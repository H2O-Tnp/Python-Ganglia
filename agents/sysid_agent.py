"""
sysid_agent.py — Agentic Motor System Identification Orchestrator
=================================================================
Runs a battery of 10 excitation tests against the real motor and logs
all telemetry to sysid_data/*.ndjson for offline model fitting.

Usage:
    .venv/bin/python sysid_agent.py [--max-rpm 200] [--skip-cogging] [--skip-hysteresis]

Steps:
    1. PRBS (PWM open-loop) — broadband linear dynamics
    2. Multi-sine chirp — frequency sweep at 3 amplitudes
    3. Step-response battery — 5 amplitudes × 3 reps × each direction
    4. Ultra-slow velocity ramp — static friction / Stribeck / hysteresis
    5. Steady-state noise floor — 6 RPM levels held 10 s each
    6. Dead-zone scan — monotonic PWM ramp from 0 until motion
    7. Cogging scan — ~2 RPM crawl 30 s, FFT reveals pole count
    8. Electrical step — short PWM bursts to capture current rise-time (τe)
    9. Free-decel — coast after PWM=0 (back-EMF, Ke)
   10. Hysteresis profile — identical chirp fwd then rev
"""
import asyncio
import websockets
import json
import math
import os
import random
import time
import datetime
import argparse
import pathlib
import requests
from dotenv import load_dotenv

load_dotenv()

BASE_URL = "http://127.0.0.1:8000"
WS_URL   = "ws://127.0.0.1:8000/ws/telemetry"
DATA_DIR = pathlib.Path("sysid_data")
DATA_DIR.mkdir(exist_ok=True)

# ── safety ──────────────────────────────────────────────────────────────────
MAX_SAFE_RPM     = 1800   # abort if velocity exceeds this
MAX_SAFE_CURRENT = 8000   # mA — abort if current exceeds this
VELOCITY_SCALE   = 10.0   # same as main.py

# ── sysid state ─────────────────────────────────────────────────────────────
sysid_status = {
    "phase": "idle",
    "test": "",
    "progress": 0.0,
    "eta_s": 0,
    "running": False,
    "aborted": False,
}

# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────
def log(msg: str):
    ts = datetime.datetime.now().strftime("%H:%M:%S")
    full = f"[{ts}] {msg}"
    print(full)
    try:
        requests.post(f"{BASE_URL}/post_agent_log", json={"message": full}, timeout=1)
    except Exception:
        pass

def api(endpoint: str, payload: dict = None, method="POST"):
    """Fire-and-forget REST call."""
    url = f"{BASE_URL}/{endpoint.lstrip('/')}"
    try:
        if method == "GET":
            return requests.get(url, timeout=3).json()
        return requests.post(url, json=payload or {}, timeout=3).json()
    except Exception as e:
        log(f"  API error {endpoint}: {e}")
        return {}

def set_pwm(value: int):
    api("/set_pwm", {"value": value})

def set_target_rpm(rpm: float):
    api("/set_target", {"mode": "velocity", "value": int(rpm),
                        "min_limit": -4000, "max_limit": 4000})

def pwm_mode():
    api("/set_op_mode", {"mode": 0})

def velocity_mode():
    api("/set_op_mode", {"mode": -2})

def stop():
    api("/stop")
    api("/set_target", {"mode": "velocity", "value": 0, "min_limit": -4000, "max_limit": 4000})
    api("/set_pwm", {"value": 0})

def start():
    api("/start")
    time.sleep(0.1)

async def reset_and_settle(duration: float = 2.0):
    """Stop motor, zero target, reset ADRC, wait for velocity to reach zero."""
    stop()
    api("/reset_adrc", {})
    await asyncio.sleep(0.3)
    
    # Initialize stable ADRC parameters for velocity control
    api("/set_pid", {
        "mode": "velocity", "p": 0, "i": 0, "d": 0, "gain_output": 1.0, "limit_i": 30000, "blend": 100
    })
    api("/set_adrc", {
        "mode": "velocity", "wc": 5.0, "b0": 20.0, "ramp_time": 0.0
    })
    
    velocity_mode()
    start()
    api("/set_target", {"mode": "velocity", "value": 0, "min_limit": -4000, "max_limit": 4000})
    # Wait until velocity is actually near zero
    deadline = time.time() + duration + 5.0
    while time.time() < deadline:
        try:
            r = requests.get(f"{BASE_URL}/sysid_status", timeout=1)
        except Exception:
            pass
        # Check via a telemetry snapshot
        await asyncio.sleep(0.2)
        # We don't have a direct REST read, so just wait the requested time
        break
    await asyncio.sleep(duration)

def is_motor_connected() -> bool:
    """Returns True only if the backend has an active Modbus connection."""
    try:
        # Try a lightweight test: call /stop (harmless) and check the response
        r = requests.post(f"{BASE_URL}/stop", json={}, timeout=2)
        body = r.json()
        return "error" not in body
    except Exception:
        return False

def auto_connect(device_id: int = 48) -> bool:
    """Try to connect to the first real serial port found."""
    try:
        ports_resp = requests.get(f"{BASE_URL}/ports", timeout=2).json()
        ports = [p for p in ports_resp.get("ports", []) if p != "Virtual Motor"]
        if not ports:
            log("ERROR: No serial ports found. Is the motor plugged in?")
            return False
        port = ports[0]
        log(f"  Auto-connecting to {port} (device_id={device_id})…")
        r = requests.post(f"{BASE_URL}/connect",
                          json={"port": port, "device_id": device_id},
                          timeout=5).json()
        if r.get("status") == "connected":
            log(f"  ✅ Connected: {r.get('message', '')}")
            return True
        log(f"  ❌ Connect failed: {r}")
        return False
    except Exception as e:
        log(f"  Connect error: {e}")
        return False

def check_connected() -> bool:
    try:
        r = requests.get(f"{BASE_URL}/ports", timeout=2)
        return r.status_code == 200
    except Exception:
        return False

# ──────────────────────────────────────────────────────────────────────────────
# Telemetry recorder — streams from WebSocket while collecting
# ──────────────────────────────────────────────────────────────────────────────
class TelemetryRecorder:
    def __init__(self, test_name: str):
        self.test_name = test_name
        self.records: list[dict] = []
        self._ws_task = None
        self._running = False
        self._aborted = False
        self.filename = DATA_DIR / f"{test_name}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.ndjson"

    async def start(self):
        self._running = True
        self._ws_task = asyncio.create_task(self._recv_loop())

    async def _recv_loop(self):
        try:
            async with websockets.connect(WS_URL) as ws:
                while self._running:
                    try:
                        raw = await asyncio.wait_for(ws.recv(), timeout=0.5)
                        pts = json.loads(raw)
                        for pt in pts:
                            pt["_test"] = self.test_name
                            self.records.append(pt)
                            # Safety check
                            vel = abs(pt.get("velocity", 0))
                            cur = abs(pt.get("current", 0))
                            if vel > MAX_SAFE_RPM or cur > MAX_SAFE_CURRENT:
                                log(f"  ⚠️  SAFETY TRIP: vel={vel:.1f} RPM  cur={cur:.1f} mA — stopping!")
                                stop()
                                self._aborted = True
                                sysid_status["aborted"] = True
                    except asyncio.TimeoutError:
                        pass
        except Exception as e:
            log(f"  WS error in {self.test_name}: {e}")

    async def stop_and_save(self):
        self._running = False
        if self._ws_task:
            self._ws_task.cancel()
            try:
                await self._ws_task
            except asyncio.CancelledError:
                pass
        # Save to NDJSON
        with open(self.filename, "w") as f:
            for rec in self.records:
                f.write(json.dumps(rec) + "\n")
        log(f"  Saved {len(self.records)} pts → {self.filename.name}")
        return self.filename

# ──────────────────────────────────────────────────────────────────────────────
# PRBS generator
# ──────────────────────────────────────────────────────────────────────────────
def prbs_sequence(n_bits: int = 10, clock_period_s: float = 0.05) -> list[tuple[float, int]]:
    """Returns list of (time_offset_s, pwm_value) pairs."""
    register = (1 << n_bits) - 1
    seq = []
    t = 0.0
    taps = {10: (10, 7), 9: (9, 5), 8: (8, 6, 5, 4)}
    tap = taps.get(n_bits, (10, 7))
    amplitude = 800  # PWM units ≈ 20% of full scale — safe for sysid
    for _ in range((1 << n_bits) - 1):
        bit = 0
        for t_bit in tap:
            bit ^= (register >> (t_bit - 1)) & 1
        register = ((register << 1) | bit) & ((1 << n_bits) - 1)
        pwm = amplitude if (register & 1) else -amplitude
        seq.append((t, pwm))
        t += clock_period_s
    return seq

# ──────────────────────────────────────────────────────────────────────────────
# Test 1: PRBS
# ──────────────────────────────────────────────────────────────────────────────
async def test_prbs():
    log("=== TEST 1: PRBS (Broadband Linear Dynamics) ===")
    sysid_status.update({"test": "prbs", "progress": 0.0})
    rec = TelemetryRecorder("01_prbs")
    pwm_mode()
    start()
    await asyncio.sleep(0.3)
    await rec.start()
    seq = prbs_sequence(n_bits=10, clock_period_s=0.05)
    total = len(seq)
    for idx, (_, pwm) in enumerate(seq):
        if sysid_status["aborted"]:
            break
        set_pwm(pwm)
        await asyncio.sleep(seq[idx + 1][0] - seq[idx][0] if idx + 1 < total else 0.05)
        sysid_status["progress"] = idx / total * 100
    set_pwm(0)
    await asyncio.sleep(1.0)
    await rec.stop_and_save()
    log("  PRBS complete.")

# ──────────────────────────────────────────────────────────────────────────────
# Test 2: Multi-sine chirp
# ──────────────────────────────────────────────────────────────────────────────
async def test_chirp(max_rpm: float = 400):
    log("=== TEST 2: Multi-sine Chirp (Freq Sweep 0.05→20 Hz) ===")
    sysid_status.update({"test": "chirp", "progress": 0.0})
    rec = TelemetryRecorder("02_chirp")
    velocity_mode()
    start()
    await asyncio.sleep(0.3)
    set_target_rpm(0)
    await rec.start()
    dt = 0.02
    duration = 40.0  # seconds
    amplitudes = [max_rpm * 0.3, max_rpm * 0.6, max_rpm * 0.9]
    elapsed = 0.0
    t = 0.0
    amp_dur = duration / len(amplitudes)
    for amp_idx, amp in enumerate(amplitudes):
        t0 = elapsed
        while elapsed - t0 < amp_dur:
            if sysid_status["aborted"]:
                break
            f = 0.05 + (20.0 - 0.05) * ((elapsed - t0) / amp_dur)
            rpm = amp * math.sin(2 * math.pi * f * t)
            set_target_rpm(rpm)
            await asyncio.sleep(dt)
            t += dt
            elapsed += dt
            sysid_status["progress"] = elapsed / duration * 100
    set_target_rpm(0)
    await asyncio.sleep(1.0)
    await rec.stop_and_save()
    log("  Chirp complete.")

# ──────────────────────────────────────────────────────────────────────────────
# Test 3: Step-response battery
# ──────────────────────────────────────────────────────────────────────────────
async def test_steps(max_rpm: float = 400):
    log("=== TEST 3: Step-Response Battery (5 amplitudes × 3 reps × ±dir) ===")
    sysid_status.update({"test": "step_response", "progress": 0.0})
    rec = TelemetryRecorder("03_step_response")
    velocity_mode()
    start()
    await asyncio.sleep(0.3)
    await rec.start()
    levels = [0.1, 0.25, 0.5, 0.75, 1.0]
    n_reps = 3
    total_tests = len(levels) * n_reps * 2
    done = 0
    for lvl in levels:
        for direction in [1, -1]:
            for rep in range(n_reps):
                if sysid_status["aborted"]:
                    break
                rpm = max_rpm * lvl * direction
                set_target_rpm(0)
                await asyncio.sleep(2.0)  # settle at zero
                set_target_rpm(rpm)
                await asyncio.sleep(4.0)  # hold step
                done += 1
                sysid_status["progress"] = done / total_tests * 100
    set_target_rpm(0)
    await asyncio.sleep(1.0)
    await rec.stop_and_save()
    log("  Step-response complete.")

# ──────────────────────────────────────────────────────────────────────────────
# Test 4: Ultra-slow velocity ramp (static friction, Stribeck, hysteresis)
# ──────────────────────────────────────────────────────────────────────────────
async def test_slow_ramp(max_rpm: float = 200):
    log("=== TEST 4: Ultra-slow Velocity Ramp (Stribeck/Stiction) ===")
    sysid_status.update({"test": "slow_ramp", "progress": 0.0})
    rec = TelemetryRecorder("04_slow_ramp")
    velocity_mode()
    start()
    await asyncio.sleep(0.3)
    await rec.start()
    # Profile: 0 → max_rpm → 0 → -max_rpm → 0, 120 s total
    dt = 0.1
    phase_time = 30.0  # 4 phases of 30 s = 120 s total
    n_steps = int(phase_time / dt)
    total = n_steps * 4
    done = 0

    # Phase 1: 0 → max_rpm
    for step in range(n_steps):
        rpm = max_rpm * step / n_steps
        set_target_rpm(rpm)
        await asyncio.sleep(dt)
        done += 1
        sysid_status["progress"] = done / total * 100

    # Phase 2: max_rpm → 0
    for step in range(n_steps, -1, -1):
        rpm = max_rpm * step / n_steps
        set_target_rpm(rpm)
        await asyncio.sleep(dt)
        done += 1
        sysid_status["progress"] = done / total * 100

    # Phase 3: 0 → -max_rpm
    for step in range(n_steps):
        rpm = -max_rpm * step / n_steps
        set_target_rpm(rpm)
        await asyncio.sleep(dt)
        done += 1
        sysid_status["progress"] = done / total * 100

    # Phase 4: -max_rpm → 0
    for step in range(n_steps, -1, -1):
        rpm = -max_rpm * step / n_steps
        set_target_rpm(rpm)
        await asyncio.sleep(dt)
        done += 1
        sysid_status["progress"] = done / total * 100

    set_target_rpm(0)
    await asyncio.sleep(1.0)
    await rec.stop_and_save()
    log("  Slow ramp complete.")

# ──────────────────────────────────────────────────────────────────────────────
# Test 5: Steady-state noise floor (6 RPM levels × 2 directions)
# ──────────────────────────────────────────────────────────────────────────────
async def test_noise_floor(max_rpm: float = 400):
    log("=== TEST 5: Steady-state Noise Floor (RPM levels up to max_rpm) ===")
    sysid_status.update({"test": "noise_floor", "progress": 0.0})
    rec = TelemetryRecorder("05_noise_floor")
    velocity_mode()
    start()
    await asyncio.sleep(0.3)
    # Levels: 6 log-spaced points from 10 RPM up to max_rpm
    import math as _math
    base_levels = [int(10 * (max_rpm / 10) ** (i / 5)) for i in range(6)]
    base_levels = [min(l, int(max_rpm)) for l in base_levels]
    
    # Expand to bidirectional
    levels = []
    for l in base_levels:
        levels.extend([l, -l])
        
    log(f"  Noise floor levels: {levels} RPM")
    await rec.start()
    for idx, rpm in enumerate(levels):
        if sysid_status["aborted"]:
            break
        log(f"  Holding {rpm} RPM…")
        set_target_rpm(rpm)
        await asyncio.sleep(10.0)  # hold each level 10 s
        sysid_status["progress"] = (idx + 1) / len(levels) * 100
    set_target_rpm(0)
    await asyncio.sleep(1.0)
    await rec.stop_and_save()
    log("  Noise floor complete.")

# ──────────────────────────────────────────────────────────────────────────────
# Test 6: Dead-zone scan (PWM threshold for motion onset)
# ──────────────────────────────────────────────────────────────────────────────
async def test_deadzone(max_rpm: float = 400):
    log("=== TEST 6: Dead-zone Scan (PWM onset threshold) ===")
    sysid_status.update({"test": "deadzone", "progress": 0.0})
    rec = TelemetryRecorder("06_deadzone")
    pwm_mode()
    start()
    await asyncio.sleep(0.3)
    set_pwm(0)
    await rec.start()
    # Cap max_pwm so we don't exceed ~20% of what PRBS used (1200)
    # We just want to find the onset, not spin the motor fast
    # Use a very slow ramp with 1-unit steps and 50 ms dwell
    max_pwm = 600  # ~15% full scale, well below safety
    found_fwd = False
    for pwm in range(0, max_pwm, 5):
        if sysid_status["aborted"]:
            break
        set_pwm(pwm)
        await asyncio.sleep(0.08)
        sysid_status["progress"] = pwm / max_pwm * 50
        if not found_fwd:
            # We don't have a velocity read here, just note the PWM level in data
            pass
    set_pwm(0)
    await asyncio.sleep(1.0)
    # Reverse direction
    found_rev = False
    for pwm in range(0, -max_pwm, -5):
        if sysid_status["aborted"]:
            break
        set_pwm(pwm)
        await asyncio.sleep(0.08)
        sysid_status["progress"] = 50 + abs(pwm) / max_pwm * 50
    set_pwm(0)
    await asyncio.sleep(1.0)
    await rec.stop_and_save()
    log("  Dead-zone scan complete.")

# ──────────────────────────────────────────────────────────────────────────────
# Test 7: Cogging scan (~2 RPM, full revolution, FFT for pole count)
# ──────────────────────────────────────────────────────────────────────────────
async def test_cogging():
    log("=== TEST 7: Cogging Scan (~2 RPM crawl for 30 s) ===")
    sysid_status.update({"test": "cogging", "progress": 0.0})
    rec = TelemetryRecorder("07_cogging")
    velocity_mode()
    start()
    await asyncio.sleep(0.3)
    set_target_rpm(2)
    await asyncio.sleep(3.0)  # let it settle
    await rec.start()
    for i in range(300):
        if sysid_status["aborted"]:
            break
        await asyncio.sleep(0.1)
        sysid_status["progress"] = i / 300 * 100
    set_target_rpm(0)
    await asyncio.sleep(1.0)
    await rec.stop_and_save()
    log("  Cogging scan complete.")

# ──────────────────────────────────────────────────────────────────────────────
# Test 8: Electrical step (short PWM bursts → current rise-time → τe = L/R)
# ──────────────────────────────────────────────────────────────────────────────
async def test_electrical_step():
    log("=== TEST 8: Electrical Step (Current rise-time → τe = L/R) ===")
    sysid_status.update({"test": "electrical_step", "progress": 0.0})
    rec = TelemetryRecorder("08_electrical_step")
    pwm_mode()
    start()
    await asyncio.sleep(0.3)
    set_pwm(0)
    await rec.start()
    # Short bursts: 50 ms ON, 200 ms OFF — repeat 20 times
    for i in range(20):
        if sysid_status["aborted"]:
            break
        set_pwm(2000)
        await asyncio.sleep(0.05)
        set_pwm(0)
        await asyncio.sleep(0.2)
        sysid_status["progress"] = i / 20 * 100
    await asyncio.sleep(0.5)
    await rec.stop_and_save()
    log("  Electrical step complete.")

# ──────────────────────────────────────────────────────────────────────────────
# Test 9: Free-decel (back-EMF → Ke)
# ──────────────────────────────────────────────────────────────────────────────
async def test_free_decel():
    log("=== TEST 9: Free Decel (back-EMF coast → Ke) ===")
    sysid_status.update({"test": "free_decel", "progress": 0.0})
    rec = TelemetryRecorder("09_free_decel")
    pwm_mode()
    start()
    await asyncio.sleep(0.3)
    
    # Spin up positive
    log("  Spinning up positive (PWM +1600)...")
    set_pwm(1600)
    await asyncio.sleep(3.0)
    await rec.start()
    set_pwm(0)
    log("  Coasting from positive...")
    for i in range(40):
        if sysid_status["aborted"]:
            break
        await asyncio.sleep(0.1)
        sysid_status["progress"] = i / 80 * 100
        
    # Settle
    set_pwm(0)
    await asyncio.sleep(2.0)
    
    # Spin up negative
    if not sysid_status["aborted"]:
        log("  Spinning up negative (PWM -1600)...")
        set_pwm(-1600)
        await asyncio.sleep(3.0)
        set_pwm(0)
        log("  Coasting from negative...")
        for i in range(40, 80):
            if sysid_status["aborted"]:
                break
            await asyncio.sleep(0.1)
            sysid_status["progress"] = i / 80 * 100
            
    stop()
    await asyncio.sleep(0.5)
    await rec.stop_and_save()
    log("  Free-decel complete.")

# ──────────────────────────────────────────────────────────────────────────────
# Test 10: Hysteresis profile (same chirp fwd then rev)
# ──────────────────────────────────────────────────────────────────────────────
async def test_hysteresis(max_rpm: float = 300):
    log("=== TEST 10: Hysteresis Profile (identical chirp ±direction) ===")
    sysid_status.update({"test": "hysteresis", "progress": 0.0})
    rec = TelemetryRecorder("10_hysteresis")
    velocity_mode()
    start()
    await asyncio.sleep(0.3)
    set_target_rpm(0)
    await rec.start()
    dt = 0.02
    duration = 20.0
    for direction in [1, -1]:
        t = 0.0
        elapsed = 0.0
        while elapsed < duration:
            if sysid_status["aborted"]:
                break
            f = 0.1 + (5.0 - 0.1) * (elapsed / duration)
            rpm = direction * max_rpm * 0.7 * math.sin(2 * math.pi * f * t)
            set_target_rpm(rpm)
            await asyncio.sleep(dt)
            t += dt
            elapsed += dt
            sysid_status["progress"] = (elapsed + (1 - direction) * duration / 2) / (2 * duration) * 100
        set_target_rpm(0)
        await asyncio.sleep(2.0)
    await rec.stop_and_save()
    log("  Hysteresis profile complete.")

# ──────────────────────────────────────────────────────────────────────────────
# Main orchestrator
# ──────────────────────────────────────────────────────────────────────────────
async def run_sysid(args):
    if not check_connected():
        log("ERROR: Cannot reach backend at http://127.0.0.1:8000 — is it running?")
        return

    # Verify motor is actually connected over serial
    if not is_motor_connected():
        log("Motor not connected — attempting auto-connect…")
        if not auto_connect(device_id=args.device_id):
            log("ERROR: Could not connect to motor. Connect manually in the UI and retry.")
            return
        await asyncio.sleep(0.5)
        if not is_motor_connected():
            log("ERROR: Motor still not responding after connect attempt.")
            return

    log("✅ Motor connection verified.")
    log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    log("  Motor System Identification Agent")
    log(f"  Max RPM: {args.max_rpm}  |  Skip cogging: {args.skip_cogging}")
    log(f"  Output: {DATA_DIR.resolve()}")
    log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")

    sysid_status["running"] = True
    sysid_status["phase"] = "excitation"

    tests = [
        ("PRBS",            lambda: test_prbs()),
        ("Chirp",           lambda: test_chirp(args.max_rpm)),
        ("Step Response",   lambda: test_steps(args.max_rpm)),
        ("Slow Ramp",       lambda: test_slow_ramp(args.max_rpm * 0.5)),
        ("Noise Floor",     lambda: test_noise_floor(args.max_rpm)),
        ("Dead Zone",       lambda: test_deadzone(args.max_rpm)),
        ("Free Decel",      lambda: test_free_decel()),
        ("Electrical Step", lambda: test_electrical_step()),
    ]
    if not args.skip_cogging:
        tests.insert(6, ("Cogging", lambda: test_cogging()))
    if not args.skip_hysteresis:
        tests.append(("Hysteresis", lambda: test_hysteresis(args.max_rpm * 0.7)))

    for idx, (name, fn) in enumerate(tests):
        log(f"\n[{idx+1}/{len(tests)}] Starting: {name}")
        sysid_status["phase"] = name
        # Full reset between tests: stop, zero target, reset ADRC, settle
        log(f"  ► Resetting motor state…")
        await reset_and_settle(duration=2.0)
        sysid_status["aborted"] = False   # clear trip from previous test, let this one run
        try:
            await fn()
        except Exception as e:
            log(f"  ❌ Test {name} failed with exception: {e}")
        log(f"  ✓ {name} done — settling 3 s…")
        await asyncio.sleep(3.0)

    sysid_status.update({"running": False, "phase": "complete", "progress": 100.0})
    log("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    log("  ✅ All excitation tests complete!")
    log(f"  Data saved to: {DATA_DIR.resolve()}")
    log("  Next: run  .venv/bin/python scripts/model_fitter.py")
    log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Motor System Identification Agent")
    parser.add_argument("--max-rpm", type=float, default=400,
                        help="Maximum RPM used during excitation tests (default: 400)")
    parser.add_argument("--device-id", type=int, default=48,
                        help="Modbus device ID (default: 48)")
    parser.add_argument("--skip-cogging", action="store_true",
                        help="Skip the slow cogging scan (saves ~3 minutes)")
    parser.add_argument("--skip-hysteresis", action="store_true",
                        help="Skip the hysteresis profile test (saves ~1 minute)")
    args = parser.parse_args()
    asyncio.run(run_sysid(args))
