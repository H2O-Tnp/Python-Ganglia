import struct
import time
import threading
import serial.tools.list_ports
import pymodbus.client
import uvicorn
import math
import collections
import pathlib

import json
import random
import numpy as np
from scipy.signal import cont2discrete

# ---------------------------------------------------------
# Load fitted motor model (if available)
# ---------------------------------------------------------
_MODEL_PATH = pathlib.Path(__file__).parent / "motor_model.json"

def _load_model():
    defaults = {
        "R": 0.5, "L": 0.002, "Ke": 0.005, "Kt": 0.05,
        "J": 1e-4, "B": 1e-4,
        "Fc": 0.02, "Fs": 0.05, "omega_s_rpm": 5.0,
        "A1": 0.0, "A2": 0.0, "A3": 0.0, "A4": 0.0, "A5": 0.0, "A6": 0.0,
        "N1": 1.0, "N2": 2.0, "N3": 3.0, "N4": 4.0, "N5": 5.0, "N6": 6.0,
        "phi1": 0.0, "phi2": 0.0, "phi3": 0.0, "phi4": 0.0, "phi5": 0.0, "phi6": 0.0,
        "sigma0": 0.5, "sigma1": 0.002,
        "deadzone_pwm_fwd": 200, "deadzone_pwm_rev": 200,
        "max_voltage": 24.0,
        # legacy brake parameters
        "BRAKE_FRICTION": 0.5,
    }
    if _MODEL_PATH.exists():
        try:
            with open(_MODEL_PATH) as f:
                data = json.load(f)
            defaults.update(data)
            print(f"[mock_sim] Loaded motor_model.json — fitted model active.")
        except Exception as e:
            print(f"[mock_sim] Could not load motor_model.json: {e} — using defaults.")
    else:
        print("[mock_sim] motor_model.json not found — using default parameters.")
    return defaults

MOTOR = _load_model()

MAX_VOLTAGE  = MOTOR["max_voltage"]
BRAKE_FRICTION = MOTOR["BRAKE_FRICTION"]  # legacy compat

# ---------------------------------------------------------
# Patching serial.tools.list_ports
# ---------------------------------------------------------
original_comports = serial.tools.list_ports.comports

class MockPortInfo:
    def __init__(self, device):
        self.device = device

def mock_comports():
    ports = original_comports()
    ports.insert(0, MockPortInfo("Virtual Motor"))
    return ports

serial.tools.list_ports.comports = mock_comports

# ---------------------------------------------------------
# Simulation State
# ---------------------------------------------------------
sim_state = {
    "velocity": 0.0,
    "position": 0.0,
    "current": 0.0,
    "target_velocity": 0.0,
    "z1": 0.0,
    "z2": 0.0,
    "z3": 0.0,
    
    "pwm_val": 0,
    "op_mode": 0,
    "brake_active": False,
    "coils": {i: False for i in range(100)},
    
    "pid_p": 0.0,
    "pid_i": 0.0,
    "pid_d": 0.0,
    "pid_integral": 0.0,
    "last_error": 0.0,
    
    "adrc_wc": 10.0,
    "adrc_b0": 50.0,
    "adrc_blend": 100,
    "adrc_z1": 0.0,
    "adrc_z2": 0.0,
    "adrc_z3": 0.0,

    "mpc_q_pos": 1.0,
    "mpc_q_vel": 1.0,
    "mpc_q_cur": 0.1,
    "mpc_r_ctrl": 0.01,
    "mpc_horizon": 10,
    "mpc_target_pos": 0.0,
    "mpc_target_vel": 0.0,
    "mpc_target_cur": 0.0,
}

telemetry_history = collections.deque(maxlen=1000)

state_lock = threading.Lock()

# ---------------------------------------------------------
# MPC Helper Functions
# ---------------------------------------------------------
def build_mpc_matrices(M, dt, N, Q_diag, R_val):
    R_m = M["R"]; L = M["L"]; Ke = M["Ke"]; Kt = M["Kt"]
    J = M["J"]; B = M["B"]
    
    Ac = np.array([
        [-B/J, Kt/J],
        [-Ke/L, -R_m/L]
    ])
    Bc = np.array([[0], [1/L]])
    Cc = np.eye(2)
    Dc = np.zeros((2,1))
    
    sys_d = cont2discrete((Ac, Bc, Cc, Dc), dt, method='zoh')
    Ad, Bd = sys_d[0], sys_d[1]
    
    nx = 2
    nu = 1
    Sx = np.zeros((N * nx, nx))
    Su = np.zeros((N * nx, N * nu))
    
    A_pow = Ad
    for i in range(N):
        Sx[i*nx:(i+1)*nx, :] = A_pow
        for j in range(i+1):
            if j == i:
                Su[i*nx:(i+1)*nx, j*nu:(j+1)*nu] = Bd
            else:
                Su[i*nx:(i+1)*nx, j*nu:(j+1)*nu] = np.linalg.matrix_power(Ad, i-j) @ Bd
        A_pow = A_pow @ Ad
        
    Q_bar = np.kron(np.eye(N), np.diag(Q_diag))
    R_bar = np.kron(np.eye(N), np.array([[R_val]]))
    
    H = 2 * (Su.T @ Q_bar @ Su + R_bar)
    L_lip = np.max(np.linalg.eigvalsh(H))
    
    return Ad, Bd, Sx, Su, Q_bar, H, L_lip

def solve_qp_fgm(H, g, lb, ub, max_iter=20):
    L_lip = np.max(np.linalg.eigvalsh(H))
    alpha = 1.0 / L_lip
    
    U = np.zeros_like(g)
    Y = np.zeros_like(g)
    t = 1.0
    
    for _ in range(max_iter):
        U_next = Y - alpha * (H @ Y + g)
        U_next = np.clip(U_next, lb, ub)
        
        t_next = (1.0 + np.sqrt(1.0 + 4.0 * t**2)) / 2.0
        Y = U_next + ((t - 1.0) / t_next) * (U_next - U)
        
        U = U_next
        t = t_next
        
    return U

# ---------------------------------------------------------
# Simulation Physics Thread
# ---------------------------------------------------------
def physics_loop():
    dt = 0.01
    obs_velocity = 0.0
    obs_current = 0.0
    while True:
        with state_lock:
            mode = sim_state["op_mode"]
            drive_enabled = sim_state.get("drive_enabled", False)
            
            voltage = 0.0
            sim_state["brake_active"] = not drive_enabled
            
            if drive_enabled:
                if mode == 0: # PWM open loop
                    voltage = (sim_state["pwm_val"] / 4000.0) * MAX_VOLTAGE
                elif mode in (1, 2, -2): # PID / ADRC (Velocity/Position/Agent)
                    # Trajectory Generator (Constant Acceleration Ramp)
                    max_accel = 100.0  # RPM/s
                    current_target = sim_state.get("ramped_target", sim_state["velocity"])
                    if current_target < sim_state["target_velocity"]:
                        current_target = min(current_target + max_accel * dt, sim_state["target_velocity"])
                    elif current_target > sim_state["target_velocity"]:
                        current_target = max(current_target - max_accel * dt, sim_state["target_velocity"])
                    sim_state["ramped_target"] = current_target

                    error = current_target - obs_velocity
                    sim_state["pid_integral"] += error * dt
                    derivative = (error - sim_state["last_error"]) / dt
                    sim_state["last_error"] = error
                    
                    pid_out = (sim_state["pid_p"] * error + 
                               sim_state["pid_i"] * sim_state["pid_integral"] + 
                               sim_state["pid_d"] * derivative)
                    
                    pid_voltage = max(min((pid_out / 4000.0) * MAX_VOLTAGE, MAX_VOLTAGE), -MAX_VOLTAGE)
                    
                    # LADRC implementation
                    blend_ratio = sim_state["adrc_blend"] / 100.0
                    wo = 3 * sim_state["adrc_wc"]  # Observer bandwidth is typically 3-5x controller bandwidth
                    b0 = sim_state["adrc_b0"]
                    applied_u = sim_state.get("last_voltage", 0.0)
                    
                    if mode in (1, -2): # Velocity Mode
                        # 1st-Order System (Velocity Control)
                        beta1 = 2 * wo
                        beta2 = wo**2
                        
                        e = sim_state["adrc_z1"] - obs_velocity
                        sim_state["adrc_z1"] += (sim_state["adrc_z2"] + b0 * applied_u - beta1 * e) * dt
                        sim_state["adrc_z2"] += (-beta2 * e) * dt
                        sim_state["adrc_z3"] = 0.0  # Not used in 1st order
                        
                        kp = sim_state["adrc_wc"]
                        u0 = kp * (current_target - sim_state["adrc_z1"])
                        
                        if b0 != 0:
                            adrc_voltage = (u0 - sim_state["adrc_z2"]) / b0
                        else:
                            adrc_voltage = 0
                            
                    else:
                        # 2nd-Order System (Position Control)
                        beta1 = 3 * wo
                        beta2 = 3 * (wo**2)
                        beta3 = wo**3
                        
                        e = sim_state["adrc_z1"] - obs_position
                        sim_state["adrc_z1"] += (sim_state["adrc_z2"] - beta1 * e) * dt
                        sim_state["adrc_z2"] += (sim_state["adrc_z3"] + b0 * applied_u - beta2 * e) * dt
                        sim_state["adrc_z3"] += (-beta3 * e) * dt
                        
                        kp = sim_state["adrc_wc"] ** 2
                        kd = 2 * sim_state["adrc_wc"]
                        u0 = kp * (sim_state["target_position"] - sim_state["adrc_z1"]) - kd * sim_state["adrc_z2"]
                        
                        if b0 != 0:
                            adrc_voltage = (u0 - sim_state["adrc_z3"]) / b0
                        else:
                            adrc_voltage = 0
                            
                    adrc_voltage = max(min(adrc_voltage, MAX_VOLTAGE), -MAX_VOLTAGE)
                    
                    voltage = (1.0 - blend_ratio) * pid_voltage + blend_ratio * adrc_voltage
                    sim_state["last_voltage"] = voltage
                elif mode == 3: # MPC
                    rebuild = False
                    if "mpc_cache" not in sim_state:
                        rebuild = True
                    else:
                        cache = sim_state["mpc_cache"]
                        if (cache["N"] != sim_state["mpc_horizon"] or
                            cache["q_pos"] != sim_state["mpc_q_pos"] or
                            cache["q_vel"] != sim_state["mpc_q_vel"] or
                            cache["q_cur"] != sim_state["mpc_q_cur"] or
                            cache["r_ctrl"] != sim_state["mpc_r_ctrl"]):
                            rebuild = True
                            
                    if rebuild:
                        N = int(sim_state.get("mpc_horizon", 10))
                        # Q_diag is [Q_vel, Q_cur]
                        q_diag = [sim_state.get("mpc_q_vel", 1.0),
                                  sim_state.get("mpc_q_cur", 0.0)]
                        r_val = sim_state.get("mpc_r_ctrl", 0.01)
                        
                        Ad, Bd, Sx, Su, Q_bar, H, L_lip = build_mpc_matrices(M, dt, N, q_diag, r_val)
                        sim_state["mpc_cache"] = {
                            "N": N, "q_vel": q_diag[0], "q_cur": q_diag[1], "r_ctrl": r_val,
                            "Sx": Sx, "Su": Su, "Q_bar": Q_bar, "H": H, "L_lip": L_lip
                        }
                    
                    cache = sim_state["mpc_cache"]
                    Sx, Su, Q_bar, H = cache["Sx"], cache["Su"], cache["Q_bar"], cache["H"]
                    N = cache["N"]
                    
                    x0 = np.array([[sim_state["velocity"]], [sim_state.get("current_A", 0.0)]])
                    trg_vel = sim_state.get("mpc_target_vel", 0.0)
                    # We only care about velocity target. Current target is 0 but we don't penalize it heavily.
                    Xr = np.tile(np.array([[trg_vel], [0.0]]), (N, 1))
                    
                    g = 2 * Su.T @ Q_bar @ (Sx @ x0 - Xr)
                    
                    lb = -MAX_VOLTAGE
                    ub = MAX_VOLTAGE
                    
                    U_opt = solve_qp_fgm(H, g, lb, ub, max_iter=20)
                    voltage = U_opt[0, 0]
                    sim_state["last_voltage"] = voltage
                    
                    X_pred = Sx @ x0 + Su @ U_opt
                    pred_vel = X_pred[0::2, 0].tolist()
                    sim_state["mpc_pred_vel"] = pred_vel
            
            # ── Nonlinear Motor Physics ──────────────────────────────────────
            # State: current (A), angular velocity (rad/s), position (rad)
            # All in SI; convert RPM → rad/s at read, rad/s → RPM at write
            M = MOTOR
            R    = M["R"]; L = M["L"]; Ke = M["Ke"]; Kt = M["Kt"]
            J    = M["J"]; B = M["B"]; Fc = M["Fc"]; Fs = M["Fs"]
            ws   = M["omega_s_rpm"] * 2 * math.pi / 60.0  # convert to rad/s

            omega = sim_state["velocity"] * 2 * math.pi / 60.0  # RPM → rad/s
            theta = sim_state["position"] * 2 * math.pi / 60.0  # position in rad
            i_cur = sim_state.get("current_A", 0.0)             # state: A

            # Calculate electromagnetic torque
            Tm = Kt * i_cur

            # Static vs Dynamic friction
            if abs(omega) < 1e-3:
                # Motor is at rest. It only starts moving if EM torque exceeds static friction (stiction).
                # The stiction torque is determined by the dead-zone PWM threshold.
                dz_pwm = M["deadzone_pwm_fwd"] if Tm >= 0 else M["deadzone_pwm_rev"]
                V_dz = (dz_pwm / 4000.0) * MAX_VOLTAGE
                T_stiction = Kt * (V_dz / R)
                
                if abs(Tm) > T_stiction:
                    # Accelerate out of stiction
                    dom_dt = (Tm - math.copysign(T_stiction, Tm)) / max(J, 1e-9)
                else:
                    # Locked by stiction
                    dom_dt = 0.0
                    omega = 0.0
            else:
                # Motor is in motion. Dynamic friction applies (Coulomb + Stribeck + Viscous)
                Tf = (Fc * math.copysign(1, omega) * (1.0 - math.exp(-abs(omega) / (ws + 1e-9)))
                    + Fs * math.exp(-(omega / (ws + 1e-9))**2) * math.copysign(1, omega)
                    + B * omega)
                
                # Cogging harmonics
                for ci in range(1, 7):
                    Tf += M[f"A{ci}"] * math.sin(M[f"N{ci}"] * theta + M[f"phi{ci}"])

                # Brake adds extra friction when drive is disabled
                if sim_state["brake_active"]:
                    Tf += math.copysign(BRAKE_FRICTION, omega)

                dom_dt = (Tm - Tf) / max(J, 1e-9)

            # Electrical ODE: Exact discrete solution to prevent numerical instability
            # since dt (0.01) >> L/R (0.0004)
            if R > 1e-6:
                i_ss = (voltage - Ke * omega) / R
                i_cur = i_ss + (i_cur - i_ss) * math.exp(-(R / max(L, 1e-9)) * dt)
            else:
                di_dt = (voltage - Ke * omega) / max(L, 1e-9)
                i_cur += di_dt * dt
                
            i_cur = max(min(i_cur, 30.0), -30.0)  # hardware current limit (A)
            omega += dom_dt * dt
            theta += omega * dt

            # Write back state
            sim_state["current_A"]   = i_cur
            sim_state["current"]     = i_cur * 1000.0           # mA for telemetry
            sim_state["velocity"]    = omega * 60.0 / (2 * math.pi)  # rad/s → RPM
            sim_state["position"]    = theta * 60.0 / (2 * math.pi)

            sim_state["z1"] = sim_state["position"]
            sim_state["z2"] = sim_state["velocity"]
            sim_state["z3"] = dom_dt * 60.0 / (2 * math.pi)  # angular accel in RPM/s

            # Heteroscedastic noise injection
            sig = M["sigma0"] + M["sigma1"] * abs(sim_state["velocity"])
            obs_velocity = sim_state["velocity"] + random.gauss(0, sig)
            obs_current  = sim_state["current"]  + random.gauss(0, 2.0)
            
            telemetry_history.append({
                "time": time.time(),
                "position": sim_state["position"] + random.gauss(0, 0.05),
                "velocity": obs_velocity,
                "current":  obs_current,
                "target_velocity": sim_state.get("ramped_target", sim_state["target_velocity"]),
                "z1": sim_state["z1"],
                "z2": sim_state["z2"],
                "z3": sim_state["z3"],
                "mpc_pred_pos": sim_state.get("mpc_pred_pos", []),
                "mpc_pred_vel": sim_state.get("mpc_pred_vel", [])
            })
            
        time.sleep(dt)

threading.Thread(target=physics_loop, daemon=True).start()

# ---------------------------------------------------------
# Mock Modbus Client
# ---------------------------------------------------------
class MockResult:
    def __init__(self, registers):
        self.registers = registers
    def isError(self):
        return False

class MockModbusClient:
    def __init__(self, port, framer, baudrate, timeout):
        self.port = port
        self.connected = False

    def connect(self):
        if self.port == "Virtual Motor":
            self.connected = True
            sim_state["op_mode"] = -2  # default to velocity mode
            return True
        return False

    def close(self):
        self.connected = False

    def read_input_registers(self, address, count, device_id):
        if address == 0 and count == 22:
            with state_lock:
                vel_raw = max(-2147483648, min(2147483647, int(sim_state["velocity"] * 10.0)))
                cur_raw = max(-2147483648, min(2147483647, int(sim_state["current"] / 4.698555425)))
                target_raw = max(-2147483648, min(2147483647, int(sim_state["target_velocity"] * 10.0)))
                
                regs = [0] * 22
                
                def pack_int(val, idx1, idx2):
                    b = struct.pack("<i", val)
                    regs[idx1], regs[idx2] = struct.unpack("<HH", b)
                    
                def pack_float(val, idx1, idx2):
                    b = struct.pack("<f", float(val))
                    regs[idx1], regs[idx2] = struct.unpack("<HH", b)
                
                pack_int(vel_raw, 2, 3)
                pack_int(cur_raw, 4, 5)
                pack_int(target_raw, 18, 19)
                pack_float(sim_state["z1"], 12, 13)
                pack_float(sim_state["z2"], 14, 15)
                pack_float(sim_state["z3"], 16, 17)
                
            return MockResult(regs)
        return MockResult([0]*count)

    def write_coil(self, address, value, device_id):
        with state_lock:
            sim_state["coils"][address] = value
            if address == 11 and value:
                sim_state["position"] = 0.0
            if address == 13:
                sim_state["drive_enabled"] = bool(value)
                
    def write_register(self, address, value, device_id):
        with state_lock:
            if address == 128:
                sim_state["op_mode"] = struct.unpack("<h", struct.pack("<H", value))[0]
            elif address == 80:
                sim_state["pwm_val"] = struct.unpack("<h", struct.pack("<H", value))[0]

    def write_registers(self, address, values, device_id):
        with state_lock:
            if address in (0, 16, 32):
                if len(values) >= 10:
                    b = struct.pack("<10H", *values[:10])
                    p, i, d, b_gain, limit, fade = struct.unpack("<ffffhH", b)
                    sim_state["pid_p"] = p
                    sim_state["pid_i"] = i
                    sim_state["pid_d"] = d
                
            if address in (10, 26, 42):
                if len(values) >= 6:
                    b = struct.pack("<6H", *values[:6])
                    val, min_l, max_l = struct.unpack("<iii", b)
                    if address == 26:
                        sim_state["target_velocity"] = val / 10.0

            if address in (368, 376, 384):  # ADRC pos/vel/cur
                if len(values) >= 8:
                    b = struct.pack("<8H", *values[:8])
                    wc, b0, ramp_time, _ = struct.unpack("<ffff", b)
                    sim_state["adrc_wc"] = wc
                    sim_state["adrc_b0"] = b0

            if address == 500:
                if len(values) >= 10:
                    b = struct.pack("<10H", *values[:10])
                    q_p, q_v, q_c, r_c, horizon = struct.unpack("<ffffi", b)
                    sim_state["mpc_q_pos"] = q_p
                    sim_state["mpc_q_vel"] = q_v
                    sim_state["mpc_q_cur"] = q_c
                    sim_state["mpc_r_ctrl"] = r_c
                    sim_state["mpc_horizon"] = max(1, min(50, horizon))
            if address == 510:
                if len(values) >= 6:
                    b = struct.pack("<6H", *values[:6])
                    t_p, t_v, t_c = struct.unpack("<fff", b)
                    sim_state["mpc_target_pos"] = t_p
                    sim_state["mpc_target_vel"] = t_v
                    sim_state["mpc_target_cur"] = t_c

pymodbus.client.ModbusSerialClient = MockModbusClient

if __name__ == "__main__":
    import main
    import pathlib
    from pydantic import BaseModel
    
    main.UI_PATH = pathlib.Path(__file__).parent / "templates" / "sim_index.html"
    
    class TuneAdrcReq(BaseModel):
        wc: float = None
        b0: float = None
        blend: int = None
        
    @main.app.post("/api/tune_adrc")
    async def tune_adrc_endpoint(req: TuneAdrcReq):
        from modbus_handler import agent_state, agent_state_lock
        with state_lock:
            if req.wc is not None: sim_state["adrc_wc"] = req.wc
            if req.b0 is not None: sim_state["adrc_b0"] = req.b0
            if req.blend is not None: sim_state["adrc_blend"] = req.blend
            
            update_msg = {
                "type": "tuning_update", 
                "wc": sim_state["adrc_wc"], 
                "b0": sim_state["adrc_b0"], 
                "blend": sim_state["adrc_blend"]
            }
            
        with agent_state_lock:
            if req.wc is not None: agent_state["agent_wc"] = req.wc
            if req.b0 is not None: agent_state["agent_b0"] = req.b0

        with main.active_ws_queues_lock:
            for q in main.active_ws_queues:
                try: q.put_nowait(update_msg)
                except: pass
        return {"status": "success", "state": update_msg}
        
    @main.app.get("/api/state")
    async def get_state():
        with state_lock:
            return dict(sim_state)
        
    @main.app.get("/api/history")
    async def get_history(count: int = 1000):
        with state_lock:
            hist = list(telemetry_history)[-count:]
        return {"data": hist}
        
    import os
    from fastapi.staticfiles import StaticFiles
    if not os.path.exists(pathlib.Path(__file__).parent / "static"):
        os.makedirs(pathlib.Path(__file__).parent / "static")
    main.app.mount("/static", StaticFiles(directory=str(pathlib.Path(__file__).parent / "static")), name="static")
        
    print("Starting Virtual Motor Simulation (with new Brake Sim UI)...")
    uvicorn.run(main.app, host="127.0.0.1", port=8000)
