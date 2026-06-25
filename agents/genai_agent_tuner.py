import asyncio
import websockets
import json
import statistics
import time
import os
import requests
from pydantic import BaseModel
from google import genai
from google.genai import types
from dotenv import load_dotenv

WS_URL = "ws://127.0.0.1:8000/ws/telemetry"
BASE_URL = "http://127.0.0.1:8000"

# Set up Gemini
load_dotenv()
api_key = os.environ.get("GEMINI_API_KEY")
if not api_key:
    print("Error: GEMINI_API_KEY environment variable not set.")
    exit(1)

client = genai.Client(api_key=api_key)

class TuningResult(BaseModel):
    reasoning: str
    wc: float
    b0: float
    ramp_time: float
    target_velocity: float

SYSTEM_PROMPT = """You are an expert control systems engineer and an autonomous agent tuning an Active Disturbance Rejection Controller (ADRC) for a motor.
Your goal is to optimize the motor's performance to smoothly hold the target RPM.

Control Theory Guide for ADRC:
1. 'wc' (Observer Bandwidth): Higher values respond to disturbances faster but amplify sensor noise and cause high-frequency vibration. Lower values are softer but may track poorly. Bounds: [1.0, 20.0].
2. 'b0' (System Gain): Represents how easily the motor accelerates. Higher b0 means the controller expects the motor to be easy to move, so it applies LESS control effort. If b0 is too high, the motor may stall or under-react. If b0 is too low, the motor may overshoot or oscillate. Bounds: [1.0, 50.0].
3. 'ramp_time': Defines the acceleration profile (seconds to reach full speed). Usually 0.0 for pure steady-state tuning. Bounds: [0.0, 5.0].

Analyze the statistical telemetry provided, as well as your recent HISTORY of changes.
- If the standard deviation of velocity is high, the system is oscillating. Try reducing wc or modifying b0.
- If the mean velocity is significantly below the target, it might be stalling because control effort is too low (b0 is too high).
- If your previous change made the standard deviation or steady-state error WORSE, reverse your previous action.
- Only make small adjustments per step (e.g., +/- 10-20%).

CRITICAL RULE: If a "USER INSTRUCTION" is provided that requests a specific target velocity, you MUST output exactly that requested target_velocity. Do NOT invent your own step responses or override the user's requested velocity for tuning purposes!
"""

async def measure_telemetry(duration=3.0):
    velocities = []
    currents = []
    
    current_state = {
        "target": 0,
        "wc": 0,
        "b0": 0,
        "ramp_time": 0.0
    }
    try:
        async with websockets.connect(WS_URL) as ws:
            start_time = time.time()
            while time.time() - start_time < duration:
                try:
                    data = await asyncio.wait_for(ws.recv(), timeout=0.5)
                    pts = json.loads(data)
                    for pt in pts:
                        if "velocity" in pt:
                            velocities.append(pt["velocity"])
                        if "current" in pt:
                            currents.append(pt["current"])
                except asyncio.TimeoutError:
                    continue
    except Exception as e:
        print(f"WS Error: {e}")
        return None, None

    # Read actual ADRC state from API (these fields are not in the telemetry stream)
    try:
        r = requests.get(f"{BASE_URL}/api/state", timeout=1)
        if r.status_code == 200:
            api_state = r.json()
            current_state["target"] = api_state.get("target_velocity", 0)
            current_state["wc"] = api_state.get("adrc_wc", 0)
            current_state["b0"] = api_state.get("adrc_b0", 0)
            current_state["ramp_time"] = 0.0
    except Exception:
        pass

    if not velocities:
        return None, None
        
    stats = {
        "velocity_mean": statistics.mean(velocities),
        "velocity_stdev": statistics.stdev(velocities) if len(velocities) > 1 else 0.0,
        "velocity_min": min(velocities),
        "velocity_max": max(velocities),
        "error_mean": statistics.mean([current_state["target"] - v for v in velocities]),
        "current_mean": statistics.mean(currents) if currents else 0.0,
        "current_stdev": statistics.stdev(currents) if len(currents) > 1 else 0.0,
    }
    return stats, current_state

def log_to_ui(msg: str):
    print(msg)
    try:
        requests.post(f"{BASE_URL}/post_agent_log", json={"message": msg}, timeout=1)
    except Exception:
        pass

def set_adrc(wc, b0, ramp=0.0, target=None):
    wc = max(1.0, min(20.0, float(wc)))
    b0 = max(1.0, min(150.0, float(b0)))
    ramp = max(0.0, min(5.0, float(ramp)))
    log_to_ui(f"Applying: wc={wc:.2f}, b0={b0:.2f}, ramp={ramp:.2f}")
    requests.post(f"{BASE_URL}/set_adrc", json={
        "mode": "velocity", "wc": wc, "b0": b0, "ramp_time": ramp
    })
    if target is not None:
        target = float(target)
        requests.post(f"{BASE_URL}/set_target", json={
            "mode": "velocity", "value": int(target), "min_limit": -4000, "max_limit": 4000
        })
        log_to_ui(f"Setting target to: {target:.2f} RPM")

async def agent_loop():
    log_to_ui("Starting GenAI Agentic Tuner Loop...")
    history = []
    while True:
        log_to_ui("\n--- Observing Telemetry (10s) ---")
        stats, state = await measure_telemetry(10.0)
        
        if stats is None:
            log_to_ui("Failed to read telemetry. Retrying...")
            await asyncio.sleep(1)
            continue
            
        log_to_ui(f"State: Target RPM = {state['target']}, wc = {state['wc']}, b0 = {state['b0']}")
        log_to_ui(f"Stats: Vel Mean = {stats['velocity_mean']:.2f}, Vel Stdev = {stats['velocity_stdev']:.2f}, Error Mean = {stats['error_mean']:.2f}")
        
        prompt = f"""Current Tuning State:
Target Velocity: {state['target']} RPM
wc: {state['wc']}
b0: {state['b0']}
ramp_time: {state['ramp_time']}

Observed Telemetry (Last 10 seconds):
Velocity Mean: {stats['velocity_mean']:.2f} RPM
Velocity Stdev: {stats['velocity_stdev']:.2f} RPM
Velocity Min/Max: {stats['velocity_min']:.2f} / {stats['velocity_max']:.2f} RPM
Steady-State Error Mean: {stats['error_mean']:.2f} RPM
Current Mean: {stats['current_mean']:.2f} mA
Current Stdev: {stats['current_stdev']:.2f} mA

"""
        if history:
            prompt += "RECENT TUNING HISTORY:\n"
            for i, h in enumerate(history[-3:]):
                prompt += f"Step -{len(history[-3:])-i}: wc={h['state']['wc']}, b0={h['state']['b0']} -> Vel Stdev={h['stats']['velocity_stdev']:.2f}, Error={h['stats']['error_mean']:.2f}\n"
        
        prompt += "\nBased on this, please provide the next wc, b0, ramp_time, and target_velocity to improve stability and tracking."

        user_prompt = ""
        try:
            r = requests.get(f"{BASE_URL}/api/agent_prompt", timeout=1)
            if r.status_code == 200:
                user_prompt = r.json().get("prompt", "")
        except:
            pass

        if user_prompt:
            prompt += f"\n\n*** USER INSTRUCTION ***\nThe user has issued a direct command: '{user_prompt}'\nYou MUST adjust parameters and target_velocity to fulfill this request!\n************************\n"
        else:
            prompt += f"\n\nNo user instruction. You MUST keep target_velocity = {state['target']} exactly. Only tune wc and b0."

        log_to_ui("Querying Gemini Agent...")
        try:
            response = client.models.generate_content(
                model='gemini-2.5-pro',
                contents=[
                    types.Content(role="user", parts=[types.Part.from_text(text=SYSTEM_PROMPT + "\n\n" + prompt)])
                ],
                config=types.GenerateContentConfig(
                    response_mime_type="application/json",
                    response_schema=TuningResult,
                    temperature=0.2,
                ),
            )
            
            result = response.parsed
            log_to_ui(f"🤖 Agent Reasoning: {result.reasoning}")
            
            history.append({
                "state": state,
                "stats": stats,
                "action": {"wc": result.wc, "b0": result.b0}
            })
            if len(history) > 5:
                history.pop(0)
            
            set_adrc(result.wc, result.b0, result.ramp_time, result.target_velocity)
            
            if user_prompt:
                try:
                    requests.post(f"{BASE_URL}/api/agent_prompt_clear", timeout=1)
                except Exception:
                    pass
            
            await asyncio.sleep(2)
        except Exception as e:
            log_to_ui(f"GenAI Error: {e}")
            await asyncio.sleep(2)

if __name__ == "__main__":
    asyncio.run(agent_loop())
