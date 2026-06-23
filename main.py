import asyncio
import struct
import time
import threading
import queue
import os
import json
import google.generativeai as genai

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import serial.tools.list_ports
from pymodbus.client import ModbusSerialClient
from pymodbus import FramerType
from pydantic import BaseModel
from pathlib import Path

app = FastAPI()
UI_PATH = Path(__file__).parent / "index.html"

# ---------------------------------------------------------
# Global State & Configurations
# ---------------------------------------------------------
modbus_client = None
DEVICE_ID = 48
modbus_lock = threading.Lock()

active_ws_queues: list[queue.Queue] = []
active_ws_queues_lock = threading.Lock()

ADDR_POS_PID   = 0
ADDR_POS_TARGET = 10
ADDR_VEL_PID   = 16
ADDR_VEL_TARGET = 26
ADDR_CUR_PID   = 32
ADDR_CUR_TARGET = 42
ADDR_PWM_VAL   = 80
ADDR_OP_MODE   = 128
ADDR_MOTOR_STAT = 0

ADDR_ADRC_POS = 368
ADDR_ADRC_VEL = 376
ADDR_ADRC_CUR = 384

# ---------------------------------------------------------
# High-Speed Polling Worker Thread  (~1 ms period)
# ---------------------------------------------------------
def modbus_polling_worker():
    while True:
        with modbus_lock:
            is_connected = modbus_client and modbus_client.connected

        if is_connected:
            try:
                with modbus_lock:
                    result = modbus_client.read_input_registers(
                        address=ADDR_MOTOR_STAT, count=22, device_id=DEVICE_ID
                    )

                if not hasattr(result, "isError") or not result.isError():
                    regs = result.registers

                    # Unpack raw values directly from the Modbus registers
                    raw_velocity = struct.unpack("<i", struct.pack("<HH", regs[2], regs[3]))[0]
                    raw_current  = struct.unpack("<i", struct.pack("<HH", regs[4], regs[5]))[0]
                    raw_target   = struct.unpack("<i", struct.pack("<HH", regs[18], regs[19]))[0]

                    # Unpack z1, z2, z3 floats
                    raw_z1 = struct.unpack("<f", struct.pack("<HH", regs[12], regs[13]))[0]
                    raw_z2 = struct.unpack("<f", struct.pack("<HH", regs[14], regs[15]))[0]
                    raw_z3 = struct.unpack("<f", struct.pack("<HH", regs[16], regs[17]))[0]

                    # --- RESTORED C++ CODELINE SCALING ---
                    # The firmware outputs fixed-point variables to protect precision.
                    # Default velocity_transfer_scale is typically 1.0 or 10.0 depending on EEPROM.
                    # If your readings are still slightly off, change this fallback divisor to 10.0.
                    VELOCITY_TRANSFER_SCALE = 10.0 
                    
                    # Convert raw values using the exact C++ structural logic
                    actual_velocity = float(raw_velocity) / VELOCITY_TRANSFER_SCALE
                    actual_target_vel = float(raw_target) / VELOCITY_TRANSFER_SCALE

                    # Board current constant calculation (remains unchanged)
                    ADC_TO_MA = 4.698555425 
                    actual_current = raw_current * ADC_TO_MA

                    pt = {
                        "timestamp": time.time(),
                        "velocity": actual_velocity,
                        "current": actual_current,
                        "target_velocity": actual_target_vel,
                        "z1": raw_z1,
                        "z2": raw_z2,
                        "z3": raw_z3,
                    }

                    with active_ws_queues_lock:
                        for q in active_ws_queues:
                            try:
                                q.put_nowait(pt)
                            except queue.Full:
                                pass

            except Exception as e:
                print(f"Polling error: {e}")

        time.sleep(0.001)

threading.Thread(target=modbus_polling_worker, daemon=True).start()

# ---------------------------------------------------------
# Pydantic Models
# ---------------------------------------------------------
class ConnectRequest(BaseModel):
    port: str
    device_id: int

class PIDRequest(BaseModel):
    mode: str
    p: float
    i: float
    d: float
    gain_output: float = 1.0
    limit_i: int = 30000
    #AI
    blend: int = 0  # เพิ่มตัวแปรผสมสัญญาณ (0 - 100)

class ADRCRequest(BaseModel):
    mode: str
    wc: float
    b0: float
    ramp_time: float

class TargetRequest(BaseModel):
    mode: str
    value: int
    min_limit: int
    max_limit: int

class OpModeRequest(BaseModel):
    mode: int

class PWMRequest(BaseModel):
    value: int

class InvertRequest(BaseModel):
    invert: bool

class SysIDRequest(BaseModel):
    waveform_type: int
    amplitude: int
    frequency: int
    offset: int
    sine_enable: bool

class ChatRequest(BaseModel):
    message: str
    context: dict

# เพิ่ม Model สำหรับรับค่าก่อนและหลังการ Transfer
class TransferRequest(BaseModel):
    mode: str
    c_pid0: float
    c_pid1: float
    c_pid2: float
    c_new0: float
    c_new1: float
    c_new2: float
    d_new1: float
    limit_i: int = 30000

# ขีดจำกัดอ้างอิงจาก HiveGround Labsheet Guide (Max 2100 RPM ที่ 12V)
MAX_SAFE_RPM = 2500.0
MAX_SAFE_CURRENT = 3000.0

# ---------------------------------------------------------
# REST API Endpoints
# ---------------------------------------------------------
@app.get("/")
def get_ui():
    return HTMLResponse(UI_PATH.read_text(encoding="utf-8"))

@app.post("/chat")
def chat_with_ai(req: ChatRequest):
    api_key = os.getenv("GEMINI_API_KEY")
    if not api_key:
        return {"response": "Error: GEMINI_API_KEY environment variable is not set on the server."}
        
    try:
        genai.configure(api_key=api_key)
        model = genai.GenerativeModel('gemini-3.1-flash-lite')
        
        system_prompt = f"""You are an AI Tutor for a DC Motor Control Lab Experiment.
The student is using a web app to tune a DC motor using PID control.
Here is the current state of their UI:
{json.dumps(req.context, indent=2)}

Use this context to answer their questions accurately. Be encouraging, educational, and avoid giving direct answers without explanation."""
        
        prompt = f"{system_prompt}\n\nStudent asks: {req.message}"
        response = model.generate_content(prompt)
        return {"response": response.text}
    except Exception as e:
        return {"response": f"AI Error: {str(e)}"}

@app.get("/ports")
def list_ports():
    ports = serial.tools.list_ports.comports()
    return {"ports": [port.device for port in ports]}

@app.post("/connect")
def connect(req: ConnectRequest):
    global modbus_client, DEVICE_ID
    with modbus_lock:
        if modbus_client and modbus_client.connected:
            modbus_client.close()
        DEVICE_ID = req.device_id
        modbus_client = ModbusSerialClient(
            port=req.port,
            framer=FramerType.ASCII,
            baudrate=2000000,
            timeout=0.2,
        )
        if not modbus_client.connect():
            return {"status": "failed", "message": "Cannot open COM port"}
    return {"status": "connected", "message": f"Port Open (ID: {DEVICE_ID})"}

@app.post("/disconnect")
def disconnect_port():
    global modbus_client
    with modbus_lock:
        if modbus_client and modbus_client.connected:
            modbus_client.close()
    return {"status": "disconnected", "message": "Port Disconnected"}

@app.post("/invert_encoder")
def invert_encoder(req: InvertRequest):
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        modbus_client.write_coil(19, req.invert, device_id=DEVICE_ID)
    return {"status": "success"}

@app.post("/reset_position")
def reset_position():
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        # Coil 11 resets absolute position according to memory layout offsets
        modbus_client.write_coil(11, True, device_id=DEVICE_ID)
        time.sleep(0.05)
        modbus_client.write_coil(11, False, device_id=DEVICE_ID)
    return {"status": "success"}

@app.post("/reset_adrc")
def reset_adrc():
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        # Coil 23 is reset_adrc
        modbus_client.write_coil(23, True, device_id=DEVICE_ID)
        time.sleep(0.05)
        modbus_client.write_coil(23, False, device_id=DEVICE_ID)
    return {"status": "success"}

@app.post("/set_op_mode")
def set_op_mode(req: OpModeRequest):
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        modbus_client.write_coil(13, False, device_id=DEVICE_ID)
        modbus_client.write_coil(3,  False, device_id=DEVICE_ID)
        modbus_client.write_coil(4,  req.mode == -1, device_id=DEVICE_ID)
        modbus_client.write_coil(5,  req.mode == -2, device_id=DEVICE_ID)
        modbus_client.write_coil(6,  req.mode == -3, device_id=DEVICE_ID)
        mode_val = struct.unpack("<H", struct.pack("<h", req.mode))[0]
        modbus_client.write_register(ADDR_OP_MODE, mode_val, device_id=DEVICE_ID)

        if req.mode != 7:
            modbus_client.write_coil(25, False, device_id=DEVICE_ID)
            restore_val_56 = struct.unpack("<2H", struct.pack("<I", 30000))
            modbus_client.write_registers(56, list(restore_val_56), device_id=DEVICE_ID)
            restore_val_58 = struct.unpack("<2H", struct.pack("<I", 0))
            modbus_client.write_registers(58, list(restore_val_58), device_id=DEVICE_ID)

    return {"status": "success"}

@app.post("/set_pid")
def set_pid(req: PIDRequest):
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        addresses = {
            "position": ADDR_POS_PID,
            "velocity": ADDR_VEL_PID,
            "current":  ADDR_CUR_PID,
        }
        addr = addresses.get(req.mode)

        #AI
        # จัดการนำค่า blend (0-100) ไปเข้าจังหวะบิตชิฟไปไว้ใน HIBYTE ของรีจิสเตอร์ที่ 10
        # ส่วน LOBYTE เป็น 0 สำหรับ reserved_for_integral_limit_accumulate
        blend_register_val = (req.blend << 8) | 0

        # Pack includes the newly exposed limit_i
        packed_bytes = struct.pack("<ffffhH", req.p, req.i, req.d, req.gain_output, req.limit_i, blend_register_val)
        regs = struct.unpack("<10H", packed_bytes)
        modbus_client.write_registers(address=addr, values=list(regs), device_id=DEVICE_ID)
    return {"status": "success"}

@app.post("/set_adrc")
def set_adrc(req: ADRCRequest):
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        addresses = {
            "position": ADDR_ADRC_POS,
            "velocity": ADDR_ADRC_VEL,
            "current":  ADDR_ADRC_CUR,
        }
        addr = addresses.get(req.mode)
        
        packed_bytes = struct.pack("<ffff", req.wc, req.b0, req.ramp_time, 0.0)
        regs = struct.unpack("<8H", packed_bytes)
        modbus_client.write_registers(address=addr, values=list(regs), device_id=DEVICE_ID)
    return {"status": "success"}

@app.post("/set_target")
def set_target(req: TargetRequest):
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
            
        addresses = {
            "position": ADDR_POS_TARGET,
            "velocity": ADDR_VEL_TARGET,
            "current":  ADDR_CUR_TARGET,
        }
        addr = addresses.get(req.mode)
        
        # Mirror the C++ Transfer Scales used in the telemetry read function
        POSITION_TRANSFER_SCALE = 1.0
        VELOCITY_TRANSFER_SCALE = 10.0
        CURRENT_TRANSFER_SCALE  = 1.0
        
        # Scale the UI input to the fixed-point format expected by the STM32
        if req.mode == "position":
            scaled_value = int(req.value * POSITION_TRANSFER_SCALE)
        elif req.mode == "velocity":
            scaled_value = int(req.value * VELOCITY_TRANSFER_SCALE)
        else:
            scaled_value = int(req.value * CURRENT_TRANSFER_SCALE)
            
        # Pack target, min_limit, and max_limit as three 32-bit integers
        packed_bytes = struct.pack("<iii", scaled_value, req.min_limit, req.max_limit)
        regs = struct.unpack("<6H", packed_bytes)
        
        modbus_client.write_registers(address=addr, values=list(regs), device_id=DEVICE_ID)
        
    return {"status": "success"}

@app.post("/start")
def start_drive():
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        modbus_client.write_coil(13, True, device_id=DEVICE_ID)
        modbus_client.write_coil(3,  True, device_id=DEVICE_ID)
    return {"status": "success"}

@app.post("/set_pwm")
def set_pwm(req: PWMRequest):
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        modbus_client.write_coil(13, True, device_id=DEVICE_ID)
        modbus_client.write_coil(3,  True, device_id=DEVICE_ID)
        val = struct.unpack("<H", struct.pack("<h", req.value))[0]
        modbus_client.write_register(ADDR_PWM_VAL, val, device_id=DEVICE_ID)
    return {"status": "success"}

@app.post("/stop")
def stop_drive():
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        modbus_client.write_coil(13, False, device_id=DEVICE_ID)
        modbus_client.write_coil(3,  False, device_id=DEVICE_ID)
    return {"status": "success"}

@app.post("/set_sysid")
def set_sysid(req: SysIDRequest):
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        
        modbus_client.write_coil(25, req.sine_enable, device_id=DEVICE_ID)
        freq_val = struct.unpack("<H", struct.pack("<h", req.frequency))[0]
        modbus_client.write_register(70, freq_val, device_id=DEVICE_ID)
        off_val = struct.unpack("<H", struct.pack("<h", req.offset))[0]
        modbus_client.write_register(71, off_val, device_id=DEVICE_ID)
        
        amp_bytes = struct.pack("<I", req.amplitude)
        amp_regs = struct.unpack("<2H", amp_bytes)
        modbus_client.write_registers(56, list(amp_regs), device_id=DEVICE_ID)
        
        wv_bytes = struct.pack("<I", req.waveform_type)
        wv_regs = struct.unpack("<2H", wv_bytes)
        modbus_client.write_registers(58, list(wv_regs), device_id=DEVICE_ID)
        
    return {"status": "success"}

@app.post("/safe_transfer")
async def safe_transfer(req: TransferRequest):
    addresses = {
        "position": ADDR_POS_PID,
        "velocity": ADDR_VEL_PID,
        "current":  ADDR_CUR_PID,
    }
    addr = addresses.get(req.mode)
    if not addr: 
        return {"error": "Invalid mode"}

    # 1. Reverse Math: แปลง C0, C1, C2 ที่รับจาก UI กลับเป็น P, I, D ที่ STM32 ต้องการ
    # (เพราะ STM32 จะแปลงมันกลับไปเป็น Target A0, A1, A2 ใหม่อีกครั้ง)
    gain_D_target = req.c_new2
    gain_P_target = -req.c_new1 - 2.0 * req.c_new2
    gain_I_target = req.c_new0 + req.c_new1 + req.c_new2
    gain_B_target = (1.0 / req.d_new1) - 1.0 if req.d_new1 != 0.0 else 0.0
    
    fade_duration_100ms = 20  # สั่ง Firmware ให้ Fade เป็นเวลา 2.0 วินาที
    
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
            
        # การบิตชิฟ (<< 8) จะนำค่า 20 ไปวางไว้ที่ reserved[1] (crossfade_time_100ms) อย่างแม่นยำ
        # และบังคับให้ integral_limit_accumulate เริ่มต้นด้วย 0
        packed_bytes = struct.pack("<ffffhH", gain_P_target, gain_I_target, gain_D_target, gain_B_target, req.limit_i, fade_duration_100ms << 8)
        regs = struct.unpack("<10H", packed_bytes)
        modbus_client.write_registers(address=addr, values=list(regs), device_id=DEVICE_ID)

    # 2. Watchdog Loop: เฝ้าระวังพฤติกรรมมอเตอร์ผ่าน Modbus ทุกๆ 50ms ระหว่างการ Fade
    tick_rate = 0.05
    steps = int(2.0 / tick_rate)
    
    for step in range(steps + 1):
        w = step / float(steps)
        
        with modbus_lock:
            result = modbus_client.read_input_registers(address=ADDR_MOTOR_STAT, count=6, device_id=DEVICE_ID)
            if not hasattr(result, "isError") or not result.isError():
                regs_stat = result.registers
                raw_vel = struct.unpack("<i", struct.pack("<HH", regs_stat[2], regs_stat[3]))[0]
                raw_cur = struct.unpack("<i", struct.pack("<HH", regs_stat[4], regs_stat[5]))[0]
                
                actual_vel = float(raw_vel) / 1.0 
                actual_cur = float(raw_cur) * 4.698555425
                
                # SAFETY TRIP: หากมอเตอร์หมุนเร็วหรือกินกระแสเกินกำหนด ให้สั่ง Snap กลับทันที!
                if abs(actual_vel) > MAX_SAFE_RPM or abs(actual_cur) > MAX_SAFE_CURRENT:
                    old_D = req.c_pid2
                    old_P = -req.c_pid1 - 2.0 * req.c_pid2
                    old_I = req.c_pid0 + req.c_pid1 + req.c_pid2
                    
                    # สั่งเขียนค่า PID เก่ากลับไป และตั้ง fade_time = 0 (ยกเลิกการ Fade ปัจจุบันและ Snap กลับทันที)
                    packed_abort = struct.pack("<ffffhH", old_P, old_I, old_D, 0.0, req.limit_i, 0)
                    regs_abort = struct.unpack("<10H", packed_abort)
                    modbus_client.write_registers(address=addr, values=list(regs_abort), device_id=DEVICE_ID)
                    
                    return {"status": "aborted", "reason": f"Safety Trip! Speed/Current spike detected."}

        # อัปเดตหน้าต่าง HTML UI Progress bar
        progress_msg = {"type": "transfer_progress", "progress": w * 100}
        with active_ws_queues_lock:
            for q in active_ws_queues:
                try: q.put_nowait(progress_msg)
                except queue.Full: pass
                    
        await asyncio.sleep(tick_rate)

    return {"status": "success"}

# ---------------------------------------------------------
# WebSocket Endpoint
# ---------------------------------------------------------
@app.websocket("/ws/telemetry")
async def telemetry_ws(websocket: WebSocket):
    await websocket.accept()
    start_time = time.time()

    q: queue.Queue = queue.Queue(maxsize=5000)
    with active_ws_queues_lock:
        active_ws_queues.append(q)

    try:
        while True:
            pts = []
            while True:
                try:
                    pt = q.get_nowait()
                    if "type" in pt and pt["type"] == "transfer_progress":
                        pts.append(pt)
                    else:
                        pts.append({
                            "time":     pt["timestamp"] - start_time,
                            "unix_time": pt["timestamp"],
                            "velocity": pt["velocity"],
                            "current":  pt["current"],
                            "target_velocity": pt.get("target_velocity", 0),
                            "z1": pt.get("z1", 0),
                            "z2": pt.get("z2", 0),
                            "z3": pt.get("z3", 0),
                        })
                except queue.Empty:
                    break

            if pts:
                await websocket.send_json(pts)

            await asyncio.sleep(0.005)

    except WebSocketDisconnect:
        pass
    except Exception as e:
        print(f"Telemetry WebSocket error: {e}")
    finally:
        with active_ws_queues_lock:
            if q in active_ws_queues:
                active_ws_queues.remove(q)