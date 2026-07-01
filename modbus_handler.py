import struct
import time
import threading
import queue
from pymodbus.client import ModbusSerialClient
from pymodbus import FramerType
from config import *
from mpc_controller import global_mpc
from lqr_controller import global_lqr

# ---------------------------------------------------------
# Global Modbus State
# ---------------------------------------------------------
modbus_client = None
DEVICE_ID = 48
modbus_lock = threading.Lock()

active_ws_queues = []
active_ws_queues_lock = threading.Lock()

agent_state = {
    "agent_target": 0.0,
    "agent_wc": 5.0,
    "agent_b0": 120.0,
    "agent_ramp": 0.25,
    "mpc_active": False,
    "lqr_active": False,
}
agent_state_lock = threading.Lock()

def get_modbus():
    """Helper to get current modbus client and state"""
    return modbus_client, DEVICE_ID, modbus_lock

def connect_modbus(port: str, device_id: int) -> bool:
    global modbus_client, DEVICE_ID
    with modbus_lock:
        if modbus_client and modbus_client.connected:
            modbus_client.close()
        DEVICE_ID = device_id
        modbus_client = ModbusSerialClient(
            port=port,
            framer=FramerType.ASCII,
            baudrate=2000000,
            timeout=0.2,
        )
        return modbus_client.connect()

def disconnect_modbus():
    global modbus_client
    with modbus_lock:
        if modbus_client and modbus_client.connected:
            modbus_client.close()

def is_connected() -> bool:
    with modbus_lock:
        return bool(modbus_client and modbus_client.connected)

def modbus_polling_worker():
    """High-Speed Polling Worker Thread (~1 ms period)"""
    while True:
        time.sleep(0.001)

        if not is_connected():
            continue

        try:
            with modbus_lock:
                result = modbus_client.read_input_registers(
                    address=ADDR_MOTOR_STAT, count=22, device_id=DEVICE_ID
                )

            if hasattr(result, "isError") and result.isError():
                continue

            registers = result.registers

            # Unpack raw values directly from the Modbus registers
            raw_velocity = struct.unpack("<i", struct.pack("<HH", registers[2], registers[3]))[0]
            raw_current  = struct.unpack("<i", struct.pack("<HH", registers[4], registers[5]))[0]
            raw_target   = struct.unpack("<i", struct.pack("<HH", registers[18], registers[19]))[0]

            # Unpack z1, z2, z3 floats
            raw_z1 = struct.unpack("<f", struct.pack("<HH", registers[12], registers[13]))[0]
            raw_z2 = struct.unpack("<f", struct.pack("<HH", registers[14], registers[15]))[0]
            raw_z3 = struct.unpack("<f", struct.pack("<HH", registers[16], registers[17]))[0]

            VELOCITY_TRANSFER_SCALE = 10.0 
            
            actual_velocity = float(raw_velocity) / VELOCITY_TRANSFER_SCALE
            actual_target_vel = float(raw_target) / VELOCITY_TRANSFER_SCALE

            ADC_TO_MA = 4.698555425 
            actual_current = raw_current * ADC_TO_MA

            # Apply Exponential Moving Average (EMA) filter to smooth out current spikes and noise
            if not hasattr(modbus_polling_worker, "filtered_current"):
                modbus_polling_worker.filtered_current = actual_current
            alpha = 0.05  # Lower = smoother but more lag. 0.05 is good for 1ms polling
            modbus_polling_worker.filtered_current = (1.0 - alpha) * modbus_polling_worker.filtered_current + alpha * actual_current

            # --- MPC & LQR Logic ---
            mpc_active = False
            lqr_active = False
            with agent_state_lock:
                mpc_active = agent_state.get("mpc_active", False)
                lqr_active = agent_state.get("lqr_active", False)
                
            if mpc_active:
                actual_target_vel = global_mpc.target_vel
            elif lqr_active:
                actual_target_vel = global_lqr.target_vel
                
            current_mpc_pred_vel = []
            current_mpc_voltage = 0.0
            
            # Execute either MPC or LQR
            if mpc_active or lqr_active:
                if not hasattr(modbus_polling_worker, "last_ctrl_time"):
                    modbus_polling_worker.last_ctrl_time = 0
                current_time = time.time()
                if current_time - modbus_polling_worker.last_ctrl_time >= 0.01:
                    modbus_polling_worker.last_ctrl_time = current_time
                    
                    if mpc_active:
                        voltage, pred_vel, max_v = global_mpc.compute_step(actual_velocity, -modbus_polling_worker.filtered_current / 1000.0)
                    else:
                        voltage, pred_vel, max_v = global_lqr.compute_step(actual_velocity, -modbus_polling_worker.filtered_current / 1000.0)
                        
                    pwm_val = int((voltage / max_v) * 4000.0)
                    pwm_val = max(min(pwm_val, 4000), -4000)
                    current_mpc_pred_vel = pred_vel
                    current_mpc_voltage = voltage
                    val = struct.unpack("<H", struct.pack("<h", pwm_val))[0]
                    with modbus_lock:
                        modbus_client.write_register(ADDR_PWM_VAL, val, device_id=DEVICE_ID)

            with agent_state_lock:
                telemetry_data_point = {
                    "timestamp": time.time(),
                    "velocity": actual_velocity,
                    "current": actual_current,
                    "target_velocity": actual_target_vel,
                    "z1": raw_z1,
                    "z2": raw_z2,
                    "z3": raw_z3,
                    "agent_target": agent_state["agent_target"],
                    "agent_wc": agent_state["agent_wc"],
                    "agent_b0": agent_state["agent_b0"],
                    "agent_ramp": agent_state["agent_ramp"],
                    "mpc_pred_vel": current_mpc_pred_vel,
                    "mpc_voltage": current_mpc_voltage,
                }

            with active_ws_queues_lock:
                for ws_queue in active_ws_queues:
                    try:
                        ws_queue.put_nowait(telemetry_data_point)
                    except queue.Full:
                        pass

        except Exception as e:
            print(f"Polling error: {e}")

# Start the worker thread
threading.Thread(target=modbus_polling_worker, daemon=True).start()
