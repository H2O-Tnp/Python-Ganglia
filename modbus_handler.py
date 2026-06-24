import struct
import time
import threading
import queue
from pymodbus.client import ModbusSerialClient
from pymodbus import FramerType
from config import *

# ---------------------------------------------------------
# Global Modbus State
# ---------------------------------------------------------
modbus_client = None
DEVICE_ID = 48
modbus_lock = threading.Lock()

active_ws_queues = []
active_ws_queues_lock = threading.Lock()

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

            telemetry_data_point = {
                "timestamp": time.time(),
                "velocity": actual_velocity,
                "current": actual_current,
                "target_velocity": actual_target_vel,
                "z1": raw_z1,
                "z2": raw_z2,
                "z3": raw_z3,
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
