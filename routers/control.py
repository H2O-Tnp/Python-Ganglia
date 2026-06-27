import struct
import time
import asyncio
import queue
from fastapi import APIRouter
from config import *
from models import *
from modbus_handler import get_modbus, active_ws_queues, active_ws_queues_lock, agent_state, agent_state_lock
from mpc_controller import global_mpc

router = APIRouter()

@router.get("/api/state")
def get_state():
    with agent_state_lock:
        return {
            "target_velocity": agent_state.get("agent_target", 0.0),
            "adrc_wc": agent_state.get("agent_wc", 0.0),
            "adrc_b0": agent_state.get("agent_b0", 0.0),
            "ramp_time": agent_state.get("agent_ramp", 0.0)
        }

@router.post("/invert_encoder")
def invert_encoder(req: InvertRequest):
    modbus_client, device_id, modbus_lock = get_modbus()
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        modbus_client.write_coil(19, req.invert, device_id=device_id)
    return {"status": "success"}

@router.post("/reset_position")
def reset_position():
    modbus_client, device_id, modbus_lock = get_modbus()
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        modbus_client.write_coil(11, True, device_id=device_id)
        time.sleep(0.05)
        modbus_client.write_coil(11, False, device_id=device_id)
    return {"status": "success"}

@router.post("/reset_adrc")
def reset_adrc():
    modbus_client, device_id, modbus_lock = get_modbus()
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        modbus_client.write_coil(23, True, device_id=device_id)
        time.sleep(0.05)
        modbus_client.write_coil(23, False, device_id=device_id)
    return {"status": "success"}

@router.post("/set_op_mode")
def set_op_mode(req: OpModeRequest):
    modbus_client, device_id, modbus_lock = get_modbus()
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        modbus_client.write_coil(13, False, device_id=device_id)
        modbus_client.write_coil(3,  False, device_id=device_id)
        modbus_client.write_coil(4,  req.mode == -1, device_id=device_id)
        modbus_client.write_coil(5,  req.mode == -2, device_id=device_id)
        modbus_client.write_coil(6,  req.mode == -3, device_id=device_id)
        
        with agent_state_lock:
            agent_state["mpc_active"] = (req.mode == 3)
            
        hardware_mode = 0 if req.mode == 3 else req.mode
        mode_val = struct.unpack("<H", struct.pack("<h", hardware_mode))[0]
        modbus_client.write_register(ADDR_OP_MODE, mode_val, device_id=device_id)

        if req.mode != 7:
            modbus_client.write_coil(25, False, device_id=device_id)
            restore_val_56 = struct.unpack("<2H", struct.pack("<I", 30000))
            modbus_client.write_registers(56, list(restore_val_56), device_id=device_id)
            restore_val_58 = struct.unpack("<2H", struct.pack("<I", 0))
            modbus_client.write_registers(58, list(restore_val_58), device_id=device_id)

    return {"status": "success"}

@router.post("/set_pid")
def set_pid(req: PIDRequest):
    modbus_client, device_id, modbus_lock = get_modbus()
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        addresses = {
            "position": ADDR_POS_PID,
            "velocity": ADDR_VEL_PID,
            "current":  ADDR_CUR_PID,
        }
        addr = addresses.get(req.mode)
        blend_register_val = (req.blend << 8) | 0
        packed_bytes = struct.pack("<ffffhH", req.p, req.i, req.d, req.gain_output, req.limit_i, blend_register_val)
        registers = struct.unpack("<10H", packed_bytes)
        modbus_client.write_registers(address=addr, values=list(registers), device_id=device_id)
    return {"status": "success"}

@router.post("/set_adrc")
def set_adrc(req: ADRCRequest):
    modbus_client, device_id, modbus_lock = get_modbus()
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
        registers = struct.unpack("<8H", packed_bytes)
        modbus_client.write_registers(address=addr, values=list(registers), device_id=device_id)

    if req.mode == "velocity":
        with agent_state_lock:
            agent_state["agent_wc"] = req.wc
            agent_state["agent_b0"] = req.b0
            agent_state["agent_ramp"] = req.ramp_time

    return {"status": "success"}

@router.post("/set_mpc")
def set_mpc(req: MPCRequest):
    global_mpc.update_params(req.q_pos, req.q_vel, req.q_cur, req.r_ctrl, req.horizon)
    # We still send to mock sim if it's connected, but on real hardware this address might not exist.
    # To avoid errors on real hardware if address 400 doesn't exist, we can just try-except or ignore.
    modbus_client, device_id, modbus_lock = get_modbus()
    with modbus_lock:
        if modbus_client and modbus_client.connected:
            packed_bytes = struct.pack("<ffffi", req.q_pos, req.q_vel, req.q_cur, req.r_ctrl, req.horizon)
            registers = struct.unpack("<10H", packed_bytes)
            # This write is mainly for the mock simulator to stay in sync
            try:
                modbus_client.write_registers(address=400, values=list(registers), device_id=device_id)
            except Exception:
                pass
    return {"status": "success"}

@router.post("/set_rpm_cap")
def set_rpm_cap(req: RPMCapRequest):
    with agent_state_lock:
        agent_state["rpm_cap"] = req.cap
    return {"status": "success"}

@router.post("/set_mpc_target")
def set_mpc_target(req: MPCTargetRequest):
    cap = agent_state.get("rpm_cap", 4000.0)
    target_vel = max(-cap, min(cap, req.target_vel))
    global_mpc.update_target(req.target_pos, target_vel, req.target_cur)
    modbus_client, device_id, modbus_lock = get_modbus()
    with modbus_lock:
        if modbus_client and modbus_client.connected:
            packed_bytes = struct.pack("<fff", req.target_pos, req.target_vel, req.target_cur)
            registers = struct.unpack("<6H", packed_bytes)
            try:
                modbus_client.write_registers(address=410, values=list(registers), device_id=device_id)
            except Exception:
                pass
    return {"status": "success"}

@router.post("/set_target")
def set_target(req: TargetRequest):
    modbus_client, device_id, modbus_lock = get_modbus()
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
            
        addresses = {
            "position": ADDR_POS_TARGET,
            "velocity": ADDR_VEL_TARGET,
            "current":  ADDR_CUR_TARGET,
        }
        addr = addresses.get(req.mode)
        
        val = req.value
        if req.mode == "velocity":
            cap = agent_state.get("rpm_cap", 4000.0)
            val = max(-cap, min(cap, val))
        
        POSITION_TRANSFER_SCALE = 1.0
        VELOCITY_TRANSFER_SCALE = 10.0
        CURRENT_TRANSFER_SCALE  = 1.0
        
        if req.mode == "position":
            scaled_value = int(val * POSITION_TRANSFER_SCALE)
        elif req.mode == "velocity":
            scaled_value = int(val * VELOCITY_TRANSFER_SCALE)
        else:
            scaled_value = int(val * CURRENT_TRANSFER_SCALE)
            
        packed_bytes = struct.pack("<iii", scaled_value, int(req.min_limit), int(req.max_limit))
        registers = struct.unpack("<6H", packed_bytes)
        modbus_client.write_registers(address=addr, values=list(registers), device_id=device_id)
        
    if req.mode == "velocity":
        with agent_state_lock:
            agent_state["agent_target"] = req.value

    return {"status": "success"}

@router.post("/start")
def start_drive():
    modbus_client, device_id, modbus_lock = get_modbus()
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        modbus_client.write_coil(13, True, device_id=device_id)
        modbus_client.write_coil(3,  True, device_id=device_id)
    return {"status": "success"}

@router.post("/set_pwm")
def set_pwm(req: PWMRequest):
    modbus_client, device_id, modbus_lock = get_modbus()
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        modbus_client.write_coil(13, True, device_id=device_id)
        modbus_client.write_coil(3,  True, device_id=device_id)
        val = struct.unpack("<H", struct.pack("<h", req.value))[0]
        modbus_client.write_register(ADDR_PWM_VAL, val, device_id=device_id)
    return {"status": "success"}

@router.post("/stop")
def stop_drive():
    modbus_client, device_id, modbus_lock = get_modbus()
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        modbus_client.write_coil(13, False, device_id=device_id)
        modbus_client.write_coil(3,  False, device_id=device_id)
    return {"status": "success"}

@router.post("/set_sysid")
def set_sysid(req: SysIDRequest):
    modbus_client, device_id, modbus_lock = get_modbus()
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        
        modbus_client.write_coil(25, req.sine_enable, device_id=device_id)
        freq_val = struct.unpack("<H", struct.pack("<h", req.frequency))[0]
        modbus_client.write_register(70, freq_val, device_id=device_id)
        off_val = struct.unpack("<H", struct.pack("<h", req.offset))[0]
        modbus_client.write_register(71, off_val, device_id=device_id)
        
        amp_bytes = struct.pack("<I", req.amplitude)
        amp_regs = struct.unpack("<2H", amp_bytes)
        modbus_client.write_registers(56, list(amp_regs), device_id=device_id)
        
        wv_bytes = struct.pack("<I", req.waveform_type)
        wv_regs = struct.unpack("<2H", wv_bytes)
        modbus_client.write_registers(58, list(wv_regs), device_id=device_id)
        
    return {"status": "success"}

@router.post("/safe_transfer")
async def safe_transfer(req: TransferRequest):
    addresses = {
        "position": ADDR_POS_PID,
        "velocity": ADDR_VEL_PID,
        "current":  ADDR_CUR_PID,
    }
    addr = addresses.get(req.mode)
    if not addr: 
        return {"error": "Invalid mode"}

    gain_D_target = req.c_new2
    gain_P_target = -req.c_new1 - 2.0 * req.c_new2
    gain_I_target = req.c_new0 + req.c_new1 + req.c_new2
    gain_B_target = (1.0 / req.d_new1) - 1.0 if req.d_new1 != 0.0 else 0.0
    
    fade_duration_100ms = 20
    
    modbus_client, device_id, modbus_lock = get_modbus()

    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
            
        packed_bytes = struct.pack("<ffffhH", gain_P_target, gain_I_target, gain_D_target, gain_B_target, req.limit_i, fade_duration_100ms << 8)
        registers = struct.unpack("<10H", packed_bytes)
        modbus_client.write_registers(address=addr, values=list(registers), device_id=device_id)

    tick_rate = 0.05
    steps = int(2.0 / tick_rate)
    
    for step in range(steps + 1):
        fade_progress_ratio = step / float(steps)
        
        with modbus_lock:
            result = modbus_client.read_input_registers(address=ADDR_MOTOR_STAT, count=6, device_id=device_id)
        
        if hasattr(result, "isError") and result.isError():
            continue

        regs_stat = result.registers
        raw_vel = struct.unpack("<i", struct.pack("<HH", regs_stat[2], regs_stat[3]))[0]
        raw_cur = struct.unpack("<i", struct.pack("<HH", regs_stat[4], regs_stat[5]))[0]
        
        actual_vel = float(raw_vel) / 1.0 
        actual_cur = float(raw_cur) * 4.698555425
        
        if abs(actual_vel) > MAX_SAFE_RPM or abs(actual_cur) > MAX_SAFE_CURRENT:
            old_D = req.c_pid2
            old_P = -req.c_pid1 - 2.0 * req.c_pid2
            old_I = req.c_pid0 + req.c_pid1 + req.c_pid2
            
            packed_abort = struct.pack("<ffffhH", old_P, old_I, old_D, 0.0, req.limit_i, 0)
            regs_abort = struct.unpack("<10H", packed_abort)
            with modbus_lock:
                modbus_client.write_registers(address=addr, values=list(regs_abort), device_id=device_id)
            
            return {"status": "aborted", "reason": f"Safety Trip! Speed/Current spike detected."}

        progress_msg = {"type": "transfer_progress", "progress": fade_progress_ratio * 100}
        with active_ws_queues_lock:
            for ws_queue in active_ws_queues:
                try: 
                    ws_queue.put_nowait(progress_msg)
                except queue.Full: 
                    pass
                    
        await asyncio.sleep(tick_rate)

    return {"status": "success"}
