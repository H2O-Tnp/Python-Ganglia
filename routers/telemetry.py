import time
import asyncio
import queue
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from modbus_handler import active_ws_queues, active_ws_queues_lock

router = APIRouter()

@router.websocket("/ws/telemetry")
async def telemetry_ws(websocket: WebSocket):
    await websocket.accept()
    start_time = time.time()

    ws_queue = queue.Queue(maxsize=5000)
    with active_ws_queues_lock:
        active_ws_queues.append(ws_queue)

    try:
        while True:     
            telemetry_points = []
            while True: 
                try:
                    telemetry_data_point = ws_queue.get_nowait()
                    if "type" in telemetry_data_point and telemetry_data_point["type"] == "transfer_progress":
                        telemetry_points.append(telemetry_data_point)
                    else:
                        telemetry_points.append({
                            "time":     telemetry_data_point["timestamp"] - start_time,
                            "unix_time": telemetry_data_point["timestamp"],
                            "velocity": telemetry_data_point["velocity"],
                            "current":  telemetry_data_point["current"],
                            "target_velocity": telemetry_data_point.get("target_velocity", 0),
                            "z1": telemetry_data_point.get("z1", 0),
                            "z2": telemetry_data_point.get("z2", 0),
                            "z3": telemetry_data_point.get("z3", 0),
                            "agent_target": telemetry_data_point.get("agent_target", 0.0),
                            "agent_wc": telemetry_data_point.get("agent_wc", 0.0),
                            "agent_b0": telemetry_data_point.get("agent_b0", 0.0),
                            "agent_ramp": telemetry_data_point.get("agent_ramp", 0.0),
                            "mpc_pred_vel": telemetry_data_point.get("mpc_pred_vel", []),
                        })
                except queue.Empty:
                    break

            if telemetry_points:
                await websocket.send_json(telemetry_points)

            await asyncio.sleep(0.005)

    except WebSocketDisconnect:
        pass
    except Exception as e:
        print(f"Telemetry WebSocket error: {e}")
    finally:
        with active_ws_queues_lock:
            if ws_queue in active_ws_queues:
                active_ws_queues.remove(ws_queue)
