from fastapi import APIRouter
import serial.tools.list_ports
from models import ConnectRequest
from modbus_handler import connect_modbus, disconnect_modbus

router = APIRouter()

@router.get("/ports")
def list_ports():
    ports = serial.tools.list_ports.comports()
    return {"ports": [port.device for port in ports]}

@router.post("/connect")
def connect(req: ConnectRequest):
    success = connect_modbus(req.port, req.device_id)
    if not success:
        return {"status": "failed", "message": "Cannot open COM port"}
    return {"status": "connected", "message": f"Port Open (ID: {req.device_id})"}

@router.post("/disconnect")
def disconnect_port():
    disconnect_modbus()
    return {"status": "disconnected", "message": "Port Disconnected"}
