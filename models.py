from pydantic import BaseModel

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
    blend: int = 0  # 0-100%

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
