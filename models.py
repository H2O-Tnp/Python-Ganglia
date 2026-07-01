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

class MPCRequest(BaseModel):
    q_pos: float
    q_vel: float
    q_cur: float
    r_ctrl: float
    horizon: int

class MPCTargetRequest(BaseModel):
    target_pos: float
    target_vel: float
    target_cur: float

class LQRRequest(BaseModel):
    q_vel: float
    q_cur: float
    r_ctrl: float

class LQRTargetRequest(BaseModel):
    target_vel: float
    target_cur: float

class TargetRequest(BaseModel):
    mode: str
    value: float
    min_limit: float = -4000.0
    max_limit: float = 4000.0

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

class RPMCapRequest(BaseModel):
    cap: float

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

class AgentLog(BaseModel):
    message: str

class AgentPromptRequest(BaseModel):
    prompt: str

class SysIDStatus(BaseModel):
    phase: str = "idle"
    test: str = ""
    progress: float = 0.0
    running: bool = False
    aborted: bool = False
