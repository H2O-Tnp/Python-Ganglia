import math
import numpy as np
from scipy.signal import cont2discrete
import scipy.linalg
import json
import pathlib
import threading

# Load motor model
_MODEL_PATH = pathlib.Path(__file__).parent / "motor_model.json"

def _load_model():
    defaults = {
        "R": 0.5, "L": 0.002, "Ke": 0.005, "Kt": 0.05,
        "J": 1e-4, "B": 1e-4,
        "max_voltage": 24.0,
    }
    if _MODEL_PATH.exists():
        try:
            with open(_MODEL_PATH) as f:
                data = json.load(f)
            defaults.update(data)
        except Exception as e:
            pass
    return defaults

MOTOR = _load_model()

def build_lqr_gain(M, dt, Q_diag, R_val):
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
    
    Q = np.diag(Q_diag)
    R = np.array([[R_val]])
    
    # Solve discrete algebraic Riccati equation
    P = scipy.linalg.solve_discrete_are(Ad, Bd, Q, R)
    
    # Compute optimal feedback gain K = (R + B^T P B)^(-1) (B^T P A)
    # Equivalently, scipy provides K directly but solve_discrete_are gives P.
    # We compute K:
    K = np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)
    
    return Ad, Bd, K

class LQRState:
    def __init__(self, dt=0.01):
        self.dt = dt
        self.lock = threading.Lock()
        
        self.q_vel = 1.0
        self.q_cur = 0.0
        self.r_ctrl = 1.0
        
        self.target_vel = 0.0
        self.target_cur = 0.0
        
        self.integral_error = 0.0
        self.cache = None
        self.rebuild_needed = True

    def update_params(self, q_vel, q_cur, r_ctrl):
        with self.lock:
            self.q_vel = q_vel
            self.q_cur = q_cur
            self.r_ctrl = r_ctrl
            self.rebuild_needed = True

    def update_target(self, target_vel, target_cur):
        with self.lock:
            self.target_vel = target_vel
            self.target_cur = target_cur
            
    def compute_step(self, current_velocity, current_current):
        with self.lock:
            if self.rebuild_needed or self.cache is None:
                q_diag = [self.q_vel, self.q_cur]
                Ad, Bd, K = build_lqr_gain(MOTOR, self.dt, q_diag, self.r_ctrl)
                self.cache = {
                    "Ad": Ad, "Bd": Bd, "K": K
                }
                self.rebuild_needed = False
                
            K = self.cache["K"]
            
            # Soft start ramp to prevent inrush current brownouts from step inputs
            max_step_rpm = 200.0 * self.dt  # 200 RPM/s acceleration limit
            if self.target_vel > getattr(self, 'current_target_vel', 0.0) + max_step_rpm:
                self.current_target_vel = getattr(self, 'current_target_vel', 0.0) + max_step_rpm
            elif self.target_vel < getattr(self, 'current_target_vel', 0.0) - max_step_rpm:
                self.current_target_vel = getattr(self, 'current_target_vel', 0.0) - max_step_rpm
            else:
                self.current_target_vel = self.target_vel
                
            # Convert RPM to rad/s for internal physics model
            current_velocity_rads = current_velocity * 2 * math.pi / 60.0
            target_vel_rads = self.current_target_vel * 2 * math.pi / 60.0
            
            x_k = np.array([[current_velocity_rads], [current_current]])
            
            # Compute required steady-state for target velocity to avoid steady-state error
            B = MOTOR.get("B", 1e-4)
            Kt = MOTOR.get("Kt", 0.05)
            Ke = MOTOR.get("Ke", 0.005)
            R_m = MOTOR.get("R", 0.5)
            
            # Integral action to eliminate steady state error
            velocity_error = target_vel_rads - current_velocity_rads
            
            self.integral_error += velocity_error * self.dt
            self.integral_error = max(min(self.integral_error, 10.0), -10.0) 
            
            # Required steady state current (ignoring non-linear friction)
            i_ss = (B / Kt) * target_vel_rads if Kt != 0 else 0.0
            
            # If the user explicitly provided a non-zero target_cur (which is in mA), convert to A and use it
            user_t_cur_A = self.target_cur / 1000.0
            t_cur = user_t_cur_A if abs(user_t_cur_A) > 1e-5 else i_ss
            
            x_ss = np.array([[target_vel_rads], [t_cur]])
            
            # Required steady state voltage + integral correction
            ki_voltage = 0.1
            u_ss = Ke * target_vel_rads + R_m * t_cur + ki_voltage * self.integral_error
            
            # Optimal Feedback Law: u_k = u_ss - K(x_k - x_ss)
            u_opt = u_ss - K @ (x_k - x_ss)
            voltage = float(u_opt[0, 0])
            
            max_v = MOTOR.get("max_voltage", 24.0)
            voltage = max(min(voltage, max_v), -max_v)
            
            # Predict next step velocity for telemetry UI
            x_next = self.cache["Ad"] @ x_k + self.cache["Bd"] @ np.array([[voltage]])
            pred_vel = [float(x_next[0, 0] * 60.0 / (2 * math.pi))]
            
            return voltage, pred_vel, max_v

# Global instance for shared state
global_lqr = LQRState(dt=0.01)
