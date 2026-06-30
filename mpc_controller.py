import math
import numpy as np
from scipy.signal import cont2discrete
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

def build_mpc_matrices(M, dt, N, Q_diag, R_val):
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
    
    nx = 2
    nu = 1
    Sx = np.zeros((N * nx, nx))
    Su = np.zeros((N * nx, N * nu))
    
    A_pow = Ad
    for i in range(N):
        Sx[i*nx:(i+1)*nx, :] = A_pow
        for j in range(i+1):
            if j == i:
                Su[i*nx:(i+1)*nx, j*nu:(j+1)*nu] = Bd
            else:
                Su[i*nx:(i+1)*nx, j*nu:(j+1)*nu] = np.linalg.matrix_power(Ad, i-j) @ Bd
        A_pow = A_pow @ Ad
        
    Q_bar = np.kron(np.eye(N), np.diag(Q_diag))
    R_bar = np.kron(np.eye(N), np.array([[R_val]]))
    
    H = 2 * (Su.T @ Q_bar @ Su + R_bar)
    L_lip = np.max(np.linalg.eigvalsh(H))
    
    return Ad, Bd, Sx, Su, Q_bar, H, L_lip

def solve_qp_fgm(H, g, lb, ub, L_lip, max_iter=20):
    alpha = 1.0 / L_lip
    
    U = np.zeros_like(g)
    Y = np.zeros_like(g)
    t = 1.0
    
    for _ in range(max_iter):
        U_next = Y - alpha * (H @ Y + g)
        U_next = np.clip(U_next, lb, ub)
        
        t_next = (1.0 + np.sqrt(1.0 + 4.0 * t**2)) / 2.0
        Y = U_next + ((t - 1.0) / t_next) * (U_next - U)
        
        U = U_next
        t = t_next
        
    return U

class MPCState:
    def __init__(self, dt=0.01):
        self.dt = dt
        self.lock = threading.Lock()
        
        self.q_pos = 1.0
        self.q_vel = 1.0
        self.q_cur = 0.1
        self.r_ctrl = 1.0
        self.horizon = 10
        
        self.target_pos = 0.0
        self.target_vel = 0.0
        self.target_cur = 0.0
        
        self.integral_error = 0.0
        self.cache = None
        self.rebuild_needed = True

    def update_params(self, q_pos, q_vel, q_cur, r_ctrl, horizon):
        with self.lock:
            self.q_pos = q_pos
            self.q_vel = q_vel
            self.q_cur = q_cur
            self.r_ctrl = r_ctrl
            self.horizon = horizon
            self.rebuild_needed = True

    def update_target(self, target_pos, target_vel, target_cur):
        with self.lock:
            self.target_pos = target_pos
            self.target_vel = target_vel
            self.target_cur = target_cur
            
    def compute_step(self, current_velocity, current_current):
        with self.lock:
            if self.rebuild_needed or self.cache is None:
                q_diag = [self.q_vel, self.q_cur]
                Ad, Bd, Sx, Su, Q_bar, H, L_lip = build_mpc_matrices(MOTOR, self.dt, self.horizon, q_diag, self.r_ctrl)
                self.cache = {
                    "N": self.horizon,
                    "Sx": Sx, "Su": Su, "Q_bar": Q_bar, "H": H, "L_lip": L_lip
                }
                self.rebuild_needed = False
                
            Sx = self.cache["Sx"]
            Su = self.cache["Su"]
            Q_bar = self.cache["Q_bar"]
            H = self.cache["H"]
            N = self.cache["N"]
            
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
            
            x0 = np.array([[current_velocity_rads], [current_current]])
            
            # Compute required steady-state for target velocity to avoid steady-state error
            B = MOTOR.get("B", 1e-4)
            Kt = MOTOR.get("Kt", 0.05)
            Ke = MOTOR.get("Ke", 0.005)
            R_m = MOTOR.get("R", 0.5)
            
            # Integral action to eliminate steady state error
            velocity_error = target_vel_rads - current_velocity_rads
            
            # Only integrate if we are somewhat close to the target, or reduce windup cap significantly
            self.integral_error += velocity_error * self.dt
            # Clamp to a much smaller value (e.g., +/- 10.0 provides +/- 5V of max correction)
            self.integral_error = max(min(self.integral_error, 10.0), -10.0) 
            
            # Required steady state current (ignoring non-linear friction)
            i_ss = (B / Kt) * target_vel_rads if Kt != 0 else 0.0
            
            # If the user explicitly provided a non-zero target_cur (which is in mA), convert to A and use it
            user_t_cur_A = self.target_cur / 1000.0
            t_cur = user_t_cur_A if abs(user_t_cur_A) > 1e-5 else i_ss
            
            # Required steady state voltage + integral correction
            ki_voltage = 0.1
            u_ss = Ke * target_vel_rads + R_m * t_cur + ki_voltage * self.integral_error
            
            Xr = np.tile(np.array([[target_vel_rads], [t_cur]]), (N, 1))
            Ur = np.tile(np.array([[u_ss]]), (N, 1))
            
            R_bar = np.kron(np.eye(N), np.array([[self.r_ctrl]]))
            
            # g includes the Ur term to penalize U - Ur instead of U - 0
            g = 2 * Su.T @ Q_bar @ (Sx @ x0 - Xr) - 2 * R_bar @ Ur
            
            max_v = MOTOR.get("max_voltage", 24.0)
            lb = -max_v
            ub = max_v
            L_lip = self.cache["L_lip"]
            
            U_opt = solve_qp_fgm(H, g, lb, ub, L_lip, max_iter=20)
            voltage = U_opt[0, 0]
            
            X_pred = Sx @ x0 + Su @ U_opt
            
            # Convert predicted velocity back to RPM for UI
            pred_vel = (X_pred[0::2, 0] * 60.0 / (2 * math.pi)).tolist()
            
            return voltage, pred_vel, max_v

# Global instance for shared state
global_mpc = MPCState(dt=0.01)
