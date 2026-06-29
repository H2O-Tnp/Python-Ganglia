import mpc_controller
import numpy as np

mpc = mpc_controller.MPCState(dt=0.01)
mpc.update_params(q_pos=0.0, q_vel=10.0, q_cur=0.0, r_ctrl=1.0, horizon=10)
voltage, pred_vel, max_v = mpc.compute_step(0.0, 0.0)

print("pred_vel type:", type(pred_vel))
print("pred_vel len:", len(pred_vel))
print("pred_vel values:", pred_vel)
