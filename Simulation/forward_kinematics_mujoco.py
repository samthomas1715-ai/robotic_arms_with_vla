import os
import time
import mujoco
import mujoco.viewer

script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "trossen_vx300s", "vx300s.xml")


model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

start_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
test_angles = [0.4, -0.5, 0.5, 0.8, -0.7, 0.0]


for i in range(len(start_angles)):
    data.qpos[i] = start_angles[i]
    data.ctrl[i] = start_angles[i] 

movement_duration = 3.0
target_reached = False
print("Starting joint movement...")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
       sim_time = data.time
       progress = min(sim_time / movement_duration, 1.0)
       for i in range(6):
          current_target = start_angles[i] + (test_angles[i] - start_angles[i]) * progress
          data.ctrl[i] = current_target
    
       mujoco.mj_step(model, data)
       if progress == 1.0 and not target_reached:
           ee_pos = data.body("gripper_link").xpos
           print("movement completed")
           print(f"target angles: {test_angles}")
           print(f"end effector position: X = {ee_pos[0]:.4f}, Y = {ee_pos[1]:.4f}, Z = {ee_pos[2]:.4f}")
           target_reached = True

       viewer.sync()
       time.sleep(0.01)
