import os
import mujoco
import time
import mujoco.viewer

script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "trossen_vx300s", "dual_scene.xml")

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)


leader_joints = [
    "leader_leader_waist", 
    "leader_leader_shoulder", 
    "leader_leader_elbow", 
    "leader_wrist_angle", 
    "leader_wrist_rotate"
]

follower_joints = [
    "follower_waist", 
    "follower_shoulder", 
    "follower_elbow", 
    "follower_wrist_angle", 
    "follower_wrist_rotate"
]
for i in range(model.njnt):
    print(model.joint(i).name)
for i in range(model.nu):
    print(model.actuator(i).name)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        for i in range(len(leader_joints)):
            current_leader_angle = data.joint(leader_joints[i]).qpos[0]
            data.actuator(follower_joints[i]).ctrl[0] = current_leader_angle
        mujoco.mj_step(model, data)
        viewer.sync()
