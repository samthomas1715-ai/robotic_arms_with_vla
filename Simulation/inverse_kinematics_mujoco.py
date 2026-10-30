import os
import time
import mujoco
import mujoco.viewer
import numpy as np

script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "trossen_vx300s", "scene.xml")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)


target_position = np.array([0.4, 0.4, 0.5]) 


step_size = 0.07     
tol = 0.01            
max_ik_steps = 100    


gripper_id = model.body("gripper_link").id

print("Launching IK solver...")

with mujoco.viewer.launch_passive(model, data) as viewer:
    
   
    viewer.user_scn.ngeom = 1
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[0], 
        mujoco.mjtGeom.mjGEOM_SPHERE, 
        np.array([0.02, 0.02, 0.02], dtype=np.float64),         
        target_position,                                         
        np.array([1, 0, 0, 0, 1, 0, 0, 0, 1], dtype=np.float64),
        np.array([1.0, 0.0, 0.0, 1.0], dtype=np.float32)         
    )
    
    while viewer.is_running():
       
        for _ in range(max_ik_steps):
           
            mujoco.mj_kinematics(model, data) 
            current_position = data.xpos[gripper_id]
            
            
            error = target_position - current_position
            
            
            if np.linalg.norm(error) < tol:
                break
                
            
            jacp = np.zeros((3, model.nv)) 
            jacr = np.zeros((3, model.nv)) 
            mujoco.mj_jacBody(model, data, jacp, jacr, gripper_id)
            
            
            dq = np.linalg.pinv(jacp) @ error
            
           
            for i in range(6): 
                data.ctrl[i] += step_size * dq[i]
                data.qpos[i] = data.ctrl[i] 
                
        
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
