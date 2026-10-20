# Leader Follower Robotic arms with VLA

## Objective
To develop a leader-follower robotic arm system for teleoperation such that follower arm imitates the leader arm, where leader arm is manually controlled, and
integration with Vision Langauge Action(VLA) model to enable autonomous single object pick and place task execution.

## Robot Kinematics
**Forward Kinematics** when the angles that links make with respect to each other are known and end-effector position is calculated using the known angles.
**Inverse Kinematics** when the end effector positions are known and orientation of links or the angles need to be calculated.
![Forward and Inverse Kinematics](images/fkik.jpg)

**DH Parameter** Denavit–Hartenberg (DH) parameters are a standardized way of describing the geometric relationship between consecutive links and joints of 
a robotic arm using four parameters.
![DH Parameter Table](images/dh-para-1.png)

**Transformation Matrix** A transformation matrix is a 4x4 matrix used to describe the position and orientation of one coordinate frame relative to another.
It contains a 3x3 rotation matrix and 3x1 translation matrix.
![DH Parameter Table](images/dh-para-2.png)

## Simulation
**Matplotlib** 3R plot
![3R-Matplotlib](images/3r.gif)

**MuJoCo** IK, FK and Teleoperation using Trossen vx300s Arm.
![Forward Kinematics](images/mfk.gif)
![Inverse Kinematics](images/mik.gif)
![Teleoperation](images/teleop_sim.gif)















