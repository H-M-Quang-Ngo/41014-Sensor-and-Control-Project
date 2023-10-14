import swift
import roboticstoolbox as rtb
import numpy as np
import matplotlib.pyplot as plt
import math
from spatialmath import SE3
from spatialmath.base import *
import spatialgeometry as geometry
import qpsolvers as qp
from math import pi
from support_funcs import *
from machinevisiontoolbox import CentralCamera
from environment import stuff

# Launch the simulator Swift
env = swift.Swift()
env.launch()

# Create a Fetch and Camera robot object
fetch = rtb.models.Fetch()
fetch_camera = rtb.models.FetchCamera()
fetch.links[1].qlim = [-2*np.pi, 2*np.pi]
qlim = fetch.qlim.T

# Set joint angles to zero configuration
fetch.q = fetch.qr
fetch_camera.q = fetch_camera.qr

# Set up a random position
rand_x = np.random.uniform(-1,0)
rand_y = np.random.uniform(-1, 1)
rand_yaw = -25*rand_y
random_base = SE3(rand_x,rand_y,0) * SE3.Rz(np.deg2rad(rand_yaw))
fetch.base = random_base
fetch_camera.base = random_base

# Create 3D target pose and points for vision:
x_3d, y_3d, z_3d = 4, 0, 1.1
size = 0.15 # edge length of QR
points = np.column_stack(([x_3d, y_3d + size/2 , z_3d + size/2],
                          [x_3d, y_3d - size/2 , z_3d + size/2],
                          [x_3d, y_3d - size/2 , z_3d - size/2],
                          [x_3d, y_3d + size/2 , z_3d - size/2]))
QR_pose = SE3(x_3d, y_3d, z_3d) * SE3.Ry(pi/2) * SE3.Rz(-pi/2)
pattern_lists = []
for i in range(points.shape[1]):
    pattern_lists.append(geometry.Sphere(radius= 0.02, pose = SE3(points[:,i]), color = (0,0,0.5,1)))

# Add a camera test object
camera = CentralCamera(f= 0.015, rho = 10e-6, imagesize = [640, 480], pp = [320,240], name = 'Fetch Camera')
cam_obj_test = geometry.Cuboid(scale= (0.3,0.1,0.5), pose = camera.pose, color = (0.3, 0.1, 0.1, 0.3))

# Camera in Fetch camera frame
T_FC_C = SE3.Ry(pi/2) * SE3.Rz(-pi/2)
update_camera_pose(camera, fetch_camera, cam_obj_test, T_FC_C)
camera.plot_point(points, pose = camera.pose)
image_plane = plt.gca()
update_camera_view(camera, points, image_plane)
plt.pause(0.01)

# ADDING ENVIRONMENT STUFF
env_stuff = stuff(env)
# Transform to fix object position
T_adjust = SE3(3.05, 0.96, 1.91) * SE3.RPY(0,-pi/2,-pi/2)    
can_pose_1 = SE3(env_stuff.my_cans_2.T)*T_adjust
can_pose_2 = SE3(env_stuff.my_cans_1.T)*T_adjust
can_poses = [can_pose_1, can_pose_2]

# Add the Fetch and other shapes to the simulator
env.add(fetch)
env.add(fetch_camera)
# env.add(cam_obj_test)
env_stuff.add_to_env()
[env.add(pattern) for pattern in pattern_lists]

if __name__ == "__main__":
    # Wait 1 sec before starting
    time.sleep(1)
    print("Fetch initial position:") 
    fetch.base.printline()
    
    # The required relative pose of goal in camera frame 
    T_Cd_G = SE3(0,0.15,1.5) # object pose is 1.5m front-to parallel and 0.1m lower to the camera plane

    # The required relative pose of goal in Fetch camera frame
    T_FCd_G = T_FC_C * T_Cd_G

    # Function to move the Fetch to the required position using Position base visual servoing
    def step_base_pbvs():
        """
        PBVS for mobile base
        """
        update_camera_pose(camera, fetch_camera)
        
        # Estimate object's pose in camera frame
        p = camera.project_point(P = points, objpose= QR_pose)
        Te_C_G = camera.estpose(P = points, p = p, frame= 'camera')    

        # Find transform between Fetch camera and target
        Te_FC_G = T_FC_C * Te_C_G

        # Find required change in Fetch camera pose
        T_delta = Te_FC_G * T_FCd_G.inv()
        
        # Find Fetch camera pose in world frame
        wTc = fetch_camera.fkine(fetch_camera.q).A
        
        # Spatial error between two frames
        et = np.sum(np.abs(T_delta.A[:3,-1]))

        # Calculate target velocity for Fetch camera to get towards target
        v_camera, _ = rtb.p_servo(wTc, wTc @ T_delta.A, 2)
        
        qd_cam = dls_vel(fetch_camera, v_camera, qdlim= fetch_camera.qdlim)
            
        if et > 0.2:
            qd_cam *= 2/et
        else:
            qd_cam *= 1.4
        
        arrived =  et < 0.05
        fetch_camera.qd = qd_cam[:fetch_camera.n]
        fetch.qd[:3] = qd_cam[:3]
        
        return arrived
    
    arrived = False
    while not arrived:
        arrived = step_base_pbvs()
        update_camera_view(camera, points, image_plane, True)
        plt.pause(0.01)
        env.step(0.01)

    # Distance that the Fetch maintains from the board
    dist_base = 1.6

    # Horizontal distance that the Fetch maintains from the objects in table
    dist_ob = 0.25

    # Function to control the Fetch with input goal pose
    def step(Tep:SE3|np.ndarray, 
             start:str = None, start_num:int = 0, 
             end: str = None, end_num:int = 0,
             err: float = 0.1):
        """
        Control Fetch motion base on input goal pose
        """
        if Tep is not SE3:
            Tep = SE3(Tep)

        # Initialize joint velocity
        fetch.qd = np.zeros(fetch.n)
        fetch_camera.qd = np.zeros(fetch_camera.n)

        # Current pose
        if start is not None and end is None:
            wT = fetch.fkine(fetch.q)
            end_num = fetch.n - 1
        elif start is None and end is not None:
            wT = fetch.fkine(fetch.q, end = end)
            start_num = 0
        elif start is not None and end is not None:
            wT = fetch.fkine(fetch.q, end = end)
        else:
            wT = fetch.fkine(fetch.q)
            start_num = 0
            end_num = fetch.n -1

        # Transform between current and goal pose
        T_delta = wT.inv() * Tep

        # Current spatial error
        et = np.sum(np.abs(T_delta.A[:3,-1]))

        # Spatial velocity to reach goal
        v, _ = rtb.p_servo(wT, Tep, 5)

        # Joint velocity to reach goal
        if start is not None and end is None:
            qd = dls_vel(fetch, v, start = start, qdlim = fetch.qdlim[start_num:])
        elif start is None and end is not None:
            qd = dls_vel(fetch, v, end = end, qdlim = fetch.qdlim[:end_num+1])
        elif start is not None and end is not None:
            qd = dls_vel(fetch, v, start = start, end = end, qdlim= fetch.qdlim[start_num:end_num+1])
        else:
            wT = dls_vel(fetch, v, qdlim = fetch.qdlim)

        if et > 0.2:
            qd *= 0.7/et
        else:
            qd *= 2
        
        fetch.qd[start_num:end_num+1] = qd
        fetch_camera.qd[:3] = fetch.qd[:3]

        arrived = et < err

        return arrived
    
    
    # Required pose of the Fetch torso link to pick object 1
    Tep_b1 = fetch.fkine(fetch.q, end= 'torso_lift_link').A
    Tep_b1[0,-1] = QR_pose.A[0,-1] - dist_base
    Tep_b1[1,-1] = can_pose_1.A[1,-1] - dist_ob

    # Required pose of the Fetch gripper to pick object 1
    Tep_p1 = can_pose_1.A
    Tep_p1[:3,:3] = SE3.Ry(pi/2).R
    # Tep_2[0,-1] -= 0.05
    Tep_p1[2,-1] -= 0.02
    
    # Action to move the base
    arrived = False
    while not arrived:
        arrived = step(Tep_b1, end= 'torso_lift_link', end_num= 2)     
        plt.pause(0.01)
        env.step(0.01)

    # Action to go to the pick position
    arrived = False
    while not arrived:
        arrived = step(Tep_p1, start= 'torso_lift_link', start_num= 2, err= 0.05)     
        print(fetch.qd)
        plt.pause(0.01)
        env.step(0.01)



    input('Enter to end!')
