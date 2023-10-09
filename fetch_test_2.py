import swift
import roboticstoolbox as rtb
import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3
from spatialmath.base import *
import spatialgeometry as geometry
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
rand_x = np.random.uniform(0,-2)
rand_y = np.random.uniform(-0.3, 0.3)
rand_yaw = np.random.uniform(-20, 20)
random_base = SE3(rand_x,rand_y,0) * SE3.Rz(np.deg2rad(rand_yaw))
fetch.base = random_base
fetch_camera.base = random_base
fix_head_tilt = np.deg2rad(0)
fetch_camera.q[-1] = fix_head_tilt

# Create target points for vision:
points = np.column_stack(([4,0.25,1.4],
                          [4,-0.25,1.4],
                          [4,-0.25,0.9],
                          [4,0.25,0.9]))
obj_pose = SE3(4,0,1.15) * SE3.Ry(pi/2) * SE3.Rz(-pi/2)
pattern_lists = []
for i in range(points.shape[1]):
    pattern_lists.append(geometry.Sphere(radius= 0.03, pose = SE3(points[:,i]), color = (0,0,0.5,1)))

# Add a camera test object
camera = CentralCamera(f= 0.015, rho = 10e-6, imagesize = [1280, 1360], pp = [640,690], name = 'Fetch Camera')
cam_obj_test = geometry.Cuboid(scale= (0.3,0.1,0.5), pose = camera.pose, color = (0.3, 0.1, 0.1, 0.3))

# camera in Fetch camera frame
T_FC_C = SE3.Ry(pi/2) * SE3.Rz(-pi/2)
update_camera_pose(camera, fetch_camera, cam_obj_test, T_FC_C)
camera.plot_point(points, pose = camera.pose)
image_plane = plt.gca()
update_camera_view(camera, points, image_plane)
plt.pause(0.01)

# ADDING ENVIRONMENT STUFF
env_stuff = stuff()

# Add the Fetch and other shapes to the simulator
env.add(fetch)
env.add(fetch_camera)
# env.add(cam_obj_test)
env_stuff.add_to_env(env)
[env.add(pattern) for pattern in pattern_lists]

if __name__ == "__main__":

    def w_lambda(et, alpha, gamma):
        return alpha * np.power(et, gamma)

    n_base = 2
    n_arm = 8
    n_camera = 2
    n = n_base + n_arm + n_camera

    update_camera_pose(camera, fetch_camera, adj_mat= T_FC_C)
    update_camera_view(camera, points, image_plane)
    
    # The required relative pose of goal in camera frame 
    T_Cd_G = SE3.Tz(1.55) # object pose is 1.55m front-to parallel to the camera plane

    # The required relative pose of goal in Fetch camera frame
    T_FCd_G = T_FC_C * T_Cd_G


    def step_base():
        """
        PBVS for mobile base
        """
    
        update_camera_pose(camera, fetch_camera)
        
        # Estimate object's pose in camera frame
        p = camera.project_point(P = points, objpose= obj_pose)
        Te_C_G = camera.estpose(P = points, p = p, frame= 'camera')    

        # Find transform between Fetch camera and target
        Te_FC_G = T_FC_C * Te_C_G

        # Find required change in Fetch camera pose
        T_delta = Te_FC_G * T_FCd_G.inv()
        
        # Find Fetch camera pose in world frame
        wTc = fetch_camera.fkine(fetch_camera.q).A
        
        # Calculate target velocity for Fetch camera to get towards target
        v_camera, _ = rtb.p_servo(wTc, wTc @ T_delta.A , 5)

        qd_cam = np.linalg.pinv(fetch_camera.jacobe(fetch_camera.q)) @ v_camera
        
        arrived =  np.linalg.norm(qd_cam) < 0.1

        fetch_camera.qd = qd_cam
        fetch.qd[:3] = qd_cam[:3]
            
        return arrived
    

    print("Fetch initial position:") 
    fetch.base.printline()

    # Wait 1 sec before starting
    time.sleep(1)
    arrived = False
    while not arrived:
        arrived = step_base()
        update_camera_view(camera, points, image_plane, True)
        plt.pause(0.01)
        env.step(0.01)

    input('Enter to end!')
    # env.hold()