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
rand_x = np.random.uniform(0,-2)
rand_y = np.random.uniform(-0.3, 0.3)
rand_yaw = np.random.uniform(-30, 30)
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

    def reduce_qd(qd, qdlim, scale = False):
        if scale:
            scaling_factor = np.minimum(np.abs(qdlim)/np.abs(qd), 1)
            qd *= scaling_factor
        for i in range(len(qd)):
            qd[i] = min(abs(qd[i]), qdlim[i])*qd[i]/abs(qd[i])
        
        
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
                
        et = np.sum(np.abs(T_delta.A[:3,-1]))

        # Quadratic component of objective function
        Q = np.eye(fetch_camera.n * 2)
        
        Q[:2, :2] *= w_lambda(et, 1, -1) # Mobile base
        Q[2,2] *= 0.01 # Torso
        Q[fetch_camera.n:-1, fetch_camera.n:-1] *= 10 # Slack fetch camera

        # Calculate target velocity for Fetch camera to get towards target
        v_camera, _ = rtb.p_servo(wTc, wTc @ T_delta.A , 5)

        # Equality constraints to achieve velocity targets
        Aeq = fetch_camera.jacobe(fetch_camera.q)
        Aeq = np.c_[Aeq, np.ones((6,fetch_camera.n))]
        beq =  v_camera.reshape((6,))
        # beq = np.r_[beq, np.ones(fetch_camera.n)]
 
        # The minimum angle (in radians) in which the joint is allowed to approach
        # to its limit
        ps = 0.1

        # The influence angle (in radians) in which the velocity damper
        # becomes active
        pi = 0.9

        # The inequality constraints for joint limit avoidance
        Ain, bin = fetch_camera.joint_velocity_damper(ps, pi, fetch_camera.n)
        Ain = np.c_[Ain, np.zeros((5,5))]
        # bin = np.r_[bin, np.zeros(fetch_camera.n)]

        # Linear component of objective function:
        c = np.zeros(fetch_camera.n * 2)
        # Get base to face end-effector
        kε = 0.5
        bTe = fetch_camera.fkine(fetch.q, include_base=False).A
        θε = math.atan2(bTe[1, -1], bTe[0, -1])
        ε = kε * θε
        c[0] = -ε

        # Lower and upper bounds on the joint velocity
        lb = -np.r_[fetch_camera.qdlim, 100 * np.ones(5)]
        ub = np.r_[fetch_camera.qdlim, 100 * np.ones(5)]

        # Solve for the joint velocities dq
        qd_cam = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb = lb, ub = ub, solver = 'daqp')
        
        if qd_cam is None:
            qd_cam = np.linalg.pinv(fetch_camera.jacobe(fetch_camera.q)) @ v_camera
            reduce_qd(qd_cam, fetch_camera.qdlim, True)
        else:
            print("Yes")
        
        arrived =  np.linalg.norm(qd_cam) < 0.1

        fetch_camera.qd = qd_cam[:fetch_camera.n]
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