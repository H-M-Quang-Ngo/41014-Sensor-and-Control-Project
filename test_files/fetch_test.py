import sys
sys.path.append('./')

import swift
import roboticstoolbox as rtb
import numpy as np
import matplotlib.pyplot as plt
import time
import threading
from spatialmath import SE3
import spatialgeometry as geometry
from math import pi
from machinevisiontoolbox import CentralCamera
from environment import stuff

STOP_FLAG = False

def update_camera_pose(cam: CentralCamera, 
                       fetch_cam: rtb.models.FetchCamera, 
                       cam_object: geometry.CollisionShape = None,
                       adj_mat: SE3 = SE3.Ry(pi/2) * SE3.Rz(-pi/2)):
    cam.pose = SE3(fetch_cam.fkine(fetch_cam.q).A @ adj_mat.A)
    if cam_object is not None:
        cam_object.T = cam.pose

def update_camera_view(camera: CentralCamera, points: np.ndarray, image_plane, single_use = False):
    while not STOP_FLAG:
        uv = camera.plot_point(P = points, pose = camera.pose, ax = image_plane)
        for i in range(uv.shape[1]):
            image_plane.text(uv[0, i], uv[1, i], str(i+1), color='red', fontsize=12)
        if single_use:
            break
        time.sleep(0.01)

def distance(q1, q2):
    """
    Euclidean distance between two joint state vector
    """
    try:
        if q1 is None or q2 is None:
            return 0
        q1_array = np.array(q1)
        q2_array = np.array(q2)
        if q1_array.shape != q2_array.shape:
            raise ValueError("Input vectors must have the same shape")
        return np.linalg.norm(q2_array - q1_array)
    except Exception as e:
        print(f"Error in distance calculation: {e}")
        return 0

def gen_traj(q1, q2, step_size = np.deg2rad(5), traj_type='trapezoidal'):
    if np.array_equal(q1, q2):
        return [q1]

    num_step = 2
    traj_function = rtb.jtraj if traj_type == 'quintic' else rtb.mtraj
    max_step = float("inf")

    while True:
        if traj_type == 'quintic':
            traj = traj_function(q1, q2, num_step).q
        else:
            traj = traj_function(tfunc=rtb.trapezoidal, q0= q1, qf= q2, t=num_step).q

        step_list = [distance(traj[i], traj[i+1]) for i in range(len(traj) - 1)]
        max_step = np.max(np.array(step_list))

        if max_step <= step_size:
            break

        num_step += 2

    return traj

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
random_base = SE3(0,0,0)
fetch.base = random_base
fetch_camera.base = random_base
fix_head_tilt = np.deg2rad(0)
fetch_camera.q[-1] = fix_head_tilt

# Create target points for vision:
points = np.column_stack(([4,0.25,1.4],
                          [4,-0.25,1.4],
                          [4,0.25,0.9],
                          [4,-0.25,0.9]))

pattern_lists = []
for i in range(points.shape[1]):
    pattern_lists.append(geometry.Sphere(radius= 0.03, pose = SE3(points[:,i]), color = (0,0,0.5,1)))

# pattern_1 = geometry.Sphere(radius= 0.03, pose = SE3(points[:,0]), color = (0,0,0.5,1))
# pattern_2 = geometry.Sphere(radius= 0.03, pose = SE3(points[:,1]), color = (0,0,0.5,1))

# Add a camera test object
camera = CentralCamera(f= 0.015, rho = 10e-6, imagesize = [1280, 1360], pp = [640,690], name = 'Fetch Camera')
cam_obj_test = geometry.Cuboid(scale= (0.3,0.1,0.5), pose = camera.pose, color = (0.3, 0.1, 0.1, 0.3))
update_camera_pose(camera, fetch_camera, cam_obj_test)
camera.plot_point(points, pose = camera.pose)
image_plane = plt.gca()
update_camera_view(camera, points, image_plane, single_use= True)

# Create a random trajectory
q_rand = np.zeros(fetch.n)
for i in range(len(q_rand)):
    if i == 0:
        continue
    if i == 1:
        # q_rand[i] = np.random.uniform(-5,5)
        q_rand[i] = 2.3
        continue
    q_rand[i] = np.random.uniform(qlim[i,0], qlim[i,1])
traj = gen_traj(fetch.q, q_rand)

# ADDING ENVIRONMENT STUFF
env_stuff = stuff()

# Add the Fetch and other shapes to the simulator
env.add(fetch)
env.add(fetch_camera)
# env.add(cam_obj_test)
env_stuff.add_to_env(env)
[env.add(pattern) for pattern in pattern_lists]

if __name__ == "__main__":

    def update_camera_view():
        camera.clf()
        uv = camera.plot_point(P = points, pose = camera.pose)
        image_plane = plt.gca()
        for i in range(uv.shape[1]):
            image_plane.text(uv[0, i], uv[1, i], str(i+1), color='red', fontsize=12)

    # cam_move_thread = threading.Thread(target= update_camera_view, args = (camera, points, image_plane))
    # cam_move_thread.start()
    update_camera_view()
    plt.pause(0.01)

    # Run the robot
    for q in traj:
        fetch.q = q
        fetch_camera.q = q[0:fetch_camera.n]
        fetch_camera.q[3:5] = [0,fix_head_tilt]
        update_camera_pose(camera, fetch_camera, cam_obj_test)
        update_camera_view()
        print("A:\n",fetch.fkine(fetch.q, end= 'base_link'))
        print("B:\n",fetch.fkine(fetch.q, end= 'base_link'))
        env.step(0)
        plt.pause(0.01)

    STOP_FLAG = True
    # cam_move_thread.join()  
    
    ## Check camera position
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # camera.plot(scale= 0.25, solid= True, ax = ax)
    # plt.show()

    plt.show()
    # env.hold()