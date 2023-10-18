## @file
#  @brief Include some supporting functions for the main code, including velocity computation, camera view update, trajectory generator, etc.
#  @author Ho Minh Quang Ngo
#  @date Oct 1, 2023

import roboticstoolbox as rtb
import numpy as np
# import time
import spatialgeometry as geometry
from spatialmath import SE3
from math import pi
from machinevisiontoolbox import CentralCamera

def update_camera_pose(cam: CentralCamera, 
                       fetch_cam: rtb.models.FetchCamera, 
                       cam_object: geometry.CollisionShape = None,
                       adj_mat: SE3 = SE3.Ry(pi/2) * SE3.Rz(-pi/2)):
    """
    Update the camera pose base on the Fetch Camera
    
    @param `cam`        : Camera object
    @param `fetch_cam`  : Fetch platform with camera only (no arm)
    @param `cam_object` : (optional) An object to represent the camera in environment 
    @param `adj_mat`    : relation matrix between camera object and Fetch camera
    """
    cam.pose = SE3(fetch_cam.fkine(fetch_cam.q).A @ adj_mat.A)
    if cam_object is not None:
        cam_object.T = cam.pose

def update_camera_view(camera: CentralCamera, points: np.ndarray, image_plane, trace = False):
    """
    Update the image plane of the camera
    
    @param `camera`:        Camera object
    @param `points`:        N points in global frame of format 3xN `ndarray`
    @param `image_plane`:   image plane of the camera
    @param `trace`:         visualize the trajectory in camera frame, `False` by default
    """
    if not hasattr(update_camera_view, '_trace_points'):
        update_camera_view._trace_points = []

    camera.clf()
    uv = camera.plot_point(np.hstack((points, points[:,0].reshape((3,1)))), 
                           'b', linewidth = 3, pose = camera.pose, ax = image_plane)
    for i in range(uv.shape[1]-1):
        image_plane.text(uv[0, i], uv[1, i], str(i+1), color='red', fontsize=12)
    
    if trace:
        update_camera_view._trace_points.append(uv)
        pts = np.stack(update_camera_view._trace_points, axis=-1)
        camera.plot_point(np.hstack((update_camera_view._trace_points[0], 
                                     update_camera_view._trace_points[0][:,0].reshape((2,1)))),
                                     'b', linewidth = 3, pose = camera.pose, ax = image_plane)
        [camera.plot_point(pts[:,i], 'r', markersize = 0.01) for i in range(pts.shape[1])]
    else:
        update_camera_view._trace_points = []

    # time.sleep(0.01)

def reduce_qd(qd, qdlim, scale = False):
    """
    Constrain the joint velocity within the joint velocity limit
    
    @param `qd`     : input joint velocity vector 1xN array-like
    @param `qdlim`  : joint velocity limit vector 1xN array-like
    @param `scale`  : if `True`, scale the velocity while keeping its initial direction
    
    @return         : valid joint velocity within limit
    """
    if scale:
        scaling_factor = np.minimum(np.abs(qdlim)/np.abs(qd), 1)
        qd *= scaling_factor
        # return qd
    else:
        for i in range(len(qd)):
            qd[i] = min(abs(qd[i]), qdlim[i])*qd[i]/abs(qd[i]) 
    return qd

def dls_vel(robot: rtb.Robot, v:np.ndarray, epsilon: float = 0.01, max_lambda: float = 0.5, 
            start: str = None, 
            end: str = None, 
            qdlim: np.ndarray = None, scale: bool = True):
    """
    Solve for joint velocity using Damped-Least Squares
    
    @param `robot`      : input robot
    @param `v`          : desired spatial velocity in `end` link frame
    @param `epsilon`    : minimum manipulability threshold
    @param `max_lambda` : maximum damping coefficient
    @param `start`      : `str` starting link, default to base link 
    @param `end`        : `str` end link, default to end-effector link
    @param `qdlim`      : joint velocity limit vector 1xN array-like
    @param `scale`      : if `True`, scale the joint velocity while keeping its initial direction

    @return joint velocity
    """
    if start is not None and end is None:
        J = robot.jacobe(robot.q, start = start)
    elif start is None and end is not None:
        J = robot.jacobe(robot.q, end = end)
    elif start is not None and end is not None:
        J = robot.jacobe(robot.q, start = start, end = end)
    else:
        J = robot.jacobe(robot.q)

    m = np.sqrt(abs(np.linalg.det(J @ J.T)))

    if m < epsilon: # If manipulability is less than given threshold
        m_lambda = (1 - m/epsilon) * max_lambda
        # print('Damped Least Square Applied!')
    else:
        # print("No DLS!")
        m_lambda = 0
    
    inv_j = np.linalg.pinv(J.T @ J + m_lambda * np.eye(J.shape[1])) @ J.T # DLS Inverse
    qd = inv_j @ v

    if qdlim is not None:
        qd = reduce_qd(qd, qdlim, scale)

    return qd

def distance(q1, q2):
    """
    Euclidean distance between two joint state vector

    @param `q1`, `q2`: input joint states

    @return Euclidean distance
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
    """
    Create a joint space trajectory between two joint states

    @param `q1`, `q2` : initial and goal joint state
    @param `step_size`: Euclidean difference between each step
    @param `traj_type`: solver method, `quintic` or `trapezoidal` 

    @return a list of intermediate states from `q1` to `q2`
    """
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