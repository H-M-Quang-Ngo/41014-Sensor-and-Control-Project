import roboticstoolbox as rtb
import numpy as np
import time
from spatialmath import SE3
import spatialgeometry as geometry
from math import pi
from machinevisiontoolbox import CentralCamera

def update_camera_pose(cam: CentralCamera, 
                       fetch_cam: rtb.models.FetchCamera, 
                       cam_object: geometry.CollisionShape = None,
                       adj_mat: SE3 = SE3.Ry(pi/2) * SE3.Rz(-pi/2)):
    """
    Update the camera pose base on the Fetch Camera
    """
    cam.pose = SE3(fetch_cam.fkine(fetch_cam.q).A @ adj_mat.A)
    if cam_object is not None:
        cam_object.T = cam.pose

def update_camera_view(camera: CentralCamera, points: np.ndarray, image_plane, trace = False):
    """
    Update the image plane of the camera
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