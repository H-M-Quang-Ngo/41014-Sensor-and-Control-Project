"""
@author Ho Minh Quang Ngo
"""

# Require libraries
import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
from spatialmath import SE3
from spatialmath.base import *
from machinevisiontoolbox import CentralCamera, mkgrid
from matplotlib.widgets import Slider, Button
import keyboard

# Useful variables
from math import pi

# -----------------------------------------------------------------------------------#
class VisionDemo:
    def __init__(self):
        plt.close('all')

        ## Define camera
        self.camera = CentralCamera(f= 0.015, rho = 10e-6, 
                                      imagesize = [640, 480], pp = [320,240], name = 'mycamera')
        ## Create 3D points 
        self.grid_of_spheres = mkgrid(2, 0.5, pose= SE3.Trans(0,0,5))

        ## Error threshold (in pixels) for tracking (min and max)
        self.error_threshold = [10, 500]

        ## Max number of times to try to track an object
        self.max_attempts = 50

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.camview_plot()

        # Set the position of the camera
        self.update_camera_and_view(transl(0,0,1) @ troty(0.1))        
        self.add_camera_controls()
    
    # -----------------------------------------------------------------------------------#
    def camview_plot(self):
        uv = self.camera.plot_point(self.grid_of_spheres, pose=self.camera.pose)
        self.ax_cam = plt.gca() # Handle for the camera view
        for i in range(uv.shape[1]):
            self.ax_cam.text(uv[0, i], uv[1, i], str(i+1), color='red', fontsize=12)
            
    # -----------------------------------------------------------------------------------#
    def redraw_ax(self):
        self.ax.cla()
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_zlim(0, 6)
        self.ax.set_box_aspect([1,1,3])
        for i in range(self.grid_of_spheres.shape[1]):
            plot_sphere(0.05, centre= self.grid_of_spheres[:,i] , ax = self.ax, color = 'b')

    # -----------------------------------------------------------------------------------#
    def update_camera_and_view(self,tr = None):
        # Update where the camera is plotted
        if tr is not None:
            self.camera.pose = SE3(tr)
        self.redraw_ax()
        self.camera.plot(scale= 0.25, solid= True, ax = self.ax)

        # Update what the camera sees
        self.camera.clf()
        uv = self.camera.plot_point(self.grid_of_spheres, pose=self.camera.pose, ax = self.ax_cam)
        for i in range(uv.shape[1]):
            self.ax_cam.text(uv[0, i], uv[1, i], str(i+1), color='red', fontsize=12)
        plt.pause(0.01)
        return uv

    # -----------------------------------------------------------------------------------#
    def add_camera_controls(self):
        """
        add_camera_controls
        """
        slider_props = dict(valstep=0.01, valfmt='%1.1f')
        self.slider_x = Slider(self.fig.add_axes([0.2, 0.02, 0.65, 0.03]), 'sliderX', -1, 3, valinit=0, **slider_props)
        self.slider_y = Slider(self.fig.add_axes([0.2, 0.07, 0.65, 0.03]), 'sliderY', -1, 3, valinit=0, **slider_props)
        self.slider_z = Slider(self.fig.add_axes([0.2, 0.12, 0.65, 0.03]), 'sliderZ', -1, 3, valinit=1, **slider_props)
        self.slider_rot_y = Slider(self.fig.add_axes([0.2, 0.17, 0.65, 0.03]), 'sliderRotY', -pi/4, pi/4, valinit=0.1, **slider_props)
        
        self.slider_x.on_changed(self._slider_callback)
        self.slider_y.on_changed(self._slider_callback)
        self.slider_z.on_changed(self._slider_callback)
        self.slider_rot_y.on_changed(self._slider_callback)

        self.button = Button(self.fig.add_axes([0.05, 0.3, 0.15, 0.07]), 'Set Target', hovercolor= 'r')
        self.button.on_clicked(self._target_user_selected_points_with_image_jacobian)
        plt.pause(0.001)
    
    # -----------------------------------------------------------------------------------#
    def _slider_callback(self, _):
        """
        _slider_callback
        """
        tr = np.eye(4)
        tr[:3, :3] = roty(self.slider_rot_y.val)
        tr[0, 3] = self.slider_x.val
        tr[1, 3] = self.slider_y.val
        tr[2, 3] = self.slider_z.val
        
        # Call the function to update camera and view with the new transformation
        self.update_camera_and_view(tr)
        plt.pause(0.001)
    
    # -----------------------------------------------------------------------------------#
    def _target_user_selected_points_with_image_jacobian(self, _):
        """
        _target_user_selected_points_with_image_jacobian
        """
        
        print('Click on the figure where you want the updated point(s) to be in order, then press Enter')
        target_uv= np.zeros([2,self.grid_of_spheres.shape[1]])
        click_count = 0
        while True:
            try:
                click_data = self.camera._fig.ginput()[0]
                target_uv[:,click_count] = np.array(click_data)
                print(f'Point {click_count+1}:', target_uv[:,click_count])
                self.ax_cam.plot(click_data[0], click_data[1], '*g')
                click_count += 1
            except:
                pass
            if click_count == self.grid_of_spheres.shape[1] or keyboard.is_pressed('enter'):
                break
        
        target_uv = target_uv[:,:click_count]
        uv = self.update_camera_and_view()
        error_features = target_uv - uv[:,:click_count]
        error_features = error_features.reshape(-1, order = 'F')
        previous_error_features = error_features.copy()
        attempts = 0
        
        # Keep going until the error is too small or large
        while self.error_threshold[0] < norm(error_features) and norm(error_features) < self.error_threshold[1]:
            tr = self.camera.pose.A
            camera_to_center_distance = norm(tr[:3,3] - np.mean(self.grid_of_spheres, 1))
            J = self.camera.visjac_p(uv[:,:click_count], camera_to_center_distance)
            
            ## Gain of the controller (should be a class property, but put here to explain)
            lambda_gain = 0.1 # If you make it too high (1 or above) it may get there in 1 step or overshoot too much and loose track
            
            delta_x = lambda_gain * linalg.pinv(J) @ error_features

            ## Play here with only allowing certain movements
            tr[:3,3] = tr[:3,3] + delta_x[:3]
            tr[:3,:3] = tr[:3,:3] @ rotx(delta_x[3]) @ roty(delta_x[4]) @ rotz(delta_x[5])
            # tr[:3,:3] = tr[:3,:3] @ roty(delta_x[4])

            uv = self.update_camera_and_view(tr)
            error_features = target_uv - uv[:,:click_count]
            error_features = error_features.reshape(-1, order = 'F')

            # Is it both less than max attempts and reducing the error
            if (attempts < self.max_attempts):
                attempts += 1
                if norm(previous_error_features) < norm(error_features):
                    print('Error is increasing!')
            else:
                print('Cannot get closer: breaking out to prevent infinitely tracking an impossible target')
                break
            previous_error_features = error_features.copy()

        if not norm(error_features) < self.error_threshold[1]:
            print(f'Current error {norm(error_features)} is larger than threshold {self.error_threshold[1]}!')
        
        if not self.error_threshold[0] < norm(error_features):
            print(f'Current error {norm(error_features)} is smaller than threshold {self.error_threshold[0]}!')

        if np.isnan(error_features).any():
            raise Exception('An impossible situation has occurred. Please reinstantiate the class and more carefully choose the targets')                      

# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    lab8_starter = VisionDemo()
    plt.show()