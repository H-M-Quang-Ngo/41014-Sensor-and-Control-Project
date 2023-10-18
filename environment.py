from spatialmath import SE3
import spatialgeometry as geometry
import os
from math import pi
from swift import Swift

class stuff():
    """ All stuff for the environment such as a table, cans, apples """

    def __init__(self, env: Swift):
        self._env = env

        # Get the directory where the current Python script is located
        script_directory = os.path.dirname(os.path.abspath(__file__))
        
        # Define the relative paths to the STL files inside the "models" folder
        cans_file_path = os.path.join(script_directory, "models", "Cans.STL")
        table_file_path = os.path.join(script_directory, "models", "Table_brown.STL")
        tray_file_path = os.path.join(script_directory, "models", "Tray.stl")

        # Create Mesh objects using the obtained file paths
        self.table = geometry.Mesh(table_file_path, pose=SE3.Rx(90, 'deg') * SE3(-1.4, -0.36, -0.6) * SE3.Ry(90, 'deg') * SE3(-1.6, 0, 4.0), color=(0.5, 0.27, 0.07, 1.0), scale=(1, 1, 1))
        self.can_1 = geometry.Mesh(cans_file_path, pose=SE3.Rx(90, 'deg') * SE3(-1.4, -0.36, -0.6) * SE3.Ry(90, 'deg') * SE3(-3.3, 0, 2.3), color=(0.3, 0.5, 0.5, 1.0), scale=(1, 1, 1))
        self.can_2 = geometry.Mesh(cans_file_path, pose=SE3.Rx(90, 'deg') * SE3(-1.4, -0.36, -0.6) * SE3.Ry(90, 'deg') * SE3(-3.0, 0, 2.5), color=(0.5, 0.5, 0.3, 1.0), scale=(1, 1, 1))
        self.tray  = geometry.Mesh(tray_file_path, pose= SE3(2.74,-0.4,0.5) * SE3.Rz(pi/2), color = (0.5, 0.39, 0.43, 1), scale = (0.0015, 0.001, 0.001))
        self.wall  = geometry.Cuboid(scale = [0.3, 5, 2], pose = SE3(4.15, 0, 1), color = (0.5, 0.5, 0.5, 0.2))

        # self.tray.T = self.tray.T @ SE3.Tz(pi/2).A
    def add_to_env(self):
        """Add objects to the specified environment."""
        self._env.add(self.table)
        self._env.add(self.can_1)
        self._env.add(self.can_2)
        self._env.add(self.wall)
        self._env.add(self.tray)