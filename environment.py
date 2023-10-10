from spatialmath import SE3
import spatialgeometry as geometry
import os
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

        # Create Mesh objects using the obtained file paths
        self.my_table = geometry.Mesh(table_file_path, pose=SE3.Rx(90, 'deg') * SE3(-1.4, -0.36, -0.6) * SE3.Ry(90, 'deg') * SE3(-1.6, 0, 4.0), color=(0.5, 0.27, 0.07, 1.0), scale=(1, 1, 1))
        self.my_cans_1 = geometry.Mesh(cans_file_path, pose=SE3.Rx(90, 'deg') * SE3(-1.4, -0.36, -0.6) * SE3.Ry(90, 'deg') * SE3(-3.0, 0, 2.5), color=(0.5, 0.5, 0.3, 1.0), scale=(1, 1, 1))
        self.my_cans_2 = geometry.Mesh(cans_file_path, pose=SE3.Rx(90, 'deg') * SE3(-1.4, -0.36, -0.6) * SE3.Ry(90, 'deg') * SE3(-3.3, 0, 2.3), color=(0.3, 0.5, 0.5, 1.0), scale=(1, 1, 1))

    def add_to_env(self):
        """Add objects to the specified environment."""
        self._env.add(self.my_table)
        self._env.add(self.my_cans_1)
        self._env.add(self.my_cans_2)