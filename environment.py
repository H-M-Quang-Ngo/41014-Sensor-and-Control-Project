from spatialmath import SE3
import spatialgeometry as geometry

class stuff():
    """ All stuff for environment such as table, cans, apples """
    
    def __init__(self):
        table_file_path = 'D:/University/UTS/2023/A_Spring 2023/41014 Sensors and Control/Project/41014-Sensor-and-Control-Project/Table_brown.stl'
        self.my_table = geometry.Mesh(table_file_path, pose = SE3(0,0,0) * SE3.Rx(90,'deg') * SE3(-1.4,-0.36,-0.6) * SE3.Ry(90,'deg') * SE3(-1.6,0,4.0) , color=(0.54, 0.27, 0.07, 1.0), scale= [1,1,1])
        cans_file_path = 'D:/University/UTS/2023/A_Spring 2023/41014 Sensors and Control/Project/41014-Sensor-and-Control-Project/Cans.stl'
        self.my_cans_1 = geometry.Mesh(cans_file_path, pose = SE3(0,0,0) * SE3.Rx(90,'deg') * SE3(-1.4,-0.36,-0.6) * SE3.Ry(90,'deg') * SE3(-3.0,0,2.5) , color=(1.0, 0, 0, 1.0), scale= [1,1,1])
        self.my_cans_2 = geometry.Mesh(cans_file_path, pose = SE3(0,0,0) * SE3.Rx(90,'deg') * SE3(-1.4,-0.36,-0.6) * SE3.Ry(90,'deg') * SE3(-3.3,0,2.3) , color=(1.0, 0, 0, 1.0), scale= [1,1,1])
        

    def add_to_env(self,env):
        """Add objects to the specified environment.

        This function adds various objects such as a table, table logo, tiles, fence, and a human model
        to the provided environment.

        Args:
            environment (object): The environment to which the objects will be added.
        """
        env.add(self.my_table)
        env.add(self.my_cans_1)
        env.add(self.my_cans_2)

        
        
    