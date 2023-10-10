## @file
#  @brief Gripper for FETCH Robot defined by standard DH parameters with 3D model
#  @author Long Thinh Le
#  @date Oct 1, 2023

from swift import Swift
import roboticstoolbox as rtb
import spatialmath.base as smb
from ir_support.robots.DHRobot3D import DHRobot3D
import os

from math import pi

class Gripper_finger_Fetch(DHRobot3D):
    """ GRIPPER RIGHT FOR UR3 ROBOT ON LINEAR RAILS
    """
    def __init__(self, env: Swift):
        # DH links
        links = self._create_DH()
        
        self._env= env
            
        # Names of the robot link files in the directory
        link3D_names = dict(
            link0 = 'blank_model',
            link1 = 'gripper_finger'
        )
        
        qtest = [0]
        
        qtest_transforms = [
            smb.transl(0,0,0),
            smb.transl(0,0,0)
        ]    
        
        current_path = os.path.abspath(os.path.dirname(__file__))
        
        current_path = os.path.join(current_path, "models")
        super().__init__( # calling the constructor of a parent class to initialize an object of the class.
            links,
            link3D_names,
            name="Gripper_Fetch",
            link3d_dir = current_path,
            qtest = qtest,
            qtest_transforms= qtest_transforms,
        )
        
        self.Gripper_base_origin = self.base

    def _create_DH(self):
        """ 
        Create Gripper RIGHT Robot's standard DH model
        """
        links = [] # BASE
        link = rtb.PrismaticDH(a = 0, alpha = 0, offset = 0, qlim = [-0.3, 0.3])
        links.append(link)

        return links

    
    def test(self):
        """
        Test the class by adding the 3D objects into a new Swift window and do a simple movement
        """
        self.q = self._qtest
        self.add_to_env(self._env)
                

        flag = True
        while True:
            if flag:
                q_goal = [0.1]
                flag = False
            else:
                # q_goal = [0,0,-pi/4,-pi/8]
                flag = True
   
            qtraj = rtb.jtraj(self.q, q_goal, 50).q
            
            for q in qtraj:
                self.q = q
                self._env.step(0.02)
            
            self._env.hold()
                



if __name__ == "__main__":
    env = Swift()
    env.launch(realtime= True)
    r = Gripper_finger_Fetch(env= env)
    r.test()





