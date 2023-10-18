## @file
#  @brief Gripper for FETCH Robot defined by standard DH parameters with 3D model
#  @author Long Thinh Le & Ho Minh Quang Ngo
#  @date Oct 1, 2023

from swift import Swift
import roboticstoolbox as rtb
import spatialmath.base as smb
from ir_support.robots.DHRobot3D import DHRobot3D
from spatialmath import SE3
import os
import time

from math import pi

class Gripper_finger_Fetch(DHRobot3D):
    """ 
    GRIPPER FINGER FOR FETCH ROBOT
    """
    def __init__(self, env: Swift):
        # DH links
        links = self._create_DH()
        
        self._env= env
            
        # Names of the robot link files in the directory
        link3D_names = dict(
            link0 = 'blank_model',
            link1 = 'gripper_finger',
            link2 = 'gripper_finger'
        )
        
        qtest = [0,0]
        
        qtest_transforms = [
            smb.transl(0,0,0),
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

        self._T_ee_gripper = SE3(0,0.0073,0) * SE3.RPY(pi/2, 0, pi)

    def _create_DH(self):
        """ 
        Create Gripper RIGHT Robot's standard DH model
        """
        links = [] # BASE
        
        link_right = rtb.PrismaticDH(a = 0, alpha = 0, offset = 0, qlim = [-0.3, 0.3])
        link_left = rtb.PrismaticDH(a=0 , alpha = 0, offset = 0, qlim = [-0.3, 0.3])
        links.append(link_right)
        links.append(link_left)

        return links
    
    def gripper_attach(self, fetch_ee:SE3):
        """
        Attach the grippers onto the Fetch end-effector.
        
        @param `fetch_ee`: end-effector pose of the Fetch arm
        """
        self.base = self.Gripper_base_origin * fetch_ee * self._T_ee_gripper
        self._update_3dmodel()
        self._env.step(0.001)
    
    def manipulate_gripper(self, open: bool|str = True):        
        """
        Open or close the gripper

        @param `open`: open the gripper if `True` and vice versa
        """
        if open: 
            gripper_range = 0.03
            # print("Gripper open!")
        elif open == 'half': 
            gripper_range = 0.01
            # print("Gripper close half!")
        else:
            gripper_range = 0
        
        q_goal = [gripper_range, -2*gripper_range - 0.015]
        qtraj = rtb.jtraj(self.q, q_goal, 50).q
        for q in qtraj:
            self.q = q
            self._env.step(0)
            time.sleep(0.02)
 
    def test(self):
        """
        Test the class by adding the 3D objects into a new Swift window and do a simple movement
        """
        self.q = self._qtest
        self.add_to_env(self._env)
                
        flag = True
        time.sleep(1)
        while True:
            if flag:
                q_goal = [0.1, -0.2]
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





