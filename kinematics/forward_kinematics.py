'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))
from autograd import jacobian
from numpy.matlib import matrix,  cos, sin, identity
import autograd.numpy as np


from recognize_posture import PostureRecognitionAgent


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       # YOUR CODE HERE
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch','RAnkleRoll'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll']
                       }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE

        # Define rotation matrices for each principal axis (X, Y, Z)
        Rx = np.array([
            [1, 0, 0, 0],
            [0, cos(joint_angle), -sin(joint_angle), 0],
            [0, sin(joint_angle), cos(joint_angle), 0],
            [0, 0, 0, 1]
        ])

        Ry = np.array([
            [cos(joint_angle), 0, sin(joint_angle), 0],
            [0, 1, 0, 0],
            [-sin(joint_angle), 0, cos(joint_angle), 0],
            [0, 0, 0, 1]
        ])

        Rz = np.array([
            [cos(joint_angle), -sin(joint_angle), 0, 0],
            [sin(joint_angle), cos(joint_angle), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Define the translation based on the joint name
        translations = {
            'HeadYaw': np.array([0.0, 0.0, 126.50]) / 1000,  # Convert from mm to meters
            'HeadPitch': np.array([0.0, 0.0, 0.00]) / 1000,
            'LShoulderPitch': np.array([0.0, 98.00, 100.00]) / 1000,
            'LShoulderRoll': np.array([0.00, 0.00, 0.00]) / 1000,
            'LElbowYaw': np.array([105.00, 15.00, 0.00]) / 1000,  # No translation as it's a rotation in place
            'LElbowRoll': np.array([0.0, 0.0, 0.0]) / 1000,
            'RShoulderPitch': np.array([0.0, 98.00, 100.00]) / 1000,
            'RShoulderRoll': np.array([0.00, 0.00, 0.00]) / 1000,
            'RElbowYaw': np.array([105.00, 15.00, 0.00]) / 1000,  # No translation as it's a rotation in place
            'RElbowRoll': np.array([0.0, 0.0, 0.0]) / 1000,
            'LHipYawPitch': np.array([0.0, 50.00, -85.00]) / 1000,
            'LHipRoll': np.array([0.0, 0.00, 0.00]) / 1000,
            'LHipPitch': np.array([0.0, 0.00, 0.00]) / 1000,
            'LKneePitch': np.array([0.0, 0.00, -100.00]) / 1000,
            'LAnklePitch': np.array([0.0, 0.00, -102.90]) / 1000,
            'LAnkleRoll': np.array([0.0, 0.00, 0.00]) / 1000,
            'RHipYawPitch': np.array([0.0, 50.00, -85.00]) / 1000,
            'RHipRoll': np.array([0.0, 0.00, 0.00]) / 1000,
            'RHipPitch': np.array([0.0, 0.00, 0.00]) / 1000,
            'RKneePitch': np.array([0.0, 0.00, -100.00]) / 1000,
            'RAnklePitch': np.array([0.0, 0.00, -102.90]) / 1000,
            'RAnkleRoll': np.array([0.0, 0.00, 0.00]) / 1000,
        }




        # Apply the appropriate rotation matrix based on the joint axis
        if 'Yaw' in joint_name:
            T[:3, :3] = Rz[:3, :3]
        elif 'Pitch' in joint_name:
            T[:3, :3] = Ry[:3, :3]
        elif 'Roll' in joint_name:
            T[:3, :3] = Rx[:3, :3]
        else:
            raise ValueError("Unknown joint axis for joint: {}".format(joint_name))

        translation = np.array([0.0, 0.00, 0.00])
        # Update the transformation matrix with the translation for the joint
        if joint_name in translations:
            # Convert the translation vector to a column vector and assign it to T
            translation_vector = translations[joint_name]
            T[:3, 3] = translation_vector.reshape(3, 1)

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics
        :param joints: {joint_name: joint_angle}
        '''
        print(joints)
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = T @ Tl  # Compound the transformation with the previous transformations
                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
