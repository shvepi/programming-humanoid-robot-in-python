'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''

from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np


class InverseKinematicsAgent(ForwardKinematicsAgent):

    def from_trans(self, T):
        # Extract translation components
        x, y, z = T[0, 3], T[1, 3], T[2, 3]

        # Extract rotation matrix
        R = T[:3, :3]

        # Compute Euler angles from rotation matrix
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)

        singular = sy < 1e-6
        if not singular:
            x_angle = np.arctan2(R[2, 1], R[2, 2])
            y_angle = np.arctan2(-R[2, 0], sy)
            z_angle = np.arctan2(R[1, 0], R[0, 0])
        else:
            x_angle = np.arctan2(-R[1, 2], R[1, 1])
            y_angle = np.arctan2(-R[2, 0], sy)
            z_angle = 0

        # Return translation and rotation (in Euler angles)
        return [x, y, z, x_angle, y_angle, z_angle]

    def inverse_kinematics(self, effector_name, transform):
        # Initialize joint angles for the specified chain
        # Define joint limits for all of NAO's joints
        joint_limits = {
            'HeadYaw': [-2.0857, 2.0857], 'HeadPitch': [-0.6720, 0.5149],
            'LShoulderPitch': [-2.0857, 2.0857], 'LShoulderRoll': [-1.3265, 0.3142],
            'LElbowYaw': [-2.0857, 2.0857], 'LElbowRoll': [0.0349, 1.5446],
            'RShoulderPitch': [-2.0857, 2.0857], 'RShoulderRoll': [-1.3265, 0.3142],
            'RElbowYaw': [-2.0857, 2.0857], 'RElbowRoll': [-0.0349, 1.5446],
            'LHipYawPitch': [-1.145303, 0.740810], 'LHipRoll': [-0.379472, 0.790477],
            'LHipPitch': [-1.773912, 0.484090], 'LKneePitch': [-0.092346, 2.112528],
            'LAnklePitch': [-1.189516, 0.922747], 'LAnkleRoll': [-0.397880, 0.769001],
            'RHipYawPitch': [-1.145303, 0.740810], 'RHipRoll': [-0.768992, 0.397880],
            'RHipPitch': [-1.186448, 0.932056], 'RKneePitch': [-0.103083, 2.120198],
            'RAnklePitch': [-1.189516, 0.922747], 'RAnkleRoll': [-0.768992, 0.397935]
        }

        # Initialize joint angles within limits for the specified effector
        joint_angles = np.array([np.random.uniform(joint_limits[joint][0], joint_limits[joint][1])
                                 for joint in self.chains[effector_name]])
        lambda_ = 1
        max_step = 0.1
        # Convert transform to a flat list
        target_transform = self.from_trans(transform)

        # Create a numpy matrix from the flat list
        target = np.matrix(target_transform).T
        max_iterations = 1000  # Set a maximum number of iterations to prevent infinite loops
        error_threshold = 1e-4
        for _ in range(max_iterations):
            # Compute the end-effector transformation matrix
            Ts = [np.identity(len(self.chains[effector_name]))] + [self.transforms[name] for name in
                                                                   self.chains[effector_name]]
            # Compute the end-effector transformation matrix
            end_effector_trans = self.from_trans(Ts[-1])
            Te = np.matrix(end_effector_trans).T
            # Calculate the error between the current end-effector position and the target
            error = target - Te
            # Clip the error to be within the specified maximum step size
            np.clip(error, -max_step, max_step, out=error)
            # Create a matrix of transformation parameters for all joints except the last one
            joint_transforms = [self.from_trans(j) for j in Ts[:-1]]
            T = np.matrix(joint_transforms).T
            # Compute the Jacobian matrix as the difference between end-effector and joint transformations
            J = Te - T
            # Shift the first three rows of the Jacobian to align rotation and translation components
            J[:3, :] = np.roll(J[:3, :], shift=1, axis=0)  # Efficiently shift rows
            # Set the last row of the Jacobian to 1 for homogeneous coordinates
            J[-1, :] = 1
            # Calculate the change in joint angles using the pseudo-inverse of the Jacobian
            d_theta = lambda_ * np.dot(np.linalg.pinv(J), error)
            # Update the joint angles
            joint_angles += np.asarray(d_theta.T)[0]
            # Check for convergence: if the change in joint angles is below a threshold, stop iterating
            if np.linalg.norm(d_theta) < error_threshold:
                break
        # Return the final calculated joint angles
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # Solve the inverse kinematics to obtain the joint angles
        joint_angles = self.inverse_kinematics(effector_name, transform)

        # Update the self.transforms dictionary with the calculated transformation
        joints = self.chains[effector_name]

        names = []
        times = []
        keys = []
        for i, joint in enumerate(joints):
            names.append(joint)
            times.append([5.60000, 8.60000])
            keys.append([[joint_angles[i], [3, -0.33333, 0.00000], [3, 0.33333, 0.00000]],
                         [joint_angles[i], [2, -0.33333, 0.00000], [2, 0.33333, 0.00000]]])
        self.keyframes = (names, times, keys)


if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
