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
from autograd import grad
import autograd.numpy as anp

class InverseKinematicsAgent(ForwardKinematicsAgent):

    def from_trans(self, T):
        # Extract translation components
        x, y, z = T[0, 3], T[1, 3], T[2, 3]

        # Extract the rotation angle
        if T[0, 0] != 1:  # Rotation is not around the X-axis
            angle = np.arctan2(T[2, 1], T[2, 2])
        elif T[1, 1] != 1:  # Rotation is not around the Y-axis
            angle = np.arctan2(T[0, 2], T[0, 0])
        elif T[2, 2] != 1:  # Rotation is not around the Z-axis
            angle = np.arctan2(T[1, 0], T[1, 1])
        else:
            angle = 0  # No rotation

        return x, y, z, angle

    def inverse_kinematics(self, effector_name, transform):
        # Initialize joint angles for the specified chain
        joint_limits = [-np.pi / 2, np.pi / 2]
        joint_angles = np.random.uniform(joint_limits[0], joint_limits[1], len(self.chains[effector_name]))
        lambda_ = 1
        max_step = 0.1
        target = np.matrix([self.from_trans(transform)]).T
        max_iterations = 1000  # Set a maximum number of iterations to prevent infinite loops
        error_threshold = 1e-4
        for _ in range(max_iterations):
            # Compute the end-effector transformation matrix
            Ts = [np.identity(len(self.chains[effector_name]))] + [self.transforms[name] for name in
                                                                   self.chains[effector_name]]
            # Compute the end-effector transformation matrix
            Te = np.matrix([self.from_trans(Ts[-1])]).T
            # Calculate the error between the current end-effector position and the target
            error = target - Te
            # Clip the error to be within the specified maximum step size
            np.clip(error, -max_step, max_step, out=error)
            # Create a matrix of transformation parameters for all joints except the last one
            T = np.matrix([self.from_trans(j) for j in Ts[:-1]]).T
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
