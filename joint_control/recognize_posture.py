'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
from keyframes import rightBackToStand
from keyframes import leftBackToStand
from keyframes import wipe_forehead
from keyframes import rightBellyToStand
from scipy.interpolate import interp1d
import numpy as np
import pickle




class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = pickle.load(open("./robot_pose.pkl", 'rb'))  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        # Relevant joints and body angles as used during classifier training
        relevant_features = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch',
                             'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch',
                             'AngleX', 'AngleY']

        # Extract and order the relevant joint angles and body angles
        features = [perception.joint[joint] if joint in perception.joint else perception.imu[i - 8]
                    for i, joint in enumerate(relevant_features)]

        # Reshape to a 2D array with one row
        features = np.array(features).reshape(1, -1)

        # Predict the posture
        predicted_label = self.posture_classifier.predict(features)[0]

        # Mapping predicted label to posture name
        posture_names = ['Frog', 'HeadBack', 'Left', 'Knee', 'Crouch', 'Back',
                         'Belly', 'Right', 'Sit', 'Stand', 'StandInit']
        posture = posture_names[predicted_label]
        print(posture)
        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
