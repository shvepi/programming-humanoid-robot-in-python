'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''

from pid import PIDAgent
from keyframes import hello
from keyframes import rightBackToStand
from keyframes import wipe_forehead
from keyframes import rightBellyToStand
from scipy.interpolate import interp1d
import time





class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        if 'LHipYawPitch' in target_joints:
            target_joints['RHipYawPitch'] = target_joints['LHipYawPitch']
        else:
            print("'LHipYawPitch' not found in target_joints; assigning a default value or skipping.")
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        names, times, keys = keyframes
        # Initialize target_joints with all joint names from perception.joint.keys()
        target_joints = {joint_name: 0.0 for joint_name in perception.joint.keys()}
        # Check for empty times list
        if not times or not any(times):
            return {}

        # Assuming total_duration is the total time span of your animation
        total_duration = max(max(times))
        # Get the current time from perception
        current_time = perception.time

        # Calculate the current relative time (normalized between 0 and 1)
        current_relative_time = (current_time % total_duration) / total_duration

        # Convert current_relative_time to the Bezier curve parameter 'i'
        i = current_relative_time

        for joint_index, joint_name in enumerate(names):
            joint_times = times[joint_index]
            joint_keys = keys[joint_index]

            # Find the segment of the current time
            for key_index in range(len(joint_times) - 1):
                # Extract control points for the current segment
                P0 = joint_keys[key_index][0]
                P3 = joint_keys[key_index + 1][0]
                # Handle control points for tangents
                handle1_duration = joint_times[key_index + 1] - joint_times[key_index]
                handle2_duration = handle1_duration
                P1 = P0 + joint_keys[key_index][2][1] * handle1_duration / 3
                P2 = P3 + joint_keys[key_index + 1][1][1] * handle2_duration / 3
                # Cubic Bezier interpolation using 'i'
                bezier_value = (
                        ((1 - i) ** 3) * P0 +
                        (3 * (1 - i) ** 2 * i) * P1 +
                        (3 * (1 - i) * (i ** 2)) * P2 +
                        (i ** 3) * P3
                )
                target_joints[joint_name] = bezier_value
                break
        return target_joints


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
