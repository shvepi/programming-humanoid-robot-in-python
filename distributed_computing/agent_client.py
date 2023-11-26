'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
import threading
import xmlrpc.client
import logging
from joint_control.keyframes import hello, leftBackToStand, rightBellyToStand, rightBackToStand, wipe_forehead
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.client = obj

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        thread = threading.Thread(target=self.client.execute_keyframes, args=(keyframes,))
        thread.start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        thread = threading.Thread(target=self.client.set_transform, args=(effector_name, transform))
        thread.start()

class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        try:
            self.client = xmlrpc.client.ServerProxy("http://localhost:7758", allow_none=True)
            self.post = PostHandler(self.client)
            logger.info('Client started')
        except Exception as e:
            logger.error(f'Failed to connect to server: {e}')
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        try:
            return self.client.get_angle(joint_name)
        except Exception as e:
            logger.error(f"Error in get_angle for joint {joint_name}: {e}")
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        try:
            return self.client.set_angle(joint_name, angle)
        except Exception as e:
            logger.error(f"Error in set_angle for joint {joint_name}: {e}")

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        try:
            return self.client.get_posture()
        except Exception as e:
            logger.error(f"Error in get_posture: {e}")

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        try:
            return self.client.execute_keyframes(keyframes)
        except Exception as e:
            logger.error(f"Error in execute_keyframes: {e}")

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        try:
            return self.client.get_transform(name)
        except Exception as e:
            logger.error(f"Error in get_transform for {name}: {e}")

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        try:
            return self.client.set_transform(effector_name, transform)
        except Exception as e:
            logger.error(f"Error in set_transform for effector {effector_name}: {e}")


if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE
    agent.execute_keyframes(hello())

