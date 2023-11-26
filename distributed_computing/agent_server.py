'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
import logging
import time
import threading
import pickle
from xmlrpc.server import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self):
        super(ServerAgent, self).__init__()
        # create server
        try:
            self.server = SimpleXMLRPCServer(('localhost', 7758), requestHandler=SimpleXMLRPCRequestHandler,
                                             allow_none=True)
            self.server.register_introspection_functions()
            self.server.register_multicall_functions()
            self.server.register_instance(self)
            self.thread = threading.Thread(target=self.server.serve_forever)
            self.thread.start()
            logger.info('Starting server...')
        except Exception as e:
            logger.error(f'Failed to start server: {e}')

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        try:
            return self.perception.joint.get(joint_name, None)
        except Exception as e:
            logger.error(f"Error in get_angle for joint {joint_name}: {e}")
            return None
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        try:
            self.target_joints[joint_name] = angle
            return True
        except Exception as e:
            logger.error(f"Error in set_angle for joint {joint_name}: {e}")
            return False

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        try:
            return self.posture
        except Exception as e:
            logger.error(f"Error in get_posture: {e}")
            return 'unknown'

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes = keyframes


        # Calculate the total duration of the keyframes
        total_duration = max([max(times) for times in keyframes[1]])

        # Start time of keyframe execution
        start_time = time.time()

        # Wait until the total duration has passed
        while (time.time() - start_time) < total_duration:
            time.sleep(0.1)  # Sleep to prevent busy waiting

        return True
    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.transforms[name]
    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.set_transforms( effector_name, transform)
if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

