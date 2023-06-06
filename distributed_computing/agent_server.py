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
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent


from xmlrpc.server import SimpleXMLRPCServer,SimpleXMLRPCRequestHandler
import threading

# restreindre à un certain path. jsp trop ce que je fais là en fait
# class RequestHandler(SimpleXMLRPCRequestHandler):
#     rpc_paths = ('/RPC2',)


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self):
        super(ServerAgent, self).__init__()   #on doit d'abord lancer le truc qui gère la co avec simspark
        server = SimpleXMLRPCServer(('localhost', 9999), logRequests=True, allow_none=True) #on créé le serveur sur mon ordi au port 9999
                                                                           #logrequests print les tentaives de connection dans le terminal
        server.register_instance(self)

        # server.register_introspection_functions()
        # server.register_multicall_functions()
        #server.serve_forever()
        arg_list = []  #pour pouvoir passer des listes au thread
        self.thread_forever = threading.Thread(target = server.serve_forever, args = arg_list)  #lance le server en tant que thread
        self.thread_forever.start()  #useful ?
        print('server is active')


    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        if joint_name in self.perception.joint:
            return str(self.perception.joint.get(joint_name)) #NB we return a string
        else:
            print('invalid joint name')
            return 'invalid joint name'
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name] = angle


    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.recognize_posture(self.perception)  #returns a string with the posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes = keyframes 

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.transforms[name]

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.set_transforms(effector_name, transform)

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

