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

from numpy.matlib import matrix, identity
from numpy import cos, sin 
import numpy as np

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
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],   #LA DOCU DONNE !R!AnkleRoll
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']  #ET !L!AnkleRoll mais c'est surement une faute alors j'ai corrigé
                       }
        
        self.link_translation = {'HeadYaw': [0., 0., 0.1265], 'HeadPitch':	[0., 0., 0.], 
                                 'LShoulderPitch':	[0., 0.098, 0.1],'LShoulderRoll': [0., 0.,	0.], 'LElbowYaw':	[0.105, 0.015, 0.], 'LElbowRoll':[0., 0., 0.], 
                                 'RShoulderPitch':	[0., -0.098, 0.1], 'RShoulderRoll': [0., 0.,	0.], 'RElbowYaw':	[0.105, -0.015, 0.], 'RElbowRoll':[0., 0., 0.], 
                                 'LHipYawPitch': [0., 0.05, -0.085], 'LHipRoll': [0., 0., 0.], 'LHipPitch': [0., 0., 0.], 'LKneePitch': [0., 0., -0.1], 'LAnklePitch': [0., 0., -0.1029], 'LAnkleRoll': [0., 0., 0.],
                                 'RHipYawPitch': [0, -0.05, -0.085], 'RHipRoll': [0., 0., 0.], 'RHipPitch': [0., 0., 0.],'RKneePitch': [0., 0., -0.1], 'RAnklePitch': [0., 0., -0.1029], 'RAnkleRoll': [0., 0., 0.]  
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
        c = cos(joint_angle)
        s = sin(joint_angle)
        if(joint_name.find('Roll') != -1):                #find returns index of searched string or -1 if not found
            R = matrix([[1, 0, 0], [0,c,-s], [0, s, c]])  #if we have roll we have Rx matrix
        if(joint_name.find('Pitch') != -1):    
            R = matrix([[c, 0, s], [0, 1, 0], [-s, 0, c]]) #pitch -> Ry
        if(joint_name.find('Yaw') != -1):
            R = matrix([[c, s, 0], [-s, c, 0], [0, 0, 1]]) #Yaw -> Rz

        T[0:3, 0:3] = R  #insert appropriate rotation matrix in top left of transform matrix


        T[-1, 0:3] = self.link_translation[joint_name]       #put the translation elements in the transfo matrix. Convention : translation goes in bottom row
        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)   
            for joint in chain_joints:
                if(joint in joints):   #we only care about the joints in joints[], not all joints in self.chains
                    angle = joints[joint]
                    Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                    T = T * Tl              #multiplication matricielle T0_N = T0_1 * T1_2 * T1_3 etc
                    self.transforms[joint] = T
                

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
