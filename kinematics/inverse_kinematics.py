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
from scipy.optimize import fmin
from math import atan2


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):  #we are not given coordinates and angles but a transform matrix from which we extract them
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        #YOUR CODE HERE 
         
        
        joint_names = self.chains[effector_name]
        longueur= len(self.chains[effector_name])
        
        end_effector_name = self.chains[effector_name][longueur-1]
        joint_angles = {key : self.perception.joint[key] 
                        for key in self.chains[effector_name]} 
        

        goal = np.matrix(self.from_trans(transform))
        
       
        func = lambda t: self.error_func(t, goal, end_effector_name)

        optimized = fmin(func, joint_angles.values)
        joint_angles_dict = dict(zip(joint_names, optimized))


        return joint_angles_dict
           
        

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.keyframes = ([], [], [])  # the result joint angles have to fill in
        angle_list = self.inverse_kinematics(effector_name, transform)
        names = self.chains(effector_name)
        times = [0, 3] * len(self.chains[effector_name])
        #for keys we put the simplest key possible, just an angle and 0 for all handles
        for i, joint_name in enumerate(names):
            keys = [[self.perception[joint_name], [3,0,0], [3,0,0]], [angle_list[i], [3,0,0], [3,0,0]]]
        self.keyframes = (names, times, keys)

    
    def from_trans(self, T):         #find x,y,z,x_angle,y_angle,z_angle from the transfo matrix
        x, y, z = T[3,0], T[3,1], T[3,2]
        x_angle, y_angle, z_angle = 0, 0, 0

        if(T[0,0] == 1):        #if we have a 1 in top right corner we have a Rx matrix inside of T
            x_angle = atan2(T[2,1], T[1,1])  #so x_angle = arctan(tan(theta))=arctan(sintheta/costheta) and all others are 0
        elif(T[1,1] == 1):      
            y_angle = atan2(T[0,2], T[0,0]) #for a Ry matrix
        elif(T[2,2] == 1):
            z_angle = atan2(T[0,1], T[0,0])  #for a Rz matrix

        return np.asarray([x,y,z,x_angle,y_angle,z_angle])
    
    def error_func(self, joint_angles, target, end_effector):
        self.forward_kinematics[joint_angles]
        total_transfo = self.transforms[end_effector] 
        intermediate = np.matrix(self.get_goal_trans(total_transfo))
        e = target -  intermediate
        return np.linalg.norm(e)      
    
   
if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05     #dernière ligne 2ème colonne    Ty ?
    T[-1, 2] = -0.26    #dernière ligne 3ème colonne    Tz ?
    agent.set_transforms('LLeg', T)
    agent.run()
