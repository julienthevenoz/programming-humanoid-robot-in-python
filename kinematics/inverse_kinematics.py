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
        joint_angles_dict = {} #make a dictionnary
        
        
        lambda_ = 0.001

        joint_angles_dict = {key : self.perception.joint[key] 
                        for key in self.chains[effector_name]} 
        all_joint_angles_dict = self.perception.joint
        

        target = np.array([self.from_trans(transform)]).T

        for i in range(3000):
            # self.forward_kinematics(all_joint_angles_dict)
            self.forward_kinematics(joint_angles_dict)


            T = [0] * len(self.chains[effector_name])
            for i, name in enumerate(self.chains[effector_name]):
                T[i] = self.transforms[name]

            Te = np.array([self.from_trans(T[-1])])
            e = target - Te
            T = np.array([self.from_trans(i) for i in T[0:len(self.chains[effector_name])]])
            J = (Te - T).T
            J[-1, :] = 1
            d_theta = lambda_ * np.dot(np.dot(J.T, np.linalg.pinv(np.dot(J, J.T))), e.T)

            for i, name in enumerate(self.chains[effector_name]):
                joint_angles_dict[name] += np.asarray(d_theta.T)[0][i]

            if np.linalg.norm(d_theta) < 1e-4:
                break
        
            #joint_angles = joint_angles_dict.values() #transform the dict back into a list
        return joint_angles_dict
           
        

    def set_transforms(self, effector_name, transform):   
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        #this function gets a transformation matrix which indicates the orientation and position of the effector
        self.keyframes = ([], [], [])  # the result joint angles have to fill in
        angle_list = self.inverse_kinematics(effector_name, transform)
        names = self.chains[effector_name]
        times = []
        for k in range(len(names)):
            times.append([0,3])     #we set the movement of every joint to last 3 seconds 
        #for keys we put the simplest key possible, just an angle and 0 for all handles
        keys=[]
        for i, joint_name in enumerate(names):
            keys.append([[self.perception.joint[joint_name], [3,0,0], [3,0,0]], [angle_list[joint_name], [3,0,0], [3,0,0]]])
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
    
    def error_func(self, joint_angles_values, joint_angles_keys, target, end_effector):
        joint_angles = dict(zip(joint_angles_keys, joint_angles_values)) #forced to do this because of fmin
        self.forward_kinematics[joint_angles]
        total_transfo = self.transforms[end_effector] 
        intermediate = np.matrix(self.get_goal_trans(total_transfo))
        e = target -  intermediate
        norm = np.linalg.norm(e)
        return norm     
    
   
if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.1     #dernière ligne 2ème colonne    Ty ?
    T[-1, 2] = -0.5    #dernière ligne 3ème colonne    Tz ?
    agent.inverse_kinematics('LLeg', T)
    agent.set_transforms('LLeg', T)
    print('here')
    agent.run()
