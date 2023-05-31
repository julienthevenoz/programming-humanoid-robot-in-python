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


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):  #we are not given coordinates and angles but a transform matrix from which we extract them
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        joint_angles =  [0] * len(self.chains[effector_name]) #on prépare la liste à return avec le bon nombre d'angles
        new_angles = [0] * len(self.chains[effector_name])  #variable intermédiaire qui fait la même taille

        #on remplit le dict angles_chain avec le nom et l'angle actuel de chaque joint dans la chain de l'effector concerné
        angles_chain = {key : self.perception.joint[key] for key in self.chains[effector_name]}  
        last_joint = self.chains[effector_name][-1] # name of the last joint in the effector's chain

        target = self.from_trans(transform).T #target is the array (transposed) of x,y,z,x-,y- and z-angle given through the 'transform' matrix
        

        for i in range(1000):
            self.forward_kinematics(angles_chain)       #execute forward kin for the chain
            forward_kin = self.transforms               #save the transforms calculated by the forward kin
            T_iteration = self.from_trans(forward_kin(last_joint))  #we get the coordinates & angles of the last joint
            e= target - T_iteration #error is the distance between where last joint should be and where it is
            err = np.linalg.norm(e)
            d = grad(err)

            new_angles = d * 
            
            joint_angles += new_angles

            if(d < 10^(-4)):
                break

        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.keyframes = ([], [], [])  # the result joint angles have to fill in


    
    def from_trans(self, T):         #find x,y,z,x_angle,y_angle,z_angle from the transfo matrix
        x, y, z = T[3,0], T[3,1], T[3,2]
        x_angle, y_angle, z_angle = 0, 0, 0

        if(T[0,0] == 1):        #if we have a 1 in top right corner we have a Rx matrix inside of T
            x_angle = np.atan2(T[2,1], T[1,1])  #so x_angle = arctan(tan(theta))=arctan(sintheta/costheta) and all others are 0
        elif(T[1,1] == 1):      
            y_angle = np.atan2(T[0,2], T[0,0]) #for a Ry matrix
        elif(T[2,2] == 1):
            z_angle = np.atan2(T[0,1], T[0,0])  #for a Rz matrix

        return np.asarray([x,y,z,x_angle,y_angle,z_angle])
    
        

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05     #dernière ligne 2ème colonne    Ty ?
    T[-1, 2] = -0.26    #dernière ligne 3ème colonne    Tz ?
    agent.set_transforms('LLeg', T)
    agent.run()
