'''In this exercise you need to implement the PID controller for joints of robot.

* Task:
    1. complete the control function in PIDController with prediction
    2. adjust PID parameters for NAO in simulation

* Hints:
    1. the motor in simulation can simple modelled by angle(t) = angle(t-1) + speed * dt 
    2. use self.y to buffer model prediction
'''


# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'software_installation'))
#sys.path.append('C:\Users\TheCh\OneDrive\Documents\Cours TUB\SommerSemester2023\Robocup\programming-humanoid-robot-in-python\software_installation')
###########target - sensor + prediction without - prediction with

import numpy as np
from collections import deque
from spark_agent import SparkAgent, JOINT_CMD_NAMES


class PIDController(object):
    '''a discretized PID controller, it controls an array of servos,
       e.g. input is an array and output is also an array
    '''
    def __init__(self, dt, size):
        '''
        @param dt: step time
        @param size: number of control values
        @param delay: delay in number of steps
        '''
        self.dt = dt
        self.u = np.zeros(size)   #array des torques ?
        self.e1 = np.zeros(size)
        self.e2 = np.zeros(size)
        # ADJUST PARAMETERS BELOW
        delay = 0
        self.Kp = 25
        self.Ki = 0.5
        self.Kd = 0.1
        self.y = deque(np.zeros((delay + 1, size)), maxlen=delay + 1) #modifié par moi. deque qui contient (delay + 1) arrays de taille (size)

    def set_delay(self, delay):
        '''
        @param delay: delay in number of steps
        '''
        self.y = deque(self.y, delay + 1)   #reformate y à la taille maxlen = delay+1 (change le nombre d'array dans le deque)

    def control(self, target, sensor):
        '''apply PID control
        @param target: reference values
        @param sensor: current values from sensor
        @return control signal
        '''
        # YOUR CODE HERE
        prediction = sensor + self.u*self.dt  #prediction de l'angle au prochain temps avec l'angle delayed et la vitesse delayed
        self.y.appendleft(prediction) #à gauche du deque on a la prediction pour le temps suivant
        e = target - (sensor + self.y[0] - self.y[-1]) #error = ref - (y_delayed + ỹ_nodelay - ỹ_delayed)
        P = (self.Kp + self.Ki*self.dt + self.Kd/self.dt)*e
        I = -(self.Kp + 2*self.Kd/self.dt)*self.e1
        D = (self.Kd / self.dt) * self.e2
        self.u = self.u + P + I + D
        
        self.e2 = self.e1  #on actualise les erreurs passées
        self.e1 = e
        return self.u #self.u sera la vitesse qu'on va ordonner aux joints


class PIDAgent(SparkAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PIDAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.joint_names = JOINT_CMD_NAMES.keys()
        number_of_joints = len(self.joint_names)
        self.joint_controller = PIDController(dt=0.01, size=number_of_joints)
        self.target_joints = {k: 0 for k in self.joint_names}

    def think(self, perception):
        action = super(PIDAgent, self).think(perception)
        '''calculate control vector (speeds) from
        perception.joint:   current joints' positions (dict: joint_id -> position (current))
        self.target_joints: target positions (dict: joint_id -> position (target)) '''
        joint_angles = np.asarray(
            [perception.joint[joint_id]  for joint_id in JOINT_CMD_NAMES])
        target_angles = np.asarray([self.target_joints.get(joint_id, 
            perception.joint[joint_id]) for joint_id in JOINT_CMD_NAMES])
        u = self.joint_controller.control(target_angles, joint_angles)
        action.speed = dict(zip(JOINT_CMD_NAMES.keys(), u))  # creates a dict: joint_id -> speed (self.u)
        return action


if __name__ == '__main__':
    agent = PIDAgent()
    agent.target_joints['HeadYaw'] = 1.0
    agent.run()
