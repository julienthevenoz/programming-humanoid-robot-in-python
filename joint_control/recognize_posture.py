'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
import pickle 
import os
import numpy as np #nécessaire ?




class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        # LOAD YOUR CLASSIFIER
        ROBOT_POSE_CLF = 'robot_pose.pkl'
        self.dir = os.path.dirname(__file__) #ça donne le path qui amène à ce fichier
        robot_pose_pkl_path = os.path.join(self.dir, ROBOT_POSE_CLF)  #ajoute le path du fichier au .pkl
        self.posture_classifier = pickle.load(open(robot_pose_pkl_path, 'rb'))
        # with open(ROBOT_POSE_CLF, 'rb') as nom_fichier:
        #     self.posture_classifier = pickle.load(nom_fichier)

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        
        joint_list = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch']
        #angles_data = np.array(len(joint_list) + 2)  #créé un array assez grand pour stocker tous les joints + AngleX & AngleY
        
        angles_data = []
        for joint, jointname in enumerate(joint_list):
          #print(perception.joint[jointname])
          angles_data.append(perception.joint[jointname])

        #ajoute les valeurs AngleX & AngleY manuellement à al fin de l'array
        angles_data.append(perception.imu[0])
        angles_data.append(perception.imu[1])
        #changing the shape of array angles data from 1d to 2d so it can be used with predict
        #angles_data = angles_data.reshape((-1,1))
        #prediction of the pose
        pose_predicted = self.posture_classifier.predict([angles_data])
        #copied from learn_posture.ipynb
        # ROBOT_POSE_DATA_DIR = 'robot_pose_data' 
        # classes = listdir(ROBOT_POSE_DATA_DIR)
        # posture = classes[pose_predicted]
        

        robot_pose_data_path = os.path.join(self.dir, 'robot_pose_data')
        posture = os.listdir(robot_pose_data_path)[pose_predicted[0]]

        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
