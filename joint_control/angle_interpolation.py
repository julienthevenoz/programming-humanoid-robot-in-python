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
#Lhand, LWristYaw, RHand et RWristYaw sont PAS dans les joint names de spark agent ?

from pid import PIDAgent
from keyframes import hello, wipe_forehead,leftBackToStand, leftBellyToStand, rightBackToStand, rightBellyToStand


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start_time = self.perception.time #attribute telling when the interpolation started

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        if('LHipYawPitch' in target_joints.keys()):
            target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        #on peut utiliser perception.time et un bougre a rajouté un attribut start_time dans la classe angleinterpolation
        names, times, keys = keyframes  #on chope les trois listes
        t_inter = perception.time - self.start_time #current time for the interpolation 
        
        for joint, jointname in enumerate(names):                      #itère les colonnes (c'est l'index des joints)
            for kf in range(len(times[joint])-1):                  #kf est l'index qui indique le keyframe de départ de la curve actuelle. On met -1 pour pas dépasser la longueur avec le kf+1 l.55
                # if(t_inter < times[joint][kf]):
                #     target_joints[jointname] = self.calculate_bezier_to_first_keyframe(perception, keyframes, t_inter, joint, kf)
                # #    print("FIRST BEZIER CURVE           ")

                if(times[joint][kf]< t_inter < times[joint][kf+1]):  #actuellement pour le joint 0 #ATT : pas de point de départ si tcurr < premier time
                    target_joints[jointname] = self.calculate_bezier_between_keyframes(keyframes, t_inter, joint, kf)
                   
        return target_joints
    
    def calculate_bezier_to_first_keyframe(self, perception, keyframes, t_inter, joint, kf):
        names, times, keys = keyframes  
        t = (t_inter - self.start_time)/(times[joint][0] - self.start_time) #t entre 0 et 1 de la 1st bezier curve
        try:
            P0 = perception.joint[names[joint]]
        except KeyError:
            P0 = 0     #angle actuel #hmm problème ?
        P1 = P0
        P3 = keys[joint][0][0]                  #angle de la première keyframe
        P2 = P3
        return P0*(1-t)**3 + P1*t*3*(1-t)**2 + P2*3*(1-t)*t**2 + P3*t**3 
    
    def calculate_bezier_between_keyframes(self, keyframes, t_inter, joint, kf):
        names, times, keys = keyframes  
        t = (t_inter - times[joint][kf]) / (times[joint][kf+1] - times[joint][kf])  #on créé t entre 0 et 1 qui indique où on est dans la courbe de Bézier actuelle
        P0 = keys[joint][kf][0]
        P1 = P0 + keys[joint][kf][2][2]
        P3 = keys[joint][kf + 1][0]
        P2 = P3 + keys[joint][kf + 1][1][2]
        return P0*(1-t)**3 + P1*t*3*(1-t)**2 + P2*3*(1-t)*t**2 + P3*t**3 #évalue la bezier curve au temps t




if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
