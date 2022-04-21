
# TITLE: Probabilistic Robotics HW5 - Hearding Simulation Robot Class
# AUTHORS: Allison Moore, Emma Bethel, Ray Rogers
# CREATED: 4/19/22


# IMPORTS

import math
import numpy as np
import cv2
import time


# CONSTANTS

REPULSIVE_CONSTANT = 8
OMEGA = 0.0000125


# ROBOT CLASS

class Robot:

    #INITIALIZERS

    def __init__(self, radius):
        ## SIMULATED VARIABLES
        self.effective_dist = radius # effective distanve of robot (since we're pushing objects, should be radius)
        self.goal_pos = np.array([50,50]) #goal position
        self.pos = np.array([10, 10])  # robot position
        self.flock_pos = np.array([30, 30]) #Center of Mass of flock
        self.flock_radius = 10 # radius of flock
        self.c_t = np.array([0.5, 0.5]) # initial c value 

    # GETTERS
    def get_pos(self):
        return self.pos
    def get_flock_pos(self):
        return self.flock_pos
    def get_flock_radius(self):
        return self.flock_radius

    #SETTERS


    def update_pos(self):
        control_vec = self.control(REPULSIVE_CONSTANT, OMEGA, self.effective_dist)
        print("control", control_vec)
        self.pos = self.pos + control_vec


    #HELPERS

    '''
    dog_influencer 
    purpose: 
    paramaters: replus - 
                d - position of the robot
                dv - 
                sbar - center of mass of the flock
                rs - 
                g - 
    returns: u  as either 1 or 0 
    '''
    def dog_influencer(self, replus, d, dv, sbar, rs, g):
        term1 = False
        term2 = False
        
        distance = np.linalg.norm(d-sbar)
        rnd = rs + dv 
        
        if distance <= rnd:
            term1 = True 

        val1 = g-d 
        val2 = sbar-d 
        val = np.linalg.det(np.c_[val1, val2])
        val_d = np.linalg.norm(g-d)
        sum = val/val_d 

        dis2 = np.linalg.norm(d-g) 
        dis3 = np.linalg.norm(sbar-g)
        if distance > rnd or dis2 <= dis3:
            term2 = True 

        if term2 and term1:
            return 1 
        else: 
            return 0


    '''
    repulsion_from_flock
    purpose:    determines the potion of the control vector preventing robot from 
                pushing herd backward while entering occlusion zone
    parameters: repulsive_constant - constant determining magnitude of repulsive 
                                    force generated to repel robot from goal
                pos - position vector of the robot
                flock_pos - position vector of the center of mass of the flock
                flock_radius - radius of the flock
                goal - position vector of the goal
                effective_dist - max distance from flock at which the robot's 
                                movements still affect those of the flock
    returns:    resulting control vector
    '''
    def repulsion_from_flock(self, repulsive_constant, pos, flock_pos, flock_radius, goal, effective_dist):
        robot_to_flock = pos - flock_pos
        dist_from_flock = np.linalg.norm(robot_to_flock)

        result = (1 / dist_from_flock + 1/(flock_radius + effective_dist))
        result *= repulsive_constant
        result *= self.dog_influencer(
            repulsive_constant,
            pos, effective_dist,
            flock_pos,
            flock_radius,
            goal 
        )
        result *= robot_to_flock / (dist_from_flock * dist_from_flock)
        
        return result      


    '''
    attraction_to_goal
    purpose:    determines the portion of the control vector pulling robot toward goal 
                once it has reach occlusion area
    parameters: c_t - current value of c weight
                pos - position vector of the robot
                flock_pos - position vector of the center of mass of the flock
                omega - positive scalar
    returns:    resulting control vector
    '''
    def attraction_to_goal(self, c_t, pos, flock_pos, omega):

        robot_to_flock = pos - flock_pos

        print("robot to fuck",robot_to_flock)

        robot_to_flock_mat = robot_to_flock.reshape(-1, 1)

        a = robot_to_flock_mat.T * math.sqrt(omega) * robot_to_flock_mat

        print("a",a)

        rho = 1 + 3*a + 3*np.matmul(a,a) + np.matmul(np.matmul(a,a),a)

        print("rho",rho)
        
        result = np.matmul(-1 * c_t, rho)
        result = np.matmul(result, a)

        result = np.matmul(result, math.sqrt(omega) * robot_to_flock)

        return result

    '''
    control
    purpose:    determine the control for the robots motion
    paramaters: repulsive_constant - weight to determine repulsion from flock
                omega - experimentally determined constant, transforms robot_to_flock_mat
                effective_dist - the distance from the robot to the flocks center of mass
    returns:    control vector for the robots motion

    '''
    def control(self, repulsive_constant, omega, effective_dist):

        robot_to_flock = self.pos - self.flock_pos

        robot_to_flock_mat = robot_to_flock.reshape(-1, 1)

        self.c_t = self.c_t + (robot_to_flock_mat.T * omega * robot_to_flock_mat)

        print("ct", self.c_t)

        a = self.repulsion_from_flock(repulsive_constant, self.pos, self.flock_pos, self.flock_radius, self.goal_pos, effective_dist)
        print("ua", a)

        b = self.attraction_to_goal(self.c_t, self.pos, self.flock_pos, omega)
        print("ub", b)


        return a + b


# MAIN FUNCTION

def main():
    r = Robot(5)
    for _ in range(2):
        print("before moving", r.get_pos())
        r.update_pos()
        print("after moving", r.get_pos())
        print('')


if __name__ == '__main__':
    main()
