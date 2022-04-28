
# TITLE: Probabilistic Robotics HW5 - Hearding Simulation Robot Class
# AUTHORS: Allison Moore, Emma Bethel, Ray Rogers
# CREATED: 4/19/22


# IMPORTS

import math
from tkinter.tix import MAX
import numpy as np
import cv2
import time
import random


# CONSTANTS

REPULSIVE_CONSTANT = 2000
OMEGA = 0.0001
MAX_SPEED = 5  # in pixels
C_0 = 2


# ROBOT CLASS

class Robot:

    #INITIALIZERS

    def __init__(self, max_x, max_y, radius, flock, goal_pos):
        ## SIMULATED VARIABLES
        self.effective_dist = radius + 10 # effective distanve of robot (since we're pushing objects, should be radius)
        self.goal_pos = goal_pos
        self.pos = np.array(
            [
                random.uniform(radius / 2.0, max_x - (radius / 2.0)),
                random.uniform(radius / 2.0, max_y - (radius / 2.0))
            ]
        ) # robot position
        self.flock = flock
        self.c_t = C_0 # initial c value 

    @property
    def flock_pos(self):
        return self.flock.get_current_pos()
    
    @property
    def flock_radius(self):
        return self.flock.radius

    # GETTERS
    def get_pos(self):
        return self.pos
    def get_flock_pos(self):
        return self.flock_pos
    def get_flock_radius(self):
        return self.flock_radius

    #SETTERS


    def update_pos(self):
        print('robot pos', self.pos, 'flock pos', self.flock_pos, "goal pos", self.goal_pos)
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
    def dog_influencer(self, d, dv, sbar, rs, g):
        term1 = False
        term2 = False

        flock_to_bot_vec = d - sbar
        flock_to_bot_dist = np.linalg.norm(flock_to_bot_vec)
        bot_to_goal_vec = g - d
        goal_to_bot_dist = np.linalg.norm(bot_to_goal_vec)
        
        term1 = flock_to_bot_dist <= rs + dv

        val = abs(np.linalg.det(np.c_[bot_to_goal_vec, sbar - d])) / goal_to_bot_dist

        # dis2 = np.linalg.norm(d-g) 
        # dis3 = np.linalg.norm(sbar-g)
        goal_to_flock_dist = np.linalg.norm(g - sbar)
        term2 = (val >= rs or goal_to_bot_dist <= goal_to_flock_dist)

        if term2 and term1:
            return 1 
        
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
        flock_to_robot = pos - flock_pos
        dist_from_flock = np.linalg.norm(flock_to_robot)

        result = 1 / dist_from_flock
        result = result -  (1 / (flock_radius + effective_dist))
        result = result * repulsive_constant

        dog_influencer = self.dog_influencer(
            pos,
            effective_dist,
            flock_pos, 
            flock_radius,
            goal 
        )
        if dog_influencer != 0:
            print('NONZERO', pos, flock_pos)
        result = result * dog_influencer
        result = result * flock_to_robot / (dist_from_flock ** 2)
        
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

        flock_to_robot = pos - flock_pos

        flock_to_robot_mat = flock_to_robot.reshape(-1, 1)

        a = np.matmul(
            flock_to_robot_mat.T,
            math.sqrt(omega) * flock_to_robot_mat
        ).item()

        # rho = 1 + 3*a + 3*np.matmul(a,a) + np.matmul(np.matmul(a,a),a)
        rho = (1 + a) ** 3
        print("rho", rho)
        return -1 * c_t * rho * a * math.sqrt(omega) * flock_to_robot

        
        # result = np.matmul(-1 * c_t, rho)
        # result = np.matmul(result, a)

        # result = np.matmul(result, math.sqrt(omega) * flock_to_robot)

        # return result

    '''
    control
    purpose:    determine the control for the robots motion
    paramaters: repulsive_constant - weight to determine repulsion from flock
                omega - experimentally determined constant, transforms robot_to_flock_mat
                effective_dist - the distance from the robot to the flocks center of mass
    returns:    control vector for the robots motion

    '''
    def control(self, repulsive_constant, omega, effective_dist):

        # update c_t
        flock_to_bot = self.pos - self.flock_pos
        flock_to_bot_mat = flock_to_bot.reshape(-1, 1)

        c_change = np.matmul(flock_to_bot_mat.T, omega * flock_to_bot_mat).item()
        self.c_t = self.c_t + c_change

        print('c_t', self.c_t)

        a = self.repulsion_from_flock(repulsive_constant, self.pos, self.flock_pos, self.flock_radius, self.goal_pos, effective_dist)
        print("ua", a)

        b = self.attraction_to_goal(self.c_t, self.pos, self.flock_pos, omega)
        print("ub", b)
        
        raw_vec = a + b

        # if overall soeed of control greater than max speed, cap the x 
        #   and y velocities while preserving vector direction
        raw_velocity = np.linalg.norm(raw_vec)
        if raw_velocity > MAX_SPEED:
            unit_vec = raw_vec / raw_velocity
            return MAX_SPEED * unit_vec

        return raw_vec


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
