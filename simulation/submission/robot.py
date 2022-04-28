# TITLE: Probabilistic Robotics HW5 - Herding Simulation Robot Class
# AUTHORS: Allison Moore, Emma Bethel, Ray Rogers
#          Largely based on Hu et. al.'s Occlusion-Based Algorithm for 
#          Autonomous Robotic Shepherding
# CREATED: 4/19/22


# IMPORTS

import math
import numpy as np
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
        # effective distance of robot (essentially how far robot should stay 
        #   from flock when swerving around it)
        self.effective_dist = radius + 10 

        self.goal_pos = goal_pos  # goal position
        self.pos = np.array(
            [
                random.uniform(radius / 2.0, max_x - (radius / 2.0)),
                random.uniform(radius / 2.0, max_y - (radius / 2.0))
            ]
        )  # robot position
        self.flock = flock
        self.c_t = C_0 # initial c value (for use in control vector calculation)

    @property
    def flock_pos(self):
        return self.flock.get_pos()
    
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
    avoid_flock
    purpose: determines whether current robot position requires swerving to 
             avoid flock (for use in ua calculation)
    paramaters: pos - position of the robot
                effective_dist - how far robot should stay 
                                 from flock when swerving around it
                flock_pos - center of mass of the flock
                flock_radius - radius of flock
                goal - goal position
    returns: 1 if robot must swerve to avoid flock when entering 
             occlusion zone, 0 otherwise
    '''
    def avoid_flock(self, pos, effective_dist, flock_pos, flock_radius, goal):
        term1 = False
        term2 = False

        flock_to_bot_vec = pos - flock_pos
        flock_to_bot_dist = np.linalg.norm(flock_to_bot_vec)
        bot_to_goal_vec = goal - pos
        goal_to_bot_dist = np.linalg.norm(bot_to_goal_vec)
        
        term1 = flock_to_bot_dist <= flock_radius + effective_dist

        val = abs(np.linalg.det(np.c_[bot_to_goal_vec, flock_pos - pos])) / goal_to_bot_dist

        goal_to_flock_dist = np.linalg.norm(goal - flock_pos)
        term2 = (val >= flock_radius or goal_to_bot_dist <= goal_to_flock_dist)

        if term2 and term1:
            return 1 
        
        return 0

    '''
    repulsion_from_flock
    purpose:    calculates portion of control vector guidign robot around herd 
                (instead of pushing it backward) while entering occlusion zone
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

        coefficient = 1 / dist_from_flock
        coefficient = coefficient -  (1 / (flock_radius + effective_dist))
        coefficient = coefficient * repulsive_constant

        dog_influencer = self.avoid_flock(
            pos,
            effective_dist,
            flock_pos, 
            flock_radius,
            goal 
        )

        coefficient = coefficient * dog_influencer

        # multiplying coefficient by a vector tangent to the one between robot 
        #   and flock, such that robot will move around flock without pushing it
        tangent = np.array([- flock_to_robot[1], flock_to_robot[0]]) / dist_from_flock

        # flipping direction of tangent vector if going clockwise will bring 
        #   robot closer to occlusion zone than counterclockwise
        goal_to_flock = flock_pos - goal
        det = np.linalg.det(np.c_[goal_to_flock, flock_to_robot])
        if det > 0:
            tangent = -1 * tangent


        return coefficient * tangent


    '''
    attraction_to_goal
    purpose: determines the portion of the control vector pulling robot 
             toward flock, which, once it has reached the occlusion zone, will 
             also push it toward goal 
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

        rho = (1 + a) ** 3

        return -1 * c_t * rho * a * math.sqrt(omega) * flock_to_robot

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

        # calculate ua (keeps robot from pushing flock backwards while moving 
        #   into occusion zone)
        a = self.repulsion_from_flock(repulsive_constant, self.pos, self.flock_pos, self.flock_radius, self.goal_pos, effective_dist)

        # calculate ub (moves robot toward flock)
        b = self.attraction_to_goal(self.c_t, self.pos, self.flock_pos, omega)
        
        # combine ua and ub into overall control vector
        raw_vec = a + b

        # if overall speed of control greater than max speed, cap the x 
        #   and y velocities while preserving vector direction
        raw_velocity = np.linalg.norm(raw_vec)
        if raw_velocity > MAX_SPEED:
            unit_vec = raw_vec / raw_velocity
            return MAX_SPEED * unit_vec

        return raw_vec


# MAIN FUNCTION -- FOR TESTING PURPOSES ONLY

def main():
    r = Robot(5)
    for _ in range(2):
        print("before moving", r.get_pos())
        r.update_pos()
        print("after moving", r.get_pos())
        print('')


if __name__ == '__main__':
    main()
