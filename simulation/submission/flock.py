# TITLE: Probabilistic Robotics HW5 - Herding Simulation Flock Class
# AUTHORS: Allison Moore, Emma Bethel, Ray Rogers
# CREATED: 4/21/22

import numpy as np
import random


class Flock:

    def __init__(self, max_x, max_y, radius):
        self.pos = np.array(
            [
                random.uniform(radius / 2.0, max_x) - (radius / 2.0),
                random.uniform(radius / 2.0, max_y) - (radius / 2.0)
            ]
        )
        self.radius = radius

    def get_pos(self):
        return self.pos
    
    def get_radius(self):
        return self.radius
    
    '''
    move
    purpose: move the flock of currently being pushed by a robot
    parameters: robot_pos - position of robot centroid
                robot_radius - radius of robot
    returns: n/a
    '''
    def move(self, robot_pos, robot_radius):
        robot_to_flock = self.pos - robot_pos
        dist_from_robot = np.linalg.norm(robot_to_flock)

        total_radius = robot_radius + self.radius

        if dist_from_robot < total_radius:
            # getting direction of movement
            direction = robot_to_flock / np.linalg.norm(robot_to_flock)

            # adding noise
            direction = direction + np.array(
                [random.gauss(0, 0.1), random.gauss(0, 0.2)]
            )

            self.pos = robot_pos + total_radius * direction
