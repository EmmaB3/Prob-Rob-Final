import math
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

    def get_current_pos(self):
        return self.pos
    
    def get_radius(self):
        return self.radius
    
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
            # x_dist = robot_pos[0] - self.pos[0]
            # y_dist = robot_pos[1] - self.pos[1]

            # theta = math.atan(y_dist / x_dist) # + random.gauss(0, 0.01)

            # self.pos = np.array(
            #     [
            #         robot_pos[0] - total_radius * math.cos(theta),
            #         robot_pos[1] - total_radius * math.sin(theta)
            #     ]
            # )



            
