from robot import Robot
from flock import Flock
import numpy as np
import cv2

ROBOT_RADIUS = 5

def main():
    max_x = 300
    max_y = 300
    flock_rad = 5
    goal_pos = np.array([50,50]) #goal position
    f = Flock(max_x, max_y, flock_rad)
    r = Robot(max_x, max_y, ROBOT_RADIUS,f, goal_pos)
    print("robot start pos", r.get_pos(), "flock start pos", f.get_pos())
    r.update_pos()
    counter = 0
    reached_goal = False
    
    while not reached_goal:
        counter = counter + 1
        #create initial image
        img = np.full((max_x, max_y, 3), 255).astype(np.uint8)

        #cv circle for goal 
        cv2.circle(img, (goal_pos[0], goal_pos[1]), 8, (134,0,255), -1)
        
        #cv circle to represent flock
        flock_pos = r.get_flock_pos().astype(int)
        r_flock_pos = r.get_flock_radius()
        robot_pos = r.get_pos().astype(int)
        cv2.circle(img, (flock_pos[0], flock_pos[1]), r_flock_pos, (0,0,255), -1)

        #cv circle to represent robot
        cv2.circle(img, robot_pos, ROBOT_RADIUS, (155,0,155), -1)
        cv2.waitKey(100)
        cv2.imshow('image', img)

        # move robot and flock
        f.move(robot_pos, ROBOT_RADIUS)
        r.update_pos()
        
        # check whether goal has been reached
        reached_goal = (np.linalg.norm(f.get_pos() - goal_pos) <= 10)
        print('')



if __name__ == '__main__':
    main()
