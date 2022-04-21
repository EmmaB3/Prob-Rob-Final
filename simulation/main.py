from robot import Robot
import numpy as np
import cv2

ROBOT_RADIUS = 5
def main():

    r = Robot(ROBOT_RADIUS)
    r.update_pos()
    counter = 0
    while True:
        counter = counter + 1
        #create initial image
        img = np.full((300, 300, 3), 255).astype(np.uint8)
        #cv circle to represent flock
        flock_pos = r.get_flock_pos()
        r_flock_pos = r.get_flock_radius()
        get_pos = r.get_pos().astype(int)
        cv2.circle(img, flock_pos, r_flock_pos, (0,0,255), -1)
        #cv circle to represent dog
        cv2.circle(img, get_pos, 5, (155,0,155), -1)
        cv2.waitKey(10)
        cv2.imshow('image', img)
        r.update_pos()


if __name__ == '__main__':
    main()
