import numpy as np
import cv2

DV = 10
RS = 10
GOAL_POS = np.array([200, 200])
FLOCK_POS = np.array([75, 75])

def dog_influencer(d, dv, sbar, rs, g):
        term1 = False
        term2 = False

        flock_to_bot_vec = d - sbar
        flock_to_bot_dist = np.linalg.norm(flock_to_bot_vec)
        bot_to_goal_vec = g - d
        goal_to_bot_dist = np.linalg.norm(bot_to_goal_vec)
        
        term1 = flock_to_bot_dist <= rs + dv

        val = abs(np.linalg.det(np.c_[bot_to_goal_vec, sbar - d])) / goal_to_bot_dist

        goal_to_flock_dist = np.linalg.norm(g - sbar)
        term2 = (val >= rs or goal_to_bot_dist <= goal_to_flock_dist)

        if term2 and term1:
            return 1 
        
        return 0


def main():
    #create initial image
    img = np.full((300, 300, 3), 255).astype(np.uint8)

    # cv2.circle(img, FLOCK_POS, DV + RS, (0, 255, 0), -1)

    for i, row in enumerate(img):
        for j in range(len(row)):
            if dog_influencer(np.array([i, j]), DV, FLOCK_POS, RS, GOAL_POS) == 1:
                img[i][j] = [255, 0, 0]

    cv2.circle(img, GOAL_POS, 3, (134,0,255), -1)
    cv2.circle(img, FLOCK_POS, RS, (0, 0, 0), -1)

    cv2.imshow('image', img)
    cv2.waitKey(0)


if __name__ == '__main__':
    main()
