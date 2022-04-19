

#IMPORTS-------------------
import math
import numpy as np
import cv2

'''
dog_influencer 
given: replus, g, sbar, rs, g
reutn: u as either 1 or 0 
'''
def dog_influencer(replus, d, dv, sbar, rs, g):
    term1 = False
    term2 = False
    
    distance = np.linalg.norm(d-sbar)
    rnd = rs + dv 
    
    if distance <= rnd:
        term1 = True 

    val1 = g-d 
    val2 = sbar-d 
    val = np.linalg.det(val1, val2)
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
purpose: determines the potion of the control vector preventing robot from 
          pushing herd backward while entering occlusion zone
parameters: repulsive_constant - constant determining magnitude of repulsive 
                                 force generated to repel robot from goal
            pos - position vector of the robot
            flock_pos - position vector of the center of mass of the flock
            flock_radius - radius of the flock
            goal - position vector of the goal
            effective_dist - max distance from flock at which the robot's 
                             movements still affect those of the flock
returns: resulting control vector
'''
def repulsion_from_flock(repulsive_constant, pos, flock_pos, flock_radius, goal, effective_dist):
    robot_to_flock = pos - flock_pos
    dist_from_flock = np.linalg.norm(robot_to_flock)

    result = (1 / dist_from_flock + 1/(flock_radius + effective_dist))
    result *= repulsive_constant
    result *= dog_influencer(
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
purpose: determines the portion of the control vector pulling robot toward goal 
         once it has reach occlusion area
parameters: c_t - current value of c weight
            pos - position vector of the robot
            flock_pos - position vector of the center of mass of the flock
            omega - positive scalar
returns: resulting control vector
'''
def attraction_to_goal(c_t, pos, flock_pos, omega):
    robot_to_flock = pos - flock_pos

    robot_to_flock_transpose = np.atleast_2d(robot_to_flock).T

    a = robot_to_flock_transpose * math.sqrt(omega) * robot_to_flock

    result = math.pow(1 + a, 2)
    result *= a * math.sqrt(omega) * robot_to_flock * -1 * c_t

    return result


def control(c_t, pos, flock_pos, flock_radius, goal_pos, repulsive_constant, 
            omega, effective_dist):

    # TODO: update c_t here

    return attraction_to_goal(c_t, pos, flock_pos, omega) + repulsion_from_flock(repulsive_constant, pos, flock_pos, flock_radius, goal_pos, effective_dist)


def draw_positions(sbar, rs, d, replus, dv,g):
        #create initial image
        img = np.full((300, 300, 3), 255).astype(np.uint8)

        #cv circle to represent flock
        cv2.circle(img, sbar, rs, (0,0,255), -1)

        #cv circle to represent flock
        cv2.circle(img, d, 5, (155,0,155), -1)


        #show image 
        cv2.imshow('initial dog heard', img)
        cv2.waitKey(0)
        
def main():

    ## SIMULATED VARIABLES
    replus = 8 #repulsive constant 
    dv = 8 # effective distanve of dog tb experimentally determine
    g = np.array([300,300]) #goal position
    d = np.array([10, 10])  # robot position
    sbar = np.array([50, 50]) #Center of Mass of flock
    rs = 10 # radius of flock
    c_init = 8 # initial c value 

    xi = d-sbar
 
    #function r
    u = dog_influencer(replus, d, dv, sbar, rs, g)

    #draw dogs 
    draw_positions(sbar, rs, d, replus, dv,g)


if __name__ == '__main__':
    main()
