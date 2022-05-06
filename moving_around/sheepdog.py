# NAME: sheepdog.py
# AUTHOR: Emma Bethel
# PURPOSE: wrapper class for interfacing with a Misty II while predicting 
#          position of herd in sheepdog algorithm

import base64
import io
import cv2
import math
import time
import numpy as np
from imageio import imread
from mistyPy.Robot import Robot
from mistyPy.Events import Events
from moving_around.find_target import find_target, get_dist_from_target

MAP_NAME = 'Map_20220506_09.29.46.UTC'

MAX_DIST_PER_MOVE = 10

METERS_PER_COORD = 0.04


class Sheepdog:
    # PURPOSE: sheepdog constructor
    # PARAMETERS: ip - ip of the Misty II acting as the sheepdog
    # RETURNS: N/A
    def __init__(self, ip):
        self.misty = Robot(ip)
        self.pos = np.array([0, 0])
        self.yaw = 0
        self.localized = False
        self.camera_ready = False
        self.meters_per_coord = 0.04
        self.target_pos = None
        self.target_pos_confidence = 0.0

        # setting up listener for robot position/status
        self.misty.RegisterEvent(
            "GetStatus",
            Events.SelfState,
            callback_function=self.update_status,
            keep_alive=True
        )
        print('misty status event registered')

        # starting SLAM
        self.misty.SetCurrentSlamMap(key=MAP_NAME)
        time.sleep(0.25)
        self.misty.StartTracking()
        self.wait_to_localize()
    
    # PURPOSE: halt code execution until Misty has located self on map
    # PARAMETERS: N/A
    # RETURNS: N/A
    def wait_to_localize(self):
        while not self.localized:
            print('waiting to localize...')
            time.sleep(0.5)
        print('localized at', self.pos)

    # PURPOSE: callback function for Misty state listener; stores relevant 
    #          position and orientation data
    # PARAMETERS: event - dict representing the robot state (send by Misty)
    # RETURNS: N/A
    def update_status(self, event):
        msg = event['message']

        if msg['location'] is not None:
            pose = msg['location']['pose']
            self.yaw = pose['yaw']

        if msg['occupancyGridCell'] is not None:
            self.pos[0] = msg['occupancyGridCell']['x']
            self.pos[1] = msg['occupancyGridCell']['y']

        if msg['slamStatus'] is not None:
            if 'HasPose' in msg['slamStatus']['statusList']:
                self.localized = True
            else:
                self.localized = False
        
        if msg['cameraStatus'] is not None:
            self.camera_ready = (msg['cameraStatus']['onboardCameraStatus'] == 'Ready')

    # PURPOSE: tell Misty to take a picture and parse output as an 
    #          OpenCV-compatible image matrix
    # PARAMETERS: N/A
    # RETURNS: the image as  BGR matrix, or NoneType if none received
    def get_camera_image(self):
        resp = self.misty.TakePicture(base64=True)
        resp_dict = resp.json()
        if resp_dict.get('error') is not None:
            print(resp_dict['error'])
            time.sleep(0.5)
            return None

        img_str = resp_dict['result']['base64']
        img = imread(io.BytesIO(base64.b64decode(img_str)))

        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    
    # PURPOSE: estimate position of an object in Misty SLAM map coordinates
    # PARAMETERS: estimated_dist - estimated distance (centimeters) of 
    #                              object away from misty (along her heading)
    # RETURNS: positioon vector (x, then y) of the object
    def calculate_target_coords(self, estimated_dist):
        meters = estimated_dist / 100.0
        displacement = np.array(
            [
                meters * math.cos(self.yaw),
                meters * math.sin(self.yaw)
            ]
        )
        displacement = displacement / self.meters_per_coord
        displacement = displacement.astype(int)
        print('displacement', displacement)

        return self.pos + displacement

    # PURPOSE: take a new camera image, search for vision target in it, and, if 
    #          applicable, update stored vision target pos accordingly
    # PARAMETERS: N/A
    # RETURNS: N/A
    # NOTE: currently a simple heuristic (update position if new measurement 
    #       confidence > old measurement confidence); should use something 
    #       smarter in the future
    def update_target_pos(self):
        img = self.get_camera_image()
        if img is not None:
            _, target_height = find_target(img)
            dist_from_target, confidence = get_dist_from_target(target_height)

            # future work: toss this into a kalman filter
            if confidence > self.target_pos_confidence:
                old_pos, old_conf = self.target_pos, self.target_pos_confidence
                self.target_pos = self.calculate_target_coords(dist_from_target)
                self.target_pos_confidence = confidence
                print(f'updating target position estimate from {old_pos} '
                      f'(confidence {old_conf}) to {self.target_pos} '
                      f'(confidence {self.target_pos_confidence})')
            else:
                print(f'not updating target position (current confidence '
                      f'{self.target_pos_confidence}, new {confidence})')
    
    # PURPOSE: drives Misty 10 grid squares (on SLAM map) toward currently 
    #          beleived position of vision target
    # PARAMETERS: N/A
    # RETURNS: N/A
    def drive_toward_target(self):
        if self.target_pos is not None:
            self.drive_toward_pos(self.target_pos)
        else:
            print('Error: no target position estimate')

    # PURPOSE: stops all current Misty movement
    # PARAMETERS: N/A
    # RETURNS: N/A
    def halt(self):
        self.misty.Halt()

    # PURPOSE: drives Misty 10 grid squares in the direction of a given 
    #          coordinate
    # PARAMETERS: coord - the coordinate. should be a valid coordinate on 
    #                     Misty's SLAM map (otherwise, empty path will be 
    #                     printed to console and Misty will not move)
    # RETURNS: N/A
    def drive_toward_pos(self, coord):
        direction = coord - self.pos
        dist = np.linalg.norm(direction)
        if dist > MAX_DIST_PER_MOVE:
            direction = (direction /dist) * MAX_DIST_PER_MOVE

        goal_pos = self.pos + direction.astype(int)
        print(f'attempting to map from {self.pos} to {goal_pos}')

        resp = self.misty.GetSlamPath(x=int(goal_pos[0]), y=int(goal_pos[1]))
        path_list = resp.json()['result']
        path = ''
        for waypoint in path_list:
            path += f'{waypoint["x"]}:{waypoint["y"]},'
        path = path[:-1]

        print(f'taking path {path}')
        self.misty.FollowPath(path=path)

    # PURPOSE: stops driving, SLAM, and event listening for Misty (should be 
    #          called on program termination)
    # PARAMETERS: N/A
    # RETURNS: N/A
    def quit(self):
        self.misty.Halt()
        self.misty.StopTracking()
        self.misty.UnregisterAllEvents()
