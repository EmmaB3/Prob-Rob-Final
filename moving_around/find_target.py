# NAME: find_target.py
# AUTHOR: Emma Bethel
# PURPOSE: finding a square, yellow vision target in an image and estimating 
#          distance from it based on known dimensions

import cv2

LOWER_THRESHOLD = (91, 77, 0)
UPPER_THRESHOLD = (107, 93, 255)

# vision target dimensions (in cm)
TARGET_WIDTH = 5
TARGET_HEIGHT = 4.75

# experimentally determined (by anaylzing target size in image taken at known
#   distance of BASE_DIST cm
FOCAL_LENGTH = 3300
BASE_DIST = 50


# PURPOSE: find a vision target in an image and compute its dimensions (in 
#          pixels)
# PARAMETERS: img - the image, as a matrix of BGR points
# RETURNS: tuple containing width and height (in pixels) of vision target in 
#          image (or 0,0 if none found)
def find_target(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, LOWER_THRESHOLD, UPPER_THRESHOLD)
    output = cv2.bitwise_and(img,img, mask= mask)
    edges = cv2.Canny(output, 10, 100)
    contours, _ = cv2.findContours(
        edges,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE
    )

    if contours == []:
        return 0, 0

    c = max(contours, key = cv2.contourArea)
    center, dims, _ = cv2.minAreaRect(c)

    return dims


# PURPOSE: given a height (in pixels), approximates how far the camera was from 
#          the vision target
# PARAMETERS: pixel_height - the height of the target in pixels
# RETURNS: dist - the calculated distance in cm
#          confidence - a value between 0 and 1 representing the probability
#                       that the approximated distance is accurate (higher
#                       closer to 50, which is the distance the pixel-to-meter
#                       conversions were initially calibrated at)
def get_dist_from_target(pixel_height):
    dist = TARGET_HEIGHT * FOCAL_LENGTH / pixel_height

    confidence = min(0.999, BASE_DIST / (2 * abs(dist - BASE_DIST)))

    return dist, confidence
