#face distance detection reference: https://www.geeksforgeeks.org/realtime-distance-estimation-using-opencv-python/

from kalmanfilter import Kalman
import cv2 as cv 


#create instrance of Kalman Filter Object
width_kalman = Kalman()
distance_kalman = Kalman()

#set reference distance from camera and width of face
REF_D= 76.2
REF_W = 14.3

#create instance of face detector 
face_detector = cv.CascadeClassifier(cv.data.haarcascades +"haarcascade_frontalface_default.xml")
 
font=cv.FONT_HERSHEY_COMPLEX
 
#find estimation of distance
def Dist(fl, face_w_estimate):

    #find distance 
    dist = (REF_W* fl)/face_w_estimate
    return dist

#calculate the width of face
def face_width(img):
    fw = 0

    #convert image to grayscale
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    #find face in image
    face = face_detector.detectMultiScale(gray,1.3,5)

    #find coordinates of each face in image
    for (x,y,h,w) in face:
        #draw rectangle around face
        cv.rectangle(img, (x, y), (x+w, y+h), (0,255, 0), 2)
        #add x,y coordinates of face to kalman filter prediction
        p = width_kalman.estimate(x, y)
        p1 = int(p[0])
        p2 = int(p[1])
        #draw predicted position of face
        cv.rectangle(img, (p1, p2), (p1+w, p2+h), (0,0, 255), 2)

        #find face width
        fw = w
    return fw

#process distances for reference image
def ref_image_process():
    ref = cv.imread("Ref_image.png")
    #get face width
    ref_fw = face_width(ref)
    #get focal length
    fl = (ref_fw*REF_D)/REF_W
    return fl


#start camera object
cam = cv.VideoCapture(0)

#loop camera frames
while True:
    _,frame = cam.read()
    fw = face_width(frame)

    original_fl = ref_image_process()

    if fw:
        dist = Dist(original_fl, fw)
        print("DIST", dist)
        #add x,y coordinates of face to kalman filter prediction
        pd = distance_kalman.estimate(dist, 0)
        print(pd)
        # Drawing Text on the screen
        cv.putText(
            frame, f"Actual Dist: {round(dist,2)} CM. Predicted: {round(pd[0],2)} CM.", (30, 35), font, 0.6, (0,255,0), 2)
 
    #show frame
    cv.imshow("face", frame)

    #quit with escape
    if cv.waitKey(1) == ord("q"):
        break
 

#cleanup
cam.release()
cv.destroyAllWindows() 
