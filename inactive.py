# inactive mode!


# Importing Required Libraries
from queue import Queue

import timeit
import numpy
import numpy as np
import cv2
import os
import imutils
import math
import time
import asyncio
#import mediapipe as mp
from pedestrian import pedestrian, pedes_container
from pedes_detect.pdec import pdetector
import gdown

import threading

# use camera or video
use_camera = True

# enable usb or not
enable_usb = True

#blur face or not
blur_face = False

if enable_usb:
    import RoboticEye2

def draw_eyes(x,y,image):
    # cv2.rectangle(image,(int(0.35*image.shape[1]),int(0.80*image.shape[0])), (int(0.65*image.shape[1]),image.shape[0]),(255,255,255),-1)
    cv2.putText(image, "Inactive Mode", (int(0.07 * image.shape[1]), int(0.93 * image.shape[0])),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA, False)
    cv2.circle(image, (int(0.43 * image.shape[1]), int(0.90 * image.shape[0])), 30, (0, 0, 0), 2)
    cv2.circle(image, (int(0.43 * image.shape[1]), int(0.90 * image.shape[0])), 28, (255, 255, 255), -1)
    cv2.circle(image, (int(0.43 * image.shape[1] - x), int(0.90 * image.shape[0] + y)), 15, (0, 0, 0), -1)
    cv2.circle(image, (int(0.56 * image.shape[1]), int(0.90 * image.shape[0])), 30, (0, 0, 0), 2)
    cv2.circle(image, (int(0.56 * image.shape[1]), int(0.90 * image.shape[0])), 28, (255, 255, 255), -1)
    cv2.circle(image, (int(0.56 * image.shape[1] - x), int(0.90 * image.shape[0] + y)), 15, (0, 0, 0), -1)

def move_timer():
    status = gaze_object
    if (abs(status.prev_angle[0] - status.angle[0]) > 3):
        print("moving from", status.prev_angle, " to ", status.angle)
        # update movements
        # movementcounter+=1
        # print("movement counter: ", movementcounter)
        eyes.move_vertical(status.angle[0])
        eyes.move_horizontal(status.angle[1])
        eyes2.move_vertical(status.angle[0])
        eyes2.move_horizontal(status.angle[1])
        status.prev_angle = status.angle
    else:
        print("Send zero movement")
        eyes.move_vertical(status.prev_angle[0])
        eyes.move_horizontal(status.prev_angle[1])
        eyes2.move_vertical(status.prev_angle[0])
        eyes2.move_horizontal(status.prev_angle[1])
        status.prev_angle = status.angle
    timer = threading.Timer(0.05, move_timer)
    timer.start()

def blur_face(image, res):
    # to blur the pedestrians (used for video demo):
    y1 = res[1][1]
    y2 = int(res[1][1] + 0.3 * (res[1][3] - res[1][1]))
    x1 = int(res[1][0] + 0.2 * (res[1][2] - res[1][0]))
    x2 = int(res[1][0] + 0.9 * (res[1][2] - res[1][0]))
    if (x1 < 0):
        x1 = 0
    if (x2 < 0):
        x2 = 0
    if (y1 < 0):
        y1 = 0
    if (y2 < 0):
        y2 = 0
    ROI = image[y1:y2, x1:x2]
    if (y2 > y1 and x2 > x1):
        blur = cv2.GaussianBlur(ROI, (5, 5), 0)
        image[y1:y2, x1:x2] = blur
    return image

def RobobicEyeInit():
    eyes = RoboticEye2.RoboticEye2(port="/dev/ttyUSB0")
    eyes2 = RoboticEye2.RoboticEye2(port="/dev/ttyUSB1")
    # right
    eyes.sid_v = 4
    eyes.sid_h = 3
    eyes2.sid_h = 1
    eyes2.sid_v = 2
    # left
    eyes.invert_h = True
    eyes.invert_v = False
    eyes2.invert_h = True
    eyes2.invert_v = False
    eyes.setup()
    eyes2.setup()
    return eyes, eyes2
############################


class status():
    def __init__(self, angle1, angle2):
        self.angle = angle1
        self.prev_angle = angle2
        self.pedestrian_found = True


# Previous angle value helps in avoiding jittering. We can check the change in angle, and if its small, we neglet that motion.


############################

def lerp(location, image):
    # this function converts co-ordinates in an image, to angles to feed into the motors.

    height = image.shape[0]
    width = image.shape[1]
    centre = (int(width / 2), int(height / 2))
    yaw_angle = int(90 * (location[0] - centre[0]) / width)
    pitch_angle = int(-90 * (location[1] - centre[1]) / height)
    if (abs(pitch_angle) > 45):
        pitch_angle = 45
    if (abs(yaw_angle) > 45):
        yaw_angle = 45
    return (yaw_angle, pitch_angle)


###########################




##########################


gaze_object = status((0, 0), (0, 0))


# The lines below helps to control the hardware. Remove three quotes on top and bottom if you are testing with the hardware and set the ports according to use


if enable_usb:
    eyes, eyes2 = RobobicEyeInit()

# global movementcounter





def main():
    main_start = time.time()
    pdec = pdetector()
    # video input
    # cap = cv2.VideoCapture("pre_record_test_video_01.mp4")
    if not use_camera:
        cap = cv2.VideoCapture("test.mp4")
    else:
        cap = cv2.VideoCapture(0)  # To Test using a webcam
    #cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('H', '2', '6', '4'))

    A = []
    # for saving time per frame
    timepframe = []
    start = time.time()
    prev_direction = (0, 0)
    main_before_while_time = time.time() - main_start
    print("main before while start's time is:" + str(main_before_while_time))

    if enable_usb:
        move_timer()

    p_container=pedes_container()

    while True:
        print("-------------------------------------------------------")
        time_loopstart = time.time()
        B = []
        (grabbed, image) = cap.read()
        if not grabbed:
            break
        image = imutils.resize(image, width=700)
        gaze_direction = (int(image.shape[1] / 2), int(image.shape[0] / 2))
        t0 = time.time()
        results = pdec(image)#pedestrian_detection
        t1 = time.time()
        print("detection time: ", t1-t0)
        for res in results:
            cv2.rectangle(image, (res[1][0], res[1][1]), (res[1][2], res[1][3]), (0, 255, 0))

        p_container.update(results)

        A=p_container.plist
        for i in range(len(A)):
            text = str(i+1)
            font = cv2.FONT_HERSHEY_SIMPLEX
            org = (A[i].location[0], A[i].location[1])
            fontScale = 0.5
            thickness = 1
            color = (0, 0, 255)
            cv2.putText(image, text, org, font, fontScale, color, thickness, cv2.LINE_AA, False)


        if p_container.any():
            gaze_direction=p_container.get_eye_direction()

        height = image.shape[0]
        width = image.shape[1]
        x = int(15 - ((30 * gaze_direction[0]) / width))
        y = int(((30 * gaze_direction[1]) / height) - 13)
        if (len(A) == 0):
            x = 0
            y = 0

        draw_eyes(x,y,image)


        # A is the array of pedestrian objects!
        # Do the below task in the above loop!
        # we need to perform a search of pedestrians | Update the location of orignal pedestrians | A to be outside the array
        # Del the pedestrians now outside the frame | Add new pedestrians | Check gaze Time etc|
        # location,size,confidence

        if len(results) == 0:
            print("No pedestrain found")
            gaze_object.pedestrian_found = False
            gaze_object.angle = gaze_object.prev_angle
        else:
            gaze_object.angle = lerp(gaze_direction, image)
            gaze_object.pedestrian_found = True
        cv2.putText(image, str(gaze_object.angle), gaze_direction, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA, False)
        # cv2.resizeWindow("Detection", 1920, 1080)
        # cv2.resize(image,(1920,1080))
        cv2.namedWindow("Detection", 0)
        cv2.imshow("Detection", image)
        # calculate how much process time for per frame, save it into timeperframe.csv
        timeperframe = time.time() - start
        # show in log (command)
        # print("took ", timeperframe, " seconds for 1 frame")
        # save into .csv
        timepframe.append(timeperframe)
        numpy.savetxt('time_per_frame.csv', timepframe)
        start = time.time()
        key = cv2.waitKey(1)

        # Exit if press ESC
        if key == 27:
            # eyes.move(1,0,True)
            # eyes.move(2,0,True)
            # eyes2.move(3, 0, True)
            # eyes2.move(4, 0, True)

            break

        net_time = time.time() - time_loopstart
        print("Net Time:" + str(net_time))
        time_gazestart = time.time()
        # if k == 0:
        # gaze(gaze_object)
        time_gazeend = time.time()
        gazetime = time_gazeend - time_gazestart
        # gazetime = int(gazetime * 1000) # ms
        # print("gaze time is:" + str(gazetime))
        time_loopend = time.time()
        looptime = time_loopend - time_loopstart
        # looptime = int(looptime) # ms
        print("loop time is:" + str(looptime))

    if enable_usb:
        eyes.__del__()
        eyes2.__del__()
    # i = 0
    print("end of this program")
    cap.release()
    cv2.destroyAllWindows()
    # exit()


############### now we build a priority system!


if __name__ == '__main__':
    main()


