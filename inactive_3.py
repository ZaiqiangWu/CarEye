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
import mediapipe as mp
from pedestrian import pedestrian
import RoboticEye
import RoboticEye2
import threading


############################


class status():
    def __init__(self, angle1, angle2):
        self.angle = angle1
        self.prev_angle = angle2


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


def issimilar(x, y):
    # This function helps to track pedestrians in different frames. If two pedestrians (one in previous frame another in subsequent frame)
    # are at small distance in the image, they are the same.
    distance = math.sqrt(
        (x.location[0] - y.location[0]) * (x.location[0] - y.location[0]) + (x.location[1] - y.location[1]) * (
                x.location[1] - y.location[1]))
    if distance > 10:
        return False
    else:
        return True


##########################
def priority(x, y):
    # We prioritize the pedestrian closer to us.
    # Usually the pedestrians who were closer, in the image, they are slightly below than those who are far.

    if x.location[1] >= y.location[1]:
        return x
    else:
        return y


##########################


# These are NMS and Confidence Thresholds used in pedestrian_detection

NMS_THRESHOLD = 0.3
MIN_CONFIDENCE = 0.7


def pedestrian_detection(image, model, layer_name, personidz=0):
    # image.shape returns as tuple of rows columns color channels
    (H, W) = image.shape[:2]
    results = []  # we store our predictions

    blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True,
                                 crop=False)  # we create a blob... scale evryy pixel intensity by 1/255, make image of standard size, swap RB channels and dont crop
    model.setInput(blob)  # model is yolo trained on yolomini dataset
    layerOutputs = model.forward(layer_name)  # output layer; list of list of detections

    boxes = []
    centroids = []
    confidences = []

    # above here we store desired output

    for output in layerOutputs:  # we loop through the detections
        for detection in output:  # for each detection

            scores = detection[
                     5:]  # class probabilities (in logits format) begin with index 5, detection= [x,y,h,w,box_score,_,_,_.. diff objects]
            classID = np.argmax(scores)  # arg of max scores
            confidence = scores[classID]  # score of most likely object

            if (classID == personidz) and confidence > MIN_CONFIDENCE:  # personidz=0 , correrponds to a person

                box = detection[0:4] * np.array([W, H, W, H])
                (centerX, centerY, width, height) = box.astype("int")

                x = int(centerX - (width / 2))
                y = int(centerY - (height / 2))

                boxes.append([x, y, int(width), int(height)])
                centroids.append((centerX, centerY))
                confidences.append(float(confidence))
    # apply non-maxima suppression to suppress weak, overlapping
    # bounding boxes
    idzs = cv2.dnn.NMSBoxes(boxes, confidences, MIN_CONFIDENCE, NMS_THRESHOLD)
    # ensure at least one detection exists
    if len(idzs) > 0:
        # loop over the indexes we are keeping
        for i in idzs.flatten():
            # extract the bounding box coordinates
            (x, y) = (boxes[i][0], boxes[i][1])
            (w, h) = (boxes[i][2], boxes[i][3])
            # update our results list to consist of the person
            # prediction probability, bounding box coordinates,
            # and the centroid
            res = (confidences[i], (x, y, x + w, y + h), centroids[i])
            results.append(res)
    # return the list of results
    return results


gaze_object = status((0, 0), (0, 0))

# The lines below helps to control the hardware. Remove three quotes on top and bottom if you are testing with the hardware and set the ports according to use


eyes=RoboticEye2.RoboticEye2(port="/dev/ttyUSB0")
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
# eyes2.setup(id_motor=1)
# eyes2.setup(id_motor=2)

eyes.setup()
eyes2.setup()


# global movementcounter

def gaze(status):
    if (abs(status.prev_angle[0] - status.angle[0]) > 3):
        print("moving from", status.prev_angle, " to ", status.angle)
        # update movements
        # movementcounter+=1
        # print("movement counter: ", movementcounter)
        # eyes.move(3, status.angle[1], True)
        # eyes2.move(1, status.angle[1], True)
        # eyes.move(4, status.angle[0], False)
        # eyes2.move(2, status.angle[0], False)
        eyes.move_vertical(status.angle[0])
        eyes.move_horizontal(status.angle[1])
        eyes2.move_vertical(status.angle[0])
        eyes2.move_horizontal(status.angle[1])
        status.prev_angle = status.angle

    else:
        print("Send zero movement")
        # eyes.move(3, status.prev_angle[1], True)
        # eyes2.move(1, status.prev_angle[1], True)
        # eyes.move(4, status.prev_angle[0], False)
        # eyes2.move(2, status.prev_angle[0], False)
        eyes.move_vertical(status.prev_angle[0])
        eyes.move_horizontal(status.prev_angle[1])
        eyes2.move_vertical(status.prev_angle[0])
        eyes2.move_horizontal(status.prev_angle[1])
        status.prev_angle = status.angle


def main():
    main_start = time.time()
    # movementcounter = 0
    # print(cv2.__version__)
    # Setting up the Pose Estimation Model
    pose_model = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    pose = pose_model.Pose(static_image_mode=True)

    # Setting up Yolov4-tiny model for pedestrian Detection
    labelsPath = "coco.names"
    LABELS = open(labelsPath).read().strip().split("\n")
    weights_path = "yolov4-tiny.weights"
    config_path = "yolov4-tiny.cfg"
    model = cv2.dnn.readNetFromDarknet(config_path, weights_path)
    # use CUDA (support now)
    model.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    model.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
    layer_name = model.getLayerNames()
    layer_name = [layer_name[i - 1] for i in model.getUnconnectedOutLayers()]

    # video input
    # cap = cv2.VideoCapture("pre_record_test_video_01.mp4")
    # cap = cv2.VideoCapture("test.mp4")
    cap = cv2.VideoCapture(0)  # To Test using a webcam
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('H', '2', '6', '4'))

    A = []
    # for saving time per frame
    timepframe = []
    start = time.time()
    prev_direction = (0, 0)
    main_before_while_time = time.time() - main_start
    print("main before while start's time is:" + str(main_before_while_time))
    k = 0
    #i = 1

    def move_timer():
        status=gaze_object
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

    move_timer()

    while True:
        print("-------------------------------------------------------")
        k = k + 1
        if k == 2:
            k = 0
        time_loopstart = time.time()
        B = []
        (grabbed, image) = cap.read()
        if not grabbed:
            break
        image = imutils.resize(image, width=700)
        gaze_direction = (int(image.shape[1] / 2), int(image.shape[0] / 2))
        p = time.time()
        results = pedestrian_detection(image, model, layer_name, personidz=LABELS.index("person"))
        p2 = time.time() - p
        print("detection time: ", p2)
        for i in range(len(results)):
            C = A  # c=[p1]
            location = (results[i][2][0], results[i][2][1])
            size = (results[i][1][2] - results[i][1][0], results[i][1][3] - results[i][1][1])
            confidence = results[i][0]
            new_pedestrian = pedestrian(location, size, confidence, (location[0], int(location[1] - size[1] * 0.3)))
            focus = new_pedestrian
            focus_top_left = (int(focus.location[0] - size[0] / 2), int(focus.location[1] - size[1] / 2))
            cropped_pedestrian = image[focus_top_left[1]:focus_top_left[1] + focus.size[1],
                                  focus_top_left[0]:focus_top_left[0] + focus.size[0]]
            # TODO detect 3 frame, if 3f have wave hand, then detect

            if (cropped_pedestrian.any()):
                rgbimg = cv2.cvtColor(cropped_pedestrian, cv2.COLOR_BGR2RGB)
                output = pose.process(rgbimg)
                if (output.pose_landmarks != None):
                    print(i)
                    mp_drawing.draw_landmarks(rgbimg, output.pose_landmarks, pose_model.POSE_CONNECTIONS,
                                              landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
                    out_img = cv2.cvtColor(rgbimg, cv2.COLOR_RGB2BGR)
                    image[focus_top_left[1]:focus_top_left[1] + focus.size[1],
                    focus_top_left[0]:focus_top_left[0] + focus.size[0]] = out_img
                    lm = output.pose_landmarks
                    lmPose = pose_model.PoseLandmark
                    # image[focus_top_left[1]:focus_top_left[1]+focus.size[1],focus_top_left[0]:focus_top_left[0]+focus.size[0]]=out_img
                    l_inner_eye = (int(lm.landmark[lmPose.LEFT_EYE_INNER].x * cropped_pedestrian.shape[1]),
                                   int(lm.landmark[lmPose.LEFT_EYE_INNER].y * cropped_pedestrian.shape[0]))
                    r_inner_eye = (int(lm.landmark[lmPose.RIGHT_EYE_INNER].x * cropped_pedestrian.shape[1]),
                                   int(lm.landmark[lmPose.RIGHT_EYE_INNER].y * cropped_pedestrian.shape[0]))
                    to_gaze = (int((l_inner_eye[0] + r_inner_eye[0]) / 2), int((l_inner_eye[1] + r_inner_eye[1]) / 2))
                    eye_coordinates = (focus_top_left[0] + to_gaze[0], focus_top_left[1] + to_gaze[1])
                    new_pedestrian.update_eyes(eye_coordinates)
                    ###################
                    l_elbow_y = int(lm.landmark[lmPose.LEFT_ELBOW].y * image.shape[1])
                    l_wrist_y = int(lm.landmark[lmPose.LEFT_WRIST].y * image.shape[1])
                    l_shoulder_x = int(lm.landmark[lmPose.LEFT_SHOULDER].x * image.shape[0])
                    l_shoulder_y = int(lm.landmark[lmPose.LEFT_SHOULDER].y * image.shape[1])
                    l_thumb_x = int(lm.landmark[lmPose.LEFT_THUMB].x * image.shape[0])
                    l_wrist_x = int(lm.landmark[lmPose.LEFT_WRIST].x * image.shape[0])
                    # right hand
                    r_elbow_y = int(lm.landmark[lmPose.RIGHT_ELBOW].y * image.shape[1])
                    r_wrist_y = int(lm.landmark[lmPose.RIGHT_WRIST].y * image.shape[1])
                    r_shoulder_x = int(lm.landmark[lmPose.RIGHT_SHOULDER].x * image.shape[0])
                    r_shoulder_y = int(lm.landmark[lmPose.RIGHT_SHOULDER].y * image.shape[1])
                    r_thumb_x = int(lm.landmark[lmPose.RIGHT_THUMB].x * image.shape[0])
                    r_wrist_x = int(lm.landmark[lmPose.RIGHT_WRIST].x * image.shape[0])
                    
                    # check if the person is facing towards camera!
                    # his left shoulder would be to more right (more x) than left shoulder
                    if (l_shoulder_x > r_shoulder_x):
                        if (l_wrist_y < l_shoulder_y or r_wrist_y < r_shoulder_y):
                            new_pedestrian.prioritize(True)
                            image = cv2.putText(image, "wave detected!", focus_top_left, cv2.FONT_HERSHEY_SIMPLEX, 1,
                                                (0, 0, 255), 2, cv2.LINE_AA)


            for res in results:
                cv2.rectangle(image, (res[1][0], res[1][1]), (res[1][2], res[1][3]), (0, 255, 0))
                """
                #to blur the pedestrians (used for video demo):
                y1=res[1][1]
                y2=int(res[1][1]+0.3*(res[1][3]-res[1][1]))
                x1=int(res[1][0]+0.2*(res[1][2]-res[1][0]))
                x2=int(res[1][0]+0.9*(res[1][2]-res[1][0]))
                if (x1<0):
                    x1=0
                if (x2<0):
                    x2=0
                if (y1<0):
                    y1=0
                if (y2<0):
                    y2=0
                ROI = image[y1:y2, x1:x2]
                if (y2>y1 and x2>x1):
                    blur = cv2.GaussianBlur(ROI,(5,5),0)
                    image[y1:y2, x1:x2]=blur
                """
            if (len(A) == 0):
                A.append(new_pedestrian)
                B.append(0)

            for i in range(len(A)):
                if issimilar(new_pedestrian, A[i]):
                    if (new_pedestrian.superior):
                        A[i].prioritize(True)
                    else:
                        A[i].prioritize(False)
                    A[i].updategazetime()
                    A[i].updatetime()
                    A[i].add_location(location)
                    if abs(lerp(new_pedestrian.eyes, image)[0] - lerp(A[i].eyes, image)[0]) > 5:
                        A[i].update_eyes(new_pedestrian.eyes)
                    B.append(i)
                    break
                if (i == len(A) - 1):
                    # new pedestrian?
                    B.append(len(A))
                    A.append(new_pedestrian)
        C = []
        for i in range(len(B)):
            C.append(A[B[i]])
        A = C

        # sort according to priority

        if len(A) > 1:
            for i in range(1, len(A)):
                key = A[i]
                j = i - 1
                while j >= 0 and priority(key, A[j]) == 1:
                    A[j + 1] = A[j]
                    j -= 1
                    A[j + 1] = key

        for i in range(len(A)):
            if A[i].superior and A[i].gaze == 0:
                A[i].changegazestatus(1)
                A[i].addstarttime()
                if (not isinstance(focus.eyes, float)):
                    gaze_direction = focus.eyes
                    image = cv2.circle(image, focus.eyes, 5, (0, 0, 255), 5, cv2.FILLED)
                break
            if (A[i].gaze == 1):
                if (A[i].gazetime > 10):
                    A[i].changegazestatus(0)
                    A[i].gazetime = 0
                    A[(i + 1) % len(A)].changegazestatus(1)
                    A[(i + 1) % len(A)].addstarttime()
                    focus = A[(i + 1) % len(A)]
                    if (not isinstance(focus.eyes, float)):
                        gaze_direction = focus.eyes
                        image = cv2.circle(image, focus.eyes, 5, (0, 0, 255), 5, cv2.FILLED)
                    break
                else:
                    A[i].updategazetime()
                    focus = A[i]
                    if (not isinstance(focus.eyes, float)):
                        gaze_direction = focus.eyes
                        image = cv2.circle(image, focus.eyes, 5, (0, 0, 255), 5, cv2.FILLED)
                    break

            if (i == len(A) - 1):
                focus = A[0]
                A[0].changegazestatus(1)
                A[0].addstarttime()
                if (not isinstance(focus.eyes, float)):
                    gaze_direction = focus.eyes
                    image = cv2.circle(image, focus.eyes, 5, (0, 0, 255), 5, cv2.FILLED)
                break

        count = 1
        for i in range(len(A)):
            text = str(count)
            font = cv2.FONT_HERSHEY_SIMPLEX
            org = (A[i].location[0], A[i].location[1])
            fontScale = 0.5
            thickness = 1
            color = (0, 0, 255)
            cv2.putText(image, text, org, font, fontScale, color, thickness, cv2.LINE_AA, False)
            count = count + 1

        height = image.shape[0]
        width = image.shape[1]
        x = int(15 - ((30 * gaze_direction[0]) / width))
        y = int(((30 * gaze_direction[1]) / height) - 15)
        if (len(A) == 0):
            x = 0
            y = 0
        # cv2.rectangle(image,(int(0.35*image.shape[1]),int(0.80*image.shape[0])), (int(0.65*image.shape[1]),image.shape[0]),(255,255,255),-1)
        cv2.putText(image, "Inactive Mode", (int(0.07 * image.shape[1]), int(0.93 * image.shape[0])),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA, False)
        cv2.circle(image, (int(0.43 * image.shape[1]), int(0.90 * image.shape[0])), 30, (0, 0, 0), 2)
        cv2.circle(image, (int(0.43 * image.shape[1]), int(0.90 * image.shape[0])), 28, (255, 255, 255), -1)
        cv2.circle(image, (int(0.43 * image.shape[1] - x), int(0.90 * image.shape[0] + y)), 15, (0, 0, 0), -1)
        cv2.circle(image, (int(0.56 * image.shape[1]), int(0.90 * image.shape[0])), 30, (0, 0, 0), 2)
        cv2.circle(image, (int(0.56 * image.shape[1]), int(0.90 * image.shape[0])), 28, (255, 255, 255), -1)
        cv2.circle(image, (int(0.56 * image.shape[1] - x), int(0.90 * image.shape[0] + y)), 15, (0, 0, 0), -1)

        # A is the array of pedestrian objects!
        # Do the below task in the above loop!
        # we need to perform a search of pedestrians | Update the location of orignal pedestrians | A to be outside the array
        # Del the pedestrians now outside the frame | Add new pedestrians | Check gaze Time etc|
        # location,size,confidence

        gaze_object.angle = lerp(gaze_direction, image)
        cv2.putText(image, str(gaze_object.angle), gaze_direction, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1,
                    cv2.LINE_AA, False)
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
            #eyes.move(1,0,True)
            # eyes.move(2,0,True)
            #eyes2.move(3, 0, True)
            #eyes2.move(4, 0, True)

            break

        net_time = time.time() - time_loopstart
        print("Net Time:" + str(net_time))
        time_gazestart = time.time()
        #if k == 0:
        #gaze(gaze_object)
        time_gazeend = time.time()
        gazetime = time_gazeend - time_gazestart
        # gazetime = int(gazetime * 1000) # ms
        # print("gaze time is:" + str(gazetime))
        time_loopend = time.time()
        looptime = time_loopend - time_loopstart
        # looptime = int(looptime) # ms
        print("loop time is:" + str(looptime))

    eyes.__del__()
    eyes2.__del__()
    # i = 0
    print("end of this program")
    cap.release()
    cv2.destroyAllWindows()
    #exit()



############### now we build a priority system!

if __name__ == '__main__':
    main()
