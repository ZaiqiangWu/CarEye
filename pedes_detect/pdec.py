
import os
import numpy as np
import cv2
# These are NMS and Confidence Thresholds used in pedestrian_detection

NMS_THRESHOLD = 0.3
MIN_CONFIDENCE = 0.7

class pdetector:
    def __init__(self):
        model, layer_name=make_pdect_model()
        self.model=model
        self.layer_name =layer_name
        labelsPath = "coco.names"
        self.LABELS = open(labelsPath).read().strip().split("\n")

    def __call__(self, image):
        return pedestrian_detection(image,self.model,self.layer_name, personidz=self.LABELS.index("person"))

def make_pdect_model():
    # Setting up Yolov4-tiny model for pedestrian Detection

    if not os.path.exists("./yolov4-tiny.weights"):
        import gdown
        id = "1-JOrNS3i1MrRvJZ19siG0nGvoSgtfZRt"
        output = "yolov4-tiny.weights"
        gdown.download(id=id, output=output, quiet=False)
    weights_path = "yolov4-tiny.weights"
    config_path = "yolov4-tiny.cfg"
    model = cv2.dnn.readNetFromDarknet(config_path, weights_path)
    # use CUDA (support now)
    model.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    model.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
    layer_name = model.getLayerNames()
    layer_name = [layer_name[i - 1] for i in model.getUnconnectedOutLayers()]
    return model, layer_name

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