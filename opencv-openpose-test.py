import cv2
import time
import numpy as np
from random import randint
import argparse

parser = argparse.ArgumentParser(description='Run keypoint detection')
parser.add_argument("--device", default="cpu", help="Device to inference on")
parser.add_argument("--image_file", default="group.jpg", help="Input image")

args = parser.parse_args()

image1 = cv2.imread(args.image_file)


# load pre-trained network
protoFile = "openpose/openpose/models/pose/coco/pose_deploy_linevec.prototxt"
weightsFile = "openpose/openpose/models/pose/coco/pose_iter_440000.caffemodel"
nPoints = 18