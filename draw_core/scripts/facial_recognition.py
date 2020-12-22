#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int8MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospkg

# Path planning
from edgesToPath import *


class FacialRecognition(object):

    def __init__(self):
        rospy.init_node("facial_recognition", anonymous=True)
        # Transfer image between ROS and Opencv
        self.bridge = CvBridge()

        # Wait for taking and processing a photo
        rospy.Subscriber("/me495/command", String, self.callback)

        # Publish the raw image to Sawyer display
        self.image_pub = rospy.Publisher("/me495/raw_image", Image, queue_size = 1)

        # Publish the drawing trajectory
        self.trajectory_pub = rospy.Publisher("/me495/trajectory", Int8MultiArray, queue_size = 1)

        # Fun image: Jarvis
        rospack = rospkg.RosPack()
        image_path = rospack.get_path("me495_vision") + "/images/beauty.jpg"
        self.jarvis = cv2.imread(image_path)


    def callback(self, string):
        # Take a photo when
        if string.data == "Take Photo":
            print("Taking Photo")

            self.state_face = False
            while self.state_face == False:
                # Read in the image and transfer through CVBridge
                image_in = rospy.wait_for_message('/camera_driver/image_raw', Image)
                try:
                    image = self.bridge.imgmsg_to_cv2(image_in, "bgr8")
                except CvBridgeError as e:
                    print(e)
                # Detect face and crop the image
                self.face = self.face_crop(image, pad_w=20, pad_h=50)

                try:
                    face_ros = self.bridge.cv2_to_imgmsg(self.face, encoding="passthrough")
                    self.image_pub.publish(face_ros)
                    print("Sending face to display")
                except:
                    print("CV Bridge fails")
                    self.state_face = False


        if string.data == "Start Processing":
            # self.photo_retake()
            self.image_process()


    def image_process(self):
        print("Processing Image")

        # Resize the photo and apply Canny Edge Detector
        # Generate edgemap and send to display
        edges = self.resize_edge(self.face, 80)
        # Save locally
        cv2.imwrite("/home/ethan/Pictures/face.jpg", self.face)
        cv2.imwrite("/home/ethan/Pictures/edge.jpg", edges)

        # Resize the edgemap image
        edges_resize = self.resize(edges, 600)
        edges_rgb = cv2.cvtColor(edges_resize, cv2.COLOR_GRAY2RGB)
        # Add Jarvis as background
        self.jarvis[0:edges_rgb.shape[0], 0:edges_rgb.shape[1]] = edges_rgb
        edges_ros = self.bridge.cv2_to_imgmsg(self.jarvis, encoding="passthrough")
        self.image_pub.publish(edges_ros)
        print("Sending edges to display")

        # Apply DFS to transfer binary image to trajectory points
        pathlist = getPointsFromEdges(edges)
        # (-1,-1) to lift up the end effector
        pathlist.append((-1,-1))
        print("Path length: ", len(pathlist))

        # Publish to drawing node
        path = Int8MultiArray()
        pathlist = np.array(pathlist).reshape(2*len(pathlist))
        path.data = pathlist
        self.trajectory_pub.publish(path)
        print("Publishing Trajectory")


    def face_crop(self, img, pad_w, pad_h):
        # pad_w, pad_h: padding margin of the crop image

        # load face training data
        rospack = rospkg.RosPack()
        xml_path = rospack.get_path("me495_vision") + "/facial_detection/haarcascade_frontalface_default.xml"
        face_cascade = cv2.CascadeClassifier(xml_path)

        # convert the baseImage to a gray-based image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # detect face
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)
        if len(faces) == 0:
            print("No faces detected!")
            print("Ready to use this photo? [y/n]")
            return

        # create rectangle for face
        for f in faces:
            # define vertices of face rectangle
            x,y,w,h = [v for v in f]
            cv2.rectangle(img,(x-pad_w,y-pad_h),(x+w+pad_w,y+h+pad_h),(255,0,0),2) # blue color
            # create cropped faces
            face_image = img[y-pad_h:y+h+pad_h,x-pad_w:x+w+pad_w]

        self.state_face = True
        return face_image


    def resize_edge(self, img, size):
        h, w = img.shape[:2]
        frame_size = size
        if h > w:
            h_new = frame_size
            w_new = (frame_size * w) / h
        else:
            w_new = frame_size
            h_new = (frame_size * h) / w
        resizes = cv2.resize(img, (w_new,h_new), cv2.INTER_AREA)

        # Canny edge
        edges = cv2.Canny(resizes, 80, 120)

        edges = np.array(edges)
        return edges


    def resize(self, img, size):
        h, w = img.shape[:2]
        frame_size = size
        if h > w:
            h_new = frame_size
            w_new = (frame_size * w) / h
        else:
            w_new = frame_size
            h_new = (frame_size * h) / w
        resizes = cv2.resize(img, (w_new,h_new), cv2.INTER_AREA)

        resizes = np.array(resizes)
        return resizes



if __name__ == "__main__":
    fr = FacialRecognition()
    while not rospy.is_shutdown():
        rospy.spin()
