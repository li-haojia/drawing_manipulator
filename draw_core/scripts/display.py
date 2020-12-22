#!/usr/bin/env python

# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import intera_interface
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from intera_core_msgs.msg import IODeviceStatus
import cv_bridge
import rospkg

class Display(object):

    def __init__(self):
        rospy.init_node("head_display", anonymous=True)
        self.head_display = intera_interface.HeadDisplay()

        # Load in camera image to show in display
        rospy.Subscriber('/me495/raw_image', Image, self.img_callback)
        # listen to button in Sawyer
        rospy.Subscriber('io/robot/navigator/state', IODeviceStatus, self.fun_callback)
        # Subscribe to control node
        self.state_camera = False
        rospy.Subscriber("/me495/command", String, self.cmd_callback)
        # Subscribe to camera data
        rospy.Subscriber('/camera_driver/image_raw', Image, self.camera_callback)
        # Show in display
        self.disp_pub = rospy.Publisher('robot/head_display', Image, latch=True, queue_size=10)

        # Load in fun images
        self.fun = False
        rospack = rospkg.RosPack()
        image_path = rospack.get_path('me495_vision') + '/images/'
        self.start_img = cv_bridge.CvBridge().cv2_to_imgmsg(cv2.imread(image_path + 'start.jpg'), encoding="bgr8")
        self.fun_img_1 = cv_bridge.CvBridge().cv2_to_imgmsg(cv2.imread(image_path + 'normal.jpg'), encoding="bgr8")
        self.fun_img_2 = cv_bridge.CvBridge().cv2_to_imgmsg(cv2.imread(image_path + 'blink.jpg'), encoding="bgr8")
        r = rospy.Rate(5)

        # Show in display
        self.useful_image = self.start_img
        self.fun_change = False
        #self.fun_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.fun:
                if self.fun_change == True:
                    self.disp_pub.publish(self.fun_img_2)
                    #if (rospy.Time.now() - self.fun_time).to_sec() > 0.2:
                    self.fun_change = False
                else:
                    self.disp_pub.publish(self.fun_img_1)
            else:
                self.disp_pub.publish(self.useful_image)
            r.sleep()

    def cmd_callback(self, cmd):
        if cmd.data == 'Show Camera':
            self.state_camera = True


    def img_callback(self, img):
        self.state_camera = False
        self.useful_image = img
        print("Printing image to the display")

    def camera_callback(self, img):
        if self.state_camera == True:
            self.useful_image = img

    def fun_callback(self, msg):
        all_signals = msg.signals
        for item in all_signals:
            if item.name == 'head_button_triangle':
                if item.data == '[2]':
                    self.fun = not self.fun
                elif item.data == '[1]':
                    self.fun_change = True

def main():
    displayer = Display()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
