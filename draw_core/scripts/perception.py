#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Point


class Perception(object):

    def __init__(self):

        rospy.init_node('perception', anonymous=True)
        self.point_pub = rospy.Publisher("/me495/target_position", Point, queue_size = 10)
        # Send target drawing location to drawing node
        self.target_location = [Point(), Point(), Point(), Point()]
        self.location = rospy.Publisher("/me495/location", Point, queue_size = 10)

        # AR_tag localization
        self.state = [0, 0, 0, 0]
        self.pointlist = [Point(), Point(), Point(), Point()]
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.callback_ar)

        # Communicate with manipulation node
        rospy.Subscriber('/me495/current_position', Point, self.callback_point)


    def callback_ar(self, alvarMarkers):
        self.state = [0, 0, 0, 0]
        self.pointlist = [Point(), Point(), Point(), Point()]
        for i in range(len(alvarMarkers.markers)):
            for j in range(4):
                if alvarMarkers.markers[i].id == j:
                    self.pointlist[j]=alvarMarkers.markers[i].pose.pose.position
                    self.state[j] = 1


    def callback_point(self, point):
        rospy.sleep(0.1)

        # (-1, 0, 0) for perceiving all four tags
        if point.x == -1:
            self.point = [Point(), Point(), Point(), Point()]
            for i in range(4):
                x = 0.0
                y = 0.0
                z = 0.0
                n_detect = 0
                for j in range(10):
                    # print(self.pointlist)
                    if self.state[i] == 1:
                        x = x + self.pointlist[i].x
                        y = y + self.pointlist[i].y
                        z = z + self.pointlist[i].z
                        n_detect = n_detect + 1
                        rospy.sleep(0.1)
                        # print(n_detect)
                # Take average
                if n_detect > 0:
                    self.point[i].x = x / n_detect + 0.1
                    self.point[i].y = y / n_detect + 0.05
                    self.point[i].z = z / n_detect + 0.25
                else:
                    self.point[i].x = 0
                    self.point[i].y = 0
                    self.point[i].z = 0
            # Send the tag position to manipulation
            for i in range(4):
                self.point_pub.publish(self.point[i])

        # (0, i, 0) for perceiving tag i
        if point.x == 0:
            print("Recording tag location")
            x = 0.0
            y = 0.0
            z = 0.0
            n_detect = 0
            for i in range(5):
                # Tag number
                num = int(point.y)
                if self.state[num] == 1:
                    x = x + self.pointlist[num].x
                    y = y + self.pointlist[num].y
                    z = z + self.pointlist[num].z
                    n_detect = n_detect + 1
                    rospy.sleep(0.1)
            if n_detect > 0:
                self.target_location[num].x = x / n_detect
                self.target_location[num].y = y / n_detect
                self.target_location[num].z = z / n_detect
            else:
                self.target_location[num].x = 0
                self.target_location[num].y = 0
                self.target_location[num].z = 0

        # (0, i, 0) for perceiving tag i
        if point.x == 1:
            x = 0.0
            y = 0.0
            z = 0.0
            for i in range(4):
                x = x + self.target_location[i].x
                y = y + self.target_location[i].y
                z = z + self.target_location[i].z
            final_point = Point()
            final_point.x = x / 4.0
            final_point.y = y / 4.0
            final_point.z = z / 4.0
            print("Sending target location", final_point)
            self.location.publish(final_point)



if __name__ == "__main__":
    perception = Perception()
    while not rospy.is_shutdown():
        rospy.spin()
