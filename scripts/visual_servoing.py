#!/usr/bin/env python

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError

class ArucoServoing:
    def __init__(self):
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters_create()
        self.bridge = CvBridge()
        self.kp = 0.1
        self.compressed_flag = True
        rospy.Subscriber("/blue_rov1/CompressedImage", CompressedImage, self.image_callback)
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def detect_marker(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=parameters)
        corners_np = np.array(corners)
        print("corners = {}".format(corners_np))
        print("corners.shape = {}".format(corners_np.shape))
        frame_markers = aruco.drawDetectedMarkers(image, corners, ids)
        cv2.imshow("image", image)
        cv2.waitKey(3)
        cv2.imshow("frame_markers", frame_markers)
        cv2.waitKey(3)


        for corner in corners:
            mid = [0, 0]
            print("corner = {}".format(corner))
            #Computing the middle point of a square using two opposite corner's points
            for point in corner:
                mid[0] = (point[0][0] + point[2][0]) / 2
                mid[1] = (point[0][1] + point[2][1]) / 2
            print("mid = {}".format(mid))
            cv2.circle(frame_markers, (int(mid[0]), int(mid[1])), 5, (0, 0, 255), -1) #-1 means filled circle
        cv2.imshow("frame_markers", frame_markers)
        cv2.waitKey(3)
        return frame_markers

    def image_callback(self, image_msg):
        if self.compressed_flag:
            try:
                np_arr = np.fromstring(image_msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))
        else:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(image_msg, "passthrough")
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))
        
        self.image_height = cv_image.shape[0]
        self.image_width = cv_image.shape[1]

        self.point_coordinates = (self.image_width/2, self.image_height/2)
        self.aruco_coordinates = self.detect_marker(cv_image)
        print('a7a')
        # error_x = self.point_coordinates[0] - self.aruco_coordinates[0]
        # error_y = self.point_coordinates[1] - self.aruco_coordinates[1]
        # vel_x = self.kp * error_x
        # vel_y = self.kp * error_y
        # print("vel_x = {}".format(vel_x.shape))
        # print("vel_y = {}".format(vel_y))

        # self.cmd_vel_publisher.publish(Twist(Vector3(float(vel_x[0]), 0, 0), Vector3(0, 0, float(vel_y[0]))))

        #########################################
        # print("vel_x = {}".format(vel_x))
        # print("vel_y = {}".format(vel_y))


if __name__ == "__main__":
    rospy.init_node("aruco_servoing")
    aruco_servoing = ArucoServoing()
    rospy.spin()

