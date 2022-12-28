#!/usr/bin/env python

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import scipy
import tf
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError

class ArucoServoing:
    def __init__(self):
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters_create()
        self.bridge = CvBridge()
        self.kp = 0.0001
        self.error_x_prev = 0
        self.error_y_prev = 0
        self.compressed_flag = True

        #Camera parameters
        self.camera_matrix = np.array([],[],[])
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])



        rospy.Subscriber("/blue_rov1/CompressedImage", CompressedImage, self.image_callback)
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # self.goal_points = np.array([[509.5, 257.5], [107.5, 257.0]])
        self.goal_points = np.array([[509.5, 257.5]])

    def detect_marker(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=parameters)
        corners_np = np.array(corners)
        print("corners = {}".format(corners_np))
        print("corners.shape = {}".format(corners_np.shape))
        frame_markers = aruco.drawDetectedMarkers(image, corners, ids)
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, self.camera_matrix, self.dist_coeffs)
        transform = cv2.aruco.drawAxis(image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
        self.transformed_position = self.transform_position(tvec, transform)
        self.quaternion = self.cv2.rodrigues(rvec)

        cv2.imshow("image", image)
        cv2.waitKey(3)
        cv2.imshow("frame_markers", frame_markers)
        cv2.waitKey(3)

        aruco_centers = []
        for corner in corners:
            mid = [0, 0]
            print("corner = {}".format(corner))
            #Computing the middle point of a square using two opposite corner's points
            for point in corner:
                mid[0] = (point[0][0] + point[2][0]) / 2
                mid[1] = (point[0][1] + point[2][1]) / 2
            print("mid = {}".format(mid))
            aruco_centers.append(mid)
            cv2.circle(frame_markers, (int(mid[0]), int(mid[1])), 5, (0, 0, 255), -1) #-1 means filled circle
        cv2.imshow("frame_markers", frame_markers)
        cv2.waitKey(3)
        return frame_markers, aruco_centers

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
        frame_markers, aruco_centers = self.detect_marker(cv_image)
        #Check if aruco is detected:
        if len(aruco_centers) == 0:
            self.cmd_vel_publisher.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
            return
        else:

            error_x = self.goal_points[0][0] - aruco_centers[0][0]
            error_y = self.goal_points[0][1] - aruco_centers[0][1]
            print("error_x = {}".format(error_x))
            print("error_y = {}".format(error_y))

            vel_x = self.kp * (error_x + self.error_x_prev)
            vel_y = self.kp * (error_y + self.error_y_prev)
            self.error_x_prev = vel_x
            self.error_y_prev = vel_y
            # vel_y = 0
            # vel_z = self.kp * error_y
            vel_z = 0
            print("vel_x = {}".format(vel_x))
            print("vel_y = {}".format(vel_y))
            print("vel_z = {}".format(vel_z))

            self.cmd_vel_publisher.publish(Twist(Vector3(float(-vel_z), float(vel_x), float(vel_y)), Vector3(0, 0, 0)))




    def transform_position(self, position, transform):
    # Create a quaternion from the rotation part of the transform matrix
        quat = tf.transformations.quaternion_from_matrix(transform)
        
        # Create a transform object from the position and quaternion
        transform = tf.Transform(translation=position, rotation=quat)
        
        # Use the transform object to transform the position to the new frame
        transformed_position = transform * position
        
        return transformed_position
    # def compute_kp(self, error):
    #     #System Dynamics
    #     A = np.array([[0, 1], [0, 0]])
    #     B = np.array([[0], [1]])
    #     C = np.array([[1, 0]])
    #     D = np.array([[0]])
    #     #LQR
    #     Q = np.array([[1, 0], [0, 1]])
    #     R = np.array([[1]])
    #     K, _, _ = scipy.linalg.lqr(A, B, Q, R)
    #     print("K = {}".format(K))

    #     return K

if __name__ == "__main__":
    rospy.init_node("aruco_servoing")
    aruco_servoing = ArucoServoing()
    rospy.spin()

