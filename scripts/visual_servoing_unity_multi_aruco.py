#!/usr/bin/env python

# Reference: https://hal.inria.fr/hal-01355384/document

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import scipy
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError
from camera_parameters import CameraParameters

import utils as ut
import math
from transform import Transform

class ArucoServoing:
    def __init__(self):
        self.cam = CameraParameters()
        self.transform = Transform()
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters_create()
        self.bridge = CvBridge()
        self.kp = 0.0001
        self.error_x_prev = 0
        self.error_y_prev = 0
        self.compressed_flag = True

        #Camera parameters
        # 813.124, 830.767, 585.5, 265
        self.camera_matrix = np.array([[813.124, 0, 320],[0,830.767, 224],[0,0,1]])
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.lambda_vs = 0.04 #Kp for visual servoing
        self.first_goal = True
        self.end_goal_reached = False

        rospy.Subscriber("/camera0/image_raw/compressed", CompressedImage, self.image_callback)
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #74
        middle_left_goal = np.array([[[122., 207.],
        [169., 207.],
        [169., 253.],
        [122., 253.]]])
        #75
        middle_right_goal = np.array([[[481., 207.],
        [527., 207.],
        [527., 253.],
        [481., 253.]]])
        #76
        upper_left_goal = np.array([[[480.,  20.],
        [527.,  20.],
        [527.,  67.],
        [481.,  67.]]])
        #77
        upper_right_goal = np.array([[[122.,  20.],
        [169.,  20.],
        [169.,  67.],
        [122.,  67.]]])
        #100
        center_goal = np.array([[[248., 202.],
        [310., 202.],
        [310., 264.],
        [248., 264.]]])
        self.goals_dict = {'74' : middle_left_goal,
                           '75' : middle_right_goal,
                           '76' : upper_left_goal,
                           '77' : upper_right_goal,
                           '100': center_goal}
        self.current_goal = None

    def detect_marker(self, image):
        # detected_markers_dict = {k:None for k in self.goals_dict.keys()}
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        parameters = aruco.DetectorParameters_create()
        all_arucos_corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=parameters)
        print("ids = {}".format(ids))
        print("all_arucos_corners = {}".format(all_arucos_corners))
        frame_markers = aruco.drawDetectedMarkers(image, all_arucos_corners, ids)
        aruco_key_found = None
        detected_markers_dict = {}
        if len(all_arucos_corners) != 0:
            for id, one_aruco_corners in zip(ids, all_arucos_corners):
                if len(one_aruco_corners) == 1:
                    detected_markers_dict[str(id[0])] = one_aruco_corners
                    aruco_key_found = str(id[0])
                    # break
        if aruco_key_found is None:
            return None, None, None, None, None
        single_aruco_corners = detected_markers_dict[aruco_key_found]
        #Selecting the goal to track
        self.current_goal = self.goals_dict[aruco_key_found]
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(single_aruco_corners, 0.1, self.camera_matrix, self.dist_coeffs)
        _ = cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
        rvec = np.array(rvec).flatten()
        self.quat = self.transform.rodrigues_to_quaternion(rvec)
        self.transformed_matrix = self.transform.transform_position(tvec, self.quat, as_quat=True)

        cv2.imshow("image", image)
        cv2.waitKey(3)
        cv2.imshow("frame_markers", frame_markers)
        cv2.waitKey(3)

        aruco_centers = []
        for corner in [single_aruco_corners]:
            mid = [0, 0]
            #Computing the middle point of a square using two opposite corner's points
            for point in corner:
                mid[0] = (point[0][0] + point[2][0]) / 2
                mid[1] = (point[0][1] + point[2][1]) / 2
            # print("mid = {}".format(mid))
            aruco_centers.append(mid)
            cv2.circle(frame_markers, (int(mid[0]), int(mid[1])), 5, (0, 0, 255), -1) #-1 means filled circle
        cv2.imshow("frame_markers", frame_markers)
        cv2.waitKey(3)
        return frame_markers, aruco_centers, single_aruco_corners, aruco_key_found, detected_markers_dict

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
        cv2.imshow("image", cv_image)
        cv2.waitKey(3)
        frame_markers, aruco_centers, aruco_corners, aruco_key_found, detected_markers_dict = self.detect_marker(cv_image)
        print("aruco_key_found={}".format(aruco_key_found))
        # print("aruco_corners = {}".format(aruco_corners))
        # print("aruco_centers = {}".format(aruco_centers))
        #Check if aruco is detected:
        print("self.first_goal = {}".format(self.first_goal))
        print("self.end_goal_reached = {}".format(self.end_goal_reached))
        if aruco_corners is None:
            self.cmd_vel_publisher.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
            return
        else:
            # if (len(detected_markers_dict) > 1) and (self.first_goal):
            search_for_goal = False
            #Pursuing the first goal:
            if self.end_goal_reached == True:
                for k,v in detected_markers_dict.items():
                    if k!="100":
                        self.first_goal = True
                        self.lambda_vs = 0.04
                        self.end_goal_reached = False
            if self.first_goal == True:
                for k,v in detected_markers_dict.items():
                    if k!="100":
                        goal_points = self.goals_dict[k].flatten()
                        current_points = np.array(v).flatten()
                        search_for_goal = True
                if search_for_goal == False:
                    vel_x, vel_y, vel_z = 0, 0, 0
                    self.cmd_vel_publisher.publish(Twist(Vector3(float(vel_x), float(-vel_z), float(-vel_y)), Vector3(0, 0, 0)))
                    return #Safety
            #Pursuing the ultimate goal:
            else:
                if "100" not in detected_markers_dict.keys():
                    vel_x, vel_y, vel_z = 0, 0, 0
                    self.cmd_vel_publisher.publish(Twist(Vector3(float(vel_x), float(-vel_z), float(-vel_y)), Vector3(0, 0, 0)))
                    return #Safety
                goal_points = self.goals_dict["100"].flatten()
                current_points = detected_markers_dict["100"].flatten()

            print("current_points = {}".format(current_points))
            print("current_points_shape = {}".format(current_points.shape))
            print("goal_points = {}".format(goal_points))
            print("goal_points_shape = {}".format(goal_points.shape))
            # exit()
            
            velocity = self.velocity_controller(current_points, goal_points)
            vel_x, vel_y, vel_z, vel_roll, vel_pitch, vel_yaw = velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5]
            # vel_x, vel_y, vel_z = 0, 0, 0
            #To avoid oscillations and weird behaviors:
            if (((np.abs(self.error_vs[0]) < 0.03) and (np.abs(self.error_vs[1]) < 0.03) and (np.abs(self.error_vs[2]) < 0.03)) and not (self.first_goal)):
                self.end_goal_reached = True
            if (((np.abs(self.error_vs[0]) > 0.03) or (np.abs(self.error_vs[1]) > 0.03) or (np.abs(self.error_vs[2]) > 0.03)) and self.first_goal):
                # vel_x = 0
                pass
            else:
                #Now first goal is reached and aruco marker = 100 will be pursued
                self.first_goal = False
                self.lambda_vs = 0.03
                
                # vel_x = 0
            # if (((np.abs(self.error_vs[0]) < 0.03) and (np.abs(self.error_vs[1]) < 0.03) and (np.abs(self.error_vs[2]) < 0.03)) and not (self.first_goal)):
            #     self.end_goal_reached = True
            
            
            self.cmd_vel_publisher.publish(Twist(Vector3(float(vel_x), float(-vel_z), float(-vel_y)), Vector3(0, 0, 0)))


    def velocity_controller(self, current_points, desired_points_vs):
        current_points_meter = self.cam.convertListPoint2meter(current_points)
        desired_points_meter = self.cam.convertListPoint2meter(desired_points_vs)

        #compute vs error
        self.error_vs = np.zeros((1, 8))
        self.error_vs = current_points_meter - desired_points_meter
        print("error_vs = {}".format(self.error_vs))

        #compute interaction matrix in the FILE ./visual_servoig.py
        L = ut.interactionMatrixFeaturePoint2DList(current_points_meter)

        #init the camera velocity
        vcam = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        #compute the velocity control law
        vcam_vs = -self.lambda_vs * np.linalg.pinv(L).dot(self.error_vs)
        vrobot = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        

        ## TODO find the control velocity expressed in the robot frame
        cam_to_rob_t = np.array([0, 0, 0])
        #Unity rotation matrix from camera to robot
        cam_to_rob_r = np.array([[0, 0, -1],
                                    [0, -1, 0],
                                    [-1, 0, 0]])
        twist_matrix = self.transform.velocityTwistMatrix(cam_to_rob_t[0], cam_to_rob_t[1], cam_to_rob_t[2],
                                            aRb=cam_to_rob_r)
        vrobot = twist_matrix.dot(vcam_vs)
        print("vrobot = {}".format(vrobot))
        return vrobot



if __name__ == "__main__":
    rospy.init_node("aruco_servoing")
    aruco_servoing = ArucoServoing()
    rospy.spin()

