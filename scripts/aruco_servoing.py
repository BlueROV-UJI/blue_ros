#Implementing visual servoing using aruco markers
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#Sources:
#https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/

class ArucoServoing():
    def __init__(self):
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters_create()
        (self.corners, self.ids, self.rejectedImgPoints) = aruco.detectMarkers(image, self.aruco_dict, parameters=self.aruco_params)
        
         # Create subscribers
        rospy.Subscriber("/camera0/image_raw", Image, self.image_callback)

    def detect_marker(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters_create()
        print("parameters = {}".format(parameters))
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        print("corners = {}".format(corners))
        print("ids = {}".format(ids))
        print("rejectedImgPoints = {}".format(rejectedImgPoints))
        frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
        return frame_markers

    def image_callback(self, image_msg):
        # print(image_msg)
        #to convert the ROS Image message to a CV2 Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "passthrough")
            cv2.imshow("Image", cv_image)
            cv2.waitKey(3)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
  
        
        
        self.image_height = cv_image.height
        self.image_width = cv_image.width

        self.point_coordinate = (self.image_width/2, self.image_height/2)
        self.aruco_coordinates = self.detect_marker(image_msg)


    def visual_servoing(self):
        # Calculate the center of the image
        # center = (self.width/2, self.height/2)
        # Calculate the error
        error = (self.point_coordinate[0] - self.point_coordinate[0], self.point_coordinate[1] - self.point_coordinate[1])
        # Calculate the proportional gain
        proportional_gain = 0.1
        # Calculate the proportional error
        proportional_error = (error[0]*proportional_gain, error[1]*proportional_gain)
        # Calculate the velocity
        velocity = (proportional_error[0], proportional_error[1])
        return velocity


if __name__ == "__main__":
    rospy.init_node("aruco_servoing")
    aruco_servoing = ArucoServoing()
    rospy.spin()

