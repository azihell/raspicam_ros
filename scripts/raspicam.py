#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#To run: ./raspicam.py /home/ubuntu/catkin_ws/src/raspicam_ros/config/raspicam.yaml

import rospy, cv2, time, yaml, argparse
import numpy as np
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

"""
  This class publishes 2 basic topics for the vision capacity of a TurtleBot3
  1) a publisher for the Raspicam ROS Image topic;
  2) a 'camera_info topic manual publisher', for the Raspicam ROS CameraInfo topic (it's necessary to provide a (absolute) path to the .yaml file as part of the '$ rosrun [...]' statement used to run this script!)
"""

class TurtleVision:

  def __init__(self):

    # Instantiates a publishing object and sets the published topic name
    self.image_pub = rospy.Publisher('raspicam/image', Image, queue_size=10)
    self.info_pub = rospy.Publisher('raspicam/camera_info', CameraInfo, queue_size=10)
 
    # Instantiates OpenCV bridge between ROS' and OpenCV's image format
    self.bridge = CvBridge()

  # Obtains the camera_info parameters to be published from a .yaml file
  def yaml_to_CameraInfo(self,yaml_fname):
    """
    Parameters
    ----------
    yaml_fname : str
        Path to .yaml file containing camera calibration parameters

    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration parameters
        data
    """
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
      calib_data = yaml.load(file_handle, Loader=yaml.FullLoader)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

  # Publishes a camera_info topic based on a loaded .yaml file.
  def pub_camera_info(self, event=None):
    # arg_parser = argparse.ArgumentParser()
    # arg_parser.add_argument('filename', help='Path to yaml file containing ' +\
    #                                         'camera calibration data')
    # args = arg_parser.parse_args()
    # filename = args.filename
    filename = "/home/ubuntu/catkin_ws/src/raspicam_ros/config/raspicam.yaml"

    # Parse yaml file
    camera_info_msg = self.yaml_to_CameraInfo(filename)
  
    # while not rospy.is_shutdown():
    self.info_pub.publish(camera_info_msg)
    rospy.loginfo_once('CameraInfo message is being published under /raspicam/camera_info')

  # Publishes acquired images from the RaspiCam
  def pub_raspicam(self, Event=None):
    
    # Open the video capture
    vidcap = cv2.VideoCapture(0, cv2.CAP_V4L2)

    # Width and height capture size (in pixels)
    vidcap.set(3, 320)
    vidcap.set(4, 240)
    # fps = vidcap.get(5)
    vidrate=rospy.Rate(20)

    # Reads the frames capture
    while (vidcap.isOpened()):
      rospy.loginfo_once('Image message is being published under /raspicam/image')

      (ret, cv2_frame) = vidcap.read()
      cv2_frame = cv2.cvtColor(cv2_frame, cv2.COLOR_BGR2GRAY)

      # Converts the OpenCV image into a ROS image
      ros_frame = self.bridge.cv2_to_imgmsg(cv2_frame, 'mono8')

      # Publishes the image
      self.image_pub.publish(ros_frame)
      vidrate.sleep()

# Main function
if __name__ == '__main__':

  # Initialize the class related node
  rospy.init_node('turtle_sight')

  # Instantiate class
  sight = TurtleVision()

  # Sets different rates for each publisher from the node
  rospy.Timer(rospy.Duration(1.0/20.0), sight.pub_camera_info)
  rospy.Timer(rospy.Duration(1.0/20.0), sight.pub_raspicam)
  
  # Keep node running
  rospy.spin()
