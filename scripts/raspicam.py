#!/usr/bin/env python3

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

    # Initialize the class related node
    rospy.init_node('turtle_sight', anonymous=True)

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
  def pub_camera_info(self):
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('filename', help='Path to yaml file containing ' +\
                                            'camera calibration data')
    args = arg_parser.parse_args()
    filename = args.filename

    # Parse yaml file
    camera_info_msg = self.yaml_to_CameraInfo(filename)
  
    # while not rospy.is_shutdown():
    self.info_pub.publish(camera_info_msg)
    rospy.loginfo_once('CameraInfo message is being published under /raspicam/camera_info')


  # Publishes acquired images from the RaspiCam
  def pub_raspicam(self):

    # Instantiates OpenCV bridge between ROS' and OpenCV's image format
    bridge = CvBridge()

    # Open the video capture
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

    # Width and height capture size (in pixels)
    cap.set(3, 320)
    cap.set(4, 240)
    
    # Reads the frames capture
    (ret, cv2_frame) = cap.read()
    """ TENTAR TORNAR AS IMAGENS P&B """

    # Converter a img cv2 em img ROS
    ros_frame = bridge.cv2_to_imgmsg(cv2_frame, 'bgr8')

    # Publishes the image
    self.image_pub.publish(ros_frame)
    rospy.loginfo_once('Image message is being published under /raspicam/image')

  def run(self):
    self.pub_raspicam()
    self.pub_camera_info()

# Funcao main
if __name__ == '__main__':
  # Instantiate class
  sight = TurtleVision()

  # Run publisher
  try:
    while not rospy.is_shutdown():
      sight.run()
  except rospy.ROSInterruptException:
    rospy.is_shutdown()
