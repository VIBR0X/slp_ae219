#!/usr/bin/env python

import rospy
from iq_gnc.py_gnc_functions import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
drone = gnc_api()

# Initialize OpenCV bridge
bridge = CvBridge()


def create_data_folder():
  """Creates the 'data' folder if it doesn't exist."""
  data_dir = "data"
  if not os.path.exists(data_dir):
    os.makedirs(data_dir)

# Callback function to process incoming images
def image_callback(msg):
    # Convert ROS image to OpenCV image
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Display the image
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    # Save the image
    create_data_folder()
    timestamp = rospy.get_time()
    filename = os.path.join("data", f"iris_camera_{timestamp}.jpg")
    cv2.imwrite(filename, cv_image)


def main():
    rospy.init_node('ortho_mosaicing_node', anonymous=True)

    rate = rospy.Rate(3)

    # Wait for FCU connection.
    drone.wait4connect()
    # Wait for the mode to be switched.
    drone.wait4start()
    # Create local reference frame.
    drone.initialize_local_frame()
    # Request takeoff with an altitude of 2m.
    drone.takeoff(10)

    # Subscribe to the image topic using the appropriate topic name
    rospy.Subscriber("/webcam/image_raw", Image, image_callback)

    drone.set_destination(x=5, y=0, z=10, psi=-90)
    rate.sleep()
    if drone.check_waypoint_reached():
        drone.land()

    # Main ROS loop
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
