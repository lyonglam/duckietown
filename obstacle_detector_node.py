#!/usr/bin/env python3
# obstacle_detector_node.py

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
import cv_bridge

class ObstacleDetector:
    def __init__(self):
        rospy.init_node('obstacle_detector_node', anonymous=True) # registers your script with the ROS master network
        
        self.bridge = cv_bridge.CvBridge() # converts ROS images to OpenCV format
        
        # 1. Ask Person 1 what they want to name this topic! 
        # We will publish True to stop, False to go.
        self.stop_pub = rospy.Publisher('/duck4/safety/emergency_stop', Bool, queue_size=1) # creates your outgoing radio channel, publishes a boolean message to the /duck4/safety/emergency_stop topic
        
        # 2. Subscribe to the live camera feed
        self.image_sub = rospy.Subscriber('/duck4/camera_node/image/compressed', CompressedImage, self.image_callback, queue_size=1) # creates your incoming radio channel, subscribes to the /duck4/camera_node/image/compressed topic
        
        print("Obstacle Detector Node Started. Watching for duckies...") # prints a message to the console

    def image_callback(self, msg):
        """
        Every time the camera takes a picture, this function runs automatically.
        """
        # Convert the compressed ROS image into an OpenCV format
        try:
            np_arr = np.frombuffer(msg.data, np.uint8) # converts the ROS image data into a numpy array
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # decodes the numpy array into an OpenCV image
        except Exception as e:
            print(f"Error decoding image: {e}")
            return

        # --- THIS IS WHERE YOUR OPENCV MAGIC WILL GO ---
        # 1. Convert to HSV color space
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) # converts the OpenCV image from the BGR color space to the HSV color space
        
        # 2. Threshold for yellow duckies (We will tune these numbers later)
        lower_yellow = np.array([20, 100, 100]) # defines the lower bound of the yellow color in the HSV color space
        upper_yellow = np.array([30, 255, 255]) # defines the upper bound of the yellow color in the HSV color space
        mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow) # creates a mask of the yellow color in the HSV color space
        
        # 3. Find how big the yellow object is
        # If it's too big, it means it is very close!
        yellow_pixels = cv2.countNonZero(mask) # counts the number of non-zero pixels in the mask
        
        stop_signal = Bool()
        stop_threshold = 5000  # Example: if more than 5000 pixels are yellow, STOP!
        
        if yellow_pixels > stop_threshold: # if the number of yellow pixels is greater than the stop threshold
            print("OBSTACLE DETECTED! STOP!") # prints a message to the console
            stop_signal.data = True # sets the stop signal to True
        else:
            stop_signal.data = False
            
        # Broadcast the decision to Person 1's code
        self.stop_pub.publish(stop_signal)
        
        # Optional: Show what the robot sees (useful for debugging in your VM)
        # cv2.imshow("Duckie Vision Mask", mask)
        # cv2.waitKey(1)

if __name__ == '__main__':
    try:
        detector = ObstacleDetector()
        rospy.spin() # Keeps the script running and listening
    except rospy.ROSInterruptException:
        pass