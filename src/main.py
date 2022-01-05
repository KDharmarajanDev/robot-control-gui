#!/usr/bin/env python3
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from gui import GUI
import rospy

def onReceiveImage(msg):
    global control_GUI, bridge
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    control_GUI.update_background_image(cv_image)

if __name__ == "__main__":
    rospy.init_node('control_gui')
    control_GUI = GUI()
    bridge = CvBridge()
    rospy.Subscriber("/camera/image_raw", Image, onReceiveImage) 
    control_GUI.root.mainloop()   