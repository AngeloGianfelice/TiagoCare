import rospy
from config import *
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image

bridge = CvBridge()

def acquire_image(task_folder):
    print("Waiting for rosbag to be played")
    msg_img = rospy.wait_for_message("/xtion/rgb/image_rect_color", Image)
    img = bridge.imgmsg_to_cv2(msg_img, "bgr8")

    img_path = task_folder+'scan.jpg'
    cv2.imwrite(img_path, img) 

    print("camera image saved")
    return img

def local_acquire_image(path):
    img_path = path + "scan.jpg"
    img = cv2.imread(img_path)

    return img
