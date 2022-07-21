#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool
import cv2
import numpy as np
import pyefd
import sys
from cv_bridge import CvBridge, CvBridgeError
import os


class HeatPoolQuality:
    
    def __init__(self):
        self.MAX_VAL = 255
        self.THRESHOLD = 240
        self.TORCH_FLAME_CENTER = (332, 478)
        self.STRENGTH_WEIGHTS = np.array([3,2,1]) #B G R
        self.image_publisher = rospy.Publisher("/camera/color/image_raw", Image, queue_size=1)
        self.poses_publisher = rospy.Publisher("/control/inputs", PoseArray, queue_size=1)
        self.calibrate_start = rospy.Publisher("/image_processing/torch_calibration/start", Bool, queue_size=1)
        self.bridge = CvBridge()
        path = "/home/chinmay/dr_ws/src/realsense_recorder/output/06272022_170520/images_06272022_170520"
        poseArray = PoseArray()

        pose1 = Pose()
        pose1.position.y =1
        pose2 = Pose()
        pose2.position.y =2
        pose3 = Pose()
        pose3.position.y =3
        pose4 = Pose()
        pose4.position.y =4
        pose5 = Pose()
        pose5.position.y =5
        pose6 = Pose()
        pose6.position.y =6
        pose7 = Pose()
        pose7.position.y =7

        poseArray.poses.append(pose1)
        poseArray.poses.append(pose2)
        poseArray.poses.append(pose3)
        poseArray.poses.append(pose4)
        poseArray.poses.append(pose5)
        poseArray.poses.append(pose6)
        poseArray.poses.append(pose7)

        print(poseArray)
        
        rospy.sleep(rospy.Duration(1))
        if os.path.isdir(path):
            images = os.listdir(path)
            images.sort()
            self.poses_publisher.publish(poseArray)
            for image_name in images:
                image_path = os.path.join(path, image_name)
                image = cv2.imread(image_path)
                # print(image)
                if image_name == "000110.jpeg":
                    rospy.loginfo("Start Calibration")
                    self.calibrate_start.publish(True)
                # msg = Image()
                # msg.header.stamp = rospy.Time.now()
                # msg.data = np.array(image).tostring()
                # self.image_publisher.publish(msg)
        

                try:
                    # self.poses_publisher.publish(poseArray)
                    self.image_publisher.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
                    rospy.loginfo("Published {}".format(image_name))
                except CvBridgeError as e:
                    print(e)

                cv2.imshow("grwhw0", image)
                cv2.waitKey(30)
    

    
def main(args):
    
    rospy.init_node("heat_pool_quality_calculator", anonymous=True)
    rospy.loginfo("Starting Node")
    
    try:
        heatPoolQuality = HeatPoolQuality()
        
    except KeyboardInterrupt:
        print("Shutting Down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)