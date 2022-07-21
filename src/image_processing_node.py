#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32, Bool
import cv2
import numpy as np
import pyefd
import sys
from cv_bridge import CvBridge, CvBridgeError


class HeatPoolQuality:
    
    def __init__(self):
        self.MAX_VAL = 255
        self.THRESHOLD = 240
        self.TORCH_FLAME_CENTER = (332, 478)
        self.STRENGTH_WEIGHTS = np.array([0.4, 0.2, 0.1]) #B G R

        self.bridge = CvBridge()

        self.image_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.processImage, queue_size = 1)
        self.calibration_subscriber = rospy.Subscriber("/image_processing/torch_calibration/start", Bool, self.calibrateCallback, queue_size = 1)
        
        self.heat_pool_strength_publisher = rospy.Publisher("/image_processing/ellipse/strength", Float32, queue_size=1)
        self.heat_pool_eccentricity_publisher = rospy.Publisher("/image_processing/ellipse/eccentricity", Float32, queue_size=1)
        self.calibration_status_publisher = rospy.Publisher("/image_processing/torch_calibration/status", Bool, queue_size=1)
        self.calibration_strength_publisher = rospy.Publisher("/image_processing/torch_calibration/strength", Float32, queue_size=1)
        
        dtype = [('strength',int), ('eccentricity',float)]
        self.ellipse_buffer_array = np.zeros(5, dtype=dtype)
        self.strength_index = 0
        self.calibrate_torch_flag = 0
        self.calibration_buffer = []
        self.calibration_strength_buffer = []
        self.calibration_strength = -1
        self.is_calibrated = False

    def processImage(self, msg):
        rospy.loginfo("Image Received")

        # Convert Image message to Opencv, numpy array
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Blur Image
        blur = cv2.GaussianBlur(image, (7, 7), 5)

        # Color Threshold the Image
        thresholded_image = self.colorThreshold(blur)

        # Fit an Ellipse
        ellipse_coeffs, ellipse_center = self.fitEllipse(thresholded_image)

        # Call the calibrate function to calibrate the torch center if the calibrate flag is set
        # If ellipse is present
        if ellipse_coeffs is not -1:
            # Get Heat Pool Quality
            ellipse_strength, ellipse_shape = self.getHeatPoolQuality(thresholded_image, ellipse_coeffs, ellipse_center)
        else:
            # If ellipse was not fit
            ellipse_strength = -1
            ellipse_shape = 1

        # Get the median value of the last 5 frames (attempt to reduce noise, not tested 07/16/2022)
        ellipse_strength, ellipse_shape = self.get_strength_median(ellipse_strength=ellipse_strength, ellipse_eccentricity=ellipse_shape)
        
        if(self.calibrate_torch_flag):
            self.calibrateTorchPosition(ellipse_center, ellipse_strength)

        # Publish the strength and eccentricity values
        self.heat_pool_strength_publisher.publish(ellipse_strength)
        self.heat_pool_eccentricity_publisher.publish(ellipse_shape)

        #If Calibration is completed publish the calibration strength and status
        if self.is_calibrated:
            self.calibration_status_publisher.publish(True)
            self.calibration_strength_publisher.publish(self.calibration_strength)

        # print(ellipse_strength, ellipse_shape)
        # cv2.waitKey(30)




    def colorThreshold(self, image):
        
        # Split into RGB Channels
        blue_channel, green_channel, red_channel = cv2.split(image)

        # Threshold Indiviual Channels
        _, blue_binary = cv2.threshold(blue_channel, self.THRESHOLD, self.MAX_VAL, cv2.THRESH_BINARY)
        _, green_binary = cv2.threshold(green_channel, self.THRESHOLD, self.MAX_VAL, cv2.THRESH_BINARY)
        _, red_binary = cv2.threshold(red_channel, self.THRESHOLD, self.MAX_VAL, cv2.THRESH_BINARY)

        # Remove Overlapping Regions (so the final images stays in R G B and not mixture of R G B colors)
        blue_bin_stack = blue_binary
        green_bin_stack = np.bitwise_and(green_binary, np.bitwise_not(blue_binary))
        red_bin_stack = np.bitwise_and(red_binary, np.bitwise_not(np.bitwise_or(green_binary, blue_binary)))

        # Merge the thresholded channels
        thresholded_image = cv2.merge([blue_bin_stack, green_bin_stack, red_bin_stack])

        return thresholded_image

    def fitEllipse(self, image):
        
        # Split into RGB Channels
        b, g, r = cv2.split(image)
        
        # Combine the channels to get a black n white image
        thresh = np.bitwise_or(b, np.bitwise_or(g, r))
        # cv2.imshow("thres", thresh)

        # Find contours in that black n white image
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        
        # Sort contours based on their area and distance from the torch
        contours = sorted(contours, key=lambda x: self.sortContour(x))
        
        # if any contour is present, try to fit an ellipse on it
        if len(contours) > 0:

            #  Get the last contour in the sorted contours
            cnt = contours[-1]
            # Calculate its centroid
            cm = cv2.moments(cnt)

            # If M00 is 0 cannot get the centroid, return -1
            if cm["m00"] == 0:
                return -1, -1
            centroid = (int(cm["m10"]/cm['m00']), int(cm["m01"]/cm["m00"]))
            
            # Reshaping contour array so that it can be used with the EFD library
            cnt.resize([cnt.shape[0],cnt.shape[2]])
            for c in range(cnt.shape[0]):
                cnt[c][0],cnt[c][1] = cnt[c][1],cnt[c][0]
            
            # Get Coefficients of the ellipse fitting the contour using EFD
            coeffs = pyefd.elliptic_fourier_descriptors(contour=cnt, order=1)
            return coeffs, centroid
        else:
            # If no contour is present return -1
            return -1, -1

    def getHeatPoolQuality(self, image, coeffs, centroid):

            # Reconstruct the Elliptical Contour
            ellipse_points = pyefd.reconstruct_contour(coeffs=coeffs, locus=(centroid[1], centroid[0]), num_points=100)
            # print(coeffs.shape)          
            for c in range(ellipse_points.shape[0]):
                ellipse_points[c][0],ellipse_points[c][1] = ellipse_points[c][1],ellipse_points[c][0]
            ellipse_points.resize([ellipse_points.shape[0], 1, ellipse_points.shape[1]])
            ellipse_points = ellipse_points.astype(np.int32)

            # get Rounded Rect for that elliptical Contour
            ellipse_rect = cv2.fitEllipse(ellipse_points)

            # Get Ellipse Parameters from the Rounded Rect
            ellipse_center = (int(ellipse_rect[0][0]),int(ellipse_rect[0][1]))
            ellipse_axes = (int(ellipse_rect[1][0]/2),int(ellipse_rect[1][1]/2))
            ellipse_angle = ellipse_rect[2]
            
            # Calculate eccentricity
            if ellipse_axes[1] == 0:
                ellipse_shape = 1
            else:
                ellipse_shape = np.sqrt(1 - (ellipse_axes[0]/ellipse_axes[1])**2)
            # print("shape = ", shape)
            
            # Create Ellipse Mask
            ellipse_mask = np.zeros(image.shape)
            cv2.ellipse(ellipse_mask, ellipse_center, ellipse_axes, ellipse_angle, 0, 360, (255, 255, 255), cv2.FILLED)
            
            # Get all the pixel coordinates within the ellipse
            ellipse_pixels = np.where(ellipse_mask == 255)

            # Get the values of pixels in the thresholded image corresponding to the coordinates within ellipse
            ellipse_pixel_values = image[ellipse_pixels[0], ellipse_pixels[1]]

            # Convert all those values from 0-255 to 0-1
            ellipse_pixel_values = ellipse_pixel_values / 255

            # Sum all the pixels along the RGB Channels to get total number of R, G & B pixels
            ellipse_sum = np.sum(ellipse_pixel_values, 0)

            # Multiply it with the weights to get the strength
            ellipse_strength = np.dot(ellipse_sum, self.STRENGTH_WEIGHTS.T)
            # print("Strength =", ellipse_strength, " Shape =", ellipse_shape, "Angle =", ellipse_angle)
            
            # cv2.imshow("Ellipse", ellipse_mask)
            
            
            return ellipse_strength, ellipse_shape

    def get_strength_median(self, ellipse_strength, ellipse_eccentricity):

        # Add the strength and eccentricity in the buffer
        self.ellipse_buffer_array[self.strength_index] = (ellipse_strength, ellipse_eccentricity)
        self.strength_index = (self.strength_index + 1) % 5

        # Make its copy
        ellipse_buffer_array = self.ellipse_buffer_array.copy()

        # sort it according to strength
        ellipse_buffer_array.sort(order='strength')
        
        # return its median value
        return ellipse_buffer_array[2] 

    # set the calibrate flag if a message is received on calibrate torch topic
    def calibrateCallback(self, msg):
        if(msg.data):
            self.is_calibrated = False
            self.calibrate_torch_flag = 1

    def calibrateTorchPosition(self, centriod, strength):
        # print("CALIBRATING...")
        # Calibrating is done by averaging the center of ellipse in 5 consecutive frames.
        # If counter has reached 6 then 5 centers are stored in the array we can average them now.

        if(self.calibrate_torch_flag == 6):

            # reset the calibration flag
            self.calibrate_torch_flag = 0
            x = 0
            y = 0

            # average all the centers in the buffer
            for i in self.calibration_buffer:
                x += i[0]
                y += i[1]
            self.TORCH_FLAME_CENTER = (int(x/5), int(y/5))
            self.calibration_strength = sum(self.calibration_strength_buffer)/5
            # Clear the buffer
            self.calibration_strength_buffer.clear()
            self.calibration_buffer.clear()
            self.is_calibrated = True
            # print("Calibrating Done, center: ",self.TORCH_FLAME_CENTER)
            return
        
        # Keep adding the centers in the buffer and incrementing the counter until counter reaches 6
        self.calibration_buffer.append(centriod)
        self.calibration_strength_buffer.append(strength)
        self.calibrate_torch_flag += 1

    def sortContour(self, x):
        cm = cv2.moments(x)
        if cm['m00'] == 0:
            return 0
        else:
            centroid = (int(cm["m10"]/cm['m00']), int(cm["m01"]/cm["m00"]))
            dist = ((centroid[0]-self.TORCH_FLAME_CENTER[0])**2 + (centroid[1]-self.TORCH_FLAME_CENTER[1])**2)**0.5
            # print(cv2.contourArea(x), dist, cv2.contourArea(x)*(1/(dist**4+(10**-5))))
            # Area / Distance Squared
            return cv2.contourArea(x)*(1/(dist**2+(10**-5)))

def main(args):

    heatPoolQuality = HeatPoolQuality()
    rospy.init_node("heat_pool_quality_calculator", anonymous=True)
    rospy.loginfo("Starting Node")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)