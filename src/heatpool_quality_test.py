import cv2
import pyefd
import os
import numpy as np
from matplotlib import pyplot as plt

MAX_VAL = 255
THRESHOLD = 250
TORCH_FLAME_CENTER = (332, 478)
STRENGTH_WEIGHTS = np.array([3,2,1]) #B G R
# def close_image

def sortContour(x):
    cm = cv2.moments(x)
    if cm['m00'] == 0:
        return 0
    else:
        centroid = (int(cm["m10"]/cm['m00']), int(cm["m01"]/cm["m00"]))
        dist = ((centroid[0]-TORCH_FLAME_CENTER[0])**2 + (centroid[1]-TORCH_FLAME_CENTER[1])**2)**0.5
        # print(cv2.contourArea(x), dist, cv2.contourArea(x)*(1/(dist**4+(10**-5))))
        return cv2.contourArea(x)*(1/(dist**4+(10**-5)))

def fit_ellipse(img):
    b, g, r = cv2.split(img)
    thresh = np.bitwise_or(b, np.bitwise_or(g, r))
    # gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # _, thresh = cv2.threshold(gray_img, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    # _, thresh = cv2.threshold(gray_img, 10, 255, 0)
    # imshow("GRAY", gray_img)
    # thresh = np.bitwise_not(thresh)
    cv2.imshow("thres", thresh)
    # edges = cv2.Canny(thresh, 100, 200)
    # imshow("edges", edges)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contours = sorted(contours, key=lambda x: sortContour(x))
    # print()
    if len(contours) > 0:
        img_copy = np.array(img)
        cnt = contours[-1]
        cv2.drawContours(img_copy, [cnt],0, (150,150,150), 2)
        # cv2.circle(img_copy, TORCH_FLAME_CENTER, 5, (255, 0, 255), cv2.FILLED)
        cm = cv2.moments(cnt)
        if cm["m00"] == 0:
            return
        centroid = (int(cm["m10"]/cm['m00']), int(cm["m01"]/cm["m00"]))
        cv2.circle(img_copy, centroid, 5, (255, 255, 0), cv2.FILLED)
        cv2.line(img_copy, TORCH_FLAME_CENTER, centroid, (0, 255, 255), 2)

        cnt.resize([cnt.shape[0],cnt.shape[2]])
        for c in range(cnt.shape[0]):
            cnt[c][0],cnt[c][1] = cnt[c][1],cnt[c][0]
        coeffs = pyefd.elliptic_fourier_descriptors(contour=cnt, order=1)
        ellipse_points = pyefd.reconstruct_contour(coeffs=coeffs, locus=(centroid[1], centroid[0]), num_points=100)
        # print(coeffs.shape)
        ellipse_image = np.zeros(img.shape)
        for c in range(ellipse_points.shape[0]):
            ellipse_points[c][0],ellipse_points[c][1] = ellipse_points[c][1],ellipse_points[c][0]
        ellipse_points.resize([ellipse_points.shape[0], 1, ellipse_points.shape[1]])
        ellipse_points = ellipse_points.astype(np.int32)
        ellipse_rect = cv2.fitEllipse(ellipse_points)
        ellipse_center = (int(ellipse_rect[0][0]),int(ellipse_rect[0][1]))
        ellipse_axes = (int(ellipse_rect[1][0]/2),int(ellipse_rect[1][1]/2))
        if ellipse_axes[1] == 0:
            ellipse_shape = 0
        else:
            ellipse_shape = ellipse_axes[0]/ellipse_axes[1]
        # print("shape = ", shape)
        theta = ellipse_rect[2]
        cv2.ellipse(ellipse_image, ellipse_center, ellipse_axes, theta, 0, 360, (255, 255, 255), cv2.FILLED)
        ellipse_pixels = np.where(ellipse_image == 255)
        ellipse_pixel_values = img[ellipse_pixels[0], ellipse_pixels[1]]
        ellipse_pixel_values = ellipse_pixel_values / 255
        ellipse_sum = np.sum(ellipse_pixel_values, 0)
        ellipse_strength = np.dot(ellipse_sum, STRENGTH_WEIGHTS.T)
        print("Strength =", ellipse_strength, " Shape = ", ellipse_shape)
        # print(ellipse_pixel_values.shape)
        # cv2.drawContours(img_copy_2, [ellipse_points], 0, (255, 255, 255), 3)
        cv2.imshow("Ellipse", ellipse_image)
        # pyefd.plot_efd(coeffs=coeffs, locus=(centroid[1], centroid[0]), contour=cnt, image=img_copy)
        # plt.pause(0.01)
        cv2.imshow("Contours", img_copy)
        return coeffs

if __name__ == '__main__':
    plt.ion()
    path = "output/06132022_161721/images_06132022_161721/"
    if os.path.isdir(path):
        images = os.listdir(path)
        images.sort()
        for image_name in images:
            image_path = os.path.join(path, image_name)
            image = cv2.imread(image_path)
            cv2.imshow("Input", image)
            blur = cv2.GaussianBlur(image, (7, 7), 5)

            blue_channel, green_channel, red_channel = cv2.split(blur)

            _, blue_binary = cv2.threshold(blue_channel, THRESHOLD, MAX_VAL, cv2.THRESH_BINARY)
            _, green_binary = cv2.threshold(green_channel, THRESHOLD, MAX_VAL, cv2.THRESH_BINARY)
            _, red_binary = cv2.threshold(red_channel, THRESHOLD, MAX_VAL, cv2.THRESH_BINARY)

            blue_bin_stack = blue_binary
            green_bin_stack = np.bitwise_and(green_binary, np.bitwise_not(blue_binary))
            red_bin_stack = np.bitwise_and(red_binary, np.bitwise_not(np.bitwise_or(green_binary, blue_binary)))

            thresholded_image = cv2.merge([blue_bin_stack, green_bin_stack, red_bin_stack])
            cv2.imshow("Thresholded", thresholded_image)
            ellipse_coeffs = fit_ellipse(thresholded_image)
            cv2.circle(image, TORCH_FLAME_CENTER, 3, (255, 0, 255), cv2.FILLED)
            
            # cv2.imshow("Red", red_channel)
            # cv2.imshow("Green", green_channel)
            # cv2.imshow("Blue", blue_channel)
            # cv2.imshow("Red Bin", red_binary)
            # cv2.imshow("Green Bin", green_binary)
            # cv2.imshow("Blue Bin", blue_binary)
            # cv2.imshow("Red Bin Stack", red_bin_stack)
            # cv2.imshow("Green Bin Stack", green_bin_stack)
            # cv2.imshow("Blue Bin Stack", blue_bin_stack)
            
            cv2.waitKey(1)