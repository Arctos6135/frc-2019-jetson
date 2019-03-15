import cv2
import numpy as np

thresh_high_h = 130
thresh_high_s = 255
thresh_high_v = 255
thresh_low_h = 80
thresh_low_s = 80
thresh_low_v = 80

thresh_high = np.array([thresh_high_h, thresh_high_s, thresh_high_v])
thresh_low = np.array([thresh_low_h, thresh_low_s, thresh_low_v])

fullness_low = 0.75
fullness_high = 1.0

morph_kernel_size = 5

max_y_diff = 50

def rect_area(rect):
    return rect[1][0] * rect[1][1]

def combined_area(contours):
    return rect_area(contours[0]) + rect_area(contours[1])

def is_valid_contour(contour, rect):
    area = cv2.contourArea(contour)
    fullness = area / rect_area(rect)
    return fullness <= fullness_high and fullness >= fullness_low

def rotatedrect_angle(rect):
    if rect[1][0] < rect[1][1]:
        return rect[2] + 180
    else:
        return rect[2] + 90

def is_valid_pair(rects):
    left = None
    right = None
    if rects[0][0][0] < rects[1][0][0]:
        left = rects[0]
        right = rects[1]
    else:
        right = rects[0]
        left = rects[1]
    return rotatedrect_angle(left) < 90 and rotatedrect_angle(right) > 90

def process_image(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
    mono = cv2.inRange(hsv, thresh_low, thresh_high)
    kern = cv2.getStructuringElement(cv.MORPH_RECT, (morph_kernel_size, morph_kernel_size))
    cv2.morphologyEx(mono, cv2.MORPH_OPEN, kern)
    cv2.morphologyEx(mono, cv2.MORPH_CLOSE, kern)

    contours = cv2.findContours(mono, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    rects = []
    for contour in contours:
        rect = cv2.minAreaRect(contour)
        
        if is_valid_contour(contour, rect):
            rects.append(bound)
    
    matching = []
    for i in range(0, len(rects)):
        for j in range(i + 1, len(rects)):
            if abs(rects[i][0][1] - rects[j][0][1]) <= max_y_diff and is_valid_pair([rects[i], rects[j]]):
                matching.append([rects[i], rects[j]])



if __name__ == "__main__":
    from glob import glob
    for img in glob("*.png"):
        process_image(cv2.imread(img))
        cv2.waitKey(-1)

