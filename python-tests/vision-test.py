
thresh_high_h = 130
thresh_high_s = 255
thresh_high_v = 255
thresh_low_h = 80
thresh_low_s = 80
thresh_low_v = 80

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



