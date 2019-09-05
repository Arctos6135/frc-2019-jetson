import cv2
import numpy as np
from math import *

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

camera_horiz_fov = 61
camera_vert_fov = 37
camera_width = 1280
camera_height = 720

camera_horiz_f = camera_width / 2 / tan(camera_horiz_fov / 2 * pi / 180)
camera_vert_f = camera_height / 2 / tan(camera_vert_fov / 2 * pi / 180)

tape_height = 5.825572030188476
tape_gap = 11.0629666927

def rect_area(rect):
    return rect[1][0] * rect[1][1]

def midpoint(a, b):
    return ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)

def get_vert_angle(y):
    slope = (y - camera_height / 2) / camera_vert_f
    return atan(slope)

def get_horiz_angle(x):
    slope = (x - camera_width / 2) / camera_horiz_f
    return atan(slope)

def get_distance_v(low, high):
    theta = get_vert_angle(low)
    phi = get_vert_angle(high)
    return tape_height / (tan(phi) - tan(theta))

def get_distance(low, high, x):
    # print("Horiz Angle", get_horiz_angle(x))
    return get_distance_v(low, high) * (1 / cos(get_horiz_angle(x)))

def get_rect_distance(rect):
    points = cv2.boxPoints(rect)
    high = max(points[0][1], max(points[1][1], max(points[2][1], points[3][1])))
    low = min(points[0][1], min(points[1][1], min(points[2][1], points[3][1])))
    return get_distance(low, high, rect[0][0])

def rank(rects):
    d0 = get_rect_distance(rects[0])
    d1 = get_rect_distance(rects[1])
    # print("Distances", d0, d1, " [Center " + str(midpoint(rects[0][0], rects[1][0])) + "]")
    return 1.0 / (d0 + d1)

def is_valid_contour(contour, rect):
    area = cv2.contourArea(contour)
    if rect_area(rect) == 0:
        return False
    fullness = area / rect_area(rect)
    return fullness <= fullness_high and fullness >= fullness_low

def rotatedrect_angle(rect):
    if rect[1][0] < rect[1][1]:
        return rect[2] + 180
    else:
        return rect[2] + 90

def is_valid_pair(rects, all_rects):
    left = None
    right = None
    if rects[0][0][0] < rects[1][0][0]:
        left = rects[0]
        right = rects[1]
    else:
        right = rects[0]
        left = rects[1]
    if rotatedrect_angle(left) < 90 and rotatedrect_angle(right) > 90:
        for rect in all_rects:
            if rect != rects[0] and rect != rects[1]:
                if rect[0][0] > left[0][0] and rect[0][0] < right[0][0]:
                    # print(left[0], right[0], rect[0])
                    return False
        return True
    else:
        return False

def get_advanced_info(rects):
    # Calculate the distances to each piece of tape from their y coordinates
    dist1 = get_rect_distance(rects[0])
    dist2 = get_rect_distance(rects[1])

    # Calculate the angle difference
    x_angle1 = get_horiz_angle(rects[0][0][0])
    x_angle2 = get_horiz_angle(rects[1][0][0])
    diff = max(x_angle1, x_angle2) - min(x_angle1, x_angle2)
    # Use the cosine law to find an estimate for the space between the two tapes
    estimated_spacing = sqrt(dist1 * dist1 + dist2 * dist2 - 2 * dist1 * dist2 * cos(diff))
    # Divide the actual by the estimate to get an error multiplier
    error_multiplier = tape_gap / estimated_spacing
    # Multiply both distances by the error multiplier to improve our estimate
    dist1 *= error_multiplier
    dist2 *= error_multiplier

    # Find out the relation of dist1 and dist2, as well as which angle is the one on the left
    left_side = 0
    right_side = 0
    left_angle = 0
    right_angle = 0
    
    # Tape 1 is on the left of tape 2
    if x_angle1 < x_angle2:
        left_side = dist1
        right_side = dist2
        left_angle = x_angle1
        right_angle = x_angle2
    # Tape 1 is on the right of tape 2
    else:
        left_side = dist2
        right_side = dist1
        left_angle = x_angle2
        right_angle = x_angle1
    
    # Calculate the angle between the line normal to the target plane and the line normal to the camera plane
    # Use the cosine law again to get the angle between the left side and the target plane
    theta = acos((left_side * left_side + tape_gap * tape_gap - right_side * right_side) / (2 * left_side * tape_gap))
    angle_offset = (pi / 2 - theta) + left_angle

    # Calculate the coordinates of the centre of the target
    # Add the pi/2 since our angles start at the y axis and not the x axis
    # Negate the angles since they go cw instead of ccw
    x1 = cos(-left_angle + pi / 2) * left_side
    y1 = sin(-left_angle + pi / 2) * left_side
    x2 = cos(-right_angle + pi / 2) * right_side
    y2 = sin(-right_angle + pi / 2) * right_side
    # Take the midpoint as the overall position
    target_x = (x1 + x2) / 2
    target_y = (y1 + y2) / 2

    # Return a tuple of the position and the offset
    return ((target_x, target_y), angle_offset)

def draw_rect(img, rect, color=(255, 0, 0), thickness=2):
    box = np.int0(cv2.boxPoints(rect))
    cv2.drawContours(img, [box], 0, color, thickness)

def draw_rect_of_rects(img, rect0, rect1, color=(255, 0, 0), thickness=4):
    points = np.append(cv2.boxPoints(rect0), cv2.boxPoints(rect1), axis=0)
    x, y, w, h = cv2.boundingRect(points)
    cv2.rectangle(img, (x,y), (x+w,y+h), color, thickness)
    position, offset = get_advanced_info((rect0, rect1))
    cv2.putText(img, "Angle Offset: {0:.5f}".format(offset), (x, y - 36), cv2.FONT_HERSHEY_DUPLEX, 1, color)
    cv2.putText(img, "Position: ({0:.3f}, {1:.3f})".format(position[0], position[1]), (x, y - 6), cv2.FONT_HERSHEY_DUPLEX, 1, color)

colors = [
    (0, 0, 0xFF),
    (0, 0xFF, 0xFF),
    (0, 0xFF, 0),
    (0xFF, 0xFF, 0),
    (0xFF, 0, 0),
    (0xFF, 0, 0xFF)
]

def get_rects(img):
    # Convert to hsv and thresh
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
    mono = cv2.inRange(hsv, thresh_low, thresh_high)
    # Apply morphological operations 
    kern = cv2.getStructuringElement(cv2.MORPH_RECT, (morph_kernel_size, morph_kernel_size))
    cv2.morphologyEx(mono, cv2.MORPH_OPEN, kern)
    cv2.morphologyEx(mono, cv2.MORPH_CLOSE, kern)

    # Find contours and min area rects
    _, contours, _ = cv2.findContours(mono, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    rects = []
    for contour in contours:
        rect = cv2.minAreaRect(contour)
        
        if is_valid_contour(contour, rect):
            rects.append(rect)

    # Filter and get matching pairs
    matching = []
    for i in range(0, len(rects)):
        for j in range(i + 1, len(rects)):
            if abs(rects[i][0][1] - rects[j][0][1]) <= max_y_diff and is_valid_pair([rects[i], rects[j]], rects):
                matching.append([rects[i], rects[j]])
    
    return matching

def process_image(img):
    matching = get_rects(img)
    if len(matching) > 0:
        _, width, _ = img.shape
        matching.sort(reverse = True, key = rank)
        for pair, color in zip(matching, colors):
            draw_rect(img, pair[0], color)
            draw_rect(img, pair[1], color)
            draw_rect_of_rects(img, pair[0], pair[1], color)
    else:
        # print("No targets found!")
        pass


if __name__ == "__main__":
    from glob import glob
    for img_file in glob("**/*.png", recursive=True):
        print(f"\u001b[1;32mProcessing {img_file}\u001b[0m")
        img = cv2.imread(img_file)
        process_image(img)
        cv2.imshow("Targets", img)
        cv2.waitKey(-1)

