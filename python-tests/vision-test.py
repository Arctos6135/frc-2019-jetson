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

def midpoint(a, b):
    return ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)

def rank(rects, width):
    combined_area = rect_area(rects[0]) + rect_area(rects[1])
    offcenterness = abs(width / 2 - midpoint(rects[0][0], rects[1][0])[0])
    return combined_area - offcenterness

def is_valid_contour(contour, rect):
    area = cv2.contourArea(contour)
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
                    print(left[0], right[0], rect[0])
                    return False
        return True
    else:
        return False

def draw_rect(img, rect, color=(255, 0, 0), thickness=2):
    box = np.int0(cv2.boxPoints(rect))
    cv2.drawContours(img, [box], 0, color, thickness)

def draw_rect_of_rects(img, rect0, rect1, color=(255, 0, 0), thickness=4):
    points = np.append(cv2.boxPoints(rect0), cv2.boxPoints(rect1), axis=0)
    x, y, w, h = cv2.boundingRect(points)
    cv2.rectangle(img, (x,y), (x+w,y+h), color, thickness)

colors = [
    (0, 0, 0xFF),
    (0, 0xFF, 0xFF),
    (0, 0xFF, 0),
    (0xFF, 0xFF, 0),
    (0xFF, 0, 0),
    (0xFF, 0, 0xFF)
]

def process_image(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
    mono = cv2.inRange(hsv, thresh_low, thresh_high)
    kern = cv2.getStructuringElement(cv2.MORPH_RECT, (morph_kernel_size, morph_kernel_size))
    cv2.morphologyEx(mono, cv2.MORPH_OPEN, kern)
    cv2.morphologyEx(mono, cv2.MORPH_CLOSE, kern)

    _, contours, _ = cv2.findContours(mono, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    rects = []
    for contour in contours:
        rect = cv2.minAreaRect(contour)
        
        if is_valid_contour(contour, rect):
            rects.append(rect)
    
    matching = []
    for i in range(0, len(rects)):
        for j in range(i + 1, len(rects)):
            if abs(rects[i][0][1] - rects[j][0][1]) <= max_y_diff and is_valid_pair([rects[i], rects[j]], rects):
                matching.append([rects[i], rects[j]])
    if len(matching) > 0:
        _, width, _ = img.shape
        matching.sort(reverse = True, key = lambda p: rank(p, width))
        for pair, color in zip(matching, colors):
            draw_rect(img, pair[0], color)
            draw_rect(img, pair[1], color)
            draw_rect_of_rects(img, pair[0], pair[1], color)
        cv2.imshow("Targets", img)
    else:
        cv2.imshow("Targets", img)
        print("No targets found!")


if __name__ == "__main__":
    from glob import glob
    for img in glob("**/*.png"):
        print(f"Processing {img}")
        process_image(cv2.imread(img))
        cv2.waitKey(-1)

