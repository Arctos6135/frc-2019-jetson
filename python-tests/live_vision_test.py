import vision_test
import cv2
import time

"""
live_vision_test

Written as a demo to be used at the subteam introductions meeting.
Note:
    * OpenCV must be installed.
    * This script was only tested on Ubuntu, with a Microsoft Lifecam.
    * If your device does not have a built-in webcam, you may need to change the line
            capture = cv2.VideoCapture(1)
        into
            capture = cv2.VideoCapture(0)
        in order to select the correct camera.
    * Sometimes the camera settings may not be set correctly (namely autoexposure). To fix this, use v4l2-ctl:
            v4l2-ctl --device=/dev/video1 --set-ctrl=exposure_auto=1
        to turn off auto exposure, and then
            v4l2-ctl --device=/dev/video1 --list-ctrls-menus
        to verify that exposure_auto is indeed off.
"""

capture = cv2.VideoCapture(1)
capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)

capture.set(3, vision_test.camera_width)

capture.set(4, vision_test.camera_height)

time.sleep(2)

capture.set(15, -8.0)

while(True):
    # Capture a frame
    ret, frame = capture.read()
    if not ret:
        # Failed!
        print("Error: Frame capture failed!")
        exit(1)
    vision_test.process_image(frame)
    cv2.imshow("Vision Demo", frame)
    
    code = cv2.waitKey(50) & 0xFF
    if code == ord('q'):
        break

capture.release()
cv2.destroyAllWindows()
