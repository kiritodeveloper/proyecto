import time

from multiprocessing import Process, Value, Array, Lock

import imutils
import numpy as np

from config_file import is_debug

import cv2

if not is_debug:
    import picamera
    from picamera.array import PiRGBArray

class RobotFrameCapturer(object):
    def __init__(self, minRange, maxRange):
        # Store X, Y and size
        self.x_object = Value('d', 0)
        self.y_object = Value('d', 0)
        self.size_object = Value('d', 0)

        # Check if finished
        self.finished = Value('b', False)

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_frame_capturer = Lock()

        # min and max range
        self.minRange = minRange
        self.maxRange = maxRange

    def start(self):
        self.p = Process(target=self.loopFrameCapturer)
        self.p.start()

    def stop(self):
        self.lock_frame_capturer.acquire()
        self.finished.value = True
        self.lock_frame_capturer.release()


    def getPosition(self):
        self.lock_frame_capturer.acquire()
        x = self.x_object.value
        y = self.y_object.value
        size = self.size_object.value
        self.lock_frame_capturer.release()
        return x, y, size

    def obtainBallPositionAndSize(self, imgBGR, minRange, maxRange):
        """
        Obtain the ball position center in pixels and its radius size
        :param imgBGR: Image in RGB
        :param minRange: Min color range to detect ball in HSV
        :param maxRange: Min color range to detect ball in HSV
        :return:
        """
        # Some part of next lines was obtained from https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
        blurred = cv2.GaussianBlur(imgBGR, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Allow two ranges of colors for red
        if minRange[0] > maxRange[0]:
            minRange0 = list(minRange)
            minRange0[0] = 0

            maxRange0 = list(maxRange)
            maxRange0[0] = maxRange[0]

            minRange1 = list(minRange)
            minRange1[0] = minRange[0]

            maxRange1 = list(maxRange)
            maxRange1[0] = 180

            mask0 = cv2.inRange(hsv, np.asarray(minRange0), np.asarray(maxRange0))

            mask1 = cv2.inRange(hsv, np.asarray(minRange1), np.asarray(maxRange1))

            mask = cv2.bitwise_or(mask1, mask0)

        else:
            mask = cv2.inRange(hsv, minRange, maxRange)

        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        x = 0
        y = 0
        radius = 0

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 5:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(imgBGR, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(imgBGR, center, 5, (0, 0, 255), -1)

        # Show image for debug only
        # output = cv2.bitwise_and(imgBGR, imgBGR, mask=mask)
        # cv2.imshow("images", np.hstack([imgBGR, output]))

        # Take images every 100 ms
        cv2.waitKey(50)

        return x, y, radius

    def loopFrameCapturer(self):
        if is_debug:
            cap = cv2.VideoCapture(0)
            while True:
                # Capture frame-by-frame
                _, imgBGR = cap.read()

                x, y, size = self.obtainBallPositionAndSize(imgBGR, self.minRange, self.maxRange)

                self.lock_frame_capturer.acquire()
                self.x_object.value = x
                self.y_object.value = y
                self.size_object.value = size
                self.lock_frame_capturer.release()

                if self.finished.value:
                    break

        else:
            cam = picamera.PiCamera()

            cam.resolution = (320, 240)
            cam.framerate = 32
            rawCapture = PiRGBArray(cam, size=(320, 240))

            # allow the camera to warmup
            time.sleep(0.1)

            for img in cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                # Capture frame
                imgBGR = img.array

                x, y, size = self.obtainBallPositionAndSize(imgBGR, self.minRange, self.maxRange)

                self.lock_frame_capturer.acquire()
                self.x_object.value = x
                self.y_object.value = y
                self.size_object.value = size
                turn_off_camera = self.finished.value
                self.lock_frame_capturer.release()

                rawCapture.truncate(0)
                cv2.waitKey(1)

                if turn_off_camera:
                    print('Hay que apagar la camara')
                    break

        print('Apago camara')
        cap.close()