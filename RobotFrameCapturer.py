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

    def getDetectorParamsConfiguration(self):
        # Setup default values for SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # These are just examples, tune your own if needed
        # Change thresholds
        params.minThreshold = 10
        params.maxThreshold = 200

        # Filter by Area
        params.filterByArea = True
        params.minArea = 500
        params.maxArea = 70000

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1

        # Filter by Color
        params.filterByColor = False
        # not directly color, but intensity on the channel input
        # params.blobColor = 0
        params.filterByConvexity = False
        params.filterByInertia = False
        return params

    def getDetector(self):
        # Create a detector with the parameters
        ver = cv2.__version__.split('.')
        if int(ver[0]) < 3:
            return cv2.SimpleBlobDetector(self.getDetectorParamsConfiguration())
        else:
            return cv2.SimpleBlobDetector_create(self.getDetectorParamsConfiguration())

    def start(self):
        self.p = Process(target=self.loop_frame_capturer)
        self.p.start()

    def stop(self):
        self.finished.value = False
        self.p.close()

    def getPosition(self):
        self.lock_frame_capturer.acquire()
        x = self.x_object.value
        y = self.y_object.value
        size = self.size_object.value
        self.lock_frame_capturer.release()
        return x, y, size

    def obtainBlobPosition(self, detector, imgBGR, minRange, maxRange):
        """

        :param detector:
        :param imgBGR:
        :param minRange: in HSV
        :param maxRange: in HSV
        :return:
        """
        # Next lines copied from https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
        blurred = cv2.GaussianBlur(imgBGR, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        #hsv = blurred

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
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(imgBGR, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(imgBGR, center, 5, (0, 0, 255), -1)

        # Show image for debug only
        #output = cv2.bitwise_and(imgBGR, imgBGR, mask=mask)
        #cv2.imshow("images", np.hstack([imgBGR, output]))

        # Take images every 100 ms
        cv2.waitKey(100)

        return x, y, radius

    def loop_frame_capturer(self):
        detector = self.getDetector()

        print("Antes de nada")
        if is_debug:
            cap = cv2.VideoCapture(0)
            while True:
                # Capture frame-by-frame
                ret, imgBGR = cap.read()

                x, y, size = self.obtainBlobPosition(detector, imgBGR, self.minRange, self.maxRange)

                self.lock_frame_capturer.acquire()
                self.x_object.value = x
                self.y_object.value = y
                self.size_object.value = size
                self.lock_frame_capturer.release()
                print(size)

                if self.finished.value:
                    break

        else:
            cam = picamera.PiCamera()

            cam.resolution = (320, 240)
            # cam.resolution = (640, 480)
            cam.framerate = 32
            rawCapture = PiRGBArray(cam, size=(320, 240))
            # rawCapture = PiRGBArray(cam, size=(640, 480))

            # allow the camera to warmup
            time.sleep(0.1)

            for img in cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                # Capture frame
                imgBGR = img.array

                x, y, size = self.obtainBlobPosition(detector, imgBGR, self.minRange, self.maxRange)

                self.lock_frame_capturer.acquire()
                self.x_object.value = x
                self.y_object.value = y
                self.size_object.value = size
                self.lock_frame_capturer.release()

                rawCapture.truncate(0)
                cv2.waitKey(1)

                if self.finished.value:
                    break
