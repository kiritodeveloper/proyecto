import time
import cv2

from RobotFrameCapturer import RobotFrameCapturer


def trackObject(colorRangeMin=(0, 0, 0), colorRangeMax=(255, 255, 255)):
    frame_capturer = RobotFrameCapturer(colorRangeMin, colorRangeMax)
    print(colorRangeMin)
    frame_capturer.start()

    finished = False

    while not finished:
        promising_blob = frame_capturer.getPosition()
        print(promising_blob)
        time.sleep(1)

    frame_capturer.stop()
    return finished


def main():
    orangeMinHSV = (10, 180, 50)
    orangeMaxHSV = (26, 255, 255)

    redMinRGB = (10, 10, 100)
    redMaxRGB = (50, 50, 255)
    trackObject(colorRangeMin=orangeMinHSV, colorRangeMax=orangeMaxHSV)


if __name__ == "__main__":
    main()
