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
    redMin = (10, 10, 100)
    redMax = (50, 50, 255)
    trackObject(colorRangeMin=redMin, colorRangeMax=redMax)


if __name__ == "__main__":
    main()
