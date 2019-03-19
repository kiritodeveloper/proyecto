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

    #redMinHSV = (0, 160, 105)
    #redMaxHSV = (3, 255, 255)
    #redMinHSV = (0, 217, 68)
    #redMaxHSV = (6, 240, 144)

    redMinRGB = (10, 10, 100)
    redMaxRGB = (50, 50, 255)

    # 0 240 68
    # 233 217 144
    # 236 240 89

    trackObject(colorRangeMin=redMinRGB, colorRangeMax=redMaxRGB)


if __name__ == "__main__":
    main()
