import time

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

    redMinHSV2 = (0, 160, 105)
    redMaxHSV2 = (3, 255, 255)

    redMinRobot = (0, 0, 130)
    redMaxRobot = (100, 60, 255)

    redMinHSV = (168, 180, 80)
    redMaxHSV = (2, 255, 255)

    redMinRGB = (10, 10, 100)
    redMaxRGB = (50, 50, 255)
    trackObject(colorRangeMin=redMinHSV, colorRangeMax=redMaxHSV)


if __name__ == "__main__":
    main()
