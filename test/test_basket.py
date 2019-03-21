from Robot import Robot


def main():
    try:
        # if args.radioD < 0:
        #    print('d must be a positive value')
        #    exit(1)

        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot()
        # 1. launch updateOdometry thread()
        robot.startOdometry()

        robot.catch('down')
        robot.catch('up')
        robot.catch('up')
        robot.catch('down')
        robot.catch('up')
        robot.catch('down')
        robot.catch('up')
        robot.catch('down')
        robot.catch('up')
        robot.catch('down')
        robot.catch('up')
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()

    except KeyboardInterrupt:
        # except the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()


if __name__ == "__main__":
    main()
