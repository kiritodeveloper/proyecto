class FakeBlockPi(object):

    def __init__(self):
        self.PORT_1 = 0x01
        self.PORT_2 = 0x02
        self.PORT_3 = 0x04
        self.PORT_4 = 0x08

        self.PORT_A = 0x01
        self.PORT_B = 0x02
        self.PORT_C = 0x04
        self.PORT_D = 0x08

    def set_motor_dps(self, motor_port_left, speed_dps_left):
        pass

    def reset_all(self):
        pass
