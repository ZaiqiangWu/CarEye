
import sys, os, math, json,time
import threading

sys.path.insert(0, '../..')
from gs2d import SerialInterface, RobotisP20

class RoboticEye():
    def __init__(self, port="/dev/ttyUSB0", baud=1000000):
        super().__init__()
        # open serial ports
        self.si = SerialInterface(device=port, baudrate=baud)
        # controlling manager for motors
        self.robotis = RobotisP20(self.si)
        # activated motor id
        self.sids = []

    def setup(self, id_motor, target_time=0.2, p_gain=400):
        ## Robitis Motor
        self.robotis.set_torque_enable(True, sid=id_motor)
        # set as initial position
        self.robotis.set_target_position(0, sid=id_motor)
        # setting the target time
        self.robotis.set_target_time(target_time, sid=id_motor)
        # P gains
        self.robotis.set_p_gain(p_gain, sid=id_motor)
        # register activated motor id
        self.sids.append(id_motor)

    def move(self, id_motor, angle, invert=False):
        #print("move")
        self.robotis.set_target_position(-angle if invert else angle, id_motor)
        
    def __del__(self):
        # disable motor
        for num in self.sids:
            self.robotis.set_torque_enable(False, sid=num)
        # close motor managers
        self.robotis.close()
        # close serial ports
        self.si.close()