import platform
from time import sleep
from gs2d import SerialInterface, RobotisP20
import threading

class RoboticEye2():
    def __init__(self, port=None, baud=1000000, hid=1, vid=2, inv_h=False, inv_v=False):
        # default device name
        if port is None:
            pf = platform.system()
            if pf == 'Windows':
                port = "COM3"
            elif pf == 'LInux':
                port = "/dev/ttyUSB0"
        # open serial ports
        self.si = SerialInterface(device=port, baudrate=baud)
        # controlling manager for motors
        self.robotis = RobotisP20(self.si)
        # default motor SIDs
        self._sid_h = hid # horizontal
        self._sid_v = vid # vertical
        # default motor direction
        self._invert_h = inv_h
        self._invert_v = inv_v
  
    def __del__(self):
        # disable motor
        self.robotis.set_torque_enable(False, sid=self._sid_h)
        self.robotis.set_torque_enable(False, sid=self._sid_v)
        # close motor managers
        self.robotis.close()
        # close serial ports
        self.si.close()

    @property
    def invert_h(self):
        return self._invert_h

    @invert_h.setter
    def invert_h(self, value):
        self._invert_h = value

    @property
    def invert_v(self):
        return self._invert_v

    @invert_v.setter
    def invert_v(self, value):
        self._invert_v = value

    @property
    def sid_h(self):
        return self._sid_h

    @sid_h.setter
    def sid_h(self, value):
        self._sid_h = value

    @property
    def sid_v(self):
        return self._sid_v

    @sid_h.setter
    def sid_v(self, value):
        self._sid_v = value

    # def check_sid(self, value):
    #     return self.robotis.get_servo_id(value)
        
    def setup(self, target_time=0.2, p_gain=400):
        self.__motor_setting(self._sid_h, target_time, p_gain)
        self.__motor_setting(self._sid_v, target_time, p_gain)
        
    def move_horizontal(self, angle):
        self.__move(self._sid_h, angle, self._invert_h)
        #self.timer = threading.Timer(0.05, self.move_horizontal,args=(angle,))
        #self.timer.start()
        
    def move_vertical(self, angle):
        self.__move(self._sid_v, angle, self._invert_v)
        #self.timer = threading.Timer(0.05, self.move_vertical, args=(angle,))
        #self.timer.start()
      
    def __move(self, id_motor, angle, invert=False):
        #print("move")
        self.robotis.set_target_position(-angle if invert else angle, id_motor)
        
    def __motor_setting(self, id_motor, target_time=0.2, p_gain=400):
        ## Robitis Motor
        self.robotis.set_torque_enable(True, sid=id_motor)
        # set as initial position
        self.robotis.set_target_position(0, sid=id_motor)
        # setting the target time
        self.robotis.set_target_time(target_time, sid=id_motor)
        # P gains
        self.robotis.set_p_gain(p_gain, sid=id_motor)

def main():
    # create instance of motor manager
    eyes = RoboticEye2(port="COM3")
    # set motor SIDs
    eyes.sid_h = 1
    eyes.sid_v = 2
    # set rotate direction
    eyes.invert_h = False
    eyes.invert_v = True
    # initialization
    # use default value (target_time=0.2, p_gain=400)
    eyes.setup()
    # controlling motor
    ## horizontal
    eyes.move_horizontal(45)
    sleep(0.5)
    eyes.move_horizontal(-45)
    sleep(0.5)
    ## vertical
    eyes.move_vertical(45)
    sleep(0.5)
    eyes.move_vertical(-45)
    sleep(0.5)
    ## both
    eyes.move_horizontal(20)
    eyes.move_vertical(30)
    sleep(0.5)
    # close motor port (destructor)
    del eyes

if __name__ == "__main__":
    main()
