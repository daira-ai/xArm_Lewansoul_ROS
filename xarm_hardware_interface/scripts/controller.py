"""
Author: Maxime Chevalier-Boisvert
https://gist.github.com/maximecb/7fd42439e8a28b9a74a4f7db68281071

Modified by: Carolina Leon Pinzon

sudo apt-get install libhidapi-hidraw0 libhidapi-libusb0
pip3 install --user hid

Notes from the original code:
- servos_off will power off the servos
- sending a movement command wakes up unpowered servos
- position readouts are sadly pretty slow, they can take up to 450ms
"""



import time
import easyhid
import numpy as np

def itos(v):
    lsb = v & 0xFF
    msb = v >> 8
    return lsb, msb

def flip(m, axis):
    if not hasattr(m, 'ndim'):
        m = asarray(m)
    indexer = [slice(None)] * m.ndim
    try:
        indexer[axis] = slice(None, None, -1)
    except IndexError:
        raise ValueError("axis=%i is invalid for the %i-dimensional input array"
                         % (axis, m.ndim))
    return m[tuple(indexer)]

class XArm():
    def __init__(self, pid=5750):

        # Stores an enumeration of all the connected USB HID devices
        en = easyhid.Enumeration()
        print (en)

        # return a list of devices based on the search parameters
        devices = en.find(product="LOBOT")#vid=1155, )

        # print a description of the devices found
        for dev in devices:
            print(dev.description())

        assert len(devices) > 0
        self.dev = devices[0]

        # open a device
        self.dev.open()
        print('Connected to xArm device')

    def __del__(self):
        print('Closing xArm device')
        self.dev.close()

    def move_to(self, id, pos, time=0):
        """
        CMD_SERVO_MOVE
        0x55 0x55 len 0x03 [time_lsb time_msb, id, pos_lsb pos_msb]
        Servo position is in range [0, 1000]
        """

        t_lsb, t_msb = itos(time)
        p_lsb, p_msb = itos(pos)
        self.dev.write([0x55, 0x55, 8, 0x03, 1, t_lsb, t_msb, id, p_lsb, p_msb])

    def move_all(self, poss, time=0):
        """
        Set the position of all servos at once
        """

        for i in range(6):
            self.move_to(id=i+1, pos=poss[i], time=time)

    def servos_off(self):
        self.dev.write([0x55, 0x55, 9, 20, 6, 1, 2, 3, 4, 5, 6])

    def read_pos(self):
        """
        Read the position of all 6 servos
        ServoPositionRead 21 (byte)count { (byte)id }; (byte)count { (byte)id (ushort)position }
        """

        self.dev.write([
            0x55, 0x55,
            9,  # Len
            21, # Cmd
            6,  # Count
            1,
            2,
            3,
            4,
            5,
            6
        ])

        ret = self.dev.read()
        count = ret[4]
        assert count == 6

        poss = []
        for i in range(6):
            id = ret[5 + 3*i]
            p_lsb = ret[5 + 3*i + 1]
            p_msb = ret[5 + 3*i + 2]
            pos = (p_msb << 8) + p_lsb
            poss.append(pos)

        return np.array(poss)

    def rest(self):
        self.move_all([500, 500, 200, 900, 800, 500], time=1500)
        time.sleep(2)
        self.servos_off()


class SafeXArm:
    """
    Wrapper to limit motion range and speed to maximize durability
    Also remaps joint angles into the [-1, 1] range
    """

    def __init__(self, **kwargs):
        self.arm = XArm(**kwargs)

        self.min_pos = np.array([
            100, # Base
            200,
            400,
            100,
            50,  # Wrist
            200, # Gripper
        ])

        self.max_pos = np.array([
            900, # Base
            800,
            900,
            600,
            850,  # Wrist
            650,  # Gripper
        ])

        # Maximum movement speed in (range/second)
        self.max_speed = 100 #250

        #self.move_all([0] * 6)
        #time.sleep(2)

    def read_pos(self):
        return flip(self.arm.read_pos())

    def rest(self):
        return self.arm.rest()

    def move_all(self, pos):
        if not isinstance(pos, np.ndarray):
            pos = np.array(pos)

        # [-1, 1] => [0, 1]
        pos = (pos + 1) / 2
        target = self.min_pos + pos * (self.max_pos - self.min_pos)

        print type(np)
        print "Before flip:", target

        target = flip(target, 0).astype(np.uint16)
        print "After flip:",target

        # TODO: compute time needed based on last position
        # Compute time needed to move each joint to target given max_speed
        #cur_pos = self.arm.read_pos()
        #time = (abs(cur_pos - target) / self.max_speed)
        #time = (time * 1000).astype(np.uint16)

        for i in range(6):
            self.arm.move_to(id=i+1, pos=target[i], time=50)


def demo():
    arm = XArm()
    print arm.read_pos()

    # Open gripper
    arm.move_to(id=1, pos=115, time=1000)
    time.sleep(2)

    # Close gripper
    arm.move_to(id=1, pos=685, time=1000)
    time.sleep(2)

    # Move Joint 2 
    arm.move_to(id=2, pos=1000, time=1000)
    time.sleep(2)
    
    # Move Joint 2 
    arm.move_to(id=2, pos=590, time=1000)
    time.sleep(2)

    # Move Joint 3 
    arm.move_to(id=3, pos=500, time=1000)
    time.sleep(2)
    
    # Move Joint 3 
    arm.move_to(id=3, pos=150, time=1000)
    time.sleep(2)


demo()
