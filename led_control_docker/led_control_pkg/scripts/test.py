#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import String
from gpiozero import RGBLED, Device
from gpiozero.pins.rpigpio import RPiGPIOFactory

Device.pin_factory = RPiGPIOFactory()
class LedControl:
    def __init__(self):
        self.detected_sign = rospy.Subscriber('/yolo/sign_detection', String, self.cbDetectedSign, queue_size=5)
        self.Colors = {
            "LEFT_TURN":     (1, 1, 0),  # yellow
            "RIGHT_TURN":    (0, 1, 0),  # green
            "INTERSECTION":  (0, 0, 1),  # blue
            "CONSTRUCTION":  (1, 0, 0),  # red
        }
        self.state = {
            "CONSTRUCTION": False,
            "LEFT_TURN":    False,
            "RIGHT_TURN":   False,
            "INTERSECTION": False
        }
        self.sign = None
        self.blink_sleep = 0.5

        self.led = RGBLED(red=17, green=27, blue=22, active_high=True)

    def cbDetectedSign(self, msg):
        self.sign = msg.data.strip()
        self._blink()

    def stateChanger(self):
        self.state[self.sign] = True

    def _blink(self):
        try:
            if self.state[self.sign] == False:
                self.led.color = self.Colors[self.sign]
                rospy.sleep(self.blink_sleep)
                self.led.off()
                self.stateChanger()
        finally:
            self.led.off()

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('LedControl')
    node = LedControl()
    node.main()
