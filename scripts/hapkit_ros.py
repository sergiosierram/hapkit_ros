#!/usr/bin/python
import rospy
import numpy
import serial

from std_msgs.msg import Float32

class Hapkit():
    def __init__(self, name):
        self.name = name
        self.initNode()
        self.initParameters()
        self.initSubscribers()
        self.initPublishers()
        self.initVariables()
        self.main()

    def initNode(self):
        self.rospy = rospy
        self.rospy.init_node(self.name, anonymous = True)
        self.rospy.loginfo("[%s] Starting node", self.name)
        pass

    def initParameters(self):
        self.errorThetaTopic = self.rospy.get_param("~error_theta_topic", "/error_theta")
        self.hapticRate = self.rospy.get_param("~haptic_parameters/rate", 100)
        self.hapticMode = self.rospy.get_param("~haptic_parameters/mode", "free")
        self.serialPort = self.rospy.get_param("~hapkit/serial_port", "/dev/ttyUSB0")
        self.baudRate = self.rospy.get_param("~hapkit/baudRate", 115200)
        pass

    def initSubscribers(self):
        self.subErrorTheta = self.rospy.Subscriber(self.errorThetaTopic, Float32, self.callbackErrorTheta)
        pass

    def initPublishers(self):
        pass

    def initVariables(self):
        self.change_error = False
        self.hapkit = serial.Serial(self.serialPort, self.baudRate)
        pass

    def callbackErrorTheta(self, msg):
        self.change_error = True
        pass

    def startup_sequence(self):
        data = ""
        while data != "00":
            self.rospy.loginfo("[%s] Performing startup_sequence", self.name)
            data = self.hapkit.readline().strip()
        self.rospy.loginfo("[%s] Initial byte received!", self.name)
        return True

    def main(self):
        self.startup_sequence()
        while not self.rospy_is_shutdown():
            if self.change_error():

        pass

if __name__ == "__main__":
    try:
        obj = Hapkit("hapkit")
    except Exception as e:
        print("Something is wrong", e)
