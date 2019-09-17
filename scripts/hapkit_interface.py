#!/usr/bin/python
import rospy
import numpy
import serial

from std_msgs.msg import Float32, String

'''
00 - Set forces equal to zero, turn off LEDs
01 - Get Pos and Vel
02 - Set Pos
03 - Set Force
04 - Wall at X
05 - Turn off wall
10 - Turn left LED off
11 - Turn left LED on
20 - Turn middle LED off
21 - Turn middle LED on
30 - Turn right LED off
31 - Turn right LED on
40 - Turn off LEDs
41 - Turn on LEDs
'''

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
        self.hapkit_pos_topic = self.rospy.get_param("~hapkit/pos_topic", "hapkit/pos")
        self.hapkit_vel_topic = self.rospy.get_param("~hapkit/vel_topic", "hapkit/vel")
        self.hapkit_set_pos_topic = self.rospy.get_param("~hapkit/set_pos_topic", "hapkit/set_point/pos")
        self.hapkit_set_force_topic = self.rospy.get_param("~hapkit/set_force_topic", "hapkit/set_point/force")
        self.hapkit_wall_topic = self.rospy.get_param("~hapkit/wall", "hapkit/set_wall")
        self.hapkit_leds_topic = self.rospy.get_param("~hapkit/leds", "hapkit/leds")
        self.hapkit_reset_topic = self.rospy.get_param("~hapkit/reset", "hapkit/reset")
        self.hapkit_rate = self.rospy.get_param("~hapkit/rate", 100)
        self.hapkit_serialPort = self.rospy.get_param("~hapkit/serial_port", "/dev/ttyUSB0")
        self.hapkit_baudRate = self.rospy.get_param("~hapkit/baudRate", 115200)
        pass

    def initSubscribers(self):
        self.subSetPos = self.rospy.Subscriber(self.hapkit_set_pos_topic, Float32, self.callbackSetPos)
        self.subSetForce = self.rospy.Subscriber(self.hapkit_set_force_topic, Float32, self.callbackSetForce)
        self.subWall = self.rospy.Subscriber(self.hapkit_wall_topic, Float32, self.callbackSetWall)
        self.subLeds = self.rospy.Subscriber(self.hapkit_leds_topic, String, self.callbackSetLeds)
        ## TODO: Make this a service ...
        self.subReset = self.rospy.Subscriber(self.hapkit_reset_topic, String, self.callbackReset)
        pass

    def initPublishers(self):
        self.pubPos = self.rospy.Publisher(self.hapkit_pos_topic, Float32, queue_size = 5)
        self.pubVel = self.rospy.Publisher(self.hapkit_vel_topic, Float32, queue_size = 5)
        pass

    def initVariables(self):
        self.change_error = False
        self.hapkit = serial.Serial(self.hapkit_serialPort, self.hapkit_baudRate)
        self.rate = self.rospy.Rate(self.hapkit_rate)
        pass

    def callbackSetPos(self, msg):
        try:
            value = msg.data
            print(value)
            self.hapkit.write("02"+str(value))
            self.rospy.loginfo("[%s] Setting %s position", self.name, str(value))
        except Exception as e:
            self.rospy.logwarn("[%s] Unable to set position")
        return

    def callbackSetForce(self, msg):
        try:
            value = msg.data
            self.hapkit.write("03"+str(value))
            self.rospy.loginfo("[%s] Setting %s force", self.name, str(value))
        except Exception as e:
            self.rospy.logwarn("[%s] Unable to set force")
        return

    def callbackSetWall(self, msg):
        try:
            value = msg.data
            if float(value) == 0:
                self.hapkit.write("05")
            else:
                self.hapkit.write("04"+str(value))
        except Exception as e:
            self.rospy.logwarn("[%s] Unable to set wall")
        pass

    def callbackSetLeds(self, msg):
        try:
            data = msg.data
            if data == "left off":
                self.hapkit.write("10")
            elif data == "left on":
                self.hapkit.write("11")
            if data == "middle off":
                self.hapkit.write("20")
            elif data == "middle on":
                self.hapkit.write("21")
            if data == "right off":
                self.hapkit.write("30")
            elif data == "right on":
                self.hapkit.write("31")
            elif data == "off":
                self.hapkit.write("40")
            elif data == "on":
                self.hapkit.write("41")
            else:
                self.rospy.logwarn("[%s] I didn't understand that", self.name)
                self.hapkit.write("40")
        except Exception as e:
            self.rospy.logwarn("[%s] Unable to set LEDs", self.name)

    def callbackReset(self, msg):
        try:
            self.hapkit.write("00")
        except Exception as e:
            self.rospy.logwarn("[%s] Unable to reset", self.name)

    def startup_sequence(self):
        data = ""
        while data != "00":
            self.rospy.loginfo("[%s] Performing startup_sequence", self.name)
            data = self.hapkit.readline().strip()
        self.rospy.loginfo("[%s] Initial byte received!", self.name)
        return True

    def publish_kinematics(self):
        try:
            self.hapkit.write("01")
            data = self.hapkit.readline()
            pos, vel = map(float, data.strip().split("%"))
            self.pubPos.publish(Float32(pos))
            self.pubVel.publish(Float32(vel))
        except Exception as e:
            self.rospy.loginfo("[%s] Unable to publish position and velocity", self.name)
            print(e)

    def main(self):
        self.startup_sequence()
        while not self.rospy.is_shutdown():
            self.publish_kinematics()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        obj = Hapkit("hapkit")
    except Exception as e:
        print("Something is wrong", e)
