#!/usr/bin/python
import rospy
import numpy
import serial
import time

from std_msgs.msg import Float32, String

'''
00 - Set forces equal to zero, turn off LEDs
01 - Get Pos and Vel
02 - Set Pos
03 - Set Force
04 - Wall at X
05 - Turn off wall
06 - Symmetric wall at X and -X
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
        self.hapkit_wall_topic = self.rospy.get_param("~hapkit/wall_topic", "hapkit/set_wall")
        self.hapkit_double_wall_topic = self.rospy.get_param("~hapkit/double_wall_topic", "hapkit/set_double_wall")
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
        self.subDoubleWall = self.rospy.Subscriber(self.hapkit_double_wall_topic, Float32, self.callbackSetDoubleWall)
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
        self.writing = False
        self.functions = {
            "reset": self.reset,
            "leds": self.leds,
            "wall": self.wall,
            "pos": self.setpos,
            "force": self.setforce,
            "dwall": self.double_wall
        }
        self.queue = False
        self.tasks = []
        pass

    def callbackSetPos(self, msg):
        self.queue = True
        self.tasks.append(["pos", msg.data])
        return

    def callbackSetForce(self, msg):
        self.queue = True
        self.tasks.append(["force", msg.data])
        return

    def callbackSetWall(self, msg):
        self.queue = True
        self.tasks.append(["wall", msg.data])
        return

    def callbackSetDoubleWall(self, msg):
        self.queue = True
        self.tasks.append(["dwall", msg.data])
        return

    def callbackSetLeds(self, msg):
        self.queue = True
        self.tasks.append(["leds", msg.data])
        return

    def callbackReset(self, msg):
        self.queue = True
        self.tasks.append(["reset", msg.data])
        return

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

    def reset(self, x):
        try:
            self.hapkit.write("00")
            data = self.hapkit.readline()
            self.rospy.loginfo("[%s] Resetting", self.name)
        except Exception as e:
            self.rospy.logwarn("[%s] Unable to reset", self.name)
            print(e)

    def leds(self, x):
        try:
            data = str(x)
            if data == "left off":
                self.hapkit.write("10")
                r = self.hapkit.readline()
            elif data == "left on":
                self.hapkit.write("11")
                r = self.hapkit.readline()
            elif data == "middle off":
                self.hapkit.write("20")
                r = self.hapkit.readline()
            elif data == "middle on":
                self.hapkit.write("21")
                r = self.hapkit.readline()
            elif data == "right off":
                self.hapkit.write("30")
                r = self.hapkit.readline()
            elif data == "right on":
                self.hapkit.write("31")
                r = self.hapkit.readline()
            elif data == "off":
                self.hapkit.write("40")
                r = self.hapkit.readline()
            elif data == "on":
                self.hapkit.write("41")
                r = self.hapkit.readline()
            else:
                self.rospy.logwarn("[%s] I didn't understand that", self.name)
                self.hapkit.write("40")
                r = self.hapkit.readline()
            self.rospy.loginfo("[%s] Setting LEDs: %s", self.name, data)
        except Exception as e:
            self.rospy.logwarn("[%s] Unable to set LEDs", self.name)
            print(e)
        pass

    def wall(self, x):
        try:
            value = x
            print(value)
            if float(value) == 0:
                self.hapkit.write("05")
                r = self.hapkit.readline()
                self.rospy.loginfo("[%s] Disabling wall", self.name)
            else:
                self.hapkit.write("04"+str(value))
                r = self.hapkit.readline()
                self.rospy.loginfo("[%s] Setting wall at: %f", self.name, float(value))
        except Exception as e:
            self.rospy.logwarn("[%s] Unable to set wall")
        pass

    def double_wall(self, x):
        try:
            value = x
            print(value)
            if float(value) == 0:
                self.hapkit.write("05")
                r = self.hapkit.readline()
                self.rospy.loginfo("[%s] Disabling wall", self.name)
            else:
                self.hapkit.write("06"+str(value))
                r = self.hapkit.readline()
                self.rospy.loginfo("[%s] Setting double wall at: %f", self.name, float(value))
        except Exception as e:
            self.rospy.logwarn("[%s] Unable to double set wall")
        pass

    def setpos(self, x):
        try:
            value = x
            print(value)
            self.hapkit.write("02"+str(value))
            r = self.hapkit.readline()
            self.rospy.loginfo("[%s] Setting %s position", self.name, str(value))
        except Exception as e:
            self.rospy.logwarn("[%s] Unable to set position")
            print(e)
        pass

    def setforce(self, x):
        try:
            value = x
            self.hapkit.write("03"+str(value))
            r = self.hapkit.readline()
            self.rospy.loginfo("[%s] Setting %s force", self.name, str(value))
        except Exception as e:
            self.rospy.logwarn("[%s] Unable to set force")
        return

    def main(self):
        self.startup_sequence()
        while not self.rospy.is_shutdown():
            self.publish_kinematics()
            if self.queue:
                for task in self.tasks:
                    f = self.functions[task[0]]
                    f(task[1])
                self.queue = False
                self.tasks = []
            self.rate.sleep()

if __name__ == "__main__":
    try:
        obj = Hapkit("hapkit")
    except Exception as e:
        print("Something is wrong", e)
