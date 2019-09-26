#!/usr/bin/python
import rospy
import serial
import numpy as np

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Wrench

class HapticInteraction():
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
        self.hapticRate = self.rospy.get_param("~haptic_parameters/rate", 30)
        self.hapticMode = self.rospy.get_param("~haptic_parameters/mode", "haptic visual")
        self.hapticParams = {
            "k1":   self.rospy.get_param("~haptic_parameters/k1", 5000),
            "k2":   self.rospy.get_param("~haptic_parameters/k2", 50)
        }
        self.hapkit_pos_topic = self.rospy.get_param("~hapkit/pos_topic", "hapkit/pos")
        self.hapkit_vel_topic = self.rospy.get_param("~hapkit/vel_topic", "hapkit/vel")
        self.hapkit_set_pos_topic = self.rospy.get_param("~hapkit/set_pos_topic", "hapkit/set_point/pos")
        self.hapkit_set_force_topic = self.rospy.get_param("~hapkit/set_force_topic", "hapkit/set_point/force")
        self.hapkit_double_wall_topic = self.rospy.get_param("~hapkit/double_wall_topic", "hapkit/set_double_wall")
        self.hapkit_wall_topic = self.rospy.get_param("~hapkit/wall", "hapkit/set_wall")
        self.hapkit_leds_topic = self.rospy.get_param("~hapkit/leds", "hapkit/leds")
        self.hapkit_reset_topic = self.rospy.get_param("~hapkit/reset", "hapkit/reset")
        self.hapkit_wrench_topic = self.rospy.get_param("~hapkit/wrench_topic", "hapkit/wrench")
        pass

    def initSubscribers(self):
        self.subErrorTheta = self.rospy.Subscriber(self.errorThetaTopic, Float32, self.callbackErrorTheta)
        self.subPos = self.rospy.Subscriber(self.hapkit_pos_topic, Float32, self.callbackHapkitPos)
        self.subVel = self.rospy.Subscriber(self.hapkit_vel_topic, Float32, self.callbackHapkitVel)
        pass

    def initPublishers(self):
        self.pubSetPos = self.rospy.Publisher(self.hapkit_set_pos_topic, Float32, queue_size = 5)
        self.pubSetForce = self.rospy.Publisher(self.hapkit_set_force_topic, Float32, queue_size = 5)
        self.pubLeds = self.rospy.Publisher(self.hapkit_leds_topic, String, queue_size = 5)
        self.pubDoubleWall = self.rospy.Publisher(self.hapkit_double_wall_topic, Float32, queue_size = 5)
        self.pubWall = self.rospy.Publisher(self.hapkit_wall_topic, Float32, queue_size = 5)
        self.pubReset = self.rospy.Publisher(self.hapkit_reset_topic, String, queue_size = 5)
        self.pubWrench = self.rospy.Publisher(self.hapkit_wrench_topic, Wrench, queue_size = 5)
        pass

    def initVariables(self):
        self.change_error = False
        self.change_pos = False
        self.change_vel = False
        self.x_max = 0.06
        self.frc = 0
        self.rate = self.rospy.Rate(self.hapticRate)
        self.theta_threshold = 10*np.pi/180
        self.led = "off"
        self.wall = -10.0
        pass

    def callbackErrorTheta(self, msg):
        self.change_error = True
        self.error = msg.data
        pass

    def callbackHapkitPos(self, msg):
        self.change_pos = True
        self.pos = msg.data
        self.msgWrench = Wrench()
        if self.hapticMode == "free" or self.hapticMode == "visual" or self.hapticMode == "haptic" or self.hapticMode == "haptic visual":
            self.msgWrench.torque.z = -1*self.hapticParams["k1"]*np.tanh(self.pos/self.hapticParams["k2"])
            if self.msgWrench.torque.z < 0.5 and self.msgWrench.torque.z > -0.5:
                self.msgWrench.torque.z = 0
        pass

    def callbackHapkitVel(self, msg):
        self.change_vel = True
        self.vel = msg.data
        pass

    def makeMsgWrench(self):
        print(self.pos)
        self.pubWrench.publish(self.msgWrench)
        return

    def makeMsgLED(self):
        #self.pubLeds.publish("off")
        msg = String()
        if self.error >= self.theta_threshold:
            msg.data = "right on"
        elif self.error <= -self.theta_threshold:
            msg.data = "left on"
        else:
            msg.data = "middle on"
        if msg.data != self.led:
            self.pubLeds.publish("off")
            self.pubLeds.publish(msg)
            self.led = msg.data
        return

    def makeMsgWall(self):
        msg = Float32()
        if self.error >= self.theta_threshold:
            #Correct action at right, so we set a wall at left side
            msg.data = -0.01
        elif self.error <= -self.theta_threshold:
            #Correct action at left, so we set a wall at right side
            msg.data = 0.01
        else:
            msg.data = 0.0
        if msg.data != self.wall:
            if msg.data == 0.0:
                self.pubLeds.publish("middle on")
                if self.pos < 0.01 and self.pos > -0.01:
                    self.pubDoubleWall.publish(0.01)
                #else:
                #    self.pubWall.publish(self.pos)
            else:
                self.pubLeds.publish("off")
                self.pubWall.publish(msg)
            self.wall = msg.data
        return

    def makeMsgWall2(self):
        msg = Float32()
        if self.error >= self.theta_threshold:
            #Correct action at right, so we set a wall at left side
            msg.data = -0.01
        elif self.error <= -self.theta_threshold:
            #Correct action at left, so we set a wall at right side
            msg.data = 0.01
        else:
            msg.data = 0.0
        if msg.data != self.wall:
            if msg.data == 0.0:
                if self.pos < 0.01 and self.pos > -0.01:
                    self.pubDoubleWall.publish(0.01)
                else:
                    self.pubWall.publish(self.pos)
            else:
                self.pubWall.publish(msg)
            self.wall = msg.data
        return

    def main(self):
        while not self.rospy.is_shutdown():
            if self.change_pos:
                if self.hapticMode == "free":
                    self.makeMsgWrench()
                if self.hapticMode == "visual" and self.change_error:
                    self.makeMsgLED()
                    self.makeMsgWrench()
                    self.change_error = False
                if self.hapticMode == "haptic" and self.change_error:
                    self.makeMsgWall()
                    self.makeMsgWrench()
                    self.change_error = False
                if self.hapticMode == "haptic visual" and self.change_error:
                    self.makeMsgLED()
                    self.makeMsgWall2()
                    self.makeMsgWrench()
                    self.change_error = False
            self.rate.sleep()
        pass

if __name__ == "__main__":
    try:
        obj = HapticInteraction("haptic_control")
    except Exception as e:
        print("Something is wrong", e)
        raise e
