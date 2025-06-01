#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Pose
import sys, getopt, math

class cmdvel2gazebo:
    def _init_(self, ns):
        self.ns = ns
        rospy.init_node('cmdvel2gazebo', anonymous=True)

        rospy.Subscriber('{}/cmd_vel'.format(ns), Twist, self.callback)
        self.pub_steerL = rospy.Publisher('{}/front_left_steering_position_controller/command'.format(ns), Float64, queue_size=1)
        self.pub_steerR = rospy.Publisher('{}/front_right_steering_position_controller/command'.format(ns), Float64, queue_size=1)
        self.pub_rearL = rospy.Publisher('{}/joint1_velocity_controller/command'.format(ns), Float64, queue_size=1)
        self.pub_rearR = rospy.Publisher('{}/joint2_velocity_controller/command'.format(ns), Float64, queue_size=1)

        self.x = 0
        self.z = 0

        self.L = 2.62
        self.T = 1.29
        self.timeout = rospy.Duration.from_sec(0.2)
        self.lastMsg = rospy.Time.now()

        self.maxsteerInside = 0.6
        rMax = self.L / math.tan(self.maxsteerInside)
        rIdeal = rMax + (self.T / 2.0)
        self.maxsteer = math.atan2(self.L, rIdeal)

        rospy.loginfo(rospy.get_caller_id() + " maximum ideal steering angle set to {0}.".format(self.maxsteer))

    def callback(self, data):
        rospy.loginfo("Received Twist Message - Linear X: {0}, Angular Z: {1}".format(data.linear.x, data.angular.z))
        self.x = 2.6101 * data.linear.x
        self.z = max(-self.maxsteer, min(self.maxsteer, data.angular.z))
        self.lastMsg = rospy.Time.now()

    def publish(self):
        if rospy.Time.now() - self.lastMsg > self.timeout:
            self.x = 0
            return

        if self.z != 0:
            T = self.T
            L = self.L
            r = L / math.fabs(math.tan(self.z))

            rL = r - (math.copysign(1, self.z) * (T / 2.0))
            rR = r + (math.copysign(1, self.z) * (T / 2.0))
            msgRearR = Float64()
            msgRearR.data = self.x * rR / r
            msgRearL = Float64()
            msgRearL.data = self.x * rL / r

            rospy.loginfo("Publishing rearR: {0}, rearL: {1}".format(msgRearR.data, msgRearL.data))

            self.pub_rearL.publish(msgRearL)
            self.pub_rearR.publish(msgRearR)

            msgSteerL = Float64()
            msgSteerR = Float64()
            msgSteerL.data = math.atan2(L, rL) * math.copysign(1, self.z)
            msgSteerR.data = math.atan2(L, rR) * math.copysign(1, self.z)

            rospy.loginfo("Publishing steerL: {0}, steerR: {1}".format(msgSteerL.data, msgSteerR.data))

            self.pub_steerL.publish(msgSteerL)
            self.pub_steerR.publish(msgSteerR)
        else:
            msgRear = Float64()
            msgRear.data = self.x
            self.pub_rearL.publish(msgRear)
            self.pub_rearR.publish(msgRear)

            msgSteer = Float64()
            msgSteer.data = self.z
            self.pub_steerL.publish(msgSteer)
            self.pub_steerR.publish(msgSteer)

def usage():
    print('cmdvel2gazebo -n catvehicle')

def main(argv):
    ns = 'catvehicle'  # Nombre de espacio para el tópico
    node = cmdvel2gazebo(ns)
    rate = rospy.Rate(100)  # run at 100Hz
    while not rospy.is_shutdown():
        node.publish()
        rate.sleep()

if _name_ == '_main_':
    main(sys.argv[1:])
