#!/usr/bin/env python

# https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/master/turtlebot3_teleop/nodes/turtlebot3_teleop_key

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
from std_msgs.msg import String

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

LAST_LIN_VEL = 0.0
LAST_ANG_VEL = 0.0

sign_list = ['nothing', 'nothing', 'nothing', 'nothing', 'nothing']

msg = """
TurtleBot3 activated
--------------------

Max. linear velocity: 0.22
Max. angular velocity: 2.84
"""

e = """Communications Failed"""


def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel, target_angular_vel)


def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output


# check velocity
def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input


# check linear velocity
def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    return vel


# check angular velocity
def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    return vel


#
def path_control():
    path_ang = 0.0
    return path_ang


# Subscribe to traffic_sign & line_detect
class burger_control:
    last_velocity = 0.0
    traffic_sign_detected = False

    def __init__(self):
        rospy.init_node('ControlTurtleBot', anonymous=False)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist)

        self.raspi_subscriber = rospy.Subscriber("/traffic_sign/detected", String, self.callback)
        rospy.loginfo("Subscribed to /traffic_sign/detected")

    # TODO Subscibe to line_detection

    def callback(self, ros_data):
        move_cmd = Twist()
        if self.traffic_sign_detected: # sign detected: do 'sign' for x seconds
            for i in range(50):  # wait for 5 seconds
                move_cmd.linear.x = self.last_velocity
                move_cmd.angular.z = 0.0
                self.cmd_vel.publish(move_cmd)
            self.last_velocity = 0.11
            move_cmd.linear.x = self.last_velocity
        else:  # no traffic sign detected, do sign_controls
            move_cmd.linear.x = self.sign_controls(ros_data)
        move_cmd.angular.z = 0.0
        self.cmd_vel.publish(move_cmd)

        rospy.loginfo(ros_data)

    # TODO Callback line_detection

    # traffic sign detected, do:
    def sign_controls(self, ros_data):

        rospy.loginfo(ros_data.data)

        sign_list.pop(0)  # delete first item in sign_list
        sign_list.append(ros_data)  # add latest item at the end

        if sign_list.count(ros_data) == 3 and ros_data.data != 'nothing':

            self.traffic_sign_detected = True

            sign_list.clear()  # queue
            sign_list.extend(['nothing', 'nothing', 'nothing', 'nothing', 'nothing'])

            if ros_data.data == "entry_forbidden":
                rospy.loginfo("entry_forbidden detected")
                self.last_velocity = 0.0
            elif ros_data.data == "main_road":
                rospy.loginfo("main road detected")
                self.last_velocity = 0.11
            elif ros_data.data == "turn_right":
                rospy.loginfo("turn right detected")
                self.last_velocity = 0.0
            elif ros_data.data == "turn_left":
                rospy.loginfo("turn left detected")
                self.last_velocity = 0.0
            elif ros_data.data == 'pedestrians':
                rospy.loginfo("pedestrians detected")
                self.last_velocity = 0.05
            elif ros_data.data == "warning":
                rospy.loginfo("warning detected")
                self.last_velocity = 0.05
            elif ros_data.data == "no_parking":
                rospy.loginfo("no parking detected")
                self.last_velocity = 0.0
            elif ros_data.data == "bus_stop":
                rospy.loginfo("bus stop detected")
                self.last_velocity = 0.0
            elif ros_data.data == "crossing":
                rospy.loginfo("crossing detected")
                self.last_velocity = 0.05
            elif ros_data.data == "slippery":
                rospy.loginfo("slippery detected")
                self.last_velocity = 0.05
            elif ros_data.data == "road_closed":
                rospy.loginfo("road closed detected")
                self.last_velocity = 0.0
            else:
                self.traffic_sign_detected = False

        return self.last_velocity


# Main
if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    turtlebot3_model = rospy.get_param("model", "burger")

    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0

    control = burger_control()

    # Wait for callback: ( while(1) )
    try:
        rospy.spin()
    except KeyboardInterrupt:
        stop_turtle = Twist()
        stop_turtle.linear.x = 0.0
        stop_turtle.angular.z = 0.0
        # pub.publish(stop_turtle)
        rospy.loginfo("Shutting down")
        # TODO set velocity to zero

    '''try:
        print msg
        while(1):
            key = getKey()
            if key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == 'x' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == 'a' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == 'd' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == ' ' or key == 's' :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print vels(target_linear_vel, target_angular_vel)
            else:
                if (key == '\x03'):
                    break

            if status == 20 :
                print msg
                status = 0

            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            pub.publish(twist)

    except:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)
    '''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
