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
from std_msgs.msg import String, Int32

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
    last_velocity = 0
    last_angular = 0
    traffic_sign_detected = False

    # initialize
    def __init__(self):
        rospy.init_node('ControlTurtleBot', anonymous=False)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist)

        # Subscribe to traffic_sign to get linear velocity
        self.ros_data_linear = rospy.Subscriber("/traffic_sign/detected", String, self.callback_linear)
        rospy.loginfo("Subscribed to /traffic_sign/detected")

        # TODO returns number between 0 and 180 -> change to Burger format
        # Subscribe to lane_assist to get angular velocity
        self.ros_data_angular = rospy.Subscriber("/lane_assist/detected", Int32, self.callback_angular)
        rospy.loginfo("Subscribed to /lane_assist/detected")

        rospy.spin()

    def callback_linear(self, ros_data_linear):
        self.movement(self.sign_controls(ros_data_linear), "linear")

    def callback_angular(self, ros_data_angular):
        self.movement(self.lane_detection(ros_data_angular), "angular")

    def movement(self, speed, kind):
        move_cmd = Twist()
        # set new linear velocity and use last angular velocity
        if kind == 'linear':
            move_cmd.linear.x = speed
            move_cmd.angular.z = self.last_angular
            self.cmd_vel.publish(move_cmd)
            rospy.loginfo('linear velocity changed to:')
            rospy.loginfo(self.last_velocity)
        # set new angular velocity and use last linear velocity
        elif kind == 'angular':
            move_cmd.linear.x = self.last_velocity
            move_cmd.angular.z = speed
            self.cmd_vel.publish(move_cmd)
            rospy.loginfo('angular velocity changed to: ' + self.last_angular)

    # lane detection, set angular velocity
    def lane_detection(self, ros_angular):
        rospy.loginfo(self, ros_angular)
        if ros_angular == 90:
            self.last_angular = 0
        elif ros_angular < 90:
            self.last_angular = ros_angular/180 * (-2.84)
        elif ros_angular.data > 90:
            self.last_angular = ros_angular/180 * 2.84

        return self.last_angular

    # traffic sign detected, do:
    def sign_controls(self, ros_data):

        if "nothing" not in ros_data.data:

            if "entry_forbidden" in ros_data.data:
                # rospy.loginfo("entry_forbidden detected")
                self.last_velocity = 0.0
            elif "main_road" in ros_data.data:
                # rospy.loginfo("main road detected")
                self.last_velocity = 0.11
            elif "turn_right" in ros_data.data:
                # rospy.loginfo("turn right detected")
                self.last_velocity = 0.0
            elif "turn_left" in ros_data.data:
                # rospy.loginfo("turn left detected")
                self.last_velocity = 0.0
            elif "pedestrians" in ros_data.data:
                # rospy.loginfo("pedestrians detected")
                self.last_velocity = 0.05
            elif "warning" in ros_data.data:
                # rospy.loginfo("warning detected")
                self.last_velocity = 0.05
            elif "no_parking" in ros_data.data:
                # rospy.loginfo("no parking detected")
                self.last_velocity = 0.0
            elif "bus_stop" in ros_data.data:
                # rospy.loginfo("bus stop detected")
                self.last_velocity = 0.0
            elif "crossing" in ros_data.data:
                # rospy.loginfo("crossing detected")
                self.last_velocity = 0.05
            elif "slippery" in ros_data.data:
                # rospy.loginfo("slippery detected")
                self.last_velocity = 0.05
            elif "road_closed" in ros_data.data:
                # rospy.loginfo("road closed detected")
                self.last_velocity = 0.0

        return self.last_velocity

    # shutdown function
    # TODO Fixme (shutdown)
    def roboter_shutdown(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_vel.publish(twist)
        rospy.loginfo("Shutting down (__exit__)")


# Main
if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    turtlebot3_model = rospy.get_param("model", "burger")

    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0

    control = burger_control()  # start turtlebot movement

    try:
        rospy.spin()
    except:
        rospy.loginfo("Something went wrong - turtle shutdown")
    finally:
        control.roboter_shutdown()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
