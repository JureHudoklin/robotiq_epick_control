#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
ROS node for controling a Robotiq EPick gripper using the Modbus RTU protocol.
The script takes as an argument the IP address of the gripper. It initializes 
a baseRobotiqEPickGripper object and adds a comModbusTcp client to it. It then 
loops forever, reading the gripper status and updating its command. The gripper 
status is published on the 'RobotiqEPickGripperRobotInput' topic using the 
'RobotiqEPickGripper_robot_input' msg type. The node subscribes to the 
'RobotiqEPickGripperRobotOutput' topic for new commands using the 
'RobotiqEPickGripper_robot_output' msg type. Examples are provided to control 
the gripper (RobotiqEPickGripperSimpleController.py) and interpreting its status
(RobotiqEPickGripperStatusListener.py).
"""

import rospy
from time import sleep


import comModBusRTU
import baseRobotiqEPick
import os
import sys
from robotiq_epick_control.msg import RobotiqEPick_robot_input as inputMsg
from robotiq_epick_control.msg import RobotiqEPick_robot_output as outputMsg


def mainLoop(device):

    print "a dela to glupav node"
    # Gripper is a EPick, ModBUS RTU communication

    
    gripper = baseRobotiqEPick.robotiqbaseRobotiqEPickGripper()
    gripper.client = comModBusRTU.communication()

    # We connect to the address received as an argument
    gripper.client.connectToDevice(device)
    
    rospy.init_node('robotiq_epick_node')
   
    # The Gripper status is published on the topic named 'RobotiqEPickGripperRobotInput'
    pub = rospy.Publisher('RobotiqEPickRobotInput',
                          inputMsg, queue_size=10)

    # The Gripper command is received from the topic named 'RobotiqEPickGripperRobotOutput'
    rospy.Subscriber('RobotiqEPickRobotOutput',
                     outputMsg, gripper.refreshCommand)
    
    # We loop
    while not rospy.is_shutdown():
        
        # Get and publish the Gripper status
        status = gripper.getStatus()
        pub.publish(status)

        # Wait a little
        rospy.sleep(0.05)

        # Send the most recent command
        gripper.sendCommand()

        # Wait a little
        rospy.sleep(0.05)
        

if __name__ == '__main__':
    
    try:
        mainLoop("/dev/ttyUSB0")
    except rospy.ROSInterruptException:
        pass
