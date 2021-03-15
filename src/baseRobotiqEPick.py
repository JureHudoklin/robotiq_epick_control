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
Module baseRobotiqEPick: defines a base class for handling command and status of the Robotiq 2F gripper.
After being instanciated, a 'client' member must be added to the object. This client depends on the communication protocol used by the Gripper. As an example, the ROS node 'Robotiq2FGripperTcpNode.py' instanciate a robotiqbaseRobotiq2FGripper and adds a client defined in the module comModbusTcp.
"""

from robotiq_epick_control.msg import RobotiqEPick_robot_input as inputMsg
from robotiq_epick_control.msg import RobotiqEPick_robot_output as outputMsg


class robotiqbaseRobotiqEPickGripper:
    """Base class (communication protocol agnostic) for sending commands and receiving the status of the Robotic 2F gripper"""

    def __init__(self):

        # Initiate output message as an empty list
        self.message = []
        self.client = None #???????

        # Note: after the instantiation, a ".client" member must be added to the object

    def verifyCommand(self, command):
        """Function to verify that the value of each variable satisfy its limits."""

        # Verify that each variable is in its correct range
        command.rACT = max(0, command.rACT)
        command.rACT = min(1, command.rACT)

        command.rMOD = max(0, command.rMOD)
        command.rMOD = min(1, command.rMOD)

        command.rGTO = max(0, command.rGTO)
        command.rGTO = min(1, command.rGTO)

        command.rATR = max(0, command.rATR)
        command.rATR = min(1, command.rATR)

        command.rPR = max(0,   command.rPR) #Max relative pressure
        command.rPR = min(255, command.rPR)

        command.rSP = max(0,   command.rSP) #GripTimeOut/Delay request
        command.rSP = min(255, command.rSP)

        command.rFR = max(0,   command.rFR) #Minimum relative pressure request
        command.rFR = min(100, command.rFR)

        # Return the modified command
        return command

    def refreshCommand(self, command):
        """Function to update the command which will be sent during the next sendCommand() call."""

        # Limit the value of each variable
        command = self.verifyCommand(command)

        # Initiate command as an empty list
        self.message = []

        # Build the command with each output variable
        # To-Do: add verification that all variables are in their authorized range
        self.message.append(
            command.rACT +(command.rMOD << 1) + (command.rGTO << 3) + (command.rATR << 4))
        self.message.append(0)
        self.message.append(0)
        self.message.append(command.rPR)
        self.message.append(command.rSP)
        self.message.append(command.rFR)

    def sendCommand(self):
        """Send the command to the Gripper."""

        self.client.sendCommand(self.message)

    def getStatus(self):
        """Request the status from the gripper and return it in the Robotiq2FGripper_robot_input msg type."""

        # Acquire status from the Gripper
        status = self.client.getStatus(6)

        # Message to output
        message = inputMsg()

        # Assign the values to their respective variables
        message.gACT = (status[0] >> 0) & 0x01
        message.gMOD = (status[0] >> 1) & 0x03
        message.gGTO = (status[0] >> 3) & 0x01
        message.gSTA = (status[0] >> 4) & 0x03
        message.gOBJ = (status[0] >> 6) & 0x03
        message.gVAS = (status[1] >> 0) & 0x03
        message.gFLT = (status[2] >> 0) & 0x0F
        message.gPR = status[3]
        message.gPO = status[4]

        return message
