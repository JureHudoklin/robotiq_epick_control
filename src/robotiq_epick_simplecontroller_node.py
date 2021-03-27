#!/usr/bin/env python

import rospy
from time import sleep

from robotiq_epick_control.msg import RobotiqEPick_robot_input as inputMsg
from robotiq_epick_control.msg import RobotiqEPick_robot_output as outputMsg


def genCommand(char, command):
    """Update the command according to the character entered by the user."""   

    if char == 'r':
        command = outputMsg()
        command.rACT = 0 

    if char == 'm':
        command = outputMsg()
        command.rGTO = 0
        
    if char == 'a':
        command = outputMsg()
        command.rACT = 1
        command.rGTO = 1


    if char == 'd':
        command = outputMsg()
        command.rACT = 1
        command.rATR = 1



    #If the command entered is a int, assign this value to rPRA
    return command


def askForCommand(command):
    """
    uint8 rACT  0 = clear fault
                1 = activate gripper
    uint8 rGTO  0 = automatic mode (griper sets vacume level, timeout, hysteresis)
                1 = advanced mode (user sets the settings)
    unit8 rMOD  0 = stop vacum generation
                1 = Follow the requested vacuum parameters in real time. When timeout is reached, rGTO must be re-asserted
    uint8 rATR  0 = normal operation
                1 = Open the valves without any timeout. After an automatic release, rACT must be re-asserted 
    uint8 rPR   ... target vacum
                0 = vacume generator always on
                1-99 = grop from 1 to 100 of cavum
                100 = Passive relese, relese to ambient pressure
                101-255 = Active relese. Release with positive preasure.
    uint8 rSP   0-255 = timieot, 1 = 100ms
    uint8 rFR   0-99 = object detection preasure in %
    """

    """Ask the user for a command to send to the gripper."""

    currentCommand = "Simple 2F Gripper Controller\n-----\nCurrent command:"
    currentCommand += ""
    currentCommand += ", rATR = " + str(command.rATR)
    currentCommand += ", rGTO = " + str(command.rGTO)
    currentCommand += ", rMOD = " + str(command.rMOD)
    currentCommand += "  rACT = " + str(command.rACT)
    currentCommand += ", rPR = " + str(command.rPR)
    currentCommand += ", rSP = " + str(command.rSP)
    currentCommand += ", rFR = " + str(command.rFR)

    print currentCommand

    strAskForCommand = "-----\nAvailable commands\n\n"
    strAskForCommand += "r: Reset\n"
    strAskForCommand += "m: Mode Automatic\n"
    strAskForCommand += "a: Activate\n"
    strAskForCommand += "d: Automatic Relese\n"
    """
    strAskForCommand += "c: Close\n"
    strAskForCommand += "o: Open\n"
    strAskForCommand += "(0-255): Go to that position\n"
    strAskForCommand += "f: Faster\n"
    strAskForCommand += "l: Slower\n"
    strAskForCommand += "i: Increase force\n"
    strAskForCommand += "d: Decrease force\n"
    """

    strAskForCommand += "-->"

    return raw_input(strAskForCommand)


def robotiq_epick_simplecontroller():
    print "test1"
    """Main loop which requests new commands and publish them on the RobotiqEPickGripperRobotOutput topic."""
    rospy.init_node("robotiq_epick_simplecontroller_node")

    pub = rospy.Publisher("RobotiqEPickRobotOutput", outputMsg, queue_size=10)

    command = outputMsg()

    while not rospy.is_shutdown():

        command = genCommand(askForCommand(command), command)
        
        pub.publish(command)

        rospy.sleep(0.1)


if __name__ == "__main__":
    robotiq_epick_simplecontroller()
