#!/usr/bin/env python3

import rospy
from time import sleep

from robotiq_epick_control.msg import RobotiqEPick_robot_input as inputMsg
from robotiq_epick_control.msg import RobotiqEPick_robot_output as outputMsg


def genCommand(char, command):
    """Update the command according to the character entered by the user."""   

    """
    uint8 rACT  0 = clear fault
                1 = activate gripper
    uint8 rMOD  0 = automatic mode (griper sets vacume level, timeout, hysteresis)
                1 = advanced mode (user sets the settings)
    unit8 rGTO  0 = stop vacum generation
                1 = Follow the requested vacuum parameters in real time. When timeout is reached, rGTO must be re-asserted
    uint8 rATR  0 = normal operation
                1 = Open the valves without any timeout. After an automatic release, rACT must be re-asserted 
    uint8 rPR   ... target vacum
                0 = vacume generator always on
                22-90 = grip from 100% to 10% of vacum. rPR = 90(Min device vacum: 10%) rPR = 22(Max device vacum: 78%)
                100 = Passive relese, relese to ambient pressure
                101-255 = Active relese. Release with positive preasure.
    uint8 rSP   0-255 = timieot, 1 = 100ms
    uint8 rFR   0-99 = object detection preasure in %
    """

    # Reset the controler (Resets all settings)
    if char == 'r':
        command.rACT = 0 
        command.rGTO = 0 

    # Set the mode to automatic
    if char == 'm':
        command.rMOD = 0
        
    # Activate suction
    if char == 'a':
        command.rACT = 1
        command.rATR = 0
        command.rGTO = 1 
        
    # Deactivate vacum generation
    if char == 'dv':
        command.rGTO = 0

    # Relese object to ambient pressure without timeout. rACT must be called after this command 
    if char == 'd':
        command.rATR = 1


    #----------------MANUAL MODE------------------
    
    # Set manual mode wtih parameters
    if char == 'mo':
        command.rMOD = 1
        command.rGTO = 1 
        command.rATR = 0
        command.rPR = 80
        command.rSP = 20
        command.rFR = 50
    
    # Active release (Only in manual MODE)
    if char == 'ad':
        command.rPR = 255


    #If the command entered is a int, assign this value to rPRA
    return command


def askForCommand(command):

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

    print(currentCommand)

    strAskForCommand = "-----\nAvailable commands\n\n"
    strAskForCommand += "r: Reset\n"
    strAskForCommand += "m: Mode Automatic\n"
    strAskForCommand += "a: Activate\n"
    strAskForCommand += "d: Automatic Relese\n"
    strAskForCommand += "mo: Manual Mode\n"
    strAskForCommand += "dv: Deactivate vacum generation\n"
    strAskForCommand += "ad: Active relese\n"
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

    return input(strAskForCommand)


def robotiq_epick_simplecontroller():
    print("test1")
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
