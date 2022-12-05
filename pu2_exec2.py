#!/usr/bin/env python

import os
import argparse
import rospy
import rospkg
import sys
import copy
import time
import rospy
import yaml

import numpy as np
from pu2_header import *
from pu2_func import *
from blob_search import *


# ========================= Student's code starts here =========================

# Position for UR3 not blocking the camera
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]

# Buffer position
bufferPOS = np.radians([180, -135, 90, -90, -90, 0])

# Wave left
waveLeft = np.radians([180, -90, 90, -180, -90, 180])

# Wave right
waveRight = np.radians([180, -90, 90, -180, -90, 0])

# Standard 1/3 pan dimensions
bowl_offset = 0.15
bowl_height = 0.05
pan_width = 0.1
pan_length = 0.2

"""
Table parameters
"""
spacing = 0.05

bowl_start = [0.2,-0.3,bowl_height+0.01]
bowl_end = [-0.1,-0.3,bowl_height+0.01]

food_end = bowl_end+[0,-0.15,0.3]

"""
Starting locations for all food
"""
whiteRice_start = [bowl_offset + pan_width/2, 0.15-pan_length-spacing, 0.05]
brownRice_start = [bowl_offset + (3*pan_width/2) + spacing, 0.15-pan_length-spacing, 0.05]

chicken_start = [bowl_offset + pan_width/2, 0.15, 0.05]
steak_start = [bowl_offset + (3*pan_width/2) + spacing, 0.15, 0.05]

cheese_start = [bowl_offset + pan_width/2, 0.15+pan_length+spacing, 0.05]
lettuce_start = [bowl_offset + (3*pan_width/2) + spacing, 0.15+pan_length+spacing, 0.05]

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

image_shape_define = False


"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

    global digital_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0

"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True

"""
Function to control the suction cup on/off
"""
def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            #rospy.loginfo("Goal is reached!")
            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

"""
Move robot arm from one position to another
"""
def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

################ Pre-defined parameters and functions no need to change above ################

def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel):

    global home

    error = 0
    # Compute required joint angles for the start and end positions
    start_pos = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], 0)
    mid_pos = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2]+0.05, 0)
    end_pos = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2], 0)

    move_arm(pub_cmd, loop_rate, bufferPOS, 4.0, 4.0)

    move_arm(pub_cmd, loop_rate, mid_pos, 4.0, 4.0)

    move_arm(pub_cmd, loop_rate, start_pos, 4.0, 4.0)

    gripper(pub_cmd, loop_rate, suction_on)

    # Delay to make sure suction cup has grasped the block
    time.sleep(1.0)
    if(digital_in_0 == 0):
        gripper(pub_cmd, loop_rate, suction_off)
        rospy.loginfo("Nothing's here!")
        return
    
    move_arm(pub_cmd, loop_rate, mid_pos, 4.0, 4.0)

    move_arm(pub_cmd, loop_rate, bufferPOS, 4.0, 4.0)

    #move to end positon and release
    move_arm(pub_cmd, loop_rate, end_pos, 4.0, 4.0)

    gripper(pub_cmd, loop_rate, suction_off)
    # Delay to make sure suction cup has grasped the block
    time.sleep(1.0)

    move_arm(pub_cmd, loop_rate, bufferPOS, 4.0, 4.0)

    return error

"""
Program run from here
"""
def main():

    global go_away
    global xw_yw_Y
    global xw_yw_G

    global home

    # Initialize ROS node
    rospy.init_node('lab5node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    vel = 4.0
    accel = 4.0
    move_arm(pub_command, loop_rate, bufferPOS, vel, accel)
    print("WELCOME TO CHIPOTLE! WHAT CAN I GET STARTED FOR YOU?")
    time.sleep(3)
    print("A BOWL? GOOD CHOICE!")
    time.sleep(1)

    move_block(pub_command, loop_rate, bowl_start, bowl_end, vel, accel)

    rice = None
    while not rice:
        rice = raw_input("Do you prefer BROWN or WHITE rice: ")
	rice = str(rice)
        if (rice != "BROWN") and (rice != "WHITE") :
            rice = None
            print("Invalid Input \n\n")

    if(rice == "WHITE"):
        move_block(pub_command, loop_rate, whiteRice_start, food_end, vel, accel)
    elif (rice == "BROWN"):
        move_block(pub_command, loop_rate, brownRice_start, food_end, vel, accel)

    protein = None
    while not protein:
        protein = raw_input("Do you prefer STEAK or CHICKEN: ")
	protein = str(protein)
        if (protein != "STEAK") and (protein != "CHICKEN") :
            protein = None
            print("Invalid Input \n\n")

    if (protein == "STEAK"):
         move_block(pub_command, loop_rate, steak_start, food_end, vel, accel)
    elif (protein == "CHICKEN"):
        move_block(pub_command, loop_rate, chicken_start, food_end, vel, accel)

    cheese = None
    while not cheese:
        cheese = raw_input("Do you want CHEESE in your bowl? [Y/N]: ")
	cheese = str(cheese)
        if (cheese != "Y") and (cheese != "N") :
            cheese = None
            print("Invalid Input \n\n")
    if (cheese == "Y"):
         move_block(pub_command, loop_rate, cheese_start, food_end, vel, accel)

    lettuce = None
    while not lettuce:
        lettuce = raw_input("Do you want lettuce in your bowl? [Y/N]: ")
	lettuce = str(lettuce)
        if (lettuce != "Y") and (lettuce != "N") :
            lettuce = None
            print("Invalid Input \n\n")
    if (lettuce == "Y"):
         move_block(pub_command, loop_rate, lettuce_start, food_end, vel, accel)


    # ========================= Student's code ends here ===========================
    
    rospy.loginfo("ENJOY YOUR MEAL!")
    move_arm(pub_command, loop_rate, bufferPOS, vel, accel)
    print("Use Ctrl+C to exit program")
    while protein:
	move_arm(pub_command, loop_rate, waveLeft,vel,accel)
        move_arm(pub_command, loop_rate, waveRight,vel,accel)
    rospy.spin()

if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
