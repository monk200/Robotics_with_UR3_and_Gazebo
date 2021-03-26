#!/usr/bin/env python

import copy
import time
import rospy
import numpy as np
from lab2_header import *
import sys

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([141.91, -117.26, 113.7, -85.71, -91.4, 6.77])

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Starts Here ##############

Q11 = np.radians([134.16, -79.48, 132.3, -143.42, -90.01, -0.15])  # top block
Q12 = np.radians([134.16, -70.24, 135.0, -155.35, -89.87, -0.12])  # middle block
Q13 = np.radians([134.16, -60.9, 136.0, -165.7, -89.75, -0.41])  # bottom block

Q21 = np.radians([148.67, -83.93, 138.31, -144.97, -90.16, 14.1])
Q22 = np.radians([148.49, -73.21, 138.72, -155.09, -90.78, 12.99])
Q23 = np.radians([148.66, -62.19, 142.39, -170.8, -89.87, 14.1])

Q31 = np.radians([165.73, -83.02, 136.08, -142.4, -90.3, 30.23])
Q32 = np.radians([165.72, -73.19, 139.02, -155.18, -90.16, 30.21])
Q33 = np.radians([165.72, -62.51, 140.24, -167.07, -90.02, 30.23])

# position well above each stack, ensures arm drops vertically instead of coming in at an angle
Q1_temp = np.radians([134.99, -93.97, 117.96, -113.3, -91.29, -0.38])
Q2_temp = np.radians([148.73, -100.15, 120.53, -108.18, -91.02, 13.56])
Q3_temp = np.radians([166.01, -97.32, 119.65, -109.15, -90.4, 31.13])

### Hint: How can you map this array to the towers?
# [top block, middle block, bottom block, above stack]
Q = [[Q11, Q12, Q13, Q1_temp], \
     [Q21, Q22, Q23, Q2_temp], \
     [Q31, Q32, Q33, Q3_temp]]

############### Your Code Ends Here ###############


thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

gripping = 0

############## Your Code Starts Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""


def gripper_input_callback(msg):
    global digital_in_0

    digital_in_0 = msg.DIGIN


############### Your Code Ends Here ###############


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

    while (at_goal == 0):

        if (abs(thetas[0] - driver_msg.destination[0]) < 0.0005 and \
                abs(thetas[1] - driver_msg.destination[1]) < 0.0005 and \
                abs(thetas[2] - driver_msg.destination[2]) < 0.0005 and \
                abs(thetas[3] - driver_msg.destination[3]) < 0.0005 and \
                abs(thetas[4] - driver_msg.destination[4]) < 0.0005 and \
                abs(thetas[5] - driver_msg.destination[5]) < 0.0005):
            at_goal = 1

        loop_rate.sleep()

        if (spin_count > SPIN_RATE * 5):
            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


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

    while (at_goal == 0):

        if (abs(thetas[0] - driver_msg.destination[0]) < 0.0005 and \
                abs(thetas[1] - driver_msg.destination[1]) < 0.0005 and \
                abs(thetas[2] - driver_msg.destination[2]) < 0.0005 and \
                abs(thetas[3] - driver_msg.destination[3]) < 0.0005 and \
                abs(thetas[4] - driver_msg.destination[4]) < 0.0005 and \
                abs(thetas[5] - driver_msg.destination[5]) < 0.0005):
            at_goal = 1
        # rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if (spin_count > SPIN_RATE * 5):
            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Starts Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q
    global SPIN_RATE
    global gripping

    loop_rate = rospy.Rate(SPIN_RATE)
    ### Hint: Use the Q array to map out your towers by location and height.

    move_arm(pub_cmd, loop_rate, Q[start_loc][3], 4.0, 4.0)  # move above block to be picked up
    move_arm(pub_cmd, loop_rate, Q[start_loc][start_height - 1], 4.0, 4.0)  # touch block to be picked up

    error = gripper(pub_cmd, loop_rate, suction_on)  # suction on
    # Delay to make sure suction cup has grasped the block
    time.sleep(1.0)

    if (digital_in_0 == 0):
        print("Gripper not holding anything")
        # we're supposed to exit
        sys.exit()

    move_arm(pub_cmd, loop_rate, Q[start_loc][3], 4.0, 4.0)  # lift block up
    move_arm(pub_cmd, loop_rate, Q[end_loc][3], 4.0, 4.0)  # move above location to be placed
    move_arm(pub_cmd, loop_rate, Q[end_loc][end_height - 1], 4.0, 4.0)  # place block

    gripper(pub_cmd, loop_rate, suction_off)  # suction off
    # Delay to make sure suction cup has released the block
    time.sleep(1.0)

    move_arm(pub_cmd, loop_rate, Q[end_loc][3], 4.0, 4.0)  # move straight up

    error = 0

    return error


############### Your Code Ends Here ###############


def main():
    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Starts Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function

    sub_gripper_input = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_input_callback)

    # TODO: modify the code below so that program can get user input

    input_init_string = 0
    input_end_string = 0
    init_pos = 0
    temp_pos = 0
    end_pos = 0

    while (not input_init_string):
        input_init_string = raw_input("Enter initial position <Either 1 2 3 or 0 to quit> ")
        print("You entered " + input_init_string + "\n")

        if (int(input_init_string) == 1):
            init_pos = 0

        elif (int(input_init_string) == 2):
            init_pos = 1

        elif (int(input_init_string) == 3):
            init_pos = 2

        elif (int(input_init_string) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")

    while (not input_end_string):
        input_end_string = raw_input("Enter ending position <Either 1 2 3 or 0 to quit> ")
        print("You entered " + input_end_string + "\n")

        if (int(input_end_string) == 1):        # if user typed '1' as the end position
            end_pos = 0                         # set the variable end_pos to be 0 (zero-indexed)
            if (init_pos == 2):
                temp_pos = 1
            else:                               # if init_pos was set to 0, the temp_pos must be 2
                temp_pos = 2
        elif (int(input_end_string) == 2):
            end_pos = 1                         # end_pos is set to 1
            if (init_pos == 0):                 # if init_pos was set to 0, the temp_pos must be 2
                temp_pos = 2
            else:                               # if init_pos was set to 1, the temp_pos must be 0
                temp_pos = 0
        elif (int(input_end_string) == 3):
            end_pos = 2                         # end_pos is set to 2
            if (init_pos == 1):                 # if init_pos was set to 1, the temp_pos must be 0
                temp_pos = 0
            else:                               # if init_pos was set to 0, the temp_pos must be 1
                temp_pos = 1
        elif (int(input_end_string) == 0):      # if the user types '0', quit the program
            print("Quitting... ")
            sys.exit()
        else:                                   # the user should input one of the selected values
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")

    ############### Your Code Ends Here ###############

    # Check if ROS is ready for operation
    while (rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Starts Here ##############
    # TODO: modify the code so that UR3 can move a tower to a new location according to user input

    move_arm(pub_command, loop_rate, home, 4.0, 4.0)        # start at a safe position
    move_block(pub_command, loop_rate, init_pos, 1, end_pos, 3)     # top block to end_pos
    move_block(pub_command, loop_rate, init_pos, 2, temp_pos, 3)    # middle block to temp_pos
    move_block(pub_command, loop_rate, end_pos, 3, temp_pos, 2)     # top block to temp_pos
    move_block(pub_command, loop_rate, init_pos, 3, end_pos, 3)     # bottom block to end_pos
    move_block(pub_command, loop_rate, temp_pos, 2, init_pos, 3)    # top block to init_pos
    move_block(pub_command, loop_rate, temp_pos, 3, end_pos, 2)     # middle block to end_pos
    move_block(pub_command, loop_rate, init_pos, 3, end_pos, 1)     # top block to end_pos
    move_arm(pub_command, loop_rate, home, 4.0, 4.0)        # end at safe position

############### Your Code Ends Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass









