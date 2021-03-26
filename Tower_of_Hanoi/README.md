# The Tower of Hanoi
This lab builds off a previous lab where students solved the Tower of Hanoi puzzle using the UR3 robotic arm's teach pendant (which has a Polyscope GUI and an 
instruction-list-type language). In this lab, the UR3 robotic arm is controlled with the ROS package in Python using skeleton code provided by the course. 
The basic concept of the Tower of Hanoi is that a tower of blocks starts in one of three positions and the goal is to move the stack to a different position, with the 
contraints that only one block can be moved at a time, no block that was originally stacked lower than another block can later be stacked on top of that block, 
and there are three locations where a block can be placed.

## Edited Files
The file containing my code can be found in [<code>src/lab2andDriver/lab2pkg_py/scripts/lab2_exec.py</code>](https://github.com/monk200/Robotics_with_UR3_and_Gazebo/blob/main/Tower_of_Hanoi/src/lab2andDriver/lab2pkg_py/scripts/lab2_exec.py)
, marked with comments. Other portions of the code were provided by the course.

## Solution
To solve the puzzle, each of the three potential locations to place a block was labeled. The inital and end positions of the stack are chosen by the user and will be 
labeled as init_pos and end_pos, respectively. The third possible location will be labeled as temp_pos. Then, each block will be reffered to as the top, middle, or bottom 
based on their configuration in the original stack. The pattern of the arm goes as follows:
1. Lift the top block off the stack at init_pos using a suction cup attachment and place it at end_pos
2. Move middle block from init_pos to temp_pos
3. Move top block from end_pos to temp_pos
4. Move the bottom block from init_pos to end_pos
5. Move the top block from temp_pos to init_pos
6. Move the middle block from temp_pos to end_pos
7. Move the top block from init_pos to end_pos

While this is the pattern of movement, the arm also needed to be programmed to position itself directly above each stack before coming down to lift or place a block. 
Without this intermediate step, the arm comes in at an angle and the tool can bump into stacks of blocks or the blocks will slide too much to be able to build a stable
tower based on the given coordinates of where each stack should be. The arm is also programming to start and end at a safe, neutral position well above any of the blocks 
so that it isn't in the way.

## Challenges
Initially there was an issue when manually entering joint angles where the arm would try to spin around to reach them. After a bit of discussion, it turned out that 
the joint angle being entered was mathematically correct but because the angles being entered were close to 300 degrees, the arm would try to rotate a larger distance. 
This was quickly solved by subtracting 360 degrees from each of the joint angles that were roughly around 300 degrees.  

Another roadblock was working with the gripper's callback function, gripper_input_callback(msg). The main cause of the issue was that this was the first lab using ROS and 
working to understand callback functions in general. After making some educated guesses on what part of the input message needed to be assigned to digital_in_0, DIGIN
ended up being the value needed.

