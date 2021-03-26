#!/usr/bin/env python

import sys
import copy
import time
import numpy as np

# For ROS
import rospy
from std_msgs.msg import String
from ur3_driver.msg import command
from ur3_driver.msg import position
from helper import lab_invk

# For Particle Filter
import turtle
import scipy as sp
import scipy.stats as st
import matplotlib.pyplot as plt
from getkey import getkey, keys


################################# ROS SETUP NO NEED TO MODIFY BELOW #################################


SPIN_RATE = 20 
PI = 3.1415926535
current_io_0 = False
current_position_set = False
thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
current_position = [120*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 0*PI/180.0]


"""
Callback function for getting current robot's world coordinate
"""
def coord_callback(msg):

    global blob_center
    blob_center = msg.data


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


"""
Check if the robot move out of the boundry of the Maze
"""
def check_boundary(pos):

    if(pos[0]>=0.5):
        pos[0] = 0.5
    elif(pos[0]<=0.1):
        pos[0] = 0.1
    else:
        pass

    if(pos[1]>=0.5):
        pos[1] = 0.5
    elif(pos[1]<=-0.1):
        pos[1] = -0.1
    else:
        pass    

    return pos


"""
Detect which key has been pressed
"""
def which_key(get_key):

    key = get_key
    key_pressed = None
    if key == keys.UP:
      # print("UP key")
      key_pressed = "3"
    elif key == keys.DOWN:
      # print("DOWN key")
      key_pressed = "1"
    elif key == keys.RIGHT:
      # print("RIGHT key")
      key_pressed ="2"
    elif key == keys.LEFT:
      # print("LEFT key")
      key_pressed = "4"
    else:
      # Handle other text characters
      key_pressed = "Wrong key is pressed!"

    return key_pressed 

################################# ROS SETUP NO NEED TO MODIFY ABOVE #################################


timsSpan = 100
numberOfParticles = 5000

oldpos0 = 0 # x
oldpos1 = 0 # y
 
num_rows = 10  
num_cols = 16   

grid_height = 25
grid_width = 25

grid_height_in_m = 0.025
grid_width_in_m = 0.025

window_height = grid_height*num_rows  # 250
window_width = grid_width*num_cols    # 400

t_step = 0
canmove = 1

xCAM = 0
yCAM = 0

# store coordinate of orange blob
blob_center = None

# end-effector coordinate in world frame
startgridx = 0  # 0 to 15
startgridy = 0  # 0 to 9

gridzero_inRobotWorld_x = 0.3875
gridzero_inRobotWorld_y = -0.0405

# initialize the location of end-effector
new_pos = [gridzero_inRobotWorld_x, gridzero_inRobotWorld_y, 0.035, -45]

"""
class for Maze
"""
class Maze(object):

    def __init__(self,dimension=2, maze = None):
        '''
        maze: 2D numpy array.
        passages are coded as a 4-bit number, with a bit value taking
        0 if there is a wall and 1 if there is no wall.
        The 1s register corresponds with a square's top edge,
        2s register the right edge,
        4s register the bottom edge,
        and 8s register the left edge.
        (numpy array)
        '''
        self.dimension = dimension       # 2
        self.grid_height = grid_height   # 25
        self.grid_width = grid_width     # 25
        self.window = turtle.Screen()
        self.window.setup (width = window_width, height = window_height) # window_width=400, window_height=250

        if maze is not None:
            self.maze = maze
            self.num_rows = maze.shape[0]
            self.num_cols = maze.shape[1]
            self.fix_maze_boundary()
            self.fix_wall_inconsistency()
        else:
            assert num_rows is not None and num_cols is not None, 'Parameters for maze should not be None.'
            self.create_maze(num_rows = num_rows, num_cols = num_cols)

        self.height = self.num_rows * self.grid_height  # 250
        self.width = self.num_cols * self.grid_width    # 400

        self.turtle_registration()

    def turtle_registration(self):
        turtle.register_shape('tri', ((-3, -2), (0, 3), (3, -2), (0, 0)))

    def check_wall_inconsistency(self):
        wall_errors = list()
        # Check vertical walls
        for i in range(self.num_rows):
            for j in range(self.num_cols-1):
                if (self.maze[i,j] & 2 != 0) != (self.maze[i,j+1] & 8 != 0):
                    wall_errors.append(((i,j), 'v'))
        # Check horizonal walls
        for i in range(self.num_rows-1):
            for j in range(self.num_cols):
                if (self.maze[i,j] & 4 != 0) != (self.maze[i+1,j] & 1 != 0):
                    wall_errors.append(((i,j), 'h'))
        return wall_errors

    def fix_wall_inconsistency(self, verbose = True):
        # Whenever there is a wall inconsistency, put a wall there.
        wall_errors = self.check_wall_inconsistency()
        if wall_errors and verbose:
            print('Warning: maze contains wall inconsistency.')
        for (i,j), error in wall_errors:
            if error == 'v':
                self.maze[i,j] |= 2
                self.maze[i,j+1] |= 8
            elif error == 'h':
                self.maze[i,j] |= 4
                self.maze[i+1,j] |= 1
            else:
                raise Exception('Unknown type of wall inconsistency.')
        return

    def fix_maze_boundary(self):
        # Make sure that the maze is bounded.
        for i in range(self.num_rows):
            self.maze[i,0] |= 8
            self.maze[i,-1] |= 2
        for j in range(self.num_cols):
            self.maze[0,j] |= 1
            self.maze[-1,j] |= 4

    def create_maze(self, num_rows, num_cols):

        self.num_rows = num_rows
        self.num_cols = num_cols

        self.maze = np.zeros((num_rows, num_cols), dtype = np.int8)
        self.maze[6,1] = 15
        self.maze[5,1] = 15
        self.maze[4,1] = 15
        self.maze[3,1] = 15
        self.maze[0,2] = 15
        self.maze[8,3] = 15
        self.maze[1,3] = 15
        self.maze[8,4] = 15
        self.maze[2,4] = 15
        self.maze[1,4] = 15
        self.maze[7,6] = 15
        self.maze[5,6] = 15
        self.maze[4,6] = 15
        self.maze[3,6] = 15
        self.maze[6,8] = 15
        self.maze[2,8] = 15
        self.maze[1,8] = 15
        self.maze[7,9] = 15
        self.maze[3,9] = 15
        self.maze[6,10] = 15
        self.maze[5,10] = 15
        self.maze[4,10] = 15
        self.maze[1,10] = 15
        self.maze[9,11] = 15
        self.maze[5,11] = 15
        self.maze[2,11] = 15
        self.maze[3,12] = 15
        self.maze[7,13] = 15
        self.maze[5,14] = 15
        self.maze[6,15] = 15        

        self.fix_maze_boundary()
        self.fix_wall_inconsistency(verbose = False)

    def permissibilities(self, cell): 
        '''
        Check if the directions of a given cell are permissible.
        Return: (down, right, up, left)
        '''
        cell_value = self.maze[cell[0], cell[1]] #(row number, col number)
        return (cell_value & 1 == 0, cell_value & 2 == 0, cell_value & 4 == 0, cell_value & 8 == 0)

    def distance_to_walls(self, coordinates):
        '''
        Measure the distance of coordinates to nearest walls at four directions.
        Return: (up, right, down, left)
        '''
        x, y = coordinates
        
        i = int(y // self.grid_height)
        j = int(x // self.grid_width)
        d1 = y - y // self.grid_height * self.grid_height
        while self.permissibilities(cell = (i,j))[0]:
            i -= 1
            d1 += self.grid_height

        i = int(y // self.grid_height)
        j = int(x // self.grid_width)
        d2 = self.grid_width - (x - x // self.grid_width * self.grid_width)
        while self.permissibilities(cell = (i,j))[1]:
            j += 1
            d2 += self.grid_width

        i = int(y // self.grid_height)
        j = int(x // self.grid_width)
        d3 = self.grid_height - (y - y // self.grid_height * self.grid_height)
        while self.permissibilities(cell = (i,j))[2]:
            i += 1
            d3 += self.grid_height

        i = int(y // self.grid_height)
        j = int(x // self.grid_width)
        d4 = x - x // self.grid_width * self.grid_width
        while self.permissibilities(cell = (i,j))[3]:
            j -= 1
            d4 += self.grid_width

        return [d1, d2, d3, d4]

    def show_maze(self):

        turtle.setworldcoordinates(0, 0, self.width * 1.005, self.height * 1.005)
        wally = turtle.Turtle()
        wally.speed(0)
        wally.width(1.5)
        wally.hideturtle()
        turtle.tracer(0, 0)

        for i in range(self.num_rows):
            for j in range(self.num_cols):
                permissibilities = self.permissibilities(cell = (i,j))
                turtle.up()
                wally.setposition((j * self.grid_width, i * self.grid_height))
                # Set turtle heading orientation
                # 0 - east, 90 - north, 180 - west, 270 - south
                wally.setheading(0)
                if not permissibilities[0]:
                    wally.down()
                else:
                    wally.up()
                wally.forward(self.grid_width)
                wally.setheading(90)
                wally.up()
                if not permissibilities[1]:
                    wally.down()
                else:
                    wally.up()
                wally.forward(self.grid_height)
                wally.setheading(180)
                wally.up()
                if not permissibilities[2]:
                    wally.down()
                else:
                    wally.up()
                wally.forward(self.grid_width)
                wally.setheading(270)
                wally.up()
                if not permissibilities[3]:
                    wally.down()
                else:
                    wally.up()
                wally.forward(self.grid_height)
                wally.up()
        turtle.update()

    def show_valid_particles(self, particles, show_frequency = 1):
        turtle.shape('tri')
        for i, particle in enumerate(particles):
            if i % show_frequency == 0:
                turtle.setposition((particle[1], particle[0]))
                turtle.setheading(90)
                turtle.color('blue')
                turtle.stamp()            
        turtle.update()

    def show_estimated_location(self, estimate):
        y_estimate, x_estimate, heading_estimate= estimate[0], estimate[1], 0
        turtle.color('orange')
        turtle.setposition((x_estimate, y_estimate))
        turtle.setheading(90 - heading_estimate)
        turtle.shape('turtle')
        turtle.stamp()
        turtle.update()

    def clear_objects(self):
        turtle.clearstamps()

    def show_robot_position(self, robotX, robotY, robotHeading=0):
        turtle.color('green')
        turtle.shape('turtle')
        turtle.shapesize(0.7, 0.7)
        turtle.setposition((robotX, robotY))
        turtle.setheading(90 - robotHeading)
        turtle.stamp()
        turtle.update()

    def finish(self):
        turtle.done()
        turtle.exitonclick()

"""
Defulat 2-D model for roomba robot in a room 
"""
class default_2D_Model:

    def __init__(self):

        self.height = num_rows * grid_height     
        self.width = num_cols * grid_width      

        self.grid_height = grid_height
        self.grid_width = grid_width

        self.num_rows = num_rows        
        self.num_cols = num_cols       

        self.y = 12 + grid_height*startgridy
        self.x = 12 + grid_width*startgridx

        self.map = Maze()
        
        self.accuracy = 15 
        self.motionNoise = 20

        self.max = [num_rows*grid_height, num_cols*grid_width]
        self.min = [0, 0]
        self.map.show_maze()


    """
    input: postion [x, y]
    return the map reading, which are the distances to the closest wall on four 
    directions at this position [d1, d2, d3, d4]
    """
    def readingMap(self, position): 
        validPosition = [0,0]
        for i in range(2):
            validPosition[i] = max(int(position[i]), self.min[i])
            validPosition[i] = min(int(position[i]), self.max[i]-1)
            
        reading = self.map.distance_to_walls((validPosition[1], validPosition[0]))
        return reading


    """
    return the sensor reading, which are the distances to the closest 
    walls on four directions [d1, d2, d3, d4]
    """
    def readingSensor(self,x_camera,y_camera):

        global grid_width_in_m
        global grid_height_in_m

        global grid_height
        global grid_width

        xpix = x_camera*(grid_width/grid_width_in_m) + 12
        ypix = y_camera*(grid_height/grid_height_in_m) + 12

        reading = self.map.distance_to_walls((xpix, ypix))

        return reading


    """
    input: the position of the previous particle [x',y'], (optional) the control 
    signal integer currentControl
    return: if the robot is at the position of the previous particle, the current 
    robot position [x,y]
    Control command:0 halt, 1 down, 2 right, 3 up, 4 left
    """
    def simulateNextPosition(self, previousEstimate, currentControl=0): 

        for i in range(2):
            previousEstimate[i] = max(previousEstimate[i], self.min[i])
            previousEstimate[i] = min(previousEstimate[i], self.max[i])
        x, y = previousEstimate[0], previousEstimate[1]
        cellX, cellY = int(x // self.grid_width), int(y // self.grid_height)

        if cellX > 15:
            cellX = 15
        
        if cellY > 9:
            cellY = 9            
        
        permissibilities = self.map.permissibilities((cellY, cellX))  #(down, right, up, left)
        
        if (currentControl == 3 and permissibilities[2]):
            y += self.grid_height
        elif (currentControl == 1 and permissibilities[0]):
            y -= self.grid_height
        elif (currentControl == 2 and permissibilities[1]):
            x += self.grid_width
        elif (currentControl == 4 and permissibilities[3]):
            x -= self.grid_width

        x += np.random.normal(0, self.motionNoise)
        y += np.random.normal(0, self.motionNoise)
        nextEstimate = np.array([x, y])
        
        for i in range(2):
            nextEstimate[i] = max(self.min[i], nextEstimate[i])
            nextEstimate[i] = min(self.max[i], nextEstimate[i])
        
        return nextEstimate


    def run(self, currentControl=0): 

        """
        Input: Control command: 0 halt, 1 down, 2 right, 3 up, 4 left
        Can only move from the center of one cell to the center of one of four neighboring cells
        """

        global canmove
        global blob_center
     
        cellX, cellY = int(self.x // self.grid_width), int(self.y // self.grid_height)
        permissibilities = self.map.permissibilities((cellY, cellX))  

        if (currentControl == 3 and permissibilities[2]):
            self.y += self.grid_height  
            canmove = 1
        elif (currentControl == 1 and permissibilities[0]):
            self.y -= self.grid_height
            canmove = 1
        elif (currentControl == 2 and permissibilities[1]):
            self.x += self.grid_width
            canmove = 1
        elif (currentControl == 4 and permissibilities[3]):
            self.x -= self.grid_width
            canmove = 1
        else: 
            canmove = 0
            
        self.map.show_robot_position(self.x, self.y, 0)

        
    def plotParticles(self, particles):
        """
        Input is 2D python list containing position of all particles: [[x1,y1], [x2,y2], ...]
        """
        self.map.show_valid_particles(particles)


    def plotEstimation(self, estimatePosition):
        """
        Input is the estimated position: [x, y]
        """
        self.map.show_estimated_location(estimatePosition)


    def readMax(self):
        """
        Return the max value at each dimension [maxX, maxY, ...]
        """
        return self.max


    def readMin(self):
        """
        Return the min value at each dimension [minX, minY, ...]
        """
        return self.min


    def readPosition(self):
        """
        Return actual position, can be used for debug
        """
        return (self.x, self.y)


  
class particleFilter:

    def __init__(self, dimension = 2, model = default_2D_Model(), numParticles = numberOfParticles, timeSpan = timsSpan, resamplingNoise = 0.01, positionStd = 5):
        
        self.model = model
        self.numParticles = numParticles 
        self.dimension = dimension 
        self.timeSpan = timeSpan

        self.std = positionStd  
        self.curMax = self.model.readMax()  
        self.curMin = self.model.readMin()  
        self.resNoise = [x*resamplingNoise for x in self.curMax] 

        ############# The initial particles are uniformely distributed #############
        ## TODO: self.particles = ? self.weights = ?
        
        # Generate uniformly distributed variables in x and y direction within [0, 1]
        # Hint: np.random.uniform(0, 1, ...) 
        
        # Spread these generated particles on the maze
        # Hint: use self.curMax, remember X direction: self.curMax[0], Y direction: self.curMax[1]
        # particles should be something like [[x1,y1], [x2,y2], ...]
        
        # Generate weight, initially all the weights for particle should be equal, namely 1/num_of_particles
        # weights should be something like [1/num_of_particles, 1/num_of_particles, 1/num_of_particles, ...]
        
        ###################################################################
        # Students finish the code below

        x = np.random.uniform(0, model.num_cols, numParticles)      # random numbers generated between 0 and num_cols
        y = np.random.uniform(0, model.num_rows, numParticles)      # random numbers generated between 0 and num_rows
        #self.particles = np.column_stack((x, y))        # tall matrix of x,y points
        self.particles = []
        for i in range(0, numParticles):
            self.particles.append([x[i], y[i]])

        #self.weights = np.full((model.num_rows, model.num_cols), 1 / numParticles)      # num_rows by num_cols array, all values are 1/numParicles
        self.weights = np.full((1, numParticles), 1 / numParticles)     # [1/numParticles, 1/numParticles, ...], len = numParticles

        ###################################################################


    def Sample_Motion_Model(self, u_t=0):

        ####### Sample the Motion Model to Propagate the Particles ########
        ## TODO: self.particles = ?
        
        # For each particle in self.particles [[x1,y1], [x2,y2], ...], get the nextEstimate
        # Hint: use self.model.simulateNextPosition(?, u_t)
        # Update self.particles
        
        ###################################################################
         # Students finish the code below
        for i in range(0, self.numParticles):       # simulates next position for each particle
            self.particles[i] = self.model.simulateNextPosition(self.particles[i], u_t)

        ####################################################################


    def Measurement_Model(self, x, y):

        ##################### Measurement Motion Model #####################
        ## TODO: update self.weights, normalized
        
        # Get the sensor measurements for robot's position
        # Hint: use self.model.readingSensor(x_camera,y_camera)
        
        # For each particle in self.particles [[x1,y1], [x2,y2], ...], get the its position
        # Hint: use self.model.readingMap(position)
        
        # Calculate distance between robot's postion and each particle's position
        # Calculate weight for each particle, w_t = exp(-distance**2/(2*self.std))
        
        # Collect all the particles' weights in a list
        # For all the weights of particles, normalized them
        # Hint: pay attention to the case that sum(weights)=0, avoid round-off to zero
        
        # Update self.weights
        
        ####################################################################
        # Students finish the code below

        botPos = self.model.readingSensor(x,y)

        w_t = []
        for i in range(0, self.numParticles):
            partPos = self.model.readingMap(self.particles[i])      # gets position of particle
            distance = np.linalg.norm(np.asarray(botPos) - np.asarray(partPos))
            #distance2 = (partPos[0] - botPos[0]) ** 2 + (partPos[1] - botPos[1]) ** 2       # calculates distance^2 between bot and particle
            w_t.append(np.exp(-distance**2 / (2 * self.std)))     # appends distance to w_t

        if (sum(w_t) == 0):
            normalized = [float(i) / .000000001 for i in w_t]
        else:
            normalized = [float(i) / sum(w_t) for i in w_t]     # normalizes each distance in w_t

        self.weights = normalized       # updates weights in model

        ####################################################################



    def calcPosition(self):

        ############# Calculate the position update estimate ###############
        ## TODO: return a list with two elements [x,y],  estimatePosition
        
        # For all the particles in direction x and y, get one estimated x, and one estimated y
        # Hint: use the normalized weights, self.weights, estimated x, y can not be out of the
        # boundary, use self.curMin, self.curMax to check

        ####################################################################
        # Students finish the code below

        estimatePosition = [0, 0]
        for i in range(0, self.numParticles):
            estimatePosition[0] += self.particles[i][0] * self.weights[i]       # adds weighted value of all x-coords of particles
            estimatePosition[1] += self.particles[i][1] * self.weights[i]       # adds weighted value of all y-coords of particles

        # makes sure no particles go beyond the maximum and minimum of the map
        estimatePosition[0] = min(estimatePosition[0], self.curMax[0])
        estimatePosition[0] = max(estimatePosition[0], self.curMin[0])
        estimatePosition[1] = min(estimatePosition[1], self.curMax[1])
        estimatePosition[1] = max(estimatePosition[1], self.curMin[1])

        return estimatePosition

        ####################################################################


    def resampling(self):

        newParticles = []

        N = len(self.particles)

        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1. # avoid round-off error

        # Resample according to indexes
        # The probability to be selected is related to weight
        for i in range(N):
            randomProb = np.random.uniform()
            index = np.searchsorted(cumulative_sum, randomProb)
            newParticles.append(self.particles[index])

        self.particles = newParticles 


#     # Method 2: Roulette Wheel
#     def resampling(self):
#         newParticles = []
#         N = len(self.particles)
#         index = int(np.random.random() * N)
#         beta = 0
#         mw = np.max(self.weights)
#         for i in range(N):
#             beta += np.random.random() * 2.0 * mw
#             while beta > self.weights[index]:
#                 beta -= self.weights[index]
#                 index = (index + 1) % N
#             newParticles.append(self.particles[index])
#         self.particles = newParticles   

#################################### NO NEED TO MODIFY BELOW ####################################

def blob_center_trans(blob_center_str):
    """
    input: blob_center_str
    """
    global gridzero_inRobotWorld_x
    global gridzero_inRobotWorld_y

    if(len(blob_center_str) == 0):
        x = -1
        y = -1
    else:
        xy_list = blob_center_str.split()
        y = gridzero_inRobotWorld_x - float(xy_list[0])
        x = float(xy_list[1]) - gridzero_inRobotWorld_y

    return (x, y)


def looprun(partfilt,step_sz,cmd,rate):

    global new_pos

    global oldpos0
    global oldpos1

    global t_step
    global blob_center

    global xCAM
    global yCAM

    print(blob_center, "blob_center test!")
    #Control command:0 halt, 1 down, 2 right, 3 up, 4 left
    control = 0

    # If not the initial round, generate new samples
    if (t_step > 0):
        partfilt.Sample_Motion_Model(control)   
    else:
        time.sleep(0.2)
        (xCAM, yCAM) = blob_center_trans(blob_center)

    print(blob_center, "blob_center test! 2")

    # Assign weights to each particles
    partfilt.Measurement_Model(xCAM,yCAM) 

    # Estimate current position
    estimatePosition = partfilt.calcPosition()  

    print('Estimated Position: ' + str(estimatePosition))

    # Resample the particles
    partfilt.resampling()     

    # Plot particles
    partfilt.model.plotParticles(partfilt.particles)

    # Plot estimated position
    partfilt.model.plotEstimation(estimatePosition)

    print('Use the arrow keys to command the robot left, right, forward and backward')
    if (t_step > 0):
        key_result = which_key(getkey())
        if(len(key_result) == 1):
            if(int(key_result) == 3):
                control = 3
                oldpos0 = new_pos[0]
                oldpos1 = new_pos[1]
                new_pos[0] -= step_sz
            elif(int(key_result) == 1):
                control = 1
                oldpos0 = new_pos[0]
                oldpos1 = new_pos[1]
                new_pos[0] += step_sz
            elif(int(key_result) == 2):
                control = 2
                oldpos0 = new_pos[0]
                oldpos1 = new_pos[1]
                new_pos[1] += step_sz
            elif(int(key_result) == 4):
                control = 4             
                oldpos0 = new_pos[0]
                oldpos1 = new_pos[1]
                new_pos[1] -= step_sz
        else:
            control = 0
            print(key_result)


    partfilt.model.map.clear_objects()
                
    partfilt.model.run(control)     

    print('Actual Postions: ' + str(partfilt.model.readPosition))   

    if canmove == 1:
        new_pos = check_boundary(new_pos)
        # print(new_pos)
        new_dest = lab_invk(new_pos[0], new_pos[1], new_pos[2], new_pos[3])
        # print(new_dest)
        move_arm(cmd, rate, new_dest, 4, 4)
        rospy.loginfo("Destination reached!")
    else:
        new_pos[0] = oldpos0  # x
        new_pos[1] = oldpos1  # y

    t_step = t_step + 1

    print('\n\nSleeping ...')

    time.sleep(0.5)

    (xCAM, yCAM) = blob_center_trans(blob_center)

    print('\n\n')


"""
Program run from here
"""
def main():

    global SPIN_RATE
    global new_pos
    global oldpos0
    global oldpos1

    pf = particleFilter(2)

    # Initialize ROS node
    rospy.init_node('lab3MoveNode')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_coord = rospy.Subscriber('/coord_center', String, coord_callback)

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    home_init = lab_invk(new_pos[0],new_pos[1],new_pos[2],new_pos[3])

    rospy.loginfo("Moving robot ...\n")
    move_arm(pub_command, loop_rate, home_init, 4, 4)
    rospy.loginfo("Home initialization finished!\n")
    time.sleep(1)   
    rospy.loginfo("Press direction keys to move the robot!")

    step_size = 0.025

    oldpos0 = new_pos[0]
    oldpos1 = new_pos[1]

    while not rospy.is_shutdown():
        looprun(pf, step_size, pub_command, loop_rate)


if __name__ == '__main__':
    
    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
