#!/usr/bin/env python
# coding: utf-8

# ### Particle Filter for ECE470 Spring 2020

# In[ ]:


import time
import turtle
import numpy as np
import scipy as sp
import scipy.stats as st
import matplotlib.pyplot as plt


# In[ ]:


timsSpan = 100                              # Set a time limit for simulation, will stop after 100 steps. 
num_rows = 10 
num_cols = 16  
grid_height = 25
grid_width = 25
numberOfParticles = 5000
window_height = grid_height*num_rows
window_width = grid_width*num_cols


# #### In the 2D example, a robot has 3 parameters: x, y, heading(in degree, ranging from 0 to 360). Here (0,0) is at the left bottom corner. The robot has 4 Lidars to measure the distance to walls in four directions (down, right, up, left). The Lidar directions are not related to robot heading. We are going to use turtle grapics package to draw the maze. Notice that when you want to stop the simulation, you need to close the turtle graphic window first, otherwise the python kernel will got stuck.  

# In[ ]:


class Maze(object):
    
    def __init__(self,dimension=2, maze = None):
        '''
        maze: 2D numpy array.
        Passages are coded as a 4-bit number, with a bit value taking
        0 if there is a wall and 1 if there is no wall.
        The 1s register corresponds with a square's top edge,
        2s register the right edge,
        4s register the bottom edge,
        and 8s register the left edge.
        (numpy array)
        '''
        self.dimension = dimension
        self.grid_height = grid_height
        self.grid_width = grid_width
        self.window = turtle.Screen()
        self.window.setup (width = window_width, height = window_height)

        if maze is not None:
            self.maze = maze
            self.num_rows = maze.shape[0]
            self.num_cols = maze.shape[1]
            self.fix_maze_boundary()
            self.fix_wall_inconsistency()
        else:
            assert num_rows is not None and num_cols is not None, 'Parameters for fixed maze should not be None.'
            self.fixed_maze(num_rows = num_rows, num_cols = num_cols)

        self.height = self.num_rows * self.grid_height
        self.width = self.num_cols * self.grid_width
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
        '''
        Whenever there is a wall inconsistency, put a wall there.
        '''
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
        '''
        Make sure that the maze is bounded.
        '''
        for i in range(self.num_rows):
            self.maze[i,0] |= 8
            self.maze[i,-1] |= 2
        for j in range(self.num_cols):
            self.maze[0,j] |= 1
            self.maze[-1,j] |= 4

            
    def fixed_maze(self, num_rows, num_cols):
        self.num_rows = num_rows   # 10
        self.num_cols = num_cols   # 16
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


    def permissibilities(self, cell):#(row number, col number)
        '''
        Check if the directions of a given cell are permissible.
        Return: (down, right, up, left)
        '''
        cell_value = self.maze[cell[0], cell[1]]
        return (cell_value & 1 == 0, cell_value & 2 == 0, cell_value & 4 == 0, cell_value & 8 == 0)

    
    def distance_to_walls(self, coordinates):
        '''
        Measure the distance of coordinates to nearest walls at four directions.
        Return: (down, right, up, left)
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
                turtle.setposition((particle[0], particle[1]))
                turtle.setheading(90)
                turtle.color('blue')
                turtle.stamp()            
        turtle.update()
        
        
    def show_estimated_location(self, estimate):
        y_estimate, x_estimate, heading_estimate= estimate[1], estimate[0], 0
        turtle.color('orange') # estimated position
        turtle.setposition((x_estimate, y_estimate))
        turtle.setheading(90 - heading_estimate)
        turtle.shape('turtle')
        turtle.stamp()
        turtle.update()

        
    def clear_objects(self):
        turtle.clearstamps()
        
        
    def show_robot_position(self, robotX, robotY, robotHeading=0):
        turtle.color('green') # ground true
        turtle.shape('turtle')
        turtle.shapesize(0.7, 0.7)
        turtle.setposition((robotX, robotY))
        turtle.setheading(90 - robotHeading)
        turtle.stamp()
        turtle.update()
    
    
    def finish(self):
        turtle.done()
        turtle.exitonclick()


# #### Defulat 2-D model for roomba robot in a room

# In[ ]:


class default_2D_Model:
    
    def __init__(self):
        
        self.height = num_rows * grid_height
        self.width = num_cols * grid_width
        self.grid_height = grid_height
        self.grid_width = grid_width
        self.num_rows = num_rows
        self.num_cols = num_cols
        self.x = 12
        self.y = 12
        self.map = Maze()
        self.accuracy = 15 # std: sigma
        self.motionNoise = 20
        
        self.max = [num_cols*grid_width-1, num_rows*grid_height-1]  
        self.min = [0, 0]
        
        self.map.show_maze()
        

    def readingMap(self, position): 
        """
        Input a postion: [x, y]
        Return the map reading, the distances to the closest wall on four directions at this position, [d1, d2, d3, d4]
        
        """
        validPosition = [0,0]
        for i in range(2):
            validPosition[i] = max(int(position[i]), self.min[i])
            validPosition[i] = min(int(position[i]), self.max[i])
        # (down, right, up, left)
        reading = self.map.distance_to_walls((validPosition[0], validPosition[1]))
        return reading
    

    def readingSensor(self):
        """
        Return the robot's sensor reading, the distances to the closest walls on four directions, [d1, d2, d3, d4]
        """
        # (down, right, up, left)
        reading = self.map.distance_to_walls((self.x, self.y))
        for i in range(len(reading)):
            reading[i] += np.random.normal(0, self.accuracy) # Add some noise
        # reading = (down+noise, right+noise, up+noise, left+noise)
        return reading
    
    
    def simulateNextPosition(self, previousEstimate, currentControl=0):   
        """
        Control command: 0 halt, 1 down, 2 right, 3 up, 4 left
        Input: the position of the previous particle [x',y'], (optional) the control signal integer currentControl
        Return: the robot next position [x,y]
        
        """
        for i in range(2):
            previousEstimate[i] = max(previousEstimate[i], self.min[i])
            previousEstimate[i] = min(previousEstimate[i], self.max[i])
            
        x, y = previousEstimate[0], previousEstimate[1]
        
        cellX, cellY = int(x // self.grid_width), int(y // self.grid_height)
        
        if cellX > 15: 
             cellX = 15
                
        if cellY > 9:  
             cellY = 9
                
        permissibilities = self.map.permissibilities((cellY, cellX))  # (down, right, up, left)
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
        cellX, cellY = int(self.x // self.grid_width), int(self.y // self.grid_height)
        
        permissibilities = self.map.permissibilities((cellY, cellX))  # (down, right, up, left)

        if (currentControl == 3 and permissibilities[2]):
            self.y += self.grid_height
        elif (currentControl == 1 and permissibilities[0]):
            self.y -= self.grid_height
        elif (currentControl == 2 and permissibilities[1]):
            self.x += self.grid_width
        elif (currentControl == 4 and permissibilities[3]):
            self.x -= self.grid_width
            
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


# In[ ]:


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
        
        # Generate uniformly distributed particles in x and y direction within [0, 1]
        # Hint: np.random.uniform(0, 1, ...)
        
        # Spread these generated particles on the maze
        # Hint: use self.curMax, remember X direction: self.curMax[0], Y direction: self.curMax[1]
        # particles should be saved in something similar to [[x1,y1], [x2,y2], ...]

        # Generate weight, initially all the weights for the particle should be equal, namely 1/num_of_particles
        # weights should be something like [1/num_of_particles, 1/num_of_particles, 1/num_of_particles, ...]

        ## Your Code start from here

        x = np.random.uniform(0, model.num_cols, numParticles)      # random numbers generated between 0 and num_cols
        y = np.random.uniform(0, model.num_rows, numParticles)      # random numbers generated between 0 and num_rows
        #self.particles = np.column_stack((x, y))        # tall matrix of x,y points
        self.particles = []
        for i in range(0, numParticles):
            self.particles.append([x[i], y[i]])

        #self.weights = np.full((model.num_rows, model.num_cols), 1 / numParticles)      # num_rows by num_cols array, all values are 1/numParicles
        self.weights = np.full((1, numParticles), 1 / numParticles)     # [1/numParticles, 1/numParticles, ...], len = numParticles

        ################################### End ###################################
    
        
    def Sample_Motion_Model(self, u_t=0):
        
        ########## Sample the Motion Model to Propagate the Particles ###########
        ## TODO: self.particles = ?
        
        # For each particle in self.particles [[x1,y1], [x2,y2], ...], get the nextEstimate
        # Hint: use self.model.simulateNextPosition(?, u_t)
        # Update self.particles
        
        ##########################################################################
        ## Your Code start from here

        for i in range(0, self.numParticles):       # simulates next position for each particle
            self.particles[i] = self.model.simulateNextPosition(self.particles[i], u_t)

        ################################### End ###################################
    
    
    def Measurement_Model(self):
        
        ##################### Measurement Motion Model #####################
        ## TODO: update self.weights, normalized
        
        # Get the sensor measurements for robot's position
        # Hint: use self.model.readingSensor()
        
        # For each particle in self.particles [[x1,y1], [x2,y2], ...], get its position
        # Hint: use self.model.readingMap()

        # Calculate distance between robot's postion and each particle's position
        # Calculate weight for each particle, w_t = exp(-distance**2/(2*self.std))
        
        # Collect all the particles' weights in a list
        # For all the weights of each particle, normalized them
        # Hint: pay attention to the case that sum(weights)=0, avoid round-off to zero

        # Update self.weights
        
        ## Your Code start from here

        botPos = self.model.readingSensor()

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

        ################################### End ###################################
    
    
    def calcPosition(self):
        
        ############# Calculate the position update estimate ###############
        ## TODO: return a list with two elements [x,y],  estimatePosition
        
        # For all the particles in direction x and y, get one estimated x, and one estimated y
        # Hint: use the normalized weights, self.weights, estimated x, y can not be out of the
        # boundary of the maze, use self.curMin, self.curMax to check
        
        ## Your Code start from here

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

        ################################### End ###################################
    
    ## Method 1
#    def resampling(self):
#
#        ## TODO: Comment the code below:
#        """
#        Universal sampling
#        Randomly chooses particles that will get resampled among all the particles available
#        """
#        newParticles = []
#        N = len(self.particles)
#        cumulative_sum = np.cumsum(self.weights)
#        cumulative_sum[-1] = 1
#        for i in range(N):
#            randomProb = np.random.uniform()
#            index = np.searchsorted(cumulative_sum, randomProb)
#            newParticles.append(self.particles[index])
#        self.particles = newParticles
        
        
    ## Method 2: Roulette Wheel
    def resampling(self):

     ## TODO: Comment the code below:
     """
     Roulette Sampling
     Prioritizes resampling particles with higher weights, but is still fairly random
     """
     newParticles = []
     N = len(self.particles)        # N = numParticles
     index = int(np.random.random() * N)
     beta = 0
     mw = np.max(self.weights)      # mw = highest weight
     for i in range(N):
         beta += np.random.random() * 2.0 * mw
         while beta > self.weights[index]:      # while beta > whatever weight we are at
             beta -= self.weights[index]        # beta -= this weight
             index = (index + 1) % N        # new index
         newParticles.append(self.particles[index])     # new particles has particle at current index
     self.particles = newParticles

    
    def runParticleFilter(self):

        # Control command: 0 halt, 1 down, 2 right, 3 up, 4 left        
        # Hard code your control input in this array
        controls = [2,3,2,2,3,2,1,2,2,2,2,2,2,3,3,2,2,3,3,3,2,3,3,2,2,3,0]
        control = 0
        
        time.sleep(1)

        for t in range(self.timeSpan - 1):

            if controls:
                control = controls.pop()
            else: 
                control = 0

            if (t > 0):
                self.Sample_Motion_Model(control)

            self.Measurement_Model()
    
            estimatePosition = self.calcPosition()

            self.resampling()

            self.model.plotParticles(self.particles)

            self.model.plotEstimation(estimatePosition)

            self.model.map.clear_objects()

            self.model.run(control)     


# In[ ]:


pf = particleFilter(2)
estimates, actual = pf.runParticleFilter() 


# In[ ]:





# In[ ]:




