#!/usr/bin/env python
# coding: utf-8

# ## Robot programming workshop 06
# 
# <div class="alert alert-block alert-info">For this workshop you will be using a new robot, the dr20, and a new environment, a short 'maze' with a curvy path to get gent out. The dr20 robot had a richer array of sensors than the line-following robot, with <b>4 left and right facing ultrasonic sensors, a front facing laser sensor and a front facing camera</b>. The goal of this workshop is to interface with the robot class and write some code to pilot the robot out of the 'maze'.  At the moment there is no definite end to the 'maze' (emerging from the walled part) so you will have to see if there is any change to the sensors to see if there is a way of detecting this state.  Make sure you have save sim.py, simConst.py (or vrepConst.py) and remoteApi.dll files to a directory and also downloaded the short_path_vrep.ttt to your working folder (a known location)</div>

# ![image-3.png](attachment:image-3.png)

# <font color="red"><b>Task 1:</b></font> Download the <b>short_path_vrep.ttt</b> to a working directory where you have saved sim.py, simConst.py (or vrepConst.py) and remoteApi.dll and change your path to that directory.  

# In[1]:


import os
desiredPath = r"C:\\Users\\mahic\\Downloads\\dir" # "C:/HOME/work/year-21-22/5CS036-sem2/SLAM/Slam-With-V-Rep" # set this path to your working folder
os.chdir(desiredPath)
print("Current working directory: {}".format(os.getcwd()))   # sanity check

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import sys


# <font color="red"><b>Task 2:</b></font> Load the other libraries we are going to use for this workshop.  

# In[2]:


from Dr20Robot import Dr20Robot_Interface

class Dr20Robot(Dr20Robot_Interface):

    def __init__(self, sim, clientID):
        Dr20Robot_Interface.__init__(self, sim, clientID)

    def behaviourTree(self):
        assert False, "Abstract, this method (behaviourTree) should be overridden" 

    def robotBehaviour(self):
        ##    print(unicode_tree(root))
        # Run the root until it returns true
        self.behaviour_tree.run()


# ### Behaviour tree engine
# 
# <font color="red"><b>Task 3:</b></font> These classes are the core components of the behaviour tree, they define the structures that the engine is built upon. <b>Note:</b> the interface between the behaviour tree and the Robot needs a scratch pad/buffer/database by which to pass values which is the purpose of the WhiteBoardStatus class.

# In[3]:


class WhiteBoardStatus:
    def __init__(self, bot):
        self.robot  = bot    # So the BT can communicate with the robot
        self.onLine = False  # Open door?
        self.sensorReading = [None] * 5 # Left, Middle, Right
        self.behaviour = None
        self.startTime = 0
        
# Base class for a leaf of a Tree Behaviour
class Leaf:
    def __init__(self):
        pass

    def run(self):
        assert Flase, "Abstract, this method (run) should be overridden"

class Behaviour(Leaf):
    """
   Base class for a node of a Tree Behavior
    """
    def __init__(self):
        Leaf.__init__(self)
#        super(Leaf, self).__init__()
        self._children = []

    def run(self):
        pass

    def add_child(self, c):
        self._children.append(c)

    def add_children(self, cs):
        self._children.extend(cs)
        

class Status:
    FAILURE = 0
    SUCCESS = 1
    RUNNING = 2
    STOP    = 4
    
#def done(tree):
#    if tree.root.status == Status.SUCCESS:
#        raise Exception("done")


# <font color="red"><b>Task 4:</b></font> The actual behaviour tree engine, this contains the classes for selectors, sequence, decorators, etc. Note: there are no concurrent/parallel variants for this implementation. 

# In[4]:


import random

class Selector(Behaviour):
    """
    Implementation of the Selector
    """
    def __init__(self):
        Behaviour.__init__(self)
#        super(Selector, self).__init__()

    def run(self):
        for c in self._children:
            status = c.run()
            while status == Status.RUNNING:
                status = c.run()
            if status == Status.SUCCESS:
                return Status.SUCCESS
        return Status.FAILURE


class Sequence(Behaviour):
    """
    Implementation of the Sequence
    """
    def __init__(self):
        Behaviour.__init__(self)
#        super(Sequence, self).__init__()

    def run(self):
        for c in self._children:
            status = c.run()
            while status == Status.RUNNING:
                status = c.run()
            if status == Status.FAILURE:
                return Status.FAILURE
        return Status.SUCCESS

class NDSelector(Selector):
    """
    Implementazione di un Selector
    """
    def __init__(self):
        Selector.__init__(self)

    def add_child(self, c):
        self._children.append(c)
        random.shuffle(self._children)


class NDSequence(Sequence):
    """
    Implementazione di un Sequence
    """
    def __init__(self):
        Sequence.__init__(self)

    def add_child(self, c):
        self._children.append(c)
        random.shuffle(self._children)

class Decorator(Leaf):
    # Implementation of the Decorator
    def __init__(self):
        Leaf.__init__(self)
        self._child = None

    def add_child(self, c):
        ## if isinstance(c, Leaf):
        self._child = c

    # This method should be overridden
    def run(self):
        return self._child.run()


# <font color="red"><b>Task 5:</b></font> These are the specific behaviours for our robot working in a v-rep/coppiliasim <i>It is complete enough for this weeks workshop demonstration but may need to be edited, for example, to use the full set of sensors on the robot, to complete the workshop task</i>. 

# In[5]:


class Stop(Behaviour):
    def __init__(self, status):
        super().__init__()
        self._status = status

    def run(self):
        print('--Stop')
        self._status.robot.stop()
        self._status.behaviour = None
        return Status.SUCCESS
    
class TreeTree(Behaviour):
    def __init__(self, status):
        super().__init__()
        self._status = status
        

    def run(self):      ## This simulates a blocking action
        status = Status.RUNNING
        sensReading = self._status.robot.readSensors()
        self._status.sensorReading = sensReading
        
        first_left = sensReading[0]
        back_left = sensReading[1]
        middle_sensor= sensReading[2]
        first_right = sensReading[3]
        back_left = sensReading[4]
        
        if middle_sensor < 0.7:
            if first_left < first_right:
                self._status.robot.rotate_right(1.0)
                self._status.behaviour = "TurnRight"
                status = Status.RUNNING
                
                
            if first_right < first_left:
                self._status.robot.rotate_left(1.0)
                self._status.behaviour = "TurnLeft"
                status = Status.RUNNING
        else:
            self._status.robot.move_forward(6)
            self._status.behaviour = "Forward"
            status = Status.RUNNING
        return status
            

    

class TurnLeft(Behaviour):
    def __init__(self, status):
        super().__init__()
        self._status = status

    def run(self):      ## This simulates a blocking action
        status = Status.RUNNING
        #if self.sensorReading[]:
        if self._status.behaviour == None:
            print('--TurnLeft')
            self._status.robot.rotate_left(1.5)
            self._status.startTime=time.time()
            self._status.behaviour = "TurnLeft"
            
        elif self._status.behaviour == "TurnLeft" and time.time()-self._status.startTime > 2:
            self._status.robot.stop()
            self._status.behaviour = None
            status = Status.SUCCESS
        #else:
        return status

class TurnRight(Behaviour):
    def __init__(self, status):
        super().__init__()
        self._status = status

    def run(self):      ## This simulates a blocking action
        status = Status.RUNNING
        if self._status.behaviour == None:
            print('--TurnRight')
            self._status.robot.rotate_right(1.5)
            self._status.startTime=time.time()
            self._status.behaviour = "TurnRight"
        elif self._status.behaviour == "TurnRight" and time.time()-self._status.startTime > 2:
            self._status.robot.stop()
            self._status.behaviour = None
            status = Status.SUCCESS
        return status

class Forward(Behaviour):
    def __init__(self, status):
        super().__init__()
        self._status = status

    def run(self):
        #print('--Forward')
        status = Status.RUNNING
        self._status.robot.move_forward(2.0)
        status = Status.SUCCESS
        return status

class ReadSensors(Behaviour):
    def __init__(self, status):
        super().__init__()
        self._status = status

    def run(self):
        self.sensorReading = self._status.robot.readSensors()    
        #pass   ## This need to be completed to Read the sensors and print the results
        print(self.sensorReading)
        status = Status.SUCCESS
        return status

# The following classes are decorators where we can specify custom behaviours
class TimeOut(Decorator):
    def __init__(self, child, duration=0):
        Decorator.__init__(self)
        self.add_child(child)
        self.delay = duration

    def run(self):
        status = self._child.run()
        if status == Status.SUCCESS:
            time.sleep(self.delay)
        return Status.SUCCESS

class DoTwice(Decorator):
    def __init__(self, child):
        Decorator.__init__(self)
        self.count = 0
        self.add_child(child)

    def run(self):
        if self.count < 2:
            self.count += 1
            self._child.run()
            return Status.RUNNING
        self.count = 0
        return Status.SUCCESS

    
class DoUntilClose(Decorator):
    def __init__(self, status, child, distance=0.0):
        Decorator.__init__(self)
        self._status = status
        self.add_child(child)
        self.distance = distance
        self.FixedDelay = 0.5

    def run(self):
        sensorReading = self._status.robot.readSensors()
        if sensorReading[2] < self.distance:
            return Status.SUCCESS
        self._child.run()
        time.sleep(self.FixedDelay)
        return Status.RUNNING


# <font color="red"><b>Task 6:</b></font> Again I am using OO design here (inheritance and overriding) to expose the behaviourTree for modification. But it is this code than finally marries together the Robot control and the behaviour tree. <b>Note: the behaviour tree does nothing useful and you will have to implement your own to complete the workshop task</b>

# In[6]:


class BTRobot(Dr20Robot):
    def __init__(self, sim, clientID):
        Dr20Robot.__init__(self, sim, clientID)
        self.behaviour_tree = self.behaviourTree()
        
    def behaviourTree(self):
        ## self puts the bot into the whiteboard
        status = WhiteBoardStatus(self)

        # Create Actions Note these are the only nodes that interact with the world
        turnRight = TurnRight(status)
        turnLeft =  TurnLeft(status)
        forward =  Forward(status)
        scan = ReadSensors(status)
        stop = Stop(status)
        
        rightTwice = DoTwice(turnRight)
        getClose = DoUntilClose(status, forward, 0.5)
        duration = TimeOut(scan,duration=0.5)

        ## Define the functional nodes
        start = NDSelector()
        root = Sequence()
        
        tree = TreeTree(status)

        ## Assemble the tree
        start.add_children([tree])
        root.add_children([start,tree])

        return root 


# <font color="red"><b>Task 7:</b></font> Implements our interface to v-rep/Coppeliasim and creates the behaviour tree robot ready for running in the simulator. 

# In[7]:


def shortPathMain(viewSensors=False, iterations = 15):
    print ('Program started')
    sim.simxFinish(-1) # just in case, close all opened connections

    clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim

    if clientID != -1:
        print ('Connected to remote API server')
    else:
        print('Connection failed!!')
        sys.exit('Could not connect')

    ## Create the robot object
    bot = BTRobot(sim, clientID)
    bot.viewSensors = viewSensors  # yeah - setters are for Java jockeys!!

    ## Run the robot loop
    for _ in range(iterations):
        bot.robotBehaviour()
        time.sleep(1.0)

    ## Stop the robot
    bot.stop()
    time.sleep(0.5)    ## delay to execute the command
    sim.simxFinish(-1) ## just in case, close all opened connections
    print("...done")


# Remember to <b>run the v-rep/CoppeliaSim scene first</b> and then run the cell below.

# In[ ]:


viewSensors = False ## I don't think I have this part working as yet!!!
try:
    shortPathMain(viewSensors)
except Exception as ex:
    print("Exception was: {}".format(ex))
    sim.simxFinish(-1) ## just in case, close all opened connections
    print("...uh oh, something went wrong!!!")


# <font color="red"><b>Task 8:</b></font> As you can see the robot behaviour is random aand rubbish. So, can you devise and code a behaviour tree strategy that will allow the robot to progress through the 'maze'. Your strategy should favour moving forward if there is no obstacle in front of the robot or turning towards a space with the greatest distance if the robot is surrounded by walls. <i>Hint: 1) Examination of the sensor reading values may allow you a good idea of what the robot sees.</i> 

# <div class="alert alert-block alert-danger">That is the end for this workbook. When you shutdown CoppeliaSim it will offer you the opportunity to save changes, select <b>No</b>. And remember to save your workbook before you shutdown. Next week: we will look at adding even more intelligent behaviour to our robot design. </div>
