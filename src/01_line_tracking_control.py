#!/usr/bin/env python
# coding: utf-8

# ## Robot programming workshop 03

# <div class="alert alert-block alert-info">For this workshop you will be developing a maze solving robot. You will connect your solution to your CoppeliaSim robot (via your robot wrapper class) and reimplement your line tracer robot with your implementation of the behaviour to solve the maze. Make sure you have save sim.py, simConst.py and remoteApi.dll files to a directory and also downloaded the 03_line_tracker_maze_vrep.ttt to the same location</div>

# <font color="red"><b>Task 1:</b></font> Change directory - to the working directory where you have saved <b>sim.py, simConst.py</b><i>(or vrepConst.py)</i> and <b>remoteApi.dll</b> and load the API library

# In[4]:


import sys, os
desiredPath = r"C:\Users\mahic\Downloads\dir\ROBOTICS WORKBOOKS\Files" # set path to working folder
os.chdir(desiredPath)
import time
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


# <font color="red"><b>Task 2:</b></font> The line tracer robot class. In future workshops this can be imported from a file in the same directory as <b>sim.py, etc</b> but is here for your reference.

# In[5]:


class LineTracerRobot:

    def __init__(self, sim, clientID):
        self._sim = sim
        self.clientID = clientID
        errCode = [0] * 5
        errCode[0], self.leftMotor = sim.simxGetObjectHandle(clientID, "DynamicLeftJoint", sim.simx_opmode_oneshot_wait )
        errCode[1], self.rightMotor = sim.simxGetObjectHandle(clientID, "DynamicRightJoint", sim.simx_opmode_oneshot_wait )
        errCode[2], self.leftSensor = sim.simxGetObjectHandle(clientID, "LeftSensor", sim.simx_opmode_oneshot_wait )
        errCode[3], self.midSensor = sim.simxGetObjectHandle(clientID, "MiddleSensor", sim.simx_opmode_oneshot_wait )
        errCode[4], self.rightSensor = sim.simxGetObjectHandle(clientID, "RightSensor", sim.simx_opmode_oneshot_wait )
        if sum(errCode) > 0:
            print("*** Error initialising robot {}".format(errCode))

    def rotate_right(self, speed=2.0):
        self._set_two_motor(speed, -speed)

    def rotate_left(self, speed=2.0):
        self._set_two_motor(-speed, speed)

    def move_forward(self, speed=2.0):
        self._set_two_motor(speed, speed)

    def move_backward(self, speed=2.0):
        self._set_two_motor(-speed, -speed)

    def _set_two_motor(self, left: float, right: float):
        self._sim.simxSetJointTargetVelocity(self.clientID, self.leftMotor, left, self._sim.simx_opmode_streaming )
        self._sim.simxSetJointTargetVelocity(self.clientID, self.rightMotor, right, self._sim.simx_opmode_streaming )

    def right_sensor(self):
        return not self._sim.simxReadVisionSensor(self.clientID,self.rightSensor,self._sim.simx_opmode_oneshot_wait)[1]

    def mid_sensor(self):
        return not self._sim.simxReadVisionSensor(self.clientID,self.midSensor,self._sim.simx_opmode_oneshot_wait)[1]

    def left_sensor(self):
        return not self._sim.simxReadVisionSensor(self.clientID,self.leftSensor,self._sim.simx_opmode_oneshot_wait)[1]

    def stop(self):
        self._set_two_motor(0.0, 0.0)


# <font color="red"><b>Task 3:</b></font> The robot should move forward until it encouters various points along the maze until it reaches the end where it should stop.

# ![image-3.png](attachment:image-3.png)

# You will have to work out the sensor settings to recognize the details of the various types of junction in the maze. A first of the algorithm for solving the maze could be characterized as follows:
# 
# <code>if At a "Cross":
# 	Go to Left, or
# 	Go to Right, or
# 	Go Straight
# else if At a "T":
# 	Go to Left, or
# 	Go to Right
# else if At a "Right Only":
# 	Go to Right
# else if At a "Left Only":
# 	Go to Left
# else if At "Straight or Left":
# 	Go to Left, or
# 	Go Straight
# else if At "Straight or Right":
# 	Go to Right, or
# 	Go Straight
# else if At a "Dead End":
# 	Go back ("U turn")
# else if At "End of Maze":
# 	Stop</code>
#     
# But to ensure the robot is able to quickly reach the end of the maze, <i>(rather than going round in circles)</i> we can use the <b>"Left-Hand Rule"</b> to solve the maze:
# 
# -	At a "Cross": Go to Left
# -	At a "T": Go to Left
# -	At a "Right Only": Go to Right
# -	At a "Left Only": Go to Left
# -	At a "Straight or Left": Go to Left
# -	At a "Straight or Right": Go Straight
# -	At a "Dead End": Go back ("U turn")
# -	At the "End of Maze": Stop
# 

# <font color="red"><b>Task 4:</b></font> To detect the kind of junction the robot is facing requires moving the robot a small distance forward and depending on what signal is detected on the sensors then deciding what action to take.</i>

# ![image.png](attachment:image.png)

# <i>Hint: One possible strategy is described in the following pseudo code:</i>
# 
# <code>if At a "DEAD END":
# 	Go back ("U turn")
# else if At a "LINE": Run an extra inch
# 	If there is a line: It is a "Cross" ==> Go to LEFT
# 	If There is no line: it is a "T" ==> Go to LEFT
# 	If there is another line: it is the "End of Maze" ==> STOP
# else if At a "RIGHT TURN": Run an extra inch
# 	if there is a line: It is a Straight or Right ==> Go STRAIGHT
# 	If there is no line: it is a Right Only ==> Go to RIGHT
# else if At a "LEFT TURN": Run an extra inch
# 	if there is a line: It is a Straight or LEFT ==> Go to LEFT
# 	If there is no line: it is a LEFT Only ==> Go to LEFT</code>
# 

# <font color="red"><b>Task 5:</b></font> Edit and modify the code below to implement your robot's behaviour. Make sure the V-rep/CoppeliaSim is on, has the 03_line_tracker_maze_vrep.ttt scene loaded and the simulation is running. 

# In[ ]:


print ('Program started')
sim.simxFinish(-1) 

clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5)

if clientID != -1:
    print ('Connected to remote API server')
else:
    print('Connection failed!!')
    sys.exit('Could not connect')


bot = LineTracerRobot(sim, clientID)


def backUp(turnType,sensorReading):
    bot.move_forward(normalSpeed)
    time.sleep(0.8)
    turnstart = time.time()
    exceededlimit = False


    if turnType == "l":
        print("rotating left")
        bot.rotate_left(normalSpeed)
        time.sleep(1)
        sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
        while sensorReading  != [False,True,False]:
            sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
            bot.rotate_left(normalSpeed * 0.15)
            if (time.time() - turnstart > 3):
                    exceededlimit == True
                    break
    
    elif turnType == "r":
        print("rotating right")
        bot.rotate_right(normalSpeed)
        time.sleep(1)

        sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
        while sensorReading  != [False,True,False]:
            sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
            bot.rotate_right(normalSpeed * 0.05)
            if (time.time() - turnstart > 3):
                    exceededlimit == True
                    break
    
    elif turnType == "u":
        print("U turn")
        bot.rotate_left(normalSpeed)
        time.sleep(1)

        sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
        while sensorReading  != [False,True,False]:
            sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
            bot.rotate_left(normalSpeed * 0.15)
            if (time.time() - turnstart > 5):
                    exceededlimit == True
                    break
    
    if  exceededlimit == True:
        print("something went wrong, re-calibrating")
        bot.move_backward(normalSpeed)
        time.sleep(1)
        if turnType == "r":
            print("rotating left")
            bot.rotate_left(normalSpeed)
            time.sleep(1)

            sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
            while sensorReading  != [False,True,False]:
                sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
                bot.rotate_left(normalSpeed * 0.15)
    
        elif turnType == "l":
            print("rotating right")
            bot.rotate_right(normalSpeed)
            time.sleep(1)

            sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
            while sensorReading  != [False,True,False]:
                sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
                bot.rotate_right(normalSpeed * 0.05)
        
        elif turnType == "u":
            print("U turn")
            bot.rotate_right(normalSpeed)
            time.sleep(0.25)

            sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
            while sensorReading  != [False,True,False]:
                sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
                bot.rotate_right(normalSpeed * 0.05)
            bot.stop

        else:
            self.move_forward(8)

def courseCorrect(direction):
    bot.move_backward(normalSpeed)
    time.sleep(1)
    turnstart = time.time()
    exceededlimit = False

    sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
    if direction == "r":    
        while sensorReading  != [False,True,False] and (time.time() - turnstart < 1):
            sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
            bot.rotate_right(normalSpeed/10)
            if (time.time() - turnstart > 1):
                    exceededlimit = True
                    break
    
    elif direction == "l":
        while sensorReading  != [False,True,False] and (time.time() - turnstart < 1):
            sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
            bot.rotate_left(normalSpeed/10)
            if (time.time() - turnstart > 1):
                    exeededlimit = True
                    break
    
    if  exceededlimit == True:
        print("something went wrong, re-calibrating")
        bot.move_backward(normalSpeed)
        time.sleep(1)
        if direction == "l":    
            while sensorReading  != [False,True,False]:
                sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
                bot.rotate_right(normalSpeed/10)
        
        elif direction == "r":
            while sensorReading  != [False,True,False]:
                sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
                bot.rotate_left(5/10)
    bot.stop()


normalSpeed = 0.1
bot.move_forward(0.8)
time.sleep(1)
bot.stop()
InTheMaze = True

startTime=time.time()
while InTheMaze == True:
    bot.move_forward(5)
    sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]


    if sensorReading  == [False,True,False]:
        bot.move_forward(5)
    
    else:
        print("-------------------------------------------")
        print("something spotted")
        time.sleep(1)
        bot.stop
        sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]

        if sensorReading  == [False,False,False]:
            #dead end
            print("dead end!")
            backUp("u",sensorReading)


        elif sensorReading  == [True,True,False]:
            print("something on the left, or off-centre?")
            time.sleep(0.25)

            sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
            if sensorReading  == [False,True,False]:
                #Straight Left !!!!!!!
                print("A left and straight, go left!")
                backUp("l",sensorReading)

            
            elif sensorReading  == [False,False,False]:
                #Left Only
                print("a left turn and nothing ahead, go left!")
                backUp("l",sensorReading)

            
            elif sensorReading  == [True,True,False]:
                print("course correct left!")
                courseCorrect("l")
            else:
                print(sensorReading)
                print("somethings wrong! go back and calibrate")
                bot.move_backward(5)
                time.sleep(0.3)
                sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
                while sensorReading  != [False,True,False]:
                    sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
                    bot.rotate_left(5/10)

        
        elif sensorReading  == [False,True,True]:
            print("something on the right, or off-centre?")
            time.sleep(1)

            sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
            if sensorReading  == [False,True,False]:
                print("A right and straight, keep going forwards!")

            
            elif sensorReading  == [False,False,False]:
                #Right Only
                print("a right turn and nothing ahead, go right!")
                backUp("r",sensorReading)

            
            elif sensorReading  == [False,True,True]:

                print("course correct right!")
                courseCorrect("r")
            else:
                print(sensorReading)
                print("somethings wrong! go back and calibrate")
                bot.move_backward(5)
                time.sleep(1)
                sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
                while sensorReading  != [False,True,False]:
                    sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
                    bot.rotate_right(normalSpeed/10)
                

        elif sensorReading  == [True,True,True]:
            time.sleep(1)

            sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
            if sensorReading  == [True,True,True]:
                print("End reached!")
                InTheMaze = False
        
            elif sensorReading  == [False,True,False]:
                print("Cross! turn left!")
                backUp("l",sensorReading)

            
            elif sensorReading  == [False,False,False]:
                print("T section! turn left!")
                backUp("l",sensorReading)

            else:
                print(sensorReading)
                print("somethings wrong! go back and calibrate")
                bot.move_backward(5)
                time.sleep(0.3)
                bot.rotate_left(5)
                time.sleep(1.5)
                sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
                while sensorReading  != [False,True,False]:
                    sensorReading = [bot.left_sensor(),bot.mid_sensor(),bot.right_sensor()]
                    bot.rotate_left(5 * 0.15)
        
        elif sensorReading  == [False,False,True]:
            courseCorrect("r")
        
        elif sensorReading  == [True,False,False]:
            courseCorrect("l")
        else:
            bot.move_forward(5)
            time.sleep(1)

bot.stop()
time.sleep(0.5)    
sim.simxFinish(-1) 
print("...done")


# <div class="alert alert-block alert-danger">That is the end for this workbook. When you shutdown CoppeliaSim it will offer you the opportunity to save changes, select <b>No</b>. And remember to save your workbook before you shutdown. Next week: we will add some logical structure and OO design to our robot code. </div>

# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:




