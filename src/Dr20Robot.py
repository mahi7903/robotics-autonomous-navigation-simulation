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
import numpy as np  # used for the camera data
try:
    import vrepConst as wlvConst
except:
    import simConst as wlvConst

class Dr20Robot_Interface:

    def __init__(self, sim, clientID):
        self._sim = sim
        self.clientID = clientID
        err, self.robHandle = sim.simxGetObjectHandle(clientID, "dr20", sim.simx_opmode_oneshot_wait )
        if err > 0:
            print("*** Error getting robot handle {}".format(err))
        ## attach actuators and sensors
        ## The robot has two senors a front bumper "dr12_bumperForceSensor_" and a camera "Vision_sensor0"
        ## and two actuators left motor "dr12_leftJoint_" and a right motor "dr12_rightJoint_"
        errCode = [0] * 7
        ## attach actuators and sensors
        errCode[0], self.leftMotor = sim.simxGetObjectHandle(clientID, "dr20_leftWheelJoint_", sim.simx_opmode_oneshot_wait )
        errCode[1], self.rightMotor = sim.simxGetObjectHandle(clientID, "dr20_rightWheelJoint_", sim.simx_opmode_oneshot_wait )

        errCode[2], self.infrared1 = sim.simxGetObjectHandle(clientID, "dr20_infraredSensor1_", sim.simx_opmode_oneshot_wait )
        errCode[3], self.infrared2 = sim.simxGetObjectHandle(clientID, "dr20_infraredSensor2_", sim.simx_opmode_oneshot_wait )
        errCode[4], self.infrared5 = sim.simxGetObjectHandle(clientID, "dr20_infraredSensor5_", sim.simx_opmode_oneshot_wait )
        errCode[5], self.infrared6 = sim.simxGetObjectHandle(clientID, "dr20_infraredSensor6_", sim.simx_opmode_oneshot_wait )
        errCode[6], self.ultasonic = sim.simxGetObjectHandle(clientID, "dr20_ultrasonicSensor_", sim.simx_opmode_oneshot_wait )

        if sum(errCode) > 0:
            print("*** Error initialising robot {}".format(errCode))

        ## initialise sensors - needs an initial call to fill the sensor buffer
        for _ in range(10):
            err_code,detectionState,detectedPoint,detectedObjectHandle, detectedSurfaceNormalVector=sim.simxReadProximitySensor(clientID,self.infrared1,sim.simx_opmode_streaming)
            err_code,detectionState,detectedPoint,detectedObjectHandle, detectedSurfaceNormalVector=sim.simxReadProximitySensor(clientID,self.infrared2,sim.simx_opmode_streaming)
            err_code,detectionState,detectedPoint,detectedObjectHandle, detectedSurfaceNormalVector=sim.simxReadProximitySensor(clientID,self.infrared5,sim.simx_opmode_streaming)
            err_code,detectionState,detectedPoint,detectedObjectHandle, detectedSurfaceNormalVector=sim.simxReadProximitySensor(clientID,self.infrared6,sim.simx_opmode_streaming)
            err_code,detectionState,detectedPoint,detectedObjectHandle, detectedSurfaceNormalVector=sim.simxReadProximitySensor(clientID,self.ultasonic,sim.simx_opmode_streaming)
            err_code, property = self._sim.simxGetModelProperty(clientID, self.robHandle, sim.simx_opmode_streaming)

        ## set up some robot constants
        self.v0=0.4 ## *s
        self.wheelDiameter=0.085 ## *s
        self.interWheelDistance=0.254 ## *s
        self.noDetectionDistance=1.0 ## *s
        self.viewSensors = False
        ## Braitenberg weights:
        self.brait_left = [0,-0.5,-1.25,-1,-0.2]

    def rotate_right(self, speed=2.0):
        ## turn the robot right
        self._set_two_motor(speed, -speed)

    def rotate_left(self, speed=2.0):
        ## turn the robot left
        self._set_two_motor(-speed, speed)

    def move_forward(self, speed=2.0):
        ## move the robot forward
        self._set_two_motor(speed, speed)

    def move_backward(self, speed=2.0):
        ## move the robot backwards
        self._set_two_motor(-speed, -speed)

    def _set_two_motor(self, left: float, right: float):
        ## set the left motor to the to the value in left,
        ## the right motor to the value in right
        self._sim.simxSetJointTargetVelocity(self.clientID, self.leftMotor, left, self._sim.simx_opmode_blocking )
        self._sim.simxSetJointTargetVelocity(self.clientID, self.rightMotor, right, self._sim.simx_opmode_blocking )

    def stop(self):
        self._set_two_motor(0.0, 0.0)

    def _readSensor(self, sensorHandle, sensorType):
        if sensorType == wlvConst.sim_object_forcesensor_type:
            result, s, f, t = self._sim.simxReadForceSensor(self.clientID, sensorHandle, self._sim.simx_opmode_blocking)
            return s, f, t
        elif sensorType == wlvConst.sim_object_proximitysensor_type:
            err_code,detectionState,detectedPoint,detectedObjectHandle, detectedSurfaceNormalVecto = self._sim.simxReadProximitySensor(self.clientID, sensorHandle, self._sim.simx_opmode_buffer)
            distance = np.linalg.norm(detectedPoint)
            if err_code == 0 and detectionState == True:
                return wlvConst.simx_return_ok, distance
            return 1, self.noDetectionDistance
        elif sensorType == wlvConst.sim_object_visionsensor_type:
            result, s, auxPackets = self._sim.simxReadVisionSensor(self.clientID, sensorHandle, self._sim.simx_opmode_blocking)
            raw_image = auxPackets[::-1]  ## flip the values to RGB
            backImage = np.array(raw_image, dtype=np.uint8)
            backImage.resize([resolution[0],resolution[0],3])
            image = np.flip(backImage, 0) ## (un)reverse the image, as it is reversed
            return s, image
        else:
            raise Exception('Unknow sensor type {} in _readSensor'.format(sensorType))

    def readSensors(self):
        sensReading = [self.noDetectionDistance,
                       self.noDetectionDistance,
                       self.noDetectionDistance,
                       self.noDetectionDistance,
                       self.noDetectionDistance]
        ## Read the sensors:
        err, dist = self._readSensor(self.infrared5, wlvConst.sim_object_proximitysensor_type)
        sensReading[0] = dist if (err == wlvConst.simx_return_ok) and (dist < self.noDetectionDistance) else self.noDetectionDistance

        err, dist = self._readSensor(self.infrared6, wlvConst.sim_object_proximitysensor_type)
        sensReading[1] = dist if (err == wlvConst.simx_return_ok) and (dist < self.noDetectionDistance) else self.noDetectionDistance

        err, dist = self._readSensor(self.ultasonic, wlvConst.sim_object_proximitysensor_type)
        sensReading[2] = dist if (err == wlvConst.simx_return_ok) and (dist < self.noDetectionDistance) else self.noDetectionDistance

        err, dist = self._readSensor(self.infrared1, wlvConst.sim_object_proximitysensor_type)
        sensReading[3] = dist if (err == wlvConst.simx_return_ok) and (dist < self.noDetectionDistance) else self.noDetectionDistance

        err, dist = self._readSensor(self.infrared2, wlvConst.sim_object_proximitysensor_type)
        sensReading[4] = dist if (err == wlvConst.simx_return_ok) and (dist < self.noDetectionDistance) else self.noDetectionDistance
        return sensReading

    def robotBehaviour(self):
        assert False, "This is the abstract robotBehaviour method, you should inherit and override this method"
