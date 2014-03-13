import numpy as np
import serial
import threading
import time 
import re

class Servo:
    def __init__(self, ind):
        self.mInd = ind
    def getInd(self):
        return self.mInd

SERVO_YAW = Servo(0)
SERVO_PITCH = Servo(1) 

COMMAND_COMPLETED = 'command completed'
COMMAND_STARTED = 'command started'
SERVO_SPEED_SET = 'servo speed set' 
reCommandCompleted = re.compile(r'Command completed (\d+)')
reCommandStarted = re.compile(r'Command started (\d+)')
reServoSpeedSet = re.compile(r'Servo speed set to')


class CommandCompletedListener:
    def commandCompleted(self, velocities, pitchYawRoll, t):
        pass

class Trackball():
    
    bufferedPitchYawRoll = []
    bufferedVelocities = []

    class TrackballThread(threading.Thread):
        def __init__(self, parent):
            threading.Thread.__init__(self)
            self.parent = parent

        def run(self):
            self.parent._start();

    thread = None

    sensorNum = 3
    
    def __init__(self, zRotAngle = 0):
        # Default mbed serial, non-blocking
        self.zRotAngle = zRotAngle
        self.serial = serial.Serial("/dev/ttyACM0", timeout=0)
        self.lines = []
        self.incomplete_line = None
        self.bufferedLines = []

        self.topOfSphere = np.array([0, 0, 1])

        self.sensor1Pos = np.array([np.sqrt(3)/2, 0, -1/2.])
        self.sensor2Pos = np.array([-np.sqrt(3)/4, -3/4., -1/2.])
        self.sensor3Pos = np.array([-np.sqrt(3)/4, 3/4., -1/2.])
        self.initSVD(zRotAngle)
        self.dataLock = threading.Lock()
        self.commandCompletedListener = None
        self.processedLines = []

    def initSVD(self, zRotAngle):
        x1, y1, z1 = rotateDeg(self.sensor1Pos, zRotAngle)
        x2, y2, z2 = rotateDeg(self.sensor2Pos, zRotAngle)
        x3, y3, z3 = rotateDeg(self.sensor3Pos, zRotAngle)
        self.A = np.array([0, z1, -y1, -z1, 0, x1, y1, -x1, 0, 0, z2, -y2, -z2, 0, x2, y2, -x2, 0, 0, z3, -y3, -z3, 0, x3, y3, -x3, 0]).reshape([9,3])
        self.A2 = np.array([0, z1, -y1, -z1, 0, x1, y1, -x1, 0, 0, z2, -y2, -z2,
                            0, x2, y2, -x2, 0]).reshape([6,3])
        self.U, self.s, self.V = np.linalg.svd(self.A)
        self.U2, self.s2, self.V2 = np.linalg.svd(self.A2)
        inv = lambda i: 0 if i == 0 else 1./i
        self.sInv = [inv(i) for i in self.s]
        self.sInv2 = [inv(i) for i in self.s2]


    def start(self):
        if (self.thread != None):
            print "Already started"
            return

        self.thread = self.TrackballThread(self)
        self.thread.start()
        print "Started"

    def _start(self):
        self.running = True 
        self._clearBuffers()
        
        #A, U, sInv, V = self.A, self.U, self.sInv, self.V
        #zRotAngle = self.zRotAngle

        while (self.running):
            A, U, sInv, V = self.A, self.U, self.sInv, self.V
            zRotAngle = self.zRotAngle


            velocities = self._processInput()
            if velocities == None:
                continue
            elif len(velocities) == 2:
                t = velocities[1]
                if velocities[0] == COMMAND_COMPLETED:
                    if self.commandCompletedListener != None:
                        self.commandCompletedListener.commandCompleted(self.bufferedVelocities,
                                                                       self.bufferedPitchYawRoll,t)
                    continue
                elif velocities[0] == COMMAND_STARTED:
                     if self.commandCompletedListener != None:
                        self.commandCompletedListener.commandStarted(t)
                     continue
                    
            elif velocities == SERVO_SPEED_SET:
                if self.commandCompletedListener != None:
                    self.commandCompletedListener.commandCompleted(self.bufferedVelocities,
                                                                   self.bufferedPitchYawRoll,
                                                                  -1)
                continue

            x1, y1, x2, y2, x3, y3 = velocities

            self.dataLock.acquire()

            wVx1, wVy1, wVz1 = (y2/2., -x2, y2*np.sqrt(3)/2.)
            wVx2, wVy2, wVz2 = (y3/2., -x3, y3*np.sqrt(3)/2.)
            wVx3, wVy3, wVz3 = (y1/2., -x1, y1*np.sqrt(3)/2.)

            wVx2, wVy2, wVz2 = rotateDeg([wVx2, wVy2, wVz2], -120 + zRotAngle)
            wVx3, wVy3, wVz3 = rotateDeg([wVx3, wVy3, wVz3], 120 + zRotAngle)

            if self.sensorNum == 3:

                b = np.array([wVx1, wVy1, wVz1, wVx2, wVy2, wVz2, wVx3, wVy3, wVz3])

                w =  U[:,0].dot(b) * V[:,0] * sInv[0] + U[:,1].dot(b) * V[:,1] * sInv[1] + U[:,2].dot(b) * V[:,2] * sInv[2]
            
            elif self.sensorNum == 2:
                U2, V2, sInv2 = self.U2, self.V2, self.sInv2
                b = np.array([wVx1, wVy1, wVz1, wVx2, wVy2, wVz2])
                w =  U2[:,0].dot(b) * V2[:,0] * sInv2[0] + U2[:,1].dot(b) * V2[:,1] * sInv2[1] + U2[:,2].dot(b) * V2[:,2] * sInv2[2]

            else:
                print "Not supported sensorNum", self.sensorNum

            #l = np.sqrt(w[0]**2 + w[1]**2 + w[2]**2)
            #w = [w[0]/l, w[1]/l, w[2]/l]

            #print np.cross(w, self.topOfSphere)
            roll, pitch, yaw = np.cross(w, self.topOfSphere)
            _, yaw, _ = np.cross(w, (1, 0, 0))

            self.bufferedVelocities += [velocities]
            self.bufferedPitchYawRoll += [(pitch, yaw, roll)]
            self.dataLock.release()
            time.sleep(1/100.)

    def setSensorNum(self, num):
        if self.sensorNum is not num:
            self.dataLock.acquire()
            self.sensorNum = num
            if num == 2:
                self.zRotAngle -= 180
            else:
                self.zRotAngle += 180
            self.initSVD(self.zRotAngle)
            self.dataLock.release()

    def stop(self):
        self.running = False

    def setCommandCompletedListener(self, listener):
        self.commandCompletedListener = listener

    def getSensorVelocities(self):
        self.dataLock.acquire()
        toreturn = list(self.bufferedVelocities) # makes a deep copy
        self._clearBuffers()
        self.dataLock.release()
        return toreturn

    def getPitchYawRoll(self):
        self.dataLock.acquire()
        toreturn = list(self.bufferedPitchYawRoll) # makes a deep copy
        self._clearBuffers()
        self.dataLock.release()
        return toreturn

    def _clearBuffers(self):
        self.bufferedVelocities = []
        self.bufferedPitchYawRoll = []

    def setServoSpeed(self, speed):
        self.serial.write("s " + str(speed) + "\n")

    def setServoAngle(self, servo, angleDeg, speed):
        if servo.__class__ != Servo:
            print "Wrong type of first argument - should be servo"
        self.serial.write(str(servo.getInd()) + " " + str(angleDeg) + "\n")

    def _processInput(self):
        lines = self.serial.readlines()
        incomplete_line = self.incomplete_line
        bufferedLines = self.bufferedLines

        if len(lines) > 0:
            if incomplete_line:
                lines[0] = incomplete_line + lines[0]
            #print "\n".join(lines)
            if lines[-1][-1] != '\n':
                incomplete_line = lines[-1]
                lines = lines[:-1]
            else:
                incomplete_line = None
            bufferedLines.extend(lines)

        self.incomplete_line = incomplete_line

        if len(bufferedLines) > 0:
            line = bufferedLines[0]
            self.processedLines += [line]
            bufferedLines = bufferedLines[1:]
            self.bufferedLines = bufferedLines
            if reCommandStarted.search(line):
                t = int(reCommandStarted.search(line).group(1))
                return (COMMAND_STARTED, t)
            if reCommandCompleted.search(line):
                t = int(reCommandCompleted.search(line).group(1))
                return (COMMAND_COMPLETED, t)
            if reServoSpeedSet.search(line):
                return SERVO_SPEED_SET
            ss = line.split()
            if len(ss) is not 8:
                None
                #print "problem ss len is not 6!"
                #print line
            else:
                try:
                    x1 = int(ss[2])
                    y1 = int(ss[3])
                    x2 = int(ss[4])
                    y2 = int(ss[5])
                    x3 = int(ss[6])
                    y3 = int(ss[7])
                    return (x1, y1, x2, y2, x3, y3)
                except ValueError:
                    print "ValueError"
        return None

# Utility functions

def rotateDeg(array, deg):
    ang = np.radians(deg)
    rot = np.array([np.cos(ang), -np.sin(ang), 0, np.sin(ang), np.cos(ang), 0, 0, 0, 1]).reshape((3, 3))
    return np.dot(rot, array)


