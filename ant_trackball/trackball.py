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
reCommandCompleted = re.compile(r'Command completed')

class CommandCompletedListener:
    def commandCompleted(self, pitchYawRoll):
        pass

class Trackball():

    class TrackballThread(threading.Thread):
        def __init__(self, parent):
            threading.Thread.__init__(self)
            self.parent = parent

        def run(self):
            self.parent._start();

    thread = None
    
    def __init__(self):
        # Default mbed serial, non-blocking
        self.serial = serial.Serial("/dev/ttyACM0", timeout=0)
        self.lines = []
        self.incomplete_line = None
        self.bufferedLines = []

        self.topOfSphere = np.array([0, 0, 1])

        sensor1Pos = np.array([np.sqrt(3)/2, 0, -1/2.])
        sensor2Pos = np.array([-np.sqrt(3)/4, -3/4., -1/2.])
        sensor3Pos = np.array([-np.sqrt(3)/4, 3/4., -1/2.])
        x1, y1, z1 = sensor1Pos
        x2, y2, z2 = sensor2Pos
        x3, y3, z3 = sensor3Pos
        self.A = np.array([0, z1, -y1, -z1, 0, x1, y1, -x1, 0, 0, z2, -y2, -z2, 0, x2, y2, -x2, 0, 0, z3, -y3, -z3, 0, x3, y3, -x3, 0]).reshape([9,3])
        self.U, self.s, self.V = np.linalg.svd(self.A)
        inv = lambda i: 0 if i == 0 else 1./i
        self.sInv = [inv(i) for i in self.s]
        self.dataLock = threading.Lock()
        self.commandCompletedListener = None

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
        
        A, U, sInv, V = self.A, self.U, self.sInv, self.V

        while (self.running):
            velocities = self._processInput()
            if velocities == None:
                continue
            elif velocities == COMMAND_COMPLETED:
                if self.commandCompletedListener != None:
                    self.commandCompletedListener.commandCompleted(self.bufferedPitchYawRoll)
                continue

            x1, y1, x2, y2, x3, y3 = velocities

            wVx1, wVy1, wVz1 = (y2/2., -x2, y2*np.sqrt(3)/2.)
            wVx2, wVy2, wVz2 = (y3/2., -x3, y3*np.sqrt(3)/2.)
            wVx3, wVy3, wVz3 = (y1/2., -x1, y1*np.sqrt(3)/2.)

            wVx2, wVy2, wVz2 = rotateDeg([wVx2, wVy2, wVz2], -120)
            wVx3, wVy3, wVz3 = rotateDeg([wVx3, wVy3, wVz3], 120)

            b = np.array([wVx1, wVy1, wVz1, wVx2, wVy2, wVz2, wVx3, wVy3, wVz3])

            w =  U[:,0].dot(b) * V[:,0] * sInv[0] + U[:,1].dot(b) * V[:,1] * sInv[1] + U[:,2].dot(b) * V[:,2] * sInv[2]

            #l = np.sqrt(w[0]**2 + w[1]**2 + w[2]**2)
            #w = [w[0]/l, w[1]/l, w[2]/l]

            #print np.cross(w, self.topOfSphere)
            roll, pitch, yaw = np.cross(w, self.topOfSphere)
            _, yaw, _ = np.cross(w, (1, 0, 0))

            self.dataLock.acquire()
            self.bufferedVelocities += [velocities]
            self.bufferedPitchYawRoll += [(pitch, yaw, roll)]
            self.dataLock.release()
            time.sleep(1/30.)


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
            bufferedLines = bufferedLines[1:]
            self.bufferedLines = bufferedLines
            if reCommandCompleted.search(line):
                return COMMAND_COMPLETED
            ss = line.split()
            if len(ss) is not 8:
                print "problem ss len is not 6!"
                print line
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


