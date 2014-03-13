import atexit
import trackball
#import trackballutil
import dateutil
import matplotlib.pyplot as plt
import threading
import time
import numpy as np

ant_trackball = trackball.Trackball(-30)
ant_trackball.start()
ant_trackball.getPitchYawRoll()
mCommandCompleted = threading.Semaphore(0)

mPitchYawRoll = []
mVelocities = []

DEFAULT_SPEED = 5

class CommandCompletedListenerImpl(trackball.CommandCompletedListener):
    def commandCompleted(self, velocities, pitchYawRoll, t):
        global mPitchYawRoll, mVelocities, mCommandCompletedTime
        print "Command completed", t
        mPitchYawRoll = pitchYawRoll
        mVelocities = velocities
        mCommandCompleted.release()
        if t > 0:
            mCommandCompletedTime = t
    def commandStarted(self, t):
        global mCommandStartedTime
        print "Command started", t
        mCommandStartedTime = t

def getPitchYawRoll():
    return mPitchYawRoll

def getVelocities():
    return mVelocities

def stopTrackball():
    ant_trackball.stop()

def init():
    ant_trackball.setServoSpeed(DEFAULT_SPEED);
    mCommandCompleted.acquire()
    ant_trackball.setServoAngle(trackball.SERVO_YAW, 0, 1);
    mCommandCompleted.acquire()
    ant_trackball.setServoAngle(trackball.SERVO_PITCH, 0, 1);
    mCommandCompleted.acquire()
    ant_trackball.getPitchYawRoll()
    print "Initialized"

mCommandCompletedListener = CommandCompletedListenerImpl()

ant_trackball.setCommandCompletedListener(mCommandCompletedListener)
atexit.register(stopTrackball)
plt.figure()

class Command:
    def run(self):
        print "Empty command"
        pass

class ServoYaw(Command):
    def __init__(self, deg):
        self.deg = deg

    def run(self):
        ant_trackball.setServoAngle(trackball.SERVO_YAW, self.deg, 1)

class ServoPitch(Command):
    def __init__(self, deg):
        self.deg = deg

    def run(self):
        ant_trackball.setServoAngle(trackball.SERVO_PITCH, self.deg, 1)

class GraphV(Command):
    def run(self):
        plt.plot(mVelocities)
        plt.legend(('s1x', 's1y', 's2x', 's2y', 's3x', 's3y'))
        plt.show()
        mCommandCompleted.release()

class Graph(Command):
    def run(self):
        plt.plot(mPitchYawRoll)
        plt.legend(('pitch', 'yaw', 'roll'))
        plt.show()
        mCommandCompleted.release()

class GraphAccum(Command):
    def run(self):
        accumPoints = accumulate(mPitchYawRoll)
        plt.plot(accumPoints)
        plt.legend(('pitch', 'yaw', 'roll'))
        plt.show()
        mCommandCompleted.release()

class GraphMovementXY(Command):
    def run(self):
        plt.xlabel('x')
        plt.ylabel('y')
        time.sleep(1)
        mPitchYawRoll = ant_trackball.getPitchYawRoll()
        accumPoints = accumulate(mPitchYawRoll)
        #XYs = map(lambda points: points[:-1], accumPoints)
        Xs = map(lambda points: points[0], accumPoints)
        Ys = map(lambda points: points[2], accumPoints)
        plt.plot(Xs, Ys)
        xyMax = max(max(Xs), max(Ys)) + 50
        xyMin = min(min(Xs), min(Ys)) - 50
        plt.axis([xyMin,xyMax,xyMin,xyMax])
        #plt.legend(('x', 'y'))
        plt.show()
        mCommandCompleted.release()



class GraphMovementXZ(Command):
    def run(self):
        plt.xlabel('x')
        plt.ylabel('z')
        time.sleep(1)
        mPitchYawRoll = ant_trackball.getPitchYawRoll()
        accumPoints = accumulate(mPitchYawRoll)
        #XYs = map(lambda points: points[:-1], accumPoints)
        Xs = map(lambda points: points[0], accumPoints)
        Ys = map(lambda points: points[1], accumPoints)
        plt.plot(Xs, Ys)
        xyMax = max(max(Xs), max(Ys)) + 50
        xyMin = min(min(Xs), min(Ys)) - 50
        plt.axis([xyMin,xyMax,xyMin,xyMax])
        #plt.legend(('x', 'y'))
        plt.show()
        mCommandCompleted.release()


class Speed(Command):
    def __init__(self, speed):
        self.speed = speed

    def run(self):
        print "Setting speed to ", self.speed
        ant_trackball.setServoSpeed(self.speed)

class TimerStart(Command):
    def run(self):
        global mTimerStartedAt
        mTimerStartedAt = mCommandStartedTime
        mCommandCompleted.release()

class Sensors(Command):
    def __init__(self, num):
        self.num = num

    def run(self):
        global ant_trackball
        ant_trackball.setSensorNum(self.num)
        mCommandCompleted.release()

class TimerStop(Command):
    def run(self):
        print mCommandCompletedTime - mTimerStartedAt
        mCommandCompleted.release()

class Speedometer(Command):
    """ Measures and prints speed of last command"""
    def __init__(self, distance):
        self.distance = distance # m 
        
    def run(self):
        t = mCommandCompletedTime - mCommandStartedTime # ms
        t = t * 0.001 # s
        print "S=", self.distance, "t=", t, "V(m/s)=", self.distance/t
        mCommandCompleted.release()

class Max(Command):
    def run(self):
        time.sleep(2)
        #mPitchYawRoll = ant_trackball.getPitchYawRoll()
        accumPoints = accumulate(mPitchYawRoll)
        Xs = map(lambda points: points[0], accumPoints)
        Ys = map(lambda points: points[1], accumPoints)
        Zs = map(lambda points: points[2], accumPoints)

        print "Max:", max(Xs), max(Ys), max(Zs)
        mCommandCompleted.release()

class Clear(Command):
    def run(self):
        mPitchYawRoll = []
        ant_trackball.getPitchYawRoll()
        mCommandCompleted.release()

def runCommandSequence(commandSequence):
    init()
    for command in commandSequence:
        command.run()
        mCommandCompleted.acquire()

def example():
    runCommandSequence([Speed(1), ServoPitch(45), Speed(10), GraphAccum(),
                        ServoYaw(-45), GraphAccum()])

def speedTest():
    runCommandSequence([Speed(1), TimerStart(), ServoPitch(45),
                        Speedometer(np.pi*0.005),
                        Speed(100), TimerStart(), ServoPitch(0),
                        Speedometer(np.pi*0.005)])

def squareTest():
    seq = [Speed(5), ServoPitch(-45), ServoYaw(45), ServoPitch(45),
           ServoYaw(90), ServoPitch(0)] + [ServoPitch(45), ServoYaw(45),
                                           ServoPitch(-45), ServoYaw(0),
                                           ServoPitch(0)] + [GraphMovementXY()]
    runCommandSequence(seq)

def xTest(speed, sensorNum, n):
    seq = [Speed(speed), Sensors(sensorNum)] + n*[ServoPitch(45), ServoPitch(0)] + [GraphMovementXY()]
    runCommandSequence(seq)

def accumulate(pitchYawRoll):
    accum = (0, 0, 0)
    accumPoints = []
    for (pitch, yaw, roll) in mPitchYawRoll:
        accumPoints += [accum]
        accum = (accum[0] + pitch, accum[1] + yaw, accum[2] + roll)

    accumPoints += [accum]

    return accumPoints 

def calibrate(angle):
    global ant_trackball
    ant_trackball.stop()
    ant_trackball = trackball.Trackball(angle)
    ant_trackball.start()
    ant_trackball.getPitchYawRoll()
    ant_trackball.setCommandCompletedListener(mCommandCompletedListener)

if __name__ == "__main__":
    speedTest()
    ant_trackball.stop()

