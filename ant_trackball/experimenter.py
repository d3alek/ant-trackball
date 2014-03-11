import atexit
import trackball
#import trackballutil
import dateutil
import matplotlib.pyplot as plt
import threading
import time
import numpy as np

ant_trackball = trackball.Trackball()
ant_trackball.start()
ant_trackball.getPitchYawRoll()
mCommandCompleted = threading.Semaphore(0)

mPitchYawRoll = []
mVelocities = []

class CommandCompletedListenerImpl(trackball.CommandCompletedListener):
    def commandCompleted(self, velocities, pitchYawRoll):
        global mPitchYawRoll, mVelocities
        print "Command completed"
        mPitchYawRoll = pitchYawRoll
        mVelocities = velocities
        mCommandCompleted.release()


def stopTrackball():
    ant_trackball.stop()

def init():
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

class Speed(Command):
    def __init__(self, speed):
        self.speed = speed

    def run(self):
        print "Setting speed to ", self.speed
        ant_trackball.setServoSpeed(self.speed)

class TimerStart(Command):
    def run(self):
        global mTimerStartedAt
        mTimerStartedAt = time.time()
        mCommandCompleted.release()

class TimerStop(Command):
    def run(self):
        print time.time() - mTimerStartedAt
        mCommandCompleted.release()

class Speedometer(Command):
    def __init__(self, distance):
        self.distance = distance # m 
        
    def run(self):
        t = time.time() - mTimerStartedAt # s
        print "Speed is (m/s)", self.distance/t
        mCommandCompleted.release()

def runCommandSequence(commandSequence):
    init()
    for command in commandSequence:
        command.run()
        mCommandCompleted.acquire()

def example():
    runCommandSequence([Speed(1), ServoYaw(45), Speed(10), Graph(), ServoYaw(-45), Graph()])

def speedTest():
    runCommandSequence([Speed(1), TimerStart(), ServoPitch(20),
                        Speedometer(np.pi*0.005),
                        Speed(10), TimerStart(), ServoPitch(0),
                        Speedometer(np.pi*0.005)])

if __name__ == "__main__":
    speedTest()

