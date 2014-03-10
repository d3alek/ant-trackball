import trackball
#import trackballutil
import dateutil
import matplotlib.pyplot as plt
import threading

ant_trackball = trackball.Trackball()
ant_trackball.start()
ant_trackball.getPitchYawRoll()
mCommandCompleted = threading.Semaphore(0)

mPitchYawRoll = []

class CommandCompletedListenerImpl(trackball.CommandCompletedListener):
    def commandCompleted(self, pitchYawRoll):
        global mPitchYawRoll
        print "Command completed"
        mPitchYawRoll = pitchYawRoll
        mCommandCompleted.release()

mCommandCompletedListener = CommandCompletedListenerImpl()

ant_trackball.setCommandCompletedListener(mCommandCompletedListener)
ant_trackball.setServoAngle(trackball.SERVO_YAW, 0, 1);
mCommandCompleted.acquire()
ant_trackball.setServoAngle(trackball.SERVO_PITCH, 0, 1);
mCommandCompleted.acquire()

print "Initialized"

class Command:
    def run(self):
        print "Empty command"
        pass

class ServoYaw(Command):
    def __init__(self, deg):
        self.deg = deg

    def run(self):
        ant_trackball.setServoAngle(trackball.SERVO_YAW, self.deg, 1)

class Graph(Command):
    def run(self):
        plt.plot(mPitchYawRoll)
        plt.show()

def runCommandSequence(commandSequence):
    for command in commandSequence:
        command.run()
        mCommandCompleted.acquire()

def example():
    runCommandSequence([ServoYaw(45), Graph()])

if __name__ == "__main__":
    example()


