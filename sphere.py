from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
import pygame
from pygame.locals import *
import sys
import numpy as np
import serial
import subprocess

TOLERANCE = 0.00001

def normalizeQuaternion(quat):
    global TOLERANCE
    x, y, z, w = quat
    mag2 = w * w + x * x + y * y + z * z
    if np.abs(mag2) > TOLERANCE and np.abs(mag2 - 1) > TOLERANCE:
        mag = sqrt(mag2)
        x /= mag
        y /= mag
        z /= mag
        w /= mag

    return (x, y, z, w)


def eulerToQuaternion(pitch, yaw, roll):
    p = pitch*np.pi/(180*2.)
    y = yaw*np.pi/(180*2.)
    r = roll*np.pi/(180*2.)

    sinp = np.sin(p)
    siny = np.sin(y)
    sinr = np.sin(r)
    cosp = np.cos(p)
    cosy = np.cos(y)
    cosr = np.cos(r)

    x = sinr * cosp * cosy - cosr * sinp * siny
    y = cosr * sinp * cosy + sinr * cosp * siny
    z = cosr * cosp * siny - sinr * sinp * cosy
    w = cosr * cosp * cosy + sinr * sinp * siny

    quat = normalizeQuaternion((x, y, z, w))

    return quat 

def multiplyQuaternions(quat1, quat2):
    x1, y1, z1, w1 = quat1
    x2, y2, z2, w2 = quat2
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2,
            w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)

def quaternionToMatrix(quat):
    x, y, z, w = quat

    x2 = x * x
    y2 = y * y
    z2 = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return [1. - 2. * (y2 + z2), 2. * (xy - wz), 2. * (xz + wy), 0.,
            2. * (xy + wz), 1. - 2. * (x2 + z2), 2. * (yz - wx), 0.,
            2. * (xz - wy), 2 * (yz + wx), 1 - 2 * (x2 + y2), 0.,
            0., 0., 0., 1.]




SCREEN_SIZE = (800, 600)

name = 'Ant Trackball'


def resize(width, height):
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(60.0, float(width)/height, 1, 1000.)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def main():

    pygame.init()

    clock = pygame.time.Clock()

    screen = pygame.display.set_mode(SCREEN_SIZE, HWSURFACE|OPENGL|DOUBLEBUF) 

    glutInit(sys.argv)

    resize(*SCREEN_SIZE)

    glClearColor(0.,0.,0.,1.)
    glShadeModel(GL_SMOOTH) # GL_FLAT?
    glEnable(GL_CULL_FACE)
    glEnable(GL_DEPTH_TEST)
    glMatrixMode(GL_MODELVIEW)
    gluLookAt(0,0,10,
              0,0,0,
              0,1,0)
    glPushMatrix()
    rot1X = rot1Y = 0
    rot2X = rot2Y = 0

    subprocess.call(['sudo', 'chmod', '777', '/dev/ttyACM0'])
    ser = serial.Serial("/dev/ttyACM0", timeout=0)

    lines = []
    incomplete_line = None

    sensor1Pos = [0, 1/np.sqrt(2), -1/np.sqrt(2)]
    sensor2Pos = [0, -1/np.sqrt(2), -1/np.sqrt(2)]

    x1, y1, z1 = sensor1Pos
    x2, y2, z2 = sensor2Pos

    A = np.array([0, z1, -y1, -z1, 0, x1, y1, -x1, 0, 0, z2, -y2, -z2, 0, x2,
                  y2, -x2, 0]).reshape([6,3])

    U, s, V = np.linalg.svd(A)

    globalQuat = eulerToQuaternion(0, 0, 0)
    bufferedLines = [] 

    oneSensor = False 

    while True:
        x1 = y1 = x2 = y2 = 0
        lines = ser.readlines()
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

        if len(bufferedLines) > 0:
            line = bufferedLines[0]
            bufferedLines = bufferedLines[1:]
            #print line
            ss = line.split()
            if len(ss) is not 6:
                print "problem ss len is not 6!"
                print line
            else:
                x1 = int(ss[2])
                y1 = int(ss[3])
                x2 = int(ss[4])
                y2 = int(ss[5])

        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
#            elif event.type == KEYDOWN:
#                if event.key == K_LEFT:
#                    print "left" + str(i)
#                    rot2X += -np.pi/2;
#                if event.key == K_RIGHT: 
#                    print "right" + str(i)
#                    rot2X += np.pi/2;
#                if event.key == K_UP:
#                    print "up" + str(i)
#                    rot2Y += np.pi/2;
#                if event.key == K_DOWN:
#                    print "down" + str(i)
#                    rot2Y += -np.pi/2;
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    pygame.event.post(pygame.event.Event(QUIT))
                if event.key == K_1:
                    oneSensor = True
                    print "Switching to one sensor"
                if event.key == K_2:
                    oneSensor = False
                    print "Switching to two sensors"

                # quaternion representing the change
        #w = [rot2X, rot2Y, 0]
        #if x1 != 0 or y1 != 0 or x2 != 0 or y2 != 0:
        #    print str(map(np.degrees, w))

        if oneSensor:
            quat = eulerToQuaternion(-y2, 0, -x2)

        elif x2 == 0 and y2 == 0:
            quat = eulerToQuaternion(y1, 0, x1)

        elif x1 == 0 and y1 == 0:
            quat = eulerToQuaternion(-y2, 0, -x2)
        else:
            b = np.array([x1, y1, 0, x2, y2, 0])
            w =   U[:,0].dot(b) * V[:,0]/s[0]
            + U[:,1].dot(b) * V[:,1]/s[1]
            + U[:,2].dot(b) * V[:,2]/s[2]

            quat = eulerToQuaternion(w[1], w[2], w[0])

        # rotate the global quaternion
        globalQuat = multiplyQuaternions(globalQuat, quat)

        display(globalQuat)
        #display(quat)
        #display((w[0], w[1], w[2]))

        pygame.display.flip()
        #clock.tick(30)
    return

def display(quaternion):
    #TODO rotate rotZ as well
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    glPushMatrix()
    #currentModelViewMatrix = glGetDouble(GL_MODELVIEW_MATRIX)
    #glRotatef(quaternion[0], currentModelViewMatrix[0][1], currentModelViewMatrix[1][1], currentModelViewMatrix[2][1])
    #currentModelViewMatrix = glGetDouble(GL_MODELVIEW_MATRIX)
    #glRotatef(quaternion[1], currentModelViewMatrix[0][0], currentModelViewMatrix[1][0], currentModelViewMatrix[2][0])
    glMultMatrixf(quaternionToMatrix(quaternion))
    color = [1.0,0.,0.,1.]
    glutWireSphere(2,20,20)
    #glutWireCube(2)
    glPopMatrix()
    return

if __name__ == '__main__': 
    main()
