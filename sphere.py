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
MOCK = False 

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
    gluLookAt(10,0,0,
              0,0,0,
              0,0,1)
    glPushMatrix()
    rot1X = rot1Y = 0
    rot2X = rot2Y = 0
    
    if not MOCK:
        subprocess.call(['sudo', 'chmod', '777', '/dev/ttyACM0'])
        ser = serial.Serial("/dev/ttyACM0", timeout=0)

    lines = []
    incomplete_line = None

#    sensor1Pos = np.dot(2, [0, 1/np.sqrt(2), -1/np.sqrt(2)])
#    sensor2Pos = np.dot(2, [0, -1/np.sqrt(2), -1/np.sqrt(2)])

    sensor1Pos = [np.sqrt(3)/2, 0, -1/2.]
    sensor2Pos = [-np.sqrt(3)/4, 3/4., -1/2.]
 

    x1, y1, z1 = sensor1Pos
    x2, y2, z2 = sensor2Pos

    A = np.array([0, z1, -y1, -z1, 0, x1, y1, -x1, 0, 0, z2, -y2, -z2, 0, x2,
                  y2, -x2, 0]).reshape([6,3])

    U, s, V = np.linalg.svd(A)
    inv = lambda i: 0 if i == 0 else 1./i
    sInv = [inv(i) for i in s]

    print "U=", str(U)
    print "s=", str(s)
    print "V=", str(V)

    globalQuat = eulerToQuaternion(0, 0, 0)
    bufferedLines = [] 

    oneSensor = False 
    calibrationMode = False

    spherePoints = genSpherePoints()

    while True:
        x1 = y1 = x2 = y2 = 0
        if not MOCK:
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
            print line
            ss = line.split()
            if len(ss) is not 6:
                print "problem ss len is not 6!"
                print line
            else:
                try:
                    x1 = int(ss[2])
                    y1 = int(ss[3])
                    x2 = int(ss[4])
                    y2 = int(ss[5])
                except ValueError:
                    x1 = y1 = x2 = y2 = 0
                    print "ValueError"

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
                if event.key == K_8:
                    ser.write("1\n")
                if event.key == K_9:
                    ser.write("2\n")
                if event.key == K_0:
                    ser.write("3\n")


                if event.key == K_c:
                    if calibrationMode:
                        print "Calibrating!"
                        print "sensor 1 X, Y", accumX1, accumY1
                        print "sensor 2 X, Y", accumX2, accumY2
                        print calibration
                        sensor1Pos, sensor2Pos = calibrate(calibration) 
                        A = np.array([0, z1, -y1, -z1, 0, x1, y1, -x1, 0, 0, z2, -y2, -z2, 0, x2, y2, -x2, 0]).reshape([6,3])

                        U, s, V = np.linalg.svd(A)
                        sInv = [inv(i) for i in s]
                        calibrationMode = False
                        print "Calibration done"
                        print U
                        print s
                        print V
                    else :
                        print "Calibration mode start"
                        calibrationMode = True
                        calibration = []
                        accumX1 = 0
                        accumY1 = 0
                        accumX2 = 0
                        accumY2 = 0
                if event.key == K_a:
                    user_in = raw_input("Enter three space separated values in degrees denoting pitch, yaw, roll: ")
                    try:
                        pitch, yaw, roll = [int(deg) for deg in user_in.split()]
                        calibration += [(accumX1, accumY1, accumX2, accumY2, pitch,
                                    yaw, roll)]
                        accumX1 = accumY1 = accumX2 = accumY2 = 0
                        print calibration[-1], " recorded!"
                    except ValueError:
                        print "Parsing error. Calibration values not saved."

                if event.key == K_p:
                    print "Sensor positions:", sensor1Pos, sensor2Pos


        if calibrationMode:
            accumX1 += x1
            accumY1 += y1
            accumX2 += x2
            accumY2 += y2

        if oneSensor:
            quat = eulerToQuaternion(-y2, 0, -x2)

        else:
            b = np.array([x1, y1, 0, x2, -y2, 0])
            w =  U[:,0].dot(b) * V[:,0] * sInv[0] + U[:,1].dot(b) * V[:,1] * sInv[1] + U[:,2].dot(b) * V[:,2] * sInv[2]

            #print str(w)
            
            if (w[2] != 0 and not np.isnan(w[2])) or (w[0] != 0 and not np.isnan(w[0])) or (w[1] !=0 and not np.isnan(w[1])):
                print "pitch", w[1], "yaw", w[0], "roll", w[2]
                #TODO maybe divide /2 in calibration?
                quat = eulerToQuaternion(w[0]/2, w[2]/2, w[1]/2)
                globalQuat = multiplyQuaternions(globalQuat, quat)
        # rotate the global quaternion

        display(globalQuat, spherePoints, [sensor1Pos, sensor2Pos])
        #display(quat)
        #display((w[0], w[1], w[2]))

        pygame.display.flip()
        #clock.tick(30)
    return

def display(quaternion, spherePoints, sensorsPos):
    #TODO rotate rotZ as well
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    for (x, y, z) in sensorsPos:
        glPushMatrix()
        glTranslate(x, y, z)
        glutSolidSphere(0.3, 10, 10)
        glPopMatrix()
    glPushMatrix()
    #currentModelViewMatrix = glGetDouble(GL_MODELVIEW_MATRIX)
    #glRotatef(quaternion[0], currentModelViewMatrix[0][1], currentModelViewMatrix[1][1], currentModelViewMatrix[2][1])
    #currentModelViewMatrix = glGetDouble(GL_MODELVIEW_MATRIX)
    #glRotatef(quaternion[1], currentModelViewMatrix[0][0], currentModelViewMatrix[1][0], currentModelViewMatrix[2][0])
    glMultMatrixf(quaternionToMatrix(quaternion))
    color = [1.0,0.,0.,1.]
    glutWireSphere(2,10,10)
   # for (x, y, z) in spherePoints:
   #     glPushMatrix()
   #     glTranslate(x, y, z)
   #     glutSolidSphere(0.1, 10, 10)
   #     glPopMatrix()
   #     #glutWireCube(2)
    glPopMatrix()
    return

def calibrate(calibration_data):
    points = genSpherePoints()
    n = len(points)
    sumSquaredErrors = []
    for i in range(0, n):
        for j in range(i, n):
            sumSquaredErr = evalSumSquared(points[i], points[j],
                                           calibration_data)
            sumSquaredErrors += [(i, j, sumSquaredErr)]

    minfunc = lambda (sensor1, sensor2, error): error

    (s1, s2, err) = min(sumSquaredErrors, key=minfunc)

    bestPlace = (s1, s2)
    print "Best place for sensors is ", points[s1], points[s2], "Error is", err
    return (points[s1], points[s2])

def genSpherePoints():
# src http://www.cmu.edu/biolphys/deserno/pdf/sphere_equi.pdf
    N = 400
    count = 0
    pi = np.pi
    r = 2
    alpha = 4*pi*r*r/N
    d = np.sqrt(alpha)
    M = int(round(pi/d))
    dTheta = pi/M
    dPhi = alpha/dTheta
    points = []
    for m in range(0, M):
        v = pi * (m + 0.5)/M
        MPhi = int(round(2*pi*np.sin(v)/dPhi))
        for n in range(0, MPhi):
            phi = 2*pi*n/MPhi
            (x, y, z) = sphericalToCartesian(v, phi, r)
            points += [(x, y, z)]
            count += 1

    return points

def sphericalToCartesian(v, phi, radius):
    return (radius*np.sin(v) * np.cos(phi), radius*np.sin(v) * np.sin(phi),
            radius*np.cos(v))

def evalSumSquared(sensor1Pos, sensor2Pos, data):

    x1, y1, z1 = sensor1Pos
    x2, y2, z2 = sensor2Pos

    A = np.array([0, z1, -y1, -z1, 0, x1, y1, -x1, 0, 0, z2, -y2, -z2, 0, x2,
                  y2, -x2, 0]).reshape([6,3])

    U, s, V = np.linalg.svd(A)

    sumSquared = 0


    for (dx1, dy1, dx2, dy2, pitch, yaw, roll) in data:
        b = np.array([dx1, dy1, 0, dx2, dy2, 0]) #TODO maybe minus? dy2
        w =  U[:,0].dot(b) * V[:,0]/s[0] + U[:,1].dot(b) * V[:,1]/s[1] + U[:,2].dot(b) * V[:,2]/s[2]
        sumSquared += error((pitch, yaw, roll), w) 


    return sumSquared

def error((pitch1, yaw1, roll1), (pitch2, yaw2, roll2)):
    return (pitch1-pitch2)**2 + (yaw1-yaw2)**2 + (roll1-roll2)**2

if __name__ == '__main__': 
    main()
