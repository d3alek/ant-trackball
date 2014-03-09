import pygame
from pygame.locals import *
import sys
import numpy as np
import serial
import subprocess

TOLERANCE = 0.00001
MOCK = False 

SCREEN_SIZE = (800, 600)

name = 'Ant Trackball'

def main():

    sensors = 4

    mTesting = False;
    
    pygame.init()

    clock = pygame.time.Clock()

    screen = pygame.display.set_mode(SCREEN_SIZE) 

    rot1X = rot1Y = 0
    rot2X = rot2Y = 0
    
    if not MOCK:
        subprocess.call(['sudo', 'chmod', '777', '/dev/ttyACM0'])
        ser = serial.Serial("/dev/ttyACM0", timeout=0)

    lines = []
    incomplete_line = None

    #sensor1Pos = np.dot(1, [np.sqrt(3)/2, 0, -1/2.])

    #sensor1Pos = np.dot(1, [-np.sqrt(3)/4, 3/4., -1/2.])
    #sensor2Pos = np.dot(1, [np.sqrt(3)/2, 0, -1/2.])

    sensor1Pos = np.dot(1, [-np.sqrt(3)/4, -3/4., -1/2.])

    topOfSphere = np.dot(1, [0, 0, 1])

    x1, y1, z1 = sensor1Pos
    #x2, y2, z2 = sensor2Pos
    #x3, y3, z3 = sensor2Pos

        #A3 = np.array([0, z1, -y1, -z1, 0, x1, y1, -x1, 0, 0, z2, -y2, -z2, 0, x2,
    #              y2, -x2, 0, 0, z3, -y3, -z3, 0, x3, y3, -x3,
    #               0]).reshape([9,3])

    if sensors == 1:
        sensor1Pos = np.dot(1, [np.sqrt(3)/2, 0, -1/2.])
        x1, y1, z1 = sensor1Pos
        A = np.array([0, z1, -y1, -z1, 0, x1, y1, -x1, 0]).reshape([3, 3])
    elif sensors == 2:
        sensor1Pos = np.dot(1, [-np.sqrt(3)/4, -3/4., -1/2.])
        x1, y1, z1 = sensor1Pos
        A = np.array([0, z1, -y1, -z1, 0, x1, y1, -x1, 0]).reshape([3, 3])
        #A = np.array([0, z1, -y1, -z1, 0, x1, y1, -x1, 0, 0, z2, -y2, -z2, 0, x2, y2, -x2, 0]).reshape([6,3])
        #U, s, V = np.linalg.svd(A)
    #U3,# s3, V3 = np.linalg.svd(A3, full_matrices=True)
        #inv = lambda i: 0 if i == 0 else 1./i
        #sInv = [inv(i) for i in s]
    #sIn#v3 = [inv(i) for i in s3]
        #print "U=", str(U)
        #print "s=", str(s)
        #print "V=", str(V)
    elif sensors == 3:
        sensor1Pos = np.dot(1, [-np.sqrt(3)/4, 3/4., -1/2.])
        x1, y1, z1 = sensor1Pos
        A = np.array([0, z1, -y1, -z1, 0, x1, y1, -x1, 0]).reshape([3, 3])
    elif sensors == 4:
        sensor1Pos = np.dot(1, [np.sqrt(3)/2, 0, -1/2.])
        sensor2Pos = np.dot(1, [-np.sqrt(3)/4, -3/4., -1/2.])
        sensor3Pos = np.dot(1, [-np.sqrt(3)/4, 3/4., -1/2.])
        x1, y1, z1 = sensor1Pos
        x2, y2, z2 = sensor2Pos
        x3, y3, z3 = sensor3Pos
        A = np.array([0, z1, -y1, -z1, 0, x1, y1, -x1, 0, 0, z2, -y2, -z2, 0, x2, y2, -x2, 0, 0, z3, -y3, -z3, 0, x3, y3, -x3, 0]).reshape([9,3])
        U, s, V = np.linalg.svd(A)
        inv = lambda i: 0 if i == 0 else 1./i
        sInv = [inv(i) for i in s]



    bufferedLines = [] 

    calibrationMode = False

    startPoint = (400, 300)
    antPath = [startPoint]

    whiteColor = pygame.Color(255, 255, 255)
    facingAngl = 0

    tw = 10 
    th = 15

    while True:
        x1 = y1 = x2 = y2 = x3 = y3 = 0
        w = [0, 0, 0]

        screen.fill(whiteColor)

        x1, y1, x2, y2, x3, y3, incomplete_line, bufferedLines = processInput(ser, incomplete_line, bufferedLines);

        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()

            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    pygame.event.post(pygame.event.Event(QUIT))
                if event.key == K_2:
                    TWO_SENSORS = True
                    print "Switching to two sensors"
                if event.key == K_3:
                    TWO_SENSORS = False 
                    print "Switching to three sensors"
                if event.key == K_8:
                    ser.write("1\n")
                if event.key == K_9:
                    ser.write("2\n")
                if event.key == K_0:
                    ser.write("3\n")
                if event.key == K_4:
                    ser.write("4\n")
                if event.key == K_5:
                    ser.write("5\n")
                if event.key == K_6:
                    ser.write("6\n")
                if event.key == K_c:
                    antPath = [startPoint]
                    facingAngl = 0

                if event.key == K_p:
                    print "Sensor positions:", sensor1Pos, sensor2Pos,
                    sensor3Pos

                if event.key == K_t:
                    print "Testing start"
                    if mTesting:
                        mTesting = False
                    else:
                        ser.write("2\n5\n"); # reset servos to 0

                        #clear
                        antPath = [startPoint]
                        facingAngl = 0

                        mTesting = True; 

        if x1 == None or (x1 == 0 and y1 == 0):
            None
        elif sensors == 1:
            print "sensor=", x2, y2
            wVx, wVy, wVz = (y2/2., -x2, y2*np.sqrt(3)/2.)
            #print "wV:", wVx, wVy, wVz
            b = np.array([wVx, wVy, wVz])
            w, _, _, _ = np.linalg.lstsq(A, b)
            l = np.sqrt(w[0]**2 + w[1]**2 + w[2]**2)
            w = [w[0]/l, w[1]/l, w[2]/l]
            print "s1 w is ", w

        elif sensors == 2:
            #print "sensor=", x3, y3
            wVx, wVy, wVz = (y3/2., -x3, y3*np.sqrt(3)/2.)
            #print "wV:", wVx, wVy, wVz
            b = np.array([wVx, wVy, wVz])
            ang = np.radians(-120)
            rot = np.array([np.cos(ang), -np.sin(ang), 0, np.sin(ang), np.cos(ang), 0, 0, 0,
                            1]).reshape((3, 3))
            brot = np.dot(rot, b)
            #print "brot:", brot
            w, _, _, _ = np.linalg.lstsq(A, brot)
            l = np.sqrt(w[0]**2 + w[1]**2 + w[2]**2)
            w = [w[0]/l, w[1]/l, w[2]/l]
            print "s2 w is ", w
            #w =  U[:,0].dot(b) * V[:,0] * sInv[0] + U[:,1].dot(b) * V[:,1] * sInv[1] + U[:,2].dot(b) * V[:,2] * sInv[2]
        elif sensors == 3:
            #print "sensor=", x1, y1
            wVx, wVy, wVz = (y1/2., -x1, y1*np.sqrt(3)/2.)
            #print "wV:", wVx, wVy, wVz
            b = np.array([wVx, wVy, wVz])
            ang = np.radians(120)
            rot = np.array([np.cos(ang), -np.sin(ang), 0, np.sin(ang), np.cos(ang), 0, 0, 0,
                            1]).reshape((3, 3))
            brot = np.dot(rot, b)
            #print "brot:", brot
            w, _, _, _ = np.linalg.lstsq(A, brot)
            l = np.sqrt(w[0]**2 + w[1]**2 + w[2]**2)
            w = [w[0]/l, w[1]/l, w[2]/l]
            print "s3 w is ", w
            #w =  U[:,0].dot(b) * V[:,0] * sInv[0] + U[:,1].dot(b) * V[:,1] * sInv[1] + U[:,2].dot(b) * V[:,2] * sInv[2]
        elif sensors == 4:
            #print "sensor=", x1, y1
            wVx1, wVy1, wVz1 = (y2/2., -x2, y2*np.sqrt(3)/2.)
            wVx2, wVy2, wVz2 = (y3/2., -x3, y3*np.sqrt(3)/2.)
            wVx3, wVy3, wVz3 = (y1/2., -x1, y1*np.sqrt(3)/2.)
            ang = np.radians(-120)
            rot = np.array([np.cos(ang), -np.sin(ang), 0, np.sin(ang), np.cos(ang), 0, 0, 0,
                            1]).reshape((3, 3))
            wVx2, wVy2, wVz2 = np.dot(rot, [wVx2, wVy2, wVz2])
            ang = np.radians(120)
            rot = np.array([np.cos(ang), -np.sin(ang), 0, np.sin(ang), np.cos(ang), 0, 0, 0,
                            1]).reshape((3, 3))
            wVx3, wVy3, wVz3 = np.dot(rot, [wVx3, wVy3, wVz3])
            #print "wV:", wVx, wVy, wVz
            b = np.array([wVx1, wVy1, wVz1, wVx2, wVy2, wVz2, wVx3, wVy3, wVz3])
            #print "brot:", brot
            w =  U[:,0].dot(b) * V[:,0] * sInv[0] + U[:,1].dot(b) * V[:,1] * sInv[1] + U[:,2].dot(b) * V[:,2] * sInv[2]
            l = np.sqrt(w[0]**2 + w[1]**2 + w[2]**2)
            w = [w[0]/l, w[1]/l, w[2]/l]
            #print "s4 w is ", w
        else:
            #b = np.array([x1, y1, 0, x2, y2, 0, x3, y3, 0])
            # xs is yw, ys is zw, zs is xw
            #z1 = calcZOnSphere(x1, y1)
            #z2 = calcZOnSphere(x2, y2)
            #z3 = calcZOnSphere(x3, y3)
            #b = np.array([z1, x1, y1, z2, x2, y2, z3, x3, y3])
            b = np.array([0, x1, y1, 0.5, x2, y2, 0.5, x3, y3])
            w =  U3[:,0].dot(b) * V3[:,0] * sInv3[0] + U3[:,1].dot(b) * V3[:,1] * sInv3[1] + U3[:,2].dot(b) * V3[:,2] * sInv3[2]

        if (w[2] != 0 and not np.isnan(w[2])) or (w[0] != 0 and not np.isnan(w[0])) or (w[1] !=0 and not np.isnan(w[1])):
            x, y, z = np.cross(w, topOfSphere)
            print "xyz=", x, y, z
            #yaw = w[1]
            yaw = 0
            roll, pitch, yaw = np.cross(w, (1, 0, 0))
            facingAngl += yaw/5
            print "facingAngle", facingAngl
            yaw = 0
            antPath += [translatePoint(rotatePoint((x, y), yaw), lastPoint)]
#            if TWO_SENSORS:
                #pitch = -w[1]
                #yaw = w[2]
                #roll = -w[0]
            #else:
                ## correct
                ##pitch = w[1]
                ##yaw = w[0]
                ##roll = w[2]

                #pitch = w[2]
                #yaw = w[0]
                #roll = w[1]
            #print w
            #lastPoint = antPath[-1]


            #x, y, z = np.cross([pitch, yaw, roll], topOfSphere)
            ##pitch = y
            ##yaw = z
            ##roll = x
            ##print "xyz=", x, y, z

            ##facingAngl = 0 
            #facingAngl = yaw
            #antPath += [translatePoint(rotatePoint((pitch, -roll),
                                                   #facingAngl), lastPoint)]



        if len(antPath) > 1:
            pygame.draw.lines(screen, (0, 0, 0), False, antPath)

        lastPoint = antPath[-1]
        triangleVertices = [translatePoint(
            rotatePoint((tw/2, 0), facingAngl),lastPoint), 
                            translatePoint(
            rotatePoint((0, -th), facingAngl), lastPoint), 
                            translatePoint(
            rotatePoint((-tw/2, 0), facingAngl), lastPoint)]

        pygame.draw.polygon(screen, (0, 0, 0), triangleVertices)
        #display(quat)
        #display((w[0], w[1], w[2]))

        pygame.display.update()
    return

def translatePoint(point, at):
    return (point[0] + at[0], point[1] + at[1])

def rotatePoint(point, angl):
    return (np.cos(angl) * point[0] + np.sin(angl) * point[1],
            -np.sin(angl) * point[0] + np.cos(angl) * point[1])

def processInput(ser, incomplete_line, bufferedLines):
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
                return (x1, y1, x2, y2, x3, y3, incomplete_line, bufferedLines)
            except ValueError:
                print "ValueError"
    return (None, None, None, None, None, None, incomplete_line, bufferedLines)

def calcZOnSphere(x, y):
    sX = x/10.
    sY = y/10.
    sq = np.sqrt(1 - sX*sX - sY*sY) - 1 
    return sq*10

if __name__ == '__main__':
    main()
