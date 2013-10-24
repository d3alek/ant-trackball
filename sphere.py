from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
import pygame
from pygame.locals import *
import sys
import numpy as np
import serial
import subprocess


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
    rotX = rotY = 0

    subprocess.call(['sudo', 'chmod', '777', '/dev/ttyACM0'])
    ser = serial.Serial("/dev/ttyACM0", timeout=0)

    lines = []
    incomplete_line = None
    
    while True:
        lines = ser.readlines()
        if incomplete_line:
            lines[0] = incomplete_line + lines[0]
        if len(lines) > 0:
            print "\n".join(lines)
            if lines[-1][-1] != '\n':
                incomplete_line = lines[-1];
                lines = lines[:-1]
            else:
                incomplete_line = None
            for line in lines:
                ss = line.split()
                x = y = 0
                if len(ss) is not 6:
                    print "problem ss len is not 6!"
                else:
                    x = int(ss[2])
                    y = int(ss[5])
                rotX += x
                rotY -= y

        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == KEYDOWN:
                if event.key == K_LEFT:
                    print "left" + str(i)
                    rotX += -np.pi/10;
                if event.key == K_RIGHT: 
                    print "right" + str(i)
                    rotX += np.pi/10;
                if event.key == K_UP:
                    print "up" + str(i)
                    rotY += np.pi/10;
                if event.key == K_DOWN:
                    print "down" + str(i)
                    rotY += -np.pi/10;
                if event.key == K_ESCAPE:
                    pygame.event.post(pygame.event.Event(QUIT))
        display(rotX, rotY)
        pygame.display.flip()
        clock.tick(30)
    return

def display(rotX, rotY):
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    glPushMatrix()
    currentModelViewMatrix = glGetDouble(GL_MODELVIEW_MATRIX)
    glRotatef(rotX, currentModelViewMatrix[0][1], currentModelViewMatrix[1][1],
              currentModelViewMatrix[2][1])
    currentModelViewMatrix = glGetDouble(GL_MODELVIEW_MATRIX)
    glRotatef(rotY, currentModelViewMatrix[0][0], currentModelViewMatrix[1][0],
              currentModelViewMatrix[2][0])
    color = [1.0,0.,0.,1.]
    glutWireSphere(2,20,20)
    glPopMatrix()
    return

if __name__ == '__main__': main()
