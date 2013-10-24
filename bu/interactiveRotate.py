#!/usr/bin/python

import pygame, sys
from pygame.locals import *
from pylab import *
import numpy as np
import time
 
## This function calculate the projected image coordinates form the 3D
## coordinates of the object vertices.
def project(D, vecs):
    return vvs[:,:2]/(vvs[:,[2,2]]-D)

def ff(x):
    return f(x[0]), f(x[1]) 
 
def f(x):
    return int(x*500)+300 


def getXRotMatrix(angle):
    return np.array([1, 0, 0, 0, np.cos(angle), -np.sin(angle), 0, np.sin(angle),
                     np.cos(angle)]).reshape((3, 3))

def getYRotMatrix(angle):
    return np.array([np.cos(angle), 0, np.sin(angle), 0, 1, 0, -np.sin(angle),
                     0, np.cos(angle)]).reshape((3, 3))
 
def getZRotMatrix(angle):
    return np.array([np.cos(angle), -np.sin(angle), 0, np.sin(angle),
                     np.cos(angle), 0, 0, 0, 1]).reshape((3, 3))
 
## The cube vertices
vs = reshape(mgrid[-1:2:2,-1:2:2,-1:2:2].T, (8,3))
 
## Generate the list of connected vertices
ed=[(j,k)
    for j in range(8)
    for k in range(j,8)
    if sum(abs(vs[j]-vs[k]))==2 ]
 
 
## Camera position
D=-5
 
## Create the figure. figsize needed to set the image sizes at the end...
pygame.init()
fpsClock = pygame.time.Clock()
windowSurfaceObj = pygame.display.set_mode((640, 480))
pygame.display.set_caption("Ant Trackball")
whiteColor = pygame.Color(255, 255, 255)
 
##
## Now start a loop over the animation frames. The index is used both
## to name the images and to calculate the rotation matrix. You could
## use this index to make some physics stuff...

objRotM = np.eye(3)
i = 0

while(True):

    rotM = np.eye(3)

    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
        elif event.type == KEYDOWN:
            if event.key == K_LEFT:
                print "left" + str(i)
                rotM = getYRotMatrix(-np.pi/10)
            if event.key == K_RIGHT:
                print "right" + str(i)
                rotM = getYRotMatrix(np.pi/10)
            if event.key == K_UP:
                print "up" + str(i)
                rotM = getXRotMatrix(np.pi/10)
            if event.key == K_DOWN:
                print "down" + str(i)
                rotM = getXRotMatrix(-np.pi/10)

            if event.key == K_ESCAPE:
                pygame.event.post(pygame.event.Event(QUIT))

    ## This is crucial, clears the figure for the new plot.
    windowSurfaceObj.fill(whiteColor)

    objRotM1 = np.dot(rotM, objRotM)

    ## Calculate the 3D coordinates of the vertices of the rotated
    ## cube. Just multiply the vectors by the rotation matrix...
    vvs=dot(vs,objRotM1)

    objRotM = np.array(objRotM1)
 
    ## Now calculate the image coordinates of the points.
    pt = project(D,vvs)
 
    ## Plot the edges.
    for j,k in ed:
        ffs = ff(pt[[j,k],0]), ff(pt[[j,k],1])
        start = [ffs[0][0], ffs[1][0]]
        end = [ffs[0][1], ffs[1][1]]
        pygame.draw.line(windowSurfaceObj, (0, 0, 0), start, end ,3)
 
    ## Plot the vertices.
#    ax.plot(pt[:,0], pt[:,1], 'bo')
    for (x, y) in pt:
        pygame.draw.circle(windowSurfaceObj, (0, 0, 0), (f(x),
                                                         f(y)), 3, 0)
 
    ## Set axes limits
#    ax.axis('equal')
#    ax.axis([-0.5,0.5,-0.5,0.5])
 
    ## Save the current frame. We need the dpi (along with the figure
    ## size up there) to set the image size.
#    savefig('anim%03d.png'%ind, dpi=100)
#    plt.draw()
#    time.sleep(0.05)
    pygame.display.update()
    fpsClock.tick(30)
    i+=1
