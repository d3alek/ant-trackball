import pygame
from pygame.locals import *
import sys
import trackball
import numpy as np

SCREEN_SIZE = (800, 600)

name = 'Ant Trackball'

def main():
    pygame.init()

    clock = pygame.time.Clock()

    screen = pygame.display.set_mode(SCREEN_SIZE)
    whiteColor = pygame.Color(255, 255, 255)

    ant_trackball = trackball.Trackball()
    ant_trackball.start()
    ant_trackball.getPitchYawRoll()

    startPoint = (400, 300)
    antPath = [startPoint]
    pitchYawRoll = []

    tw = 10 
    th = 15
    facingAngl = 0

    while True:
        screen.fill(whiteColor)

        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                ant_trackball.stop()
                sys.exit()

            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    pygame.event.post(pygame.event.Event(QUIT))
                if event.key == K_2:
                    None
                    #TWO_SENSORS = True
                    #print "Switching to two sensors"
                if event.key == K_3:
                    None
                    #TWO_SENSORS = False 
                    #print "Switching to three sensors"
                if event.key == K_8:
                    ant_trackball.setServoAngle(
                        trackball.SERVO_YAW,
                        30,
                        1)
                if event.key == K_9:
                    ant_trackball.setServoAngle(
                        trackball.SERVO_YAW,
                        0,
                        1)
                if event.key == K_0:
                    ant_trackball.setServoAngle(
                        trackball.SERVO_YAW,
                        -30,
                        1)

                if event.key == K_4:
                    ant_trackball.setServoAngle(
                        trackball.SERVO_PITCH,
                        -30,
                        1)
                if event.key == K_5:
                    ant_trackball.setServoAngle(
                        trackball.SERVO_PITCH,
                        0,
                        1)


                    #ser.write("5\n")
                if event.key == K_6:
                    ant_trackball.setServoAngle(
                        trackball.SERVO_PITCH,
                        30,
                        1)


                    #ser.write("6\n")
                if event.key == K_c:
                    antPath = [startPoint]
                    facingAngl = 0
            
        pitchYawRoll += ant_trackball.getPitchYawRoll()
        
        if len(pitchYawRoll) > 0:
            pitch, yaw, roll = pitchYawRoll[0]
            pitchYawRoll = pitchYawRoll[1:]

            lastPoint = antPath[-1]
            antPath += [translatePoint((pitch, roll), lastPoint)]
            facingAngl += yaw/100
            #print yaw/100

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

        pygame.display.update()
        clock.tick(300)

def translatePoint(point, at):
    return (point[0] + at[0], point[1] + at[1])

def rotatePoint(point, angl):
    return (np.cos(angl) * point[0] + np.sin(angl) * point[1],
            -np.sin(angl) * point[0] + np.cos(angl) * point[1])

if __name__ == '__main__':
    main()
