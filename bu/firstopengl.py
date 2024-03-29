SCREEN_SIZE = (800, 600)

from math import radians 
import numpy as np

from OpenGL.GL import *
from OpenGL.GLU import *

import pygame
from pygame.locals import *


def resize(width, height):
    
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(60.0, float(width)/height, .1, 1000.)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init():
    
    glEnable(GL_DEPTH_TEST)
    
    glShadeModel(GL_FLAT)
    glClearColor(1.0, 1.0, 1.0, 0.0)

    glEnable(GL_COLOR_MATERIAL)
    
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)        
    glLight(GL_LIGHT0, GL_POSITION,  (0, 1, 1, 0))    
    
    
class Cube(object):
    
    
    def __init__(self, position, color):
        
        self.position = position
        self.color = color
    
    num_faces = 6
        
    vertices = [ (0.0, 0.0, 1.0),
                 (1.0, 0.0, 1.0),
                 (1.0, 1.0, 1.0),
                 (0.0, 1.0, 1.0),
                 (0.0, 0.0, 0.0),
                 (1.0, 0.0, 0.0),
                 (1.0, 1.0, 0.0),
                 (0.0, 1.0, 0.0) ]
        
    normals = [ (0.0, 0.0, +1.0),  # front
                (0.0, 0.0, -1.0),  # back
                (+1.0, 0.0, 0.0),  # right
                (-1.0, 0.0, 0.0),  # left 
                (0.0, +1.0, 0.0),  # top
                (0.0, -1.0, 0.0) ] # bottom
    
    vertex_indices = [ (0, 1, 2, 3),  # front
                       (4, 5, 6, 7),  # back
                       (1, 5, 6, 2),  # right
                       (0, 4, 7, 3),  # left
                       (3, 2, 6, 7),  # top
                       (0, 1, 5, 4) ] # bottom    

    def render(self):                
        
        glColor( self.color )
    
        # Adjust all the vertices so that the cube is at self.position
        vertices = [tuple(Vector3(v) + self.position) for v in self.vertices]
            
        # Draw all 6 faces of the cube
        glBegin(GL_QUADS)
    
        for face_no in xrange(self.num_faces):
                        
            glNormal3dv( self.normals[face_no] )
            
            v1, v2, v3, v4 = self.vertex_indices[face_no]
                    
            glVertex( vertices[v1] )
            glVertex( vertices[v2] )
            glVertex( vertices[v3] )
            glVertex( vertices[v4] )            
        
        glEnd()
        

class Sphere():
    
    def __init__(self):
        self.display_list = None
        self.cubes = []
        gl_col = (255.0, 0.0, 0.0)
        self.cubes.append(Cube(np.array([0, 0, 0]), gl_col))
    
    def render(self):
                
        if self.display_list is None:
            
            # Create a display list
            self.display_list = glGenLists(1)                
            glNewList(self.display_list, GL_COMPILE)
            
            # Draw the cubes
            for cube in self.cubes:
                cube.render()
                
            # End the display list
            glEndList()
            
        else:
            # Render the display list            
            glCallList(self.display_list)
                    

def run():
    
    pygame.init()
    screen = pygame.display.set_mode(SCREEN_SIZE, HWSURFACE|OPENGL|DOUBLEBUF)
    
    resize(*SCREEN_SIZE)
    init()
    
    clock = pygame.time.Clock()    
    
    glMaterial(GL_FRONT, GL_AMBIENT, (0.1, 0.1, 0.1, 1.0))    
    glMaterial(GL_FRONT, GL_DIFFUSE, (1.0, 1.0, 1.0, 1.0))

    # This object renders the 'map'
    map = Sphere()        

    # Camera transform matrix
    camera_matrix = Matrix44()
    camera_matrix.translate = (10.0, .6, 10.0)

    # Initialize speeds and directions
    rotation_direction = Vector3()
    rotation_speed = radians(90.0)
    movement_direction = Vector3()
    movement_speed = 5.0    

    while True:
        
        for event in pygame.event.get():
            if event.type == QUIT:
                return
            if event.type == KEYUP and event.key == K_ESCAPE:
                return                
            
        # Clear the screen, and z-buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                        
        #time_passed = clock.tick()
        #time_passed_seconds = time_passed / 1000.
        
        pressed = pygame.key.get_pressed()
        
        # Reset rotation and movement directions
        rotation_direction.set(0.0, 0.0, 0.0)
        movement_direction.set(0.0, 0.0, 0.0)
        
        # Modify direction vectors for key presses
        if pressed[K_LEFT]:
            rotation_direction.y = +1.0
        elif pressed[K_RIGHT]:
            rotation_direction.y = -1.0
        if pressed[K_UP]:
            rotation_direction.x = -1.0
        elif pressed[K_DOWN]:
            rotation_direction.x = +1.0
        if pressed[K_z]:
            rotation_direction.z = -1.0
        elif pressed[K_x]:
            rotation_direction.z = +1.0            
        
        # Calculate rotation matrix and multiply by camera matrix    
        rotation = rotation_direction * rotation_speed * time_passed_seconds
        rotation_matrix = Matrix44.xyz_rotation(*rotation)        
        camera_matrix *= rotation_matrix
        
        # Upload the inverse camera matrix to OpenGL
        glLoadMatrixd(camera_matrix.get_inverse().to_opengl())
                
        # Light must be transformed as well
        glLight(GL_LIGHT0, GL_POSITION,  (0, 1.5, 1, 0)) 
                
        # Render the map
        map.render()
        
                
        # Show the screen
        pygame.display.flip()
        fpsClock.tick(30)

run()
