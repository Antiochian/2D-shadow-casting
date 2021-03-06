#! /usr/bin/env python

"""
Main function for raycasting
"""
import sys
import math
import pygame
import time
import numpy as np
import numpy.linalg as la

global col, Nx, Ny
Nx, Ny = 400,640 #screen size
col = {
        "BGCOLOR" : (100,100,100),
        "LIGHTCOL" : (255,255,255),
        "SHAPECOL" : (255,0,0),
        "TRANSPARENT" : (0,0,0,0)
        }

class Ray:
    #ray objects have a position and unit direction vector, and have the "cast" method to detect collisions
    def __init__(self,pos,direction):
        self.pos = np.array(pos) #originating location
        self.direction = 10*np.array(direction)/la.norm(np.array(direction)) #pointing vector
        self.color = (255,255,255)

    def show(self):
        global RAY_SURF
        end = self.cast()
        if end:
            #round end:
            end = int(end[0]), int(end[1])
            pygame.draw.line(RAY_SURF, self.color, self.pos, end,1)

    def cast(self):
        """
        Req for valid intersection: 
        maths part: https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
        0 < t < 1 #inside of wall
        0 < u #in front of ray
        Complexity increases linearly with number of edges in scene
        """
        global WALLS
        #RAY LINE
        x1 , y1  = tuple(self.pos)
        x2, y2  = tuple(self.pos + self.direction)
        best = float('inf') #pick closest collision, initial best = inf
        res = None
        for wall in WALLS:
            x3, y3 = tuple(wall.start)
            x4, y4 = tuple(wall.end)
            
            unum = (x1-x2)*(y1-y3) - (y1-y2)*(x1-x3)
            tnum = (x1 - x3)*(y3-y4) - (y1-y3)*(x3-x4)
            denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
            if denom == 0:
                denom = 0.001
            t = tnum/denom
            u = -unum/denom #MINUS SIGN IMPORTANT

            if t >= 0 and 0 <= u <= 1:
                Px = x1 + t*(x2-x1)
                Py = y1 + t*(y2-y1)
                dist = (Px-x1)**2 + (Py-y1)**2
                if dist < best:
                    best = dist
                    res = (Px,Py)
        return res


class Boundary:
    def __init__(self,ax,ay,bx,by):
        global WALLS
        self.start = np.array( (ax,ay) )
        self.end = np.array( (bx,by) )
        self.color = (255,0,0)
        if any(self.start != self.end):
            WALLS.append(self)

    def show(self):
        #for debugging, usually redundant due to seperate draw.polygon()
        global GEOMETRY_SURF
        pygame.draw.line(GEOMETRY_SURF, self.color, self.start, self.end)

def angle2pointer(rad):
    """ convert angle to pointer vector"""
    return ( math.cos(rad), math.sin(rad) )
    
def make_polygon(*coords):
    """ make polygon from list of coords """
    global GEOMETRY_SURF, POLYGONS,col
    if len(coords) < 3:
        print("Warning: Invalid polygon passed, ignoring...")
        return
    start = coords[0]
    prev = coords[0]
    for coord in coords:
        POLYGONS |= {coord}
        line = Boundary(prev[0],prev[1],coord[0],coord[1]) #add segment to WALL list
        prev = coord
    line = Boundary(prev[0], prev[1],start[0],start[1])
    #now draw poly
    pygame.draw.polygon(GEOMETRY_SURF,col["SHAPECOL"], coords)
    return

def decorate_scene():
    """ Place various polygons around screen"""
    make_polygon( (100,100),(120,140),(270,70) )
    make_polygon( (300,10), (300,550), (340,452),(380,300), (330,50))
    make_polygon( (200,450), (100,450), (100,500), (200,500) )
    make_polygon( (130,320), (150,300), (140,280) )
    return

def cast_rays(pos):
    """Cast rays all around from pos"""
    global POLYGONS
    dtheta = 0.01
    coll = []
    for vertex in POLYGONS: 
        dx = vertex[0] - pos[0]
        dy = vertex[1] - pos[1]
        angle = math.atan2(dy,dx)
        rays = (Ray(pos,angle2pointer(angle-dtheta)) , Ray(pos,angle2pointer(angle)) , Ray(pos,angle2pointer(angle+dtheta)))
        opts = (rays[0].cast(), rays[1].cast(), rays[2].cast())
        if opts[0] != None:
            coll.append(( angle-dtheta, (int(opts[0][0]),int(opts[0][1])) ))
        if opts[1] != None:
            coll.append(( angle, (int(opts[1][0]),int(opts[1][1])) ))
        if opts[2] != None:
            coll.append(( angle+dtheta, (int(opts[2][0]),int(opts[2][1])) ))
    shader_vertices = [x[1] for x in sorted(coll)]
    return shader_vertices

def main():
    global col,Nx, Ny, RAY_SURF, GEOMETRY_SURF, WALLS, POLYGONS
    pygame.init()
    clock = pygame.time.Clock()
    window = pygame.display.set_mode( (Nx,Ny) )
    window.fill(col["BGCOLOR"])
    GEOMETRY_SURF = pygame.Surface( (Nx,Ny), pygame.SRCALPHA )
    GEOMETRY_SURF.fill(col["TRANSPARENT"])
    RAY_SURF = pygame.Surface ( (Nx,Ny) , pygame.SRCALPHA)
    RAY_SURF.fill(col["TRANSPARENT"])
      
    decorate_scene()
    
    window.blits( ( (RAY_SURF, (0,0)), (GEOMETRY_SURF, (0,0)) ))
    pygame.display.flip()
    print(len(WALLS), " walls generated") 
    FPS = 30
    t0 = time.time()
    while True:
        #print(time.time()-t0)
        #t0 = time.time()
        clock.tick(FPS)
        for event in pygame.event.get():
            if event.type == pygame.QUIT or pygame.key.get_pressed()[27]: #detect attempted exit
                pygame.quit()
                sys.exit()
        RAY_SURF.fill((0,0,0))
        shader_vertices = cast_rays(pygame.mouse.get_pos())
        pygame.draw.polygon(RAY_SURF, col["LIGHTCOL"], shader_vertices)
        window.blits( ( (RAY_SURF, (0,0)), (GEOMETRY_SURF, (0,0)) ) ) 
        pygame.display.update()

global WALLS, POLYGONS, RAY_SURF, GEOMETRY_SURF
#WALLS is a list of Boundary objects in the scene. Initialises with the 4 sides of the screen
WALLS = [Boundary(0,0, Nx, 0), Boundary(0,0, 0 ,Ny), Boundary(0,Ny,Nx, Ny), Boundary(Nx,0,Nx,Ny)]
#POLYGONS is a set all geometry vertexes on the map, that will later become a polygon to flood-fill
POLYGONS = set(( (0,0), (0,Ny), (Nx,0), (Nx,Ny)  )) #initialise with the 4 corners of the screen

if __name__ == "__main__":
    main()
