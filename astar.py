import pygame
#from PIL import Image, ImageDraw
import time
import numpy as np
import math
from scipy.stats import linregress

class astar:
    def __init__(self, area, poly, rect, cir):
        self.extreme_point = area
        self.poly = poly
        self.rect = rect
        self.cir = cir
        self.obs_array = []
        #self.disp = pygame.display.set_mode(self.extreme_point)
        #self.build_world()

    def getChild(self, c_point):
        returnList = []
        #print c_point
        if (c_point[1] - 1 >= 0):
            returnList.append((c_point[0], c_point[1] - 1))
        if (c_point[1] + 1 < 150):
            returnList.append((c_point[0], c_point[1] + 1))
        if (c_point[0] - 1 >= 0):
            returnList.append((c_point[0] - 1, c_point[1]))
        if (c_point[0] + 1) < 250:
            returnList.append((c_point[0]  + 1, c_point[1]))
        if (c_point[0] + 1 < 250) and (c_point[1] - 1 >= 0):
            returnList.append((c_point[0] + 1, c_point[1] - 1))
        if (c_point[0] + 1 < 250) and (c_point[1] + 1 < 150):
            returnList.append((c_point[0] + 1, c_point[1] + 1))
        if (c_point[0] - 1 >= 0) and (c_point[1] - 1 >= 0):
            returnList.append((c_point[0] - 1, c_point[1] - 1))
        if (c_point[0] - 1 >= 0) and (c_point[1] + 1 < 150):
            returnList.append((c_point[0] - 1, c_point[1] + 1))

        return returnList

    def ccw(self,A,B,C):
        #Checking the range
        return ((C[1]-A[1]) * (B[0]-A[0])) > ((B[1] - A[1]) * (C[0] - A[0]))

    def intersect(self, pos, line_segment):
        #This function checks if the Ray of the point is interecting the line segments of the polygons.
        #for estimating the optimal polygon points, need to check with more than one array which I haven't done.
        a = self.ccw(pos, self.extreme_point, line_segment[0]) != self.ccw(pos, self.extreme_point, line_segment[1]) and self.ccw(pos, line_segment[0], line_segment[1]) != self.ccw(self.extreme_point, line_segment[0], line_segment[1])
        if a == True:
            return 1
        if a == False:
            return 0


    def check_cir(self, pos):
        if math.sqrt((pos[0] - ((self.cir[0] + self.cir[2])/2))**2 + (pos[1] - ((self.cir[1] + self.cir[3])/2))**2) <= 15 :
            return True
        else:
            return False


    def check_rect(self, pos):
        if (pos[0] >= self.rect[0][0]) & (pos[0] <= self.rect[1][0]) & (pos[1] >= self.rect[0][1]) & (pos[1] <= self.rect[1][1]) :
            return True
        else:
            return False

    def check_poly(self, pos):
        #Plan is to check the intersection
        container_var = 0
        for x in range(len(poly)):
            container_var = container_var + self.intersect(pos, [poly[x-1], poly[x]])
            #print container_var
        if (container_var % 2) == 0:
            return False
        else:
            return True


    def make_obs_hp(self, pos):
        #checking the point if it's in the obstacle
        if self.check_rect(pos) or self.check_cir(pos) or self.check_poly(pos):
            return True
        else:
            return False

    def start_astar(self, s_pos, g_pos):
        self.sp = s_pos
        self.gp = g_pos



if __name__ == '__main__':
    print "Arena starts is of size (0-249)(0-149), so define the points accordingly."
    start_point = tuple(map(int,raw_input("Enter Starting point in the format: example - 'x,y' : ").split(',')))
    goal_point = tuple(map(int,raw_input("Enter goal point in the format: example - 'x,y' : ").split(',')))
    #goal_point = (250,149-149)
    start_point = (start_point[0], 149 - start_point[1])
    goal_point = (goal_point[0], 149 - goal_point[1])
    poly = [(120, 149 - 55), (158, 149 - 51), (165, 149 - 89), (188, 149 - 51), (168, 149 - 14), (145, 149 - 14)]
    circ = (180 - 15, 149 - 120 - 15, 180 + 15, 149 - 120 + 15)
    square = [( 55 , 149 - 90 - (45/2)), (55 + 50 , 149 - 90 + (45/2))]
    area = (250,150)
    a = astar(area, poly, square, circ)
