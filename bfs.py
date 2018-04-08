import pygame
#from PIL import Image, ImageDraw
import time
import numpy as np
import math
from scipy.stats import linregress

class bfs:
    def __init__(self, area, poly, rect, cir):
        self.extreme_point = area
        self.poly = poly
        self.rect = rect
        self.cir = cir
        self.obs_array = []
        self.disp = pygame.display.set_mode(self.extreme_point)
        self.build_world()

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

    def build_world(self):
        #This is for the gui
        blank_world = np.zeros(self.extreme_point, dtype='int')
        for x in range(self.extreme_point[0]):
            for y in range(self.extreme_point[1]):
                if self.make_obs_hp((x,y)):
                    blank_world[x][y] = 1
                    pygame.draw.line(self.disp, (255, 0, 0), (x,y), (x,y))
        pygame.display.update()


    def start_bfs(self, pos, epos):
        start_time = time.time()
        self.sp = pos
        closed_point = []
        queue = []
        self.node_info = {}
        current_point = pos
        while(current_point != epos):
            childs = self.getChild(current_point)
            valid_child = [c for c in childs if hash(c) not in closed_point]
            valid_child = [c for c in valid_child if c not in queue]
            #print valid_child
            permited_child = []

            for obj in valid_child:
                temp = self.make_obs_hp(obj)
                #print temp
                if temp == False:
                    permited_child.append(obj)
            #print permited_child
            for obj in permited_child:
                pygame.draw.line(self.disp, (0, 255, 0), obj, obj)
                pygame.display.update()
                self.node_info[obj] = current_point

            #print permited_child

            queue.extend(permited_child)
            closed_point.append(hash(current_point))
            #print queue
            #print closed_point
            try:
                current_point = queue.pop(0)

                #print current_point
                #print len(closed_point)
                #aa = raw_input()
            except:
                break
        total_time = time.time() - start_time
        print 'This BFS took ' + str(total_time) + ' seconds'
        self.find_path(epos)

    def find_path(self, goal):
        gp = goal
        while (gp != self.sp):
            pygame.draw.line(self.disp, (100, 0, 255), gp, gp)
            pygame.display.update()
            gp = self.node_info[gp]
            #print gp
        aa = raw_input('Press anything to terminate')
        exit()

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
    a = bfs(area, poly, square, circ)
    #Checking if the goal point is in the obstacle
    if a.make_obs_hp(start_point):
        print "Start point is in the obtacle";
        exit()
    if a.make_obs_hp(goal_point):
        print "Goal point is in the obtacle";
        exit()
    a.start_bfs(start_point, goal_point)
    #a.find_path(goal_point)
