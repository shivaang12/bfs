import pygame
#from PIL import Image, ImageDraw
import time
import numpy as np
import math
from sys import maxint
from heapq import heappop, heappush
#from scipy.stats import linregress


class astar:
    def __init__(self, area, poly, rect, cir):
        self.extreme_point = area
        self.poly = poly
        self.rect = rect
        self.cir = cir
        self.obs_array = []
        self.node_info = {}
        #self.graphix = graphix
        self.PARENT = 0
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
            returnList.append((c_point[0] + 1, c_point[1]))
        if (c_point[0] + 1 < 250) and (c_point[1] - 1 >= 0):
            returnList.append((c_point[0] + 1, c_point[1] - 1))
        if (c_point[0] + 1 < 250) and (c_point[1] + 1 < 150):
            returnList.append((c_point[0] + 1, c_point[1] + 1))
        if (c_point[0] - 1 >= 0) and (c_point[1] - 1 >= 0):
            returnList.append((c_point[0] - 1, c_point[1] - 1))
        if (c_point[0] - 1 >= 0) and (c_point[1] + 1 < 150):
            returnList.append((c_point[0] - 1, c_point[1] + 1))

        return returnList

    def ccw(self, A, B, C):
        # Checking the range
        return ((C[1] - A[1]) * (B[0] - A[0])) > ((B[1] - A[1]) * (C[0] - A[0]))

    def intersect(self, pos, line_segment):
        # This function checks if the Ray of the point is interecting the line segments of the polygons.
        # for estimating the optimal polygon points, need to check with more than one array which I haven't done.
        a = self.ccw(pos, self.extreme_point, line_segment[0]) != self.ccw(pos, self.extreme_point, line_segment[1]) and self.ccw(
            pos, line_segment[0], line_segment[1]) != self.ccw(self.extreme_point, line_segment[0], line_segment[1])
        if a == True:
            return 1
        if a == False:
            return 0

    def check_cir(self, pos):
        if math.sqrt((pos[0] - ((self.cir[0] + self.cir[2]) / 2))**2 + (pos[1] - ((self.cir[1] + self.cir[3]) / 2))**2) <= 15:
            return True
        else:
            return False

    def check_rect(self, pos):
        if (pos[0] >= self.rect[0][0]) & (pos[0] <= self.rect[1][0]) & (pos[1] >= self.rect[0][1]) & (pos[1] <= self.rect[1][1]):
            return True
        else:
            return False

    def check_poly(self, pos):
        # Plan is to check the intersection
        container_var = 0
        for x in range(len(poly)):
            container_var = container_var + \
                self.intersect(pos, [poly[x - 1], poly[x]])
            #print container_var
        if (container_var % 2) == 0:
            return False
        else:
            return True

    def make_obs_hp(self, pos):
        # checking the point if it's in the obstacle
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

    def get_h_cost(self, s_pos):
        g_pos = self.gp
        return math.sqrt((s_pos[0] - g_pos[0])**2 + (s_pos[1] - g_pos[1])**2)

    def get_g_cost(self, s_pos, g_pos):
        return math.sqrt((s_pos[0] - g_pos[0])**2 + (s_pos[1] - g_pos[1])**2)

    def start_astar(self, s_pos, g_pos):
        self.sp = s_pos
        self.gp = g_pos

        # Setting up some params
        # Vector maps names corresponding number
        F, H, NUM, G, POS, OPEN, VALID, PARENT = xrange(8)
        self.PARENT = PARENT
        number = iter(xrange(maxint))
        start_h = self.get_h_cost(s_pos)
        start_node = [0 + start_h, start_h, number.next(), 0, s_pos, True, True, None]
        self.node_info[s_pos] = start_node
        heap = [start_node]
        best_node = start_node
        #print best_node

        while heap:
            current_node = heappop(heap)
            current_node[OPEN] = False

            if current_node[POS] == self.gp:
                best_node = current_node
                break

            childs = self.getChild(current_node[POS])
            permited_childs = []
            for obj in childs:
                temp = self.make_obs_hp(obj)
                #print temp
                if temp == False:
                    permited_childs.append(obj)
            #print permited_childs
            #aa = raw_input('Press anything to terminate')
            #################################################################
            #for obj in permited_childs:
            #	if self.graphix == True:            		
	        #        pygame.draw.line(self.disp, (0, 255, 0), obj, obj)
	        #        pygame.display.update()

            for each_child in permited_childs:
                new_child_g = current_node[G] + self.get_g_cost(current_node[POS], each_child)
                new_child_node = self.node_info.get(each_child)
                # We will make a new one or we will update if exist
                if new_child_node is None:
                    new_child_h = self.get_h_cost(each_child)
                    new_child_node = [new_child_g + new_child_h, new_child_h, number.next(), new_child_g, each_child, True, True, current_node[POS]]
                    self.node_info[each_child] = new_child_node
                    heappush(heap, new_child_node)
                    #print best_node
                    if new_child_h < best_node[H]:
                        best_node = new_child_node

                elif new_child_g < new_child_node[G]:
                    if new_child_node[OPEN]:
                        #We will not change it but add new one which will be the updated version of the previous one and make the previous one in valid
                        #if self.graphix == True:
                        #	pygame.draw.line(self.disp, (0, 255, 0), each_child, each_child)
                        #	pygame.display.update()
                        new_child_node[VALID] = False
                        self.node_info[each_child] = new_child_node = new_child_node[:]
                        new_child_node[F] = new_child_g + new_child_node[H]
                        new_child_node[NUM] = number.next()
                        new_child_node[G] = new_child_g
                        new_child_node[VALID] = True
                        new_child_node[PARENT] = current_node[POS]
                        heappush(heap, new_child_node)
                    else:
                        new_child_node[F] = new_child_g + new_child_node[H]
                        new_child_node[G] = new_child_g
                        new_child_node[PARENT] = current_node[POS]
                        new_child_node[OPEN] = True
                        heappush(heap, new_child_node)

            while heap and not heap[0][VALID]:
                heappop(heap)
        self.global_best = best_node

    def make_path(self):
        path = []
        current_node = self.global_best
        PARENT = self.PARENT
        while current_node[PARENT] is not None:
            path.append(current_node[4])
            current_node = self.node_info[current_node[PARENT]]
        path.reverse()
        for i in path:
            pygame.draw.line(self.disp, (100, 0, 255), i, i)
            pygame.display.update()
        aa = raw_input('Press anything to terminate')
        exit()


if __name__ == '__main__':
    print "Arena starts is of size (0-249)(0-149), so define the points accordingly."
    start_point = tuple(map(int, raw_input(
        "Enter Starting point in the format: example - 'x,y' : ").split(',')))
    goal_point = tuple(map(int, raw_input(
        "Enter goal point in the format: example - 'x,y' : ").split(',')))
    #goal_point = (250,149-149)
    #graph = str(raw_input("Want to turn on the process graphix? t or f : "))
    #if graph=='t':
    #    graphix = True
    #if graph=='f':
    #    graphix = False
    start_point = (start_point[0], 149 - start_point[1])
    goal_point = (goal_point[0], 149 - goal_point[1])
    poly = [(120, 149 - 55), (158, 149 - 51), (165, 149 - 89),
            (188, 149 - 51), (168, 149 - 14), (145, 149 - 14)]
    circ = (180 - 15, 149 - 120 - 15, 180 + 15, 149 - 120 + 15)
    square = [(55, 149 - 90 - (45 / 2)), (55 + 50, 149 - 90 + (45 / 2))]
    area = (250, 150)
    a = astar(area, poly, square, circ)
    if a.make_obs_hp(start_point):
        print "Start point is in the obtacle";
        exit()
    if a.make_obs_hp(goal_point):
        print "Goal point is in the obtacle";
        exit()
    a.start_astar(start_point, goal_point)
    a.make_path()
