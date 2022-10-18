__author__ = 'Jacques saraydaryan'

from AbstractShortPath import AbstractShortPath
import math
import rospy
from visualization_msgs.msg import MarkerArray


# sys.path.append('../')

class AStar(AbstractShortPath):
    SLEEP_TIME_BEFORE_NEXT_ITERATION = 0.001
    def __init__(self):
        print('')

    def goto(self, source, target, matrix, pub_marker, marker_container):
        start = {'x': source['x'], 'y': source['y']}

        prev = {}
        #nodes to visit
        closedlist = []
        openlist = []

        #dic with node score
        fscore = {}
        gscore = {}
        
        INF = 9999

        # Condition to stop the path finding algo
        end = False
        print('start processing')

        # Intialisation
        for i in range(len(matrix)):
            for j in range(len(matrix[0])):
                fscore[str(i) + '_' + str(j)] = INF
                gscore[str(i) + '_' + str(j)] = INF
    
        # score of the start node is set to 0
        fscore[str(source['x']) + '_' + str(source['y'])] = 0
        gscore[str(source['x']) + '_' + str(source['y'])] = 0
        openlist.append(start)
        print('end initialisation phase')

        # while their is node to process or goal is reached (early exit)
        while len(openlist) and not end:
            # get the node with the lowest score
            u = None
            u_score = INF
            for h in openlist:
                score = fscore[str(h['x']) + '_' + str(h['y'])]
                if score < u_score:
                    u_score = score
                    u = h
            #print('current Node:' + str(u))
            if str(u['x']) + '_' + str(u['y']) == str(target['x']) + '_' + str(target['y']):
                # end the path computation
                end = True
            
            openlist.remove(u)
            closedlist.append(u)
            self.createClosedMarkerPt(u, marker_container)

            # get the list of the neighbors of the current node
            currentNeighbors = self.getNeighbors(u, matrix)
            # for all neighbors
            for v in currentNeighbors:
                
                if self.inU(v, closedlist):
                   continue
                    
                v_score = gscore[str(u['x']) + '_' + str(u['y'])] + self.hn(matrix, u, v)
                if not self.inU(v, openlist):
                    self.createFontierUnitMarkerPt(v, marker_container)
                    openlist.append(v)
                elif v_score >= gscore[str(v['x']) + '_' + str(v['y'])]:
                    continue
                        
                prev[str(v['x']) + '_' + str(v['y'])] = str(u['x']) + '_' + str(u['y'])
                gscore[str(v['x']) + '_' + str(v['y'])] = v_score
                fscore[str(v['x']) + '_' + str(v['y'])] = gscore[str(v['x']) + '_' + str(v['y'])] + self.hn(matrix, v, target)
            
            pub_marker.publish(marker_container)
            rospy.sleep(self.SLEEP_TIME_BEFORE_NEXT_ITERATION)
            
        return prev

        ### TODO
        ###########################################################
        ################### Function Paramters ###################
        ###########################################################
        ### source: coordinate of the robot position source['x'] return the x position, source['y'] return the y position
        ###
        ### target: coordinate of the target position target['x'] return the x position, target['y'] return the y position
        ###
        ### matrix: rescaled map (including obstacles) matrix[i][j] return the value of the cell i,j of the matrix
        ###
        ### elf.MAP_OBSTACLE_VALUE: value of an obstacle into the matrix (-100)
        ###
        ### pub_marker: marker publisher to visualize information into rviz (usage pub_marker.publish(marker_container) )
        ###
        ### marker_container: marker container where where new marker are added as point
        ###
        ###########################################################
        ################### Function Toolboxes ###################
        ###########################################################
        #   # create a visual information
        #   self.createFontierUnitMarker(v, marker_container)
        #
        #    # publish visual information
        #    pub_marker.publish(marker_containers)
        #
        #    # create a visual information
        #    self.createClosedMarker(u, marker_container)
        #
        #
        #
        #
        #
        #
        #                       TODO
        #
        #
        #
        #
        #
        #
        ###
        ### prev:  disctionary holding node precedence
        ### CAUTION prev dictionary has to be completed as follow:
        ###
        ### prev[str(v['x']) + '_' + str(v['y'])] = str(u['x']) + '_' + str(u['y'])
        ###
        ### where v['x'] return the x position of the node v in the resized map
        ### where v['y'] return the y position of the node v in the resized map
        

    def minScore(self, fscore, frontier):
        """ Return the node that has the lowest score, information return like u={'x':5,'y':3}"""
        min = 9999
        min_coord = ''
        for n in frontier:
            if fscore[str(n['x']) + '_' + str(n['y'])] < min:
                min = fscore[str(n['x']) + '_' + str(n['y'])]
                min_coord = n
        return min_coord

    def getNeighbors(self, currentNode, matrix):
        """ Compute Neighbors of the current point, Return the list of the point neighbors in Cfree"""
        x_c = currentNode['x']
        y_c = currentNode['y']
        neighbors = []
        self.checkAndAdd(neighbors, x_c + 1, y_c, matrix)
        self.checkAndAdd(neighbors, x_c, y_c + 1, matrix)
        self.checkAndAdd(neighbors, x_c - 1, y_c, matrix)
        self.checkAndAdd(neighbors, x_c, y_c - 1, matrix)
        return neighbors

    def checkAndAdd(self, neighbors, x, y, matrix):
        """ Check that the candidate neighbor is valid == not an obstacle, in current bound, add the nieghbor node to
        the node list """
        if x > 0 and x < self.map_width and y > 0 and y < self.map_height:
            if matrix[y][x] != self.MAP_OBSTACLE_VALUE:
                neighbors.append({'x': x, 'y': y})
        return neighbors

    def inU(self, v, frontier):
        """ Check if the node is into the list, return boolean """
        return v in frontier

    def hn(self, matrix, source, destination):
        """Compute the distance between the given node and the target, the result is an estimation of the distance
        without taking into account obstacles """
        return math.sqrt(math.pow(source['x'] - destination['x'], 2) + math.pow(source['y'] - destination['y'], 2))


        
