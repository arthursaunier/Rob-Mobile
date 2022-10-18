__author__ = 'Jacques saraydaryan'

from AbstractShortPath import AbstractShortPath
from visualization_msgs.msg import MarkerArray
import math
import rospy


# import sys
# sys.path.append('../')


class Dijsktra(AbstractShortPath):
    SLEEP_TIME_BEFORE_NEXT_ITERATION = 0.01
    def __init__(self):
        print('')


    def goto(self, source, target, matrix, pub_marker, marker_container):
        prev = {}
        #nodes to visit
        unvisited = []
        #dic with node score
        score = {}
        
        INF = 9999

        # Condition to stop the path finding algo
        end = False
        print('start processing')

        # Intialisation
        for i in range(len(matrix)):
            for j in range(len(matrix[0])):
                # all nodes receive a score of INF
                score[str(i) + '_' + str(j)] = INF
                # all nodes are added to the list to process
                unvisited.append({'x': i, 'y': j})
        # score of the start node is set to 0
        score[str(source['x']) + '_' + str(source['y'])] = 0
        print('end initialisation phase')

        # while their is node to process or goal is reached (early exit)
        while len(unvisited) != 0 and not end:
            # get the node with the lowest score
            u = self.minScore(score, unvisited)
            #print('current Node:' + str(u))
            # remove the current node to the node to process list
            unvisited.remove(u)
            # create a visual information
            #self.createClosedMarker(u, marker_container)
            self.createClosedMarkerPt(u, marker_container)

            # get the list of the neighbors of the current node
            currentNeighbors = self.getNeighbors(u, matrix)
            # for all neighbors
            for v in currentNeighbors:
                # check that the current node has not already be processed
                if self.inU(v, unvisited):
                    # create a visual information
                    # self.createFontierUnitMarker(v, marker_array)
                    self.createFontierUnitMarkerPt(v, marker_container)
                    # update the score of the current neighbor with the estimate distance between the neighbors and
                    # the target (heuristic)
                    current_score = score[str(u['x']) + '_' + str(u['y'])] +1

                    if current_score < score[str(v['x']) + '_' + str(v['y'])]: 
                        #print('Neighbor:' + str(v) + ', hn:' + str(self.hn(matrix, v, target)))
                        # update precedence of the current neighbor
                        score[str(v['x']) + '_' + str(v['y'])] = current_score
                        prev[str(v['x']) + '_' + str(v['y'])] = str(u['x']) + '_' + str(u['y'])
                    # check if the current neighbor is the target
                    if str(v['x']) + '_' + str(v['y']) == str(target['x']) + '_' + str(target['y']):
                        # end the path computation
                        end = True
            # publish visual information
            pub_marker.publish(marker_container)
            #marker_container = self._create_marker_container()
            # wait before next iteration
            #rospy.sleep(self.SLEEP_TIME_BEFORE_NEXT_ITERATION)
        print(str(prev))
        return prev

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
        
