import pygame
from RRTclasses import RRTGraph
from RRTclasses import RRTmap
import time
import csv
import numpy as np


def obstacle_loc():
    with open('position.csv', newline='') as csv_file:
        reader = csv.reader(csv_file, delimiter=',')
        cm = list(reader)
    return cm

def main():
    cm = obstacle_loc()
    # converting only first array in list to float for display.. can be changed easily for multiple obstacle
    for i in range(0, len(cm[0])):
        cm[0][i] = float(cm[0][i])
    dimension = (600, 1000)
    start = 50, 50
    goal = (1000, 450)
    ObsDim = 30
    ObsNum = 20
    # ObsNum = len(cm) /multiple obstacle
    iteration = 0
    start_time = time.time()
    obsnum2 = 1
    extracted_dim = [349, 249, 61, 147]
    pygame.init()
    map = RRTmap(start, goal, dimension, ObsDim, ObsNum, cm[0])
    graph = RRTGraph(start, goal, dimension, ObsDim, ObsNum, cm[0])
    obstacles = graph.makeextractedobs()

    # map = RRTmap(start, goal, dimension, ObsDim, ObsNum, extracted_dim)
    # graph = RRTGraph(start, goal, dimension, ObsDim, ObsNum, extracted_dim)
    # obstacles = graph.makeObs()
    map.drawMap(obstacles)

    while not graph.path_to_goal():
        elapsed_time = time.time()-start_time
        start_time = time.time()
        if elapsed_time > 10:
            raise

        if iteration % 10 == 0:
            u, v, parent = graph.bias(goal)
            pygame.draw.circle(map.Map, map.grey, (u[-1], v[-1]), map.nodeRadius+2, 0)
            pygame.draw.line(map.Map, map.Blue, (u[-1], v[-1]), (u[parent[-1]], v[parent[-1]]), map.edgeThickness)
        else:
            u, v, parent = graph.expand()
            pygame.draw.circle(map.Map, map.grey, (u[-1], v[-1]), map.nodeRadius + 2, 0)
            pygame.draw.line(map.Map, map.Blue, (u[-1], v[-1]), (u[parent[-1]], v[parent[-1]]), map.edgeThickness)

        if iteration % 10 == 0:
            pygame.display.update()
        iteration += 1

    map.drawPath(graph.getPathCords())
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)


if __name__ == '__main__':
    result = False
    while not result:
        try:
            main()
            result = True
        except:
            result = False
