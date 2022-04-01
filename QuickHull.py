from mailbox import linesep
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import math, random, time

frames = []
frames_temp = []

def CrossProd2D(line_p1, line_p2, point):
    x = 0
    y = 1
    # <point, line_p1> x <line_p1, line_p2> or P1P x P2P1
    return (point[y] - line_p1[y]) * (line_p2[x] - line_p1[x]) - (line_p2[y] - line_p1[y]) * (point[x] - line_p1[x])

def lowerOrUpper(line_p1, line_p2, point):

    distMetric: int = CrossProd2D(line_p1, line_p2, point)
    
    if distMetric == 0:
        return 0
    else:
        return distMetric / abs(distMetric)

def distPointToLine(line_p1, line_p2, point):
    return abs(CrossProd2D(line_p1, line_p2, point)) #not distnace but an approximation

def sortCounterClockwise(points, centre = None):
  if centre:
    centre_x, centre_y = centre
  else:
    length = len(points)
    if length == 0:
        return points
    centre_x, centre_y = sum([x for x,_ in points])/length, sum([y for _,y in points])/length

  angles = [math.atan2(y - centre_y, x - centre_x) for x, y in points]

  counterclockwise_indices = sorted(range(len(points)), key=lambda i: angles[i])
  counterclockwise_points = [points[i] for i in counterclockwise_indices]

  return counterclockwise_points


def createSubPlot(side_points, convex_points):
    pointsX, pointsY = map(list, zip(*side_points)) #create two list from list of tuples
    
    output = sortCounterClockwise(convex_points)
    output.append(output[0])
    outputX, outputY = map(list, zip(*output)) 
   
    global frames, frames_temp
    frames.append((outputX, outputY))
    frames_temp.append((pointsX, pointsY))


def QuickHullRec2(points: list[tuple], line_p1: tuple, line_p2: tuple, side: int):

    convex_points = []
    same_side_points = []
    max_distance = -1
    max_point = None

    for point in points:
        distance = distPointToLine(line_p1, line_p2, point)
        point_side = lowerOrUpper(line_p1, line_p2, point)
        if point_side == side:
            same_side_points += [point]
            if distance > max_distance:
                max_distance = distance
                max_point = point
        
    if max_distance == -1:
        return [line_p1, line_p2]
    else:
        convex_points += [max_point]

    createSubPlot(same_side_points, convex_points + [line_p1, line_p2])

    convex_points += QuickHullRec2(points, line_p1, max_point, lowerOrUpper(line_p1, line_p2, max_point))
    convex_points += QuickHullRec2(points, line_p2, max_point, lowerOrUpper(line_p2, line_p1, max_point))
    return convex_points

def QuickHull2(points: list[tuple]):
    if len(points) < 3:
        return points

    convex_points = []
    points.sort(key = lambda x : (x[0], x[1]))
    minPoint = points[0]
    maxPoint = points[-1]

    createSubPlot(points, [minPoint, maxPoint])

    convex_points += QuickHullRec2(points, minPoint, maxPoint, 1) # upper side
    convex_points += QuickHullRec2(points, minPoint, maxPoint, -1) # lower side
    
    return set(convex_points)

def animateConvexHull(points, pause_interval = 0.5, generate_random_points = True, pointRange = 10000, num_of_points = 100):

    if generate_random_points == True:
        possiblePoint = range(int(pointRange))
        for i in range(num_of_points):
            point = tuple(random.sample(possiblePoint, 2))
            points += [point]

    outputPoints = QuickHull2(points)
    print('done quickhull2')
    outputPoints = sortCounterClockwise(list(outputPoints))
    print('done sorting counter clockwise')
    print(outputPoints)

    tempPoints = []
    
    pointHash = {}
    pointsInCluster = []
    # x = 0; y = 1

    # ax1 = plt.subplot()
    for temp_points in reversed(frames_temp[1:]):
        temp = []
        c = 0

        for x, y in zip(temp_points[0], temp_points[1]):
            if pointHash.get((x, y)) is None:
                temp += plt.plot(x, y, 'bo')
                pointHash[(x, y)] = True
                c += 1
    
        pointsInCluster.append(c)
        tempPoints.append(temp)
    
    tempPoints.reverse()
    pointsInCluster.reverse()
    counter = -1
    tempLines = []
    tPL = len(tempPoints)

    for grp in frames:
        plt.plot(grp[0], grp[1], 'ro')
        tempLines += plt.plot(grp[0], grp[1], 'r-')
        # plt.pause(pause_interval)

        if counter > -1 and counter < tPL:
            for ij in tempPoints[counter]:
                ij.remove()
                plt.pause(pause_interval / 5)
        print(counter, "num of iters", len(frames_temp))
        counter += 1
        plt.pause(pause_interval)
    
    outputPointsX = []
    outputPointsY = []
    length = len(outputPoints)
    counter = 0
    for i in range(length + 1):
        index = i % length
        index2 = (index + 1) % length
        
        plt.plot([outputPoints[index][0], outputPoints[index2][0]] , [outputPoints[index][1], outputPoints[index2][1]],'go-')
        plt.pause(pause_interval)
    
    for k in range(len(tempLines)):
        tempLines[k].remove()
        plt.pause(pause_interval)

    print(plt.get_fignums())
    plt.show()

def animateConvexHull2(points):
    interval: float = 1

    pointsX = []
    pointsY = []

    tempPoints = []
    for i in points:
        pointsX.append(i[0])
        pointsY.append(i[1])
        tempPoints += plt.plot(pointsX, pointsY, 'bo')
        plt.pause(interval)
    
    output = QuickHull2(points)
    output = sortCounterClockwise(list(output))

    outputX = []
    outputY = []
    

    length = len(output)    
    for i in range(length + 1): #extra point to connect everything
        ii =  i % length
        outputX += [output[ii][0]]
        outputY += [output[ii][1]]
        
        plt.plot(output[ii][0], output[ii][1], 'ro')
        plt.plot(outputX, outputY, 'r-', lw = 2)
        plt.pause(interval)

    for i in tempPoints:
        i.remove()
        plt.pause(interval)

    print(outputX)

    plt.show()

def animateConvexHull3(points, pause_interval = 0.5, generate_random_points = True, pointRange = 10000, num_of_points = 100):

    if generate_random_points == True:
        possiblePoint = range(int(pointRange))
        for i in range(num_of_points):
            point = tuple(random.sample(possiblePoint, 2))
            points += [point]

    outputPoints = QuickHull2(points)
    print('done quickhull2')
    outputPoints = sortCounterClockwise(list(outputPoints))
    print('done sorting counter clockwise')
    print(outputPoints)

    tempPoints = []
    
    pointHash = {}
    pointsInCluster = []
    # x = 0; y = 1

    
    for temp_points in reversed(frames_temp[1:]):
        tempPoints += plt.plot(temp_points[0], temp_points[1], 'bo')
    
    
    tempPoints.reverse()
    pointsInCluster.reverse()
    counter = -1
    tempLines = []
    tPL = len(tempPoints)

    for grp in frames:
        plt.plot(grp[0], grp[1], 'ro')
        tempLines += plt.plot(grp[0], grp[1], 'r-')

        if counter > -1 and counter < tPL:
            tempPoints[counter].remove()
            plt.pause(pause_interval / 5)
        
        counter += 1
        plt.pause(pause_interval)
    
    # outputPointsX = []
    # outputPointsY = []
    length = len(outputPoints)
    counter = 0
    for i in range(length + 1):
        index = i % length
        index2 = (index + 1) % length
        plt.plot([outputPoints[index][0], outputPoints[index2][0]] , [outputPoints[index][1], outputPoints[index2][1]],'go-')
        plt.pause(pause_interval)
    
    for k in range(len(tempLines)):
        tempLines[k].remove()
        plt.pause(pause_interval)

    
    plt.show()
if __name__ == '__main__':

    plt.title("Creating Convex Hull from Random Points")
    # points = [ (0, 3), (1, 1), (2, 2), (4, 4),
    #            (0, 0), (1, 2), (3, 1), (3, 3)] # ans: {(4, 4), (0, 3), (3, 1), (0, 0)}

    # points = [ (0, 3), (1, 1), (2, 2), (2, 1),
    #            (3, 0), (0, 0), (3, 3)] # ans: {(0, 3), (3, 3), (3, 0), (0, 0)}

    # points = [(0, 0), (0, 4), (-4, 0), (5, 0), 
    #           (0, -6), (1, 0)] # ans: (-4, 0), (5, 0), (0, -6), (0, 4)

    points = [(0, 0), (1, -4), (-1, -5), (-5, -3), 
              (-3, -1), (-1, -3), (-2, -2), (-1, -1),
              (-2, -1), (-1, 1)] # ans: (-5, 3), (-1, -5), (-1, -4), (0, 0), (-1, 1)  
    
    animateConvexHull3(points, generate_random_points = False, pause_interval = 1, pointRange = 1e9)
    # animateConvexHull2(points)
    # # points = []
    # possible = range(1000000)
    # # print(possible)
    # # for i in range(10000):
    # #     point = tuple(random.sample(possible, 2))
    # #     points += [point]

    # print("done")
    # # pointsX = []
    # # pointsY = []
    # # for i in points:
    # #     pointsX.append(i[0])
    # #     pointsY.append(i[1])
    # #     plt.plot(pointsX, pointsY, 'go')
    # #     plt.pause(0.01)
    
    
    # # animateConvexHull2(points)
    # outputPoints = QuickHull2(points)
    # print('done quickhull2')
    # outputPoints = sortCounterClockwise(list(outputPoints))
    # print('done sorting counter clockwise')
    # print(outputPoints)
    # # fig, ax = plt.subplots()
    # # ax.plot(pointsX, pointsY, 'yo')
    # # # plt.show()

    # pointsX: list = []
    # pointsY: list = []
    # tempPoints = []
    
    # pointHash = {}
    # pointsInCluster = []
    # for temp_points in reversed(frames_temp[1:]):

    #     temp1 = temp_points[0]
    #     temp2 = temp_points[1]
    #     temp3 = []
    #     c = 0
    #     for x, y in tuple(zip(temp1, temp2)):
    #         if pointHash.get((x,y)) is None:
    #             temp3 += plt.plot(x, y, 'bo')
    #             pointHash[(x, y)] = True
    #             c += 1
    #             # plt.pause(0.01)
    
    #     pointsInCluster += [c]
    #     tempPoints += [temp3]
    #     # pointsX.append(temp1)
    #     # pointsY.append(temp2)
    
    # tempPoints.reverse()
    # pointsInCluster.reverse()
    # counter = -1
    # tempLines = []
    # tempLinesP = []
    # tPL = len(tempPoints)
    # for grp in frames:
    #     plt.plot(grp[0], grp[1], 'ro')
    #     tempLines += plt.plot(grp[0], grp[1], 'r-')
    #     plt.pause(0.1)
    #     # for k in tempPoints:
    #     #     counter += 1
    #     if counter > -1 and counter < tPL:
    #         for ij in tempPoints[counter]:
    #             ij.set_visible(False)
    #             # plt.pause(0.01)
    #     print(counter, "num of iters", len(frames_temp))
    #     counter += 1
    #     plt.pause(0.5)
    
    # outputPointsX = []
    # outputPointsY = []
    # length = len(outputPoints)
    # counter = 0
    # for i in range(length + 1):
    #     index = i % length
    #     outputPointsX.append(outputPoints[index][0])
    #     outputPointsY.append(outputPoints[index][1])

        
    #     plt.plot(outputPointsX, outputPointsY, 'go')
    #     plt.plot(outputPointsX, outputPointsY, 'g-')
    #     plt.pause(.5)
    
    # for k in range(len(tempLines)):
    #     tempLines[k].remove()
    #     plt.plot(outputPointsX[0], outputPointsY[0], 'go')
    #     plt.pause(.5)

    # plt.show()    