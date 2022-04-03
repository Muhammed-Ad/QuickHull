from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
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


def QuickHullRec(points: list[tuple], line_p1: tuple, line_p2: tuple, side: int, createSubplot = True):

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

    if createSubplot:
        createSubPlot(same_side_points, convex_points + [line_p1, line_p2])

    convex_points += QuickHullRec(same_side_points, line_p1, max_point, lowerOrUpper(line_p1, line_p2, max_point), createSubplot)
    convex_points += QuickHullRec(same_side_points, line_p2, max_point, lowerOrUpper(line_p2, line_p1, max_point), createSubplot)
    return convex_points

def QuickHull(points: list[tuple], createSubplot = True):
    if len(points) < 3:
        return set(points)

    convex_points = []
    points.sort(key = lambda x : (x[0], x[1]))
    minPoint = points[0]
    maxPoint = points[-1]

    if createSubplot:
        createSubPlot(points, [minPoint, maxPoint])

    convex_points += QuickHullRec(points, minPoint, maxPoint, 1, createSubplot) # upper side
    convex_points += QuickHullRec(points, minPoint, maxPoint, -1, createSubplot) # lower side
    
    return set(convex_points)

def QuickHullWrapper(points, generate_random_points = True, pointRange = 10000, num_of_points = 100, createSubplot = True):
    if generate_random_points == True:
        possiblePoint = range(int(pointRange))
        num_of_points = int(num_of_points)
        dim = 2
        Li = random.sample(possiblePoint, dim*num_of_points) #create list of num_of_point group of dim tuples from possiblePoint range 
        points += zip(*[iter(Li)]*dim) #make dim copies of iterators and break-up those dim copies up to create of list of tuples that dim in length
        

    start = time.time()
    outputPoints = QuickHull(points, createSubplot)
    amt = time.time() - start 
    print(f'done quickhull in: {amt}')

    start1 = time.time()
    outputPoints = sortCounterClockwise(list(outputPoints))
    amt1 = time.time() - start1
    print(f'done sorting counter clockwise in: {amt1}')
    print(outputPoints)

    return outputPoints

def animateConvexHull(points, pause_interval = 0.5, generate_random_points = True, pointRange: int = 10000, num_of_points: int = 100):
    outputPoints = QuickHullWrapper(points, generate_random_points, pointRange, num_of_points)

    tempPoints = []
    pointHash = {}

    for temp_points in reversed(frames_temp[1:]):
        temp = []
        c = 0

        for x, y in zip(temp_points[0], temp_points[1]):
            if pointHash.get((x, y)) is None:
                temp += plt.plot(x, y, 'bo')
                pointHash[(x, y)] = True
                c += 1
    
        tempPoints.append(temp)
    
    tempPoints.reverse()
    counter = -1
    tempLines = []
    tPL = len(tempPoints)

    for grp in frames:
        tempLines += plt.plot(grp[0], grp[1], 'ro-')

        if counter > -1 and counter < tPL:
            for ij in tempPoints[counter]:
                ij.remove()
                plt.pause(pause_interval / 5)
        print(counter, "num of iters", len(frames_temp))
        counter += 1
        plt.pause(pause_interval)
    
    
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

    # print(plt.get_fignums())
    plt.show()

def animateConvexHull2(points, pause_interval = 0.01, generate_random_points = False, point_range = 1e9, num_of_points = 1e6):
    interval: float = 0.01
    
    output = QuickHullWrapper(points, pointRange = point_range, num_of_points = num_of_points)

    tempPoints = []
    for i in points:
        tempPoints += plt.plot(i[0], i[1], 'bo')
        plt.pause(interval)
    
    length = len(output)    
    for i in range(length + 1): #extra point to connect everything
        ii =  i % length
        iii = (ii + 1) % length
        
        plt.plot([output[ii][0], output[iii][0]], [output[ii][1], output[iii][1]], 'ro-', lw = 2)
        plt.pause(interval)

    for i in tempPoints:
        i.remove()
        plt.pause(interval)

    plt.show()

def animateConvexHull3(points, pause_interval = 0.5, generate_random_points = True, pointRange = 10000, num_of_points = 100):

    outputPoints = QuickHullWrapper(points, generate_random_points, pointRange, num_of_points)

    tempPoints = []
    
    for temp_points in reversed(frames_temp[1:]):
        tempPoints += plt.plot(temp_points[0], temp_points[1], 'bo')
    
    
    tempPoints.reverse()
    counter = -1
    tempLines = []
    tPL = len(tempPoints)

    for grp in frames:
        tempLines += plt.plot(grp[0], grp[1], 'ro-')

        if counter > -1 and counter < tPL:
            tempPoints[counter].remove()
            plt.pause(pause_interval / 5)
        
        counter += 1
        plt.pause(pause_interval)
    
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

def animationGovernor(points, generate_random_points, num_of_points, just_outline = False):
    plt.title("Creating Convex Hull from Random Points")

    if just_outline:
        animateConvexHull2(points, pause_interval = 1)
    else:
        if num_of_points > 1e4 and generate_random_points:
            animateConvexHull3(points, pause_interval = 1, generate_random_points = generate_random_points, pointRange = 1e3 * num_of_points, num_of_points = 1e6)
        else:
            animateConvexHull(points, pause_interval = 1, generate_random_points = generate_random_points, pointRange = 1e3 * num_of_points, num_of_points = 1e6)
        
        

if __name__ == '__main__':

    
    # points = [ (0, 3), (1, 1), (2, 2), (4, 4),
    #            (0, 0), (1, 2), (3, 1), (3, 3)] # ans: {(4, 4), (0, 3), (3, 1), (0, 0)}

    # points = [ (0, 3), (1, 1), (2, 2), (2, 1),
    #            (3, 0), (0, 0), (3, 3)] # ans: {(0, 3), (3, 3), (3, 0), (0, 0)}

    # points = [(0, 0), (0, 4), (-4, 0), (5, 0), 
    #           (0, -6), (1, 0)] # ans: (-4, 0), (5, 0), (0, -6), (0, 4)

    points = [(0, 0), (1, -4), (-1, -5), (-5, -3), 
              (-3, -1), (-1, -3), (-2, -2), (-1, -1),
              (-2, -1), (-1, 1)] # ans: (-5, 3), (-1, -5), (-1, -4), (0, 0), (-1, 1)  
    
    QuickHullWrapper(points, generate_random_points=True, pointRange=1e12, num_of_points=1e9, createSubplot = False)
    #  animationGovernor(points, generate_random_points = False, num_of_points = 1e6, just_outline = True)   