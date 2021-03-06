from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import math, random, time, enum, sys, os, psutil, gc, json
# from memory_profiler import profile

frames = []
frames_temp = []
results = []

# Makes code more readable, using enuams for shape and side of lines
class Shape(enum.Enum): 
    RandomSample = 0
    FilledCircle = 1
    Square = 2
    OutlinedCircle = 3
    Cluster = 4
    
class Side(enum.Enum): 
    lower = -1
    inline = 0
    upper = 1

# line_p1 and line_p2 are the two points that make up the line
# The point is passed in and the cross product is used to determine if that
# is to the lower or upper part of the line. The magnitude is then used as an
# Approximation of relative distance since we don't need exact distance 
def CrossProd2D(line_p1, line_p2, point):
    x = 0
    y = 1
    # <point, line_p1> x <line_p1, line_p2> or P1P x P2P1
    return (point[y] - line_p1[y]) * (line_p2[x] - line_p1[x]) - (line_p2[y] - line_p1[y]) * (point[x] - line_p1[x])

# 1, 0, -1 depending on side
def lowerOrUpper(line_p1, line_p2, point):

    distMetric: int = CrossProd2D(line_p1, line_p2, point)
    
    if distMetric == 0:
        return Side.inline.value
    else:
        return distMetric / abs(distMetric)

# not distance but an approximation
def distPointToLine(line_p1, line_p2, point):
    return abs(CrossProd2D(line_p1, line_p2, point)) 

# From StackOverflow: https://stackoverflow.com/a/69104076
# Sort point counter clockwise around calculated center, used to make animation
# QuickHull Process accurate
def sortCounterClockwise(points, center = None):
  if center:
    center_x, center_y = center
  else:
    length = len(points)
    if length == 0:
        return points
    center_x, center_y = sum([x for x,_ in points])/length, sum([y for _,y in points])/length

  angles = [math.atan2(y - center_y, x - center_x) for x, y in points]

  counterclockwise_indices = sorted(range(len(points)), key=lambda i: angles[i])
  counterclockwise_points = [points[i] for i in counterclockwise_indices]

  return counterclockwise_points

# Creates Queue of frames to generate animation with at the end
def createSubPlot(side_points, convex_points):
    pointsX, pointsY = map(list, zip(*side_points)) #create two list from list of tuples
    
    output = sortCounterClockwise(convex_points)
    output.append(output[0])
    outputX, outputY = map(list, zip(*output)) 
   
    global frames, frames_temp
    frames.append((outputX, outputY))
    frames_temp.append((pointsX, pointsY))


# Recursive QuickHull function to deal with partitioning points and calculating Convex Hull
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

def QuickHullMod(points: list[tuple], createSubplot = True):
    if len(points) < 3:
        return set(points)

    convex_points = list([])
    points.sort(key = lambda x : (x[0], x[1]))
    minPoint = points[0]
    maxPoint = points[-1]

    def QuickHullRecMod(line_p1: tuple, line_p2: tuple, side: int, createSubplot = True):
        
        same_side_points = []
        max_distance = -1
        max_point = None
        
        for point in points:
            distance = distPointToLine(line_p1, line_p2, point)
            point_side = lowerOrUpper(line_p1, line_p2, point)
            if point_side == side:
                if createSubplot:
                    same_side_points += [point]
                if distance > max_distance:
                    max_distance = distance
                    max_point = point

        if max_distance == -1:
            nonlocal convex_points
            convex_points += [line_p1, line_p2]
            return
        else:
            convex_points += [max_point]

        if createSubplot:
            createSubPlot(same_side_points, convex_points + [line_p1, line_p2])

        QuickHullRecMod(line_p1, max_point, lowerOrUpper(line_p1, line_p2, max_point), createSubplot)
        QuickHullRecMod(line_p2, max_point, lowerOrUpper(line_p2, line_p1, max_point), createSubplot)
        

    if createSubplot:
        createSubPlot(points, [minPoint, maxPoint])

    QuickHullRecMod(minPoint, maxPoint, Side.upper.value, createSubplot) # upper side
    QuickHullRecMod(minPoint, maxPoint, Side.lower.value, createSubplot) # lower side
    
    return set(convex_points)


# Non-Recursive QuickHull function to call QuickHull Recursive from
def QuickHull(points: list[tuple], createSubplot = True):
    if len(points) < 3:
        return set(points)

    convex_points = []
    points.sort(key = lambda x : (x[0], x[1]))
    minPoint = points[0]
    maxPoint = points[-1]

    if createSubplot:
        createSubPlot(points, [minPoint, maxPoint])

    convex_points += QuickHullRec(points, minPoint, maxPoint, Side.upper.value, createSubplot) # upper side
    convex_points += QuickHullRec(points, minPoint, maxPoint, Side.lower.value, createSubplot) # lower side
    
    return set(convex_points)

# QuickHull Wrapper function to generate different point shapes and call the on-recursive QuickHull
def QuickHullWrapper(points, generate_random_points = True, pointRange = 10000, num_of_points = 100, createSubplot = True, shape = 0):
    write_file = False
    if generate_random_points == True:
        R = int(pointRange)
        possiblePoints = range(R)
        num_of_points = int(num_of_points)
        dim = 2

        

        if shape == 0: # Average case
            Li = random.sample(possiblePoints, dim*num_of_points) # create list of num_of_point group of dim tuples from possiblePoint range 
            points += zip(*[iter(Li)]*dim)  # make dim copies of iterators and break-up those dim copies up to create of list of tuples that dim in length
        
        elif shape == 1: # Close to best case, better than avg case
            for i in range(num_of_points):
                r = R * math.sqrt(random.uniform(0, 1.0))
                theta = random.uniform(0, 1.0) * 2 * math.pi
                points.append((r*math.cos(theta), r*math.sin(theta)))

        elif shape == 2: # Square, similar case to random sample
            points += [(random.uniform(0, R), random.uniform(0, R)) for _ in range(num_of_points)]
        
        elif shape == 3: # Worst case, outline of circle
            r = R
            for i in range(num_of_points):
                theta = random.uniform(0, 1.0) * 2 * math.pi
                points.append((r*math.cos(theta), r*math.sin(theta)))
            
        elif shape == 4: # Cluster of points, Best Case
            mean = (R - 1) / 2 # Sum of numbers from 0 - R-1 inclusive divided by R to get mean (mean of first R -1 naural numbers)

            standard_deviation = (((R**2) - 1)/ 12) ** 0.5 # std devation of n natural numbers (not exactly std of n-1 natural numbers but differnece is n minimal with large sets)
            limit = int(1e9)

            if num_of_points <= limit:
                points += [(random.normalvariate(mean, standard_deviation), random.normalvariate(mean, standard_deviation)) for _ in range(num_of_points)]
            else:
                write_file = True
                fp = open('randPoints.txt', 'w')
                for i in range(num_of_points):
                    points += [(random.normalvariate(mean, standard_deviation), random.normalvariate(mean, standard_deviation))]
                    if i % (limit * 0.5) == 0:
                        for point in points:
                            fp.write("%d %d\n" % point)
                        
                        del points[:]
                        gc.collect()
                fp.close()

    
    if write_file:
        with open('randPoints.txt', 'r') as fp:
            points = [tuple(map(float, i.split())) for i in fp]

    # psutil.Process(os.getpid()).memory_info().rss ---Gives memory in bytes
    s = Shape(shape)
    print("Shape: ", s)
    start = time.perf_counter()
    outputPoints = QuickHull(points, createSubplot)
    amt = time.perf_counter() - start 
    print(f'done quickhull in: {amt}s')
    numPo = len(outputPoints)
    print("Number of points on hull:", numPo)
    memUsed = psutil.Process(os.getpid()).memory_info().rss / 1024 ** 3
    print("Memory Used:", memUsed, "GB")

    global results
    results += [{
        'Shape': s.name,
        'Time': amt,
        'Points on Hull': numPo,
        'Memory Used': memUsed
    }]

    # start = time.time()
    # outputPoints = QuickHullMod(points, createSubplot) # SLOW, VERY SLOW
    # amt = time.time() - start 
    # print(f'done quickhull in: {amt}s')

    start1 = time.perf_counter()
    outputPoints = sortCounterClockwise(list(outputPoints))
    amt1 = time.perf_counter() - start1
    print(f'done sorting counter clockwise in: {amt1}s')
    # print(outputPoints)

    return outputPoints

# animating QuickHull by animating each point individually (slowest, most animation)
def animateConvexHull(points, pause_interval = 0.5, generate_random_points = True, pointRange: int = 10000, num_of_points: int = 100, shape = 0):
    outputPoints = QuickHullWrapper(points, generate_random_points, pointRange, num_of_points, shape = shape)

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

    plt.show()

# animating QuickHull by animating batches of point (Faster than first version, less animation)
def animateConvexHull2(points, pause_interval = 0.01, generate_random_points = False, point_range = 1e9, num_of_points = 1e6, shape = 0):
    interval: float = 0.01
    
    output = QuickHullWrapper(points, pointRange = point_range, num_of_points = num_of_points, shape = shape)

    tempPoints = []
    for i in points:
        tempPoints += plt.plot(i[0], i[1], 'bo')
        plt.pause(interval)
    
    length = len(output)    
    for i in range(length + 1): # extra point to connect everything
        ii =  i % length
        iii = (ii + 1) % length
        
        plt.plot([output[ii][0], output[iii][0]], [output[ii][1], output[iii][1]], 'ro-', lw = 2)
        plt.pause(interval)

    for i in tempPoints:
        i.remove()
        plt.pause(interval)

    plt.show()

# animate outline of QuickHull
def animateConvexHull3(points, pause_interval = 0.5, generate_random_points = True, pointRange = 10000, num_of_points = 100, shape = 0):

    outputPoints = QuickHullWrapper(points, generate_random_points, pointRange, num_of_points, shape = shape)

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

def animationGovernor(generate_random_points, num_of_points, just_outline = False, pause_interval = 1, shape = 0):
    plt.title("Creating Convex Hull from Random Points") # implement figures to draw multiple shapes at once
    points = []

    if just_outline:
        animateConvexHull2(points, pause_interval = pause_interval, shape = shape)
    else:
        if num_of_points >= 1e4 and generate_random_points:
            animateConvexHull3(points, pause_interval = pause_interval, generate_random_points = generate_random_points, pointRange = 1e3 * num_of_points, num_of_points = num_of_points, shape = shape)
        else:
            animateConvexHull(points, pause_interval = pause_interval, generate_random_points = generate_random_points, pointRange = 1e3 * num_of_points, num_of_points = num_of_points, shape = shape)
        
def measureSpeedNoAnimation(num_of_points = 1e2, trials = 3):
    points = []
    trials = 3
    
    for shape in Shape:
        if shape.value == Shape.OutlinedCircle.value and num_of_points > int(1e7):
            continue
        else:
            for trial in range(trials):
                QuickHullWrapper(points, generate_random_points = True, pointRange=num_of_points*1e3, num_of_points=num_of_points, createSubplot = False, shape = shape.value)
                del points[:]
                gc.collect()

def inputVal(arg):
    try: 
        num = int(arg)
    except ValueError:
        print('Invalid input! Cannot convert %s to base 10 integer. Program Closing...' % arg)
        sys.exit(1)
    else:
        return num

if __name__ == '__main__':

    if len(sys.argv) < 3:
        raise ValueError('Need number of points to perform QuickHull on and Number of trials!')
    
    type = sys.argv[1]
    num = inputVal(sys.argv[2])

    if type == 'a':
        animationGovernor(generate_random_points = True, num_of_points = num, just_outline = False, pause_interval = .1, shape = Shape.Cluster.value)   
    
    else:
        if len(sys.argv) < 4:
            raise ValueError('Need number of points to perform QuickHull on and Number of trials!')
        trials = inputVal(sys.argv[3])
        
        measureSpeedNoAnimation(num_of_points = num, trials = trials)

        with open('json_data' + '_' + str(num) + '_' + str(trials) +'.json', 'w') as outfile:
            json.dump(results, outfile, indent=4)
    
    
        

    