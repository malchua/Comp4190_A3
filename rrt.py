__author__ = 'Jacky Baltes <jacky@cs.umanitoba.ca>'

import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import numpy as np
import math
import random
from pathplanning import PathPlanningProblem, Rectangle
import time

MAX_ITERS = 100000 # Just so it doesn't run forever...

def essentiallyEqual( x, y ):
    return abs( x - y ) < 1e-1

def hitGoal( goal, node ):
    line = goal - node
    if lineLength( line ) < 1e-1:
        return True

    return False

def getClosestNode( newPoint, tree ):
    closest = None
    distance = float('inf')

    for point in tree:
        x = newPoint[0] - point[0]
        y = newPoint[1] - point[1]

        # distance
        z = math.sqrt( x*x + y*y )

        if z < distance:
            distance = z
            closest = point

    return closest, distance

def lineLength( line ):
    return math.sqrt( line[0]*line[0] + line[1]*line[1] )

def ExploreDomain( domain, initial, steps, goal ):
    paths = []
    pos = np.array(initial)
    tree = [pos]
    dd = 0.1
    total_steps = int( steps / dd )
    diagLength = math.sqrt( 50*50 + 50*50 )
    goal = np.array( goal )

    atGoal = False
    iter = 0
    while atGoal == False and iter < MAX_ITERS:
        newSpot = None
        log = np.zeros( ( total_steps, 2 ) )
        rand = random.randint( 1, 4 )
        if rand < 4:
            theta = random.uniform(-180.0/180.0 * math.pi, 180.0/180.0 * math.pi)
            while( theta >= math.pi ):
                theta = theta - 2 * math.pi
            while( theta < - math.pi ):
                theta = theta + 2 * math.pi

            # get a random spot.
            newSpot = [domain.width/2, domain.height/2];
            newSpot = np.array( newSpot )
            newDist = random.uniform( 0.0, diagLength )
            newSpot = newSpot + newDist * np.array([newDist * math.cos( theta ), newDist * math.sin( theta )])
        else:
            newSpot = goal

        # Get the cloeset node to the new point.
        pos, distance = getClosestNode( newSpot, tree )
        if pos is not None:
            pos = np.array( pos )
            log[0,:] = pos
            distanceRemaining = distance
            lineToMove = newSpot - pos
            lengthOfLine = lineLength( lineToMove )
            lineToMove[0] /= lengthOfLine
            lineToMove[1] /= lengthOfLine
            i = 1
            stop = False
            while i < total_steps and not stop:
                # newpos = pos + dd * np.array([dd * math.cos(theta), dd * math.sin(theta)])
                newpos = np.array( pos + dd * np.array( lineToMove ) )
                distanceRemaining -= dd
                r = Rectangle(newpos[0], newpos[1], 0.1, 0.1)
                if ( newpos[0] >= 0.0 ) and ( newpos[0] < domain.width ) and ( newpos[1] >= 0.0 ) and ( newpos[1] < domain.height ):
                    if ( not domain.CheckOverlap( r ) ):
                        pos = newpos
                        log[i,:] = pos
                        # Check if we reached the goal.
                        atGoal = hitGoal( goal, pos )
                        if atGoal:
                            stop = True

                    else:
                        stop = True

                else:
                    stop = True

                i = i + 1
                # we are at the new node.
                if distanceRemaining < 1e-9:
                    stop = True

            # add the final position to the tree.
            tree.append( pos )
            i = 1
            while i < total_steps and log[i][0] != 0:
                i = i + 1

            log = log[:i, :i]
            if len( log ) > 1:
                paths.append( log )
        else:
            print( "no closest node found.... This is an error" )

        iter = iter + 1
        atGoal = hitGoal( goal, tree[len( tree ) - 1] )
        if atGoal == True:
            print( "Found it!" )

    return paths, atGoal

def main( argv = None ):
    if ( argv == None ):
        argv = sys.argv[1:]

    width = 100.0
    height = 100.0
    maxOWidth = 50.0
    maxOHeight = 50.0

    numObstacles = 5
    if len(argv) > 0:
        numObstacles = int(argv[0])

    pp = PathPlanningProblem( width, height, numObstacles, maxOWidth, maxOHeight)
    #pp.obstacles = [ Obstacle(0.0, 0.0, pp.width, pp.height / 2.2, '#555555' ) ]
    # pp.obstacles = []
    initial, goals = pp.CreateProblemInstance()

    fig = plt.figure()
    ax = fig.add_subplot(1,1,1, aspect='equal')
    ax.set_xlim(0.0, width)
    ax.set_ylim(0.0, height)

    for o in pp.obstacles:
        ax.add_patch(o.patch)
#        ax.add_patch(g)

    start_time = time.time()
    paths, found = ExploreDomain( pp, initial, 5, goals[0] )
    ax.set_title('RRT Domain')

    print( "Total paths traversed: ", len(paths) )
    if found:
        toGoal = []
        toGoal.append( paths[len( paths ) - 1] )
        paths.pop( len( paths ) - 1 )
        atStart = False
        if toGoal[0][0][0] == initial[0] and toGoal[0][0][1] == initial[1]:
            atStart = True
        while not atStart:
            pathToConnect = toGoal[len( toGoal ) - 1]
            i = 0
            for path in paths:
                if path[len( path ) - 1][0] == pathToConnect[0][0] and path[len( path ) - 1][1] == pathToConnect[0][1]:
                    toGoal.append( path )
                    paths.pop( i )
                    atStart = path[0][0] == initial[0] and path[0][1] == initial[1]
                    break
                else:
                    i += 1

        print( "Total steps to goal: ", len( toGoal ) )

        total_time = time.time() - start_time
        path_length = 0
        #calculate the length.
        for path in toGoal:
            path_length += lineLength( path[0] - path[len( path ) - 1] )

        print( "Found the goal in: ", total_time, "\nWith path of length: ", path_length )

        for path in paths:
            plt.plot(path[:,0], path[:,1], 'b-')

        for path in toGoal:
            plt.plot(path[:,0], path[:,1], 'y-')

    else:
        print("Did not find goal.")
        for path in paths:
            plt.plot(path[:,0], path[:,1], 'b-')

    # These don't show up...
    ip = plt.Rectangle((initial[0],initial[1]), .5, .5, facecolor='#ff0000')
    ax.add_patch(ip)
    for g in goals:
        g = plt.Rectangle((g[0],g[1]), 0.5, 0.5, facecolor='#00ff00')
        ax.add_patch(g)

    plt.show()

if ( __name__ == '__main__' ):
    main()
