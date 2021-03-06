__author__ = 'Jacky Baltes <jacky@cs.umanitoba.ca>'

import sys
import matplotlib.pyplot as plt
import numpy as np
import math
import copy
from pathplanning import PathPlanningProblem, Rectangle
import time

graph = []

class CellDecomposition:
    def __init__(self, domain, minimumSize):
        self.domain = domain
        self.minimumSize = minimumSize
        self.root = [Rectangle(0.0, 0.0, domain.width, domain.height), 'unknown', [], [], float('inf'), False]

    def Draw(self, ax, node = None):
            if ( node == None ):
                node = self.root
            r = plt.Rectangle((node[0].x, node[0].y), node[0].width, node[0].height, fill=False, facecolor=None, alpha=0.5)
            if ( node[1] == 'mixed' ):
                color = '#5080ff'
                if ( node[2] == [] ):
                    r.set_fill(True)
                    r.set_facecolor(color)
            elif ( node[1] == 'free' ):
                color = '#ffff00'
                # if node in graph:
                #     color = "#ff0000"
                if node[5] == True:
                    color = "#ff00ff"
                r.set_fill(True)
                r.set_facecolor(color)
            elif ( node[1] == 'obstacle'):
                color = '#5050ff'
                r.set_fill(True)
                r.set_facecolor(color)
            else:
                print("Error: don't know how to draw cell of type", node[1])
            #print('Draw node', node)
            ax.add_patch(r)
            for c in node[2]:
                self.Draw(ax, c)

    def CountCells(self, node = None ):
        if ( node is None ):
            node = self.root
        sum = 0
        if ( node[2] != [] ):
            sum = 0
            for c in node[2]:
                sum = sum + self.CountCells(c)
        else:
            sum = 1
        return sum

class QuadTreeDecomposition(CellDecomposition):
    def __init__(self, domain, minimumSize):
        super().__init__(domain, minimumSize)
        self.root = self.Decompose(self.root)

    def Decompose(self, node):
        cell = 'free'
        r = node[0]
        rx = r.x
        ry = r.y
        rwidth = r.width
        rheight = r.height

        for o in self.domain.obstacles:
            if ( o.CalculateOverlap(r) >= rwidth * rheight ):
                cell = 'obstacle'
                break
            elif ( o.CalculateOverlap(r) > 0.0 ):
                cell = 'mixed'
                break
        if ( cell == 'mixed'):
            if (rwidth / 2.0 > self.minimumSize) and (rheight / 2.0 > self.minimumSize):
                childt1 = [Rectangle(rx, ry, rwidth/2.0, rheight/2.0), 'unknown', [], [], float('inf'), False ]
                qchild1 = self.Decompose( childt1 )
                childt2 = [Rectangle(rx + rwidth/2.0, ry, rwidth/2.0, rheight/2.0), 'unknown', [], [], float('inf'), False ]
                qchild2 = self.Decompose( childt2 )
                childt3 = [Rectangle(rx, ry + rheight/2.0, rwidth/2.0, rheight/2.0), 'unknown', [], [], float('inf'), False ]
                qchild3 = self.Decompose( childt3 )
                childt4 = [Rectangle(rx + rwidth/2.0, ry + rheight/2.0, rwidth/2.0, rheight/2.0), 'unknown', [], [], float('inf'), False ]
                qchild4 = self.Decompose( childt4 )
                children = [ qchild1, qchild2, qchild3, qchild4 ]
                node[2] = children
            else:
                cell = 'obstacle'

        elif cell == 'free':
            graph.append( node )

        node[1] = cell
        return node

def nodeContains( a ):
    for node in graph:
        r = node[0]
        if a[0] >= r.x and a[0] <= r.x + r.width and a[1] >= r.y and a[1] <= r.y + r.height:
            return node

    return None

def assignHeuristics( goal ):
    curr = nodeContains( goal )
    if curr is None:
        print( 'Goal node is None' )

    curr[4] = 0
    changed = True
    while changed:
        changed = False
        for node in graph:
            if node[4] < float('inf'):
                est = node[4] + 1
                for adjNode in node[3]:
                    if adjNode[4] > est:
                        adjNode[4] = est
                        changed = True


def checkAdjacency( a, b ):
    # ra = a[0]
    # rb = b[0]
    # if ra.x <= rb.x + rb.width and ra.x + ra.width >= rb.x and ra.y <= rb.y + rb.height and ra.y + ra.height >= rb.y:
    #     # They overlap somewhere.
    #     a[3].append( b )
    #     b[3].append( a )
    adjacent = False
    rectangle1 = a[0]
    rectangle2 = b[0]

    r1MinX = rectangle1.x
    r1MaxX = rectangle1.x + rectangle1.width
    r1MinY = rectangle1.y
    r1MaxY = rectangle1.y + rectangle1.height

    r2MinX = rectangle2.x
    r2MaxX = rectangle2.x + rectangle2.width
    r2MinY = rectangle2.y
    r2MaxY = rectangle2.y + rectangle2.height

    if ( r1MinX == r2MaxX and r1MinY <= r2MaxY and r1MaxY >= r2MinY):
        adjacent = True
    elif ( r1MaxX == r2MinX and r1MinY <= r2MaxY and r1MaxY >= r2MinY ):
        adjacent = True
    elif ( r1MinY == r2MaxY and r1MinX <= r2MaxX and r1MaxX >= r2MinX ):
        adjacent = True
    elif ( r1MaxY == r2MinY and r1MinX <= r2MaxX and r1MaxX >= r2MinX ):
        adjacent = True

    if adjacent:
        a[3].append( b )
        b[3].append( a )

def markAdjacentNodes():
    for i in range( 0, len( graph ) - 1 ):
        node = graph[i]
        for j in range( i + 1, len( graph ) - 1 ):
            checkNode = graph[j]
            checkAdjacency( node, checkNode )

def lineLength( line ):
    return math.sqrt( line[0]*line[0] + line[1]*line[1] )

def getCentre( node ):
    r = node[0]
    return np.array( [r.x + r.width / 2, r.y + r.height / 2] )

def aStart( initial, goal ):
    pos = np.array( initial )
    checked = []
    checked.append( nodeContains( initial ) )
    if checked[0] is None:
        print("Starting node not found.")
        return 0

    checked[0][5] = True
    checked[0][4] = lineLength( getCentre( checked[0] ) - pos )
    atGoal = False
    madeMove = True

    while not atGoal and madeMove:
        madeMove = False
        if nodeContains( goal ) is checked[len( checked ) - 1]:
            print( 'Found goal near: ', pos )
            atGoal = True
        else:
            nextNode = None
            bestCost = float('inf')
            for node in checked:
                cost = node[4]
                for adjNode in node[3]:
                    if adjNode[5] == False:
                        estCost = cost + adjNode[4]
                        if estCost < bestCost:
                            bestCost = estCost
                            nextNode = adjNode



            if nextNode is not None:
                madeMove = True
                nextNode[5] = True
                centre = getCentre( nextNode )
                #calc the total cost up to the centre of this node.
                totalCost = cost + lineLength( centre - pos )
                # store the cost in node.
                nextNode[4] = totalCost
                # move to new location
                pos = centre
                checked.append( nextNode )

            else:
                print( "No move made." )

        return checked[len( checked ) - 1][4], atGoal



def main( argv = None ):
    if ( argv == None ):
        argv = sys.argv[1:]

    width = 100.0
    height = 100.0
    maxOWidth = 50.0
    maxOHeight = 50.0

    numObstacles = 10
    if len(argv) > 0:
        numObstacles = int(argv[0])

    pp = PathPlanningProblem( width, height, numObstacles, maxOWidth, maxOHeight)
    #pp.obstacles = [ Obstacle(0.0, 0.0, pp.width, pp.height / 2.2, '#555555' ) ]
    initial, goals = pp.CreateProblemInstance()

    fig = plt.figure()
    ax = fig.add_subplot(1,2,1, aspect='equal')
    ax.set_xlim(0.0, width)
    ax.set_ylim(0.0, height)

    for o in pp.obstacles:
        ax.add_patch(copy.copy(o.patch) )
    ip = plt.Rectangle((initial[0],initial[1]), 0.1, 0.1, facecolor='#ff0000')
    ax.add_patch(ip)

    for g in goals:
        g = plt.Rectangle((g[0],g[1]), 0.1, 0.1, facecolor='#00ff00')
        ax.add_patch(g)

    start_time = time.time()
    qtd = QuadTreeDecomposition(pp, 0.1)
    markAdjacentNodes()
    assignHeuristics( goals[0] )

    length, found = aStart( initial, goals[0] )
    total_time = time.time() - start_time

    if found == False:
        print( "Did not find goal." )
    else:
        print( "Found goal in: ", total_time, ". Path length: ", length )

    qtd.Draw(ax)
    n = qtd.CountCells()
    ax.set_title('Quadtree Decomposition\n{0} cells'.format(n))

    plt.show()

if ( __name__ == '__main__' ):
    main()
