__author__ = 'Jacky Baltes <jacky@cs.umanitoba.ca>'

import sys
import matplotlib.pyplot as plt
import numpy as np
import math
import copy
from pathplanning import PathPlanningProblem, Rectangle

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
                # if node[4] > 5:
                #     color = "#ff00ff"
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
    ra = a[0]
    rb = b[0]
    if ra.x <= rb.x + rb.width and ra.x + ra.width >= rb.x and ra.y <= rb.y + rb.height and ra.y + ra.height >= rb.y:
        # They overlap somewhere.
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

def aStart( initial ):
    pos = np.array( initial )
    checked = []
    checked.append( nodeContains( initial ) )
    checked[0][5] = True
    atGoal = False
    madeMove = True

    while not atGoal and madeMove:
        madeMove = False
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
            #calc the total cost up to the centre of this node.
            totalCost = cost # + lineLength( centre of node - pos )
            # pos = centre of node.
            # store the cost in node.
            nextNode[4] = totalCost



def main( argv = None ):
    if ( argv == None ):
        argv = sys.argv[1:]

    width = 10.0
    height = 10.0
    maxOWidth = 5.0
    maxOHeight = 5.0

    numObstacles = 3
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

    qtd = QuadTreeDecomposition(pp, 0.1)
    markAdjacentNodes()
    assignHeuristics( goals[0] )
    qtd.Draw(ax)
    n = qtd.CountCells()
    ax.set_title('Quadtree Decomposition\n{0} cells'.format(n))

    plt.show()

if ( __name__ == '__main__' ):
    main()
