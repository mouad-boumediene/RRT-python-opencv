# MIT License
#
# Copyright (c) [2020] [mouad boumediene]
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from RRTbase import RRTGraph
from RRTbase import RRTMap
from cv2 import waitKey

def main():
    dimensions=(512,512)  # map dimetions
    # start and coords
    start=(0,0)
    goal=(510,510)
    map=RRTMap(start,goal ,dimensions )
    graph=RRTGraph(start,goal,dimensions)
    # make obstacles randomly
    obstacles=graph.makeobs()
    # draw the map
    map.drawMap(obstacles)
    i=1 # iteration counter
    while(not graph.path_to_goal(goal)):
        # biasing the tree
        if i%10 == 0 :
            X,Y,Parent=graph.bias(goal)
            map.drawNode([X[-1],Y[-1]], nodeType="N")
            map.drawEdge( (X[-1],Y[-1]) , (X[Parent[-1]],Y[Parent[-1]])  )
            map.refreshMap()
        # expanding  the tree
        else:
            X,Y,Parent=graph.expand()
            map.drawNode([X[-1], Y[-1]], nodeType="N")
            map.drawEdge((X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]))
            map.refreshMap()
        i+=1
    # extract the coordinates of the path waypoints
    graph.path_to_goal(goal)
    # draw the path
    map.drawPath(graph.getPathCoords())

    waitKey(0)


if __name__ == '__main__':
    main()

