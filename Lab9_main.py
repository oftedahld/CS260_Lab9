#
#  Lab9_main.py
#
#  Created by Jim Bailey on 6/2/2020
#  Licensed under a Creative Commons Attribution 4.0 International License.
#
from WGraph import WGraph


def main():
    # define graph
    print("Testing weighted non-directed graph")
    graph = WGraph()

    # Add nodes to graph
    graph.addNode('A')
    graph.addNode('B')
    graph.addNode('C')
    graph.addNode('D')
    graph.addNode('E')
    graph.addNode('F')
    graph.addNode('G')
    graph.addNode('H')
    graph.addNode('I')
    graph.addNode('J')

    # add edges to the graph
    graph.addEdge('A', 'B', 4)
    graph.addEdge('A', 'H', 8)
    graph.addEdge('B', 'C', 8)
    graph.addEdge('B', 'H', 11)
    graph.addEdge('C', 'D', 7)
    graph.addEdge('C', 'F', 4)
    graph.addEdge('C', 'I', 2)
    graph.addEdge('D', 'E', 9)
    graph.addEdge('D', 'F', 14)
    graph.addEdge('E', 'F', 10)
    graph.addEdge('F', 'G', 2)
    graph.addEdge('G', 'H', 1)
    graph.addEdge('G', 'I', 6)
    graph.addEdge('H', 'I', 7)

    print("The list of nodes ")
    print(" expected A B C D E F G H I J " )
    print(" actually " + graph.listNodes(), end = "\n\n")

    print("The adjacency list is:")
    print(graph.displayAdjacency(), end="\n")
    
    print("The adjacency matrix is:")
    print(graph.displayMatrix(), end="\n")

    print("The breadth first traversal starting at A")
    print(" expected A H B I G C F D E Unreached J")
    print(" actually " + graph.breadthFirst('A'), end="\n")

    print("The depth first traversal starting at A" )
    print(" expected A H I G F E D C B Unreached J" )
    print(" actually " + graph.depthFirst('A'), end="\n")

    #uncomment the following lines to test min cost tree
    print("The min cost tree starting at A ")
    print(" Expected A: A-B A-H H-G G-F F-C C-I C-D D-E Unreached J")
    print(" Actually " + graph.minCostTree('A'), end="\n")
    
    print("The min cost tree starting at D ")
    print(" Expected D: D-C C-I C-F F-G G-H C-B B-A D-E Unreached J")
    print(" Actually " + graph.minCostTree('D'), end="\n")
    
    #uncomment the following lines to test minCostPath
    print("The minimum cost paths starting at A ")
    print(" Expected B(4) C(12) D(19) E(21) F(11) G(9) H(8) I(14) J(inf) ")
    print(" Actually " + graph.minCostPaths('A'), end="\n")

    print("The minimum cost paths starting at D ")
    print(" Expected A(19) B(15) C(7) E(9) F(11) G(13) H(14) I(9) J(inf) ")
    print(" Actually " + graph.minCostPaths('D'), end="\n")

if __name__ == '__main__':
    main()