from collections import deque

class Edge:
    def __init__(self, endIndex=None, next=None):
        self.endIndex = endIndex
        self.next = next

    def __str__(self):
        if self.endIndex != None:
            endIndex = str(self.endIndex)
        else:
            endIndex = ""
        strOut = "[endIndex=" + endIndex + "]"
        return strOut
    
    def getEndIndex(self):
        return self.endIndex

    def setEndIndex(self, value):
        self.endIndex = value
    
    def getNext(self):
        return self.next
        
    def setNext(self, next):
        self.next = next


class Node:
    def __init__(self, name="", visited="", adjacency=None):
        self.name = name
        self.visited = visited
        self.adjacency = adjacency

    def __str__(self):
        if self.adjacency == None:
            adjcy = ""
        else:
            adjcy = self.adjacency
        strOut = "[N=" + str(self.name) + "|V=" + str(self.visited) + "|C=" + str(adjcy) + "]"
        return strOut

    def getName(self):
        return self.name

    def getVisited(self):
        return self.visited
    
    def getAdjacency(self):
        return self.adjacency
    
    def setName(self, name):
        self.name = name

    def setVisited(self, visited):
        self.visited = visited

    def setAdjacency(self, adjacency):
        self.adjacency = adjacency



class WGraph:
    def __init__(self):
        """Init for DGraph class"""
        self.size = 20
        self.numNodes = 0
        self.nodeList = [None] * self.size
        self.edgeList = []
        self.adjacencyMatrix = []
        for i in range (self.size):
            self.adjacencyMatrix.append(["" for i in range(self.size)])



    
    def addNode(self, name):
        """"""
        if self.numNodes >= self.size:
            raise OverflowError('Graph size exceeded!') 
        temp = Node()
        temp.setName(name)
        temp.setVisited(False)
        temp.setAdjacency(None)
        self.nodeList[self.numNodes] = temp
        self.numNodes += 1
        #print(f"Node added {temp}") #TEMP PRINT FOR TESTING
        
        

    def addEdge(self, starts, ends):
        """"""
        #print (f"Adding edge with starts = {starts} and ends = {ends}") #TEMP PRINT FOR TESTING        
        if starts == ends:
            return False
        startIndex = self.findNode(starts)
        endIndex = self.findNode(ends)
        if startIndex == -1 or endIndex == -1:
            return False
        self.adjacencyMatrix[startIndex][endIndex] = True
        self.adjacencyMatrix[endIndex][startIndex] = True

        startEnd = Edge()
        startEnd.setEndIndex(endIndex)
        startEnd.setNext(self.nodeList[startIndex].getAdjacency())
        self.nodeList[startIndex].setAdjacency(startEnd)
        self.edgeList.append(f"[{starts} --> {ends}]")

        #Disabled below is for non-directed graphs
        # endStart = Edge()
        # endStart.setEndIndex(startIndex)
        # endStart.setNext(self.nodeList[endIndex].getAdjacency())
        # self.nodeList[endIndex].setAdjacency(endStart)
        # self.edgeList.append(endStart)

        return True


    
    def findNode(self, value):
        """Retuns the index of the node from the nodelist"""
        self.numNodes
        for i in range(0, self.numNodes):
            if self.nodeList[i].getName() == value:
                return i
        return -1

    def listNodes(self):
        """"""
        theList = "The Nodes are: "
        for i in range(0, self.numNodes):
            theList += self.nodeList[i].getName()
            theList += " "
        return theList

    def displayAdjacency(self):
        """"""
        buffer = "The Adjacency List is: (Node: its adjacencies)\n"
        for i in range (0, self.numNodes):
            buffer += self.nodeList[i].getName()
            buffer +=": "
            ptr = self.nodeList[i].getAdjacency()
            while ptr != None:
                buffer += self.nodeList[ptr.getEndIndex()].getName()
                buffer += " "
                ptr = ptr.getNext()
            buffer += '\n'
        return buffer

    def displayEdges(self):
        """"""
        output  = ""
        for i in self.edgeList:
            output += i + "\n"
        return output

    def displayMatrix(self):
        """"""
        theList = "|     "
        for i in range (0, self.numNodes):
            theList += str(self.nodeList[i].getName()) + "  "
        theList += "|\n"
        for i in range (0, self.numNodes):
            theList += "|  " + str(self.nodeList[i].getName()) + "  "
            for j in range (0, self.numNodes):
                if self.adjacencyMatrix[i][j] == True:
                    addToList = "1"
                else:
                    addToList = "0"
                theList += addToList + "  "
            theList += '|\n'
        return theList
        


    def breadthFirst(self, name):
        """"""
        output = ""
        queue = deque()
        startNode = self.nodeList[self.findNode(name)]
        startNode.setVisited(True)
        queue.append(startNode)
        while queue:
            checkNode = queue.popleft()
            if output == "": #For first item (the input node) include a colon after the name
                output += str(checkNode.getName()) + ": "
            else:
                output += str(checkNode.getName()) + " "
            checkEdge = checkNode.getAdjacency()
            if checkEdge != None:
                neighborsRemaining = True
            else:
                neighborsRemaining = False
            while neighborsRemaining == True:
                nextNode = self.nodeList[checkEdge.getEndIndex()]
                if nextNode.getVisited() != True:
                    nextNode.setVisited(True)
                    queue.append(nextNode)
                if checkEdge.getNext() != None:
                    checkEdge = checkEdge.getNext()
                else:
                    neighborsRemaining = False
            #print (f"{output}") #FOR TESTING ONLY
        self.resetVisited() #After traversal has completed, reset the visited status for each node
        return output

    def depthFirst(self, name):
        
        stack  = []
        startNode = self.nodeList[self.findNode(name)]
        startNode.setVisited(True)
        stack.append(startNode)
        output = f"{name}: "
    
        while stack:
            peekNode = stack[-1]
            neighborPath = peekNode.getAdjacency() #Get path to neighbor
            neighborFound = False
            if neighborPath == None: #If there was no neighbor
                stack.pop()#Pop the peeked node
            while neighborPath != None and neighborFound == False: #If there was a neighbor
                neighbor = self.nodeList[neighborPath.getEndIndex()]
                if neighbor.getVisited() == False: #if the neighbor hasn't been visited yet
                    neighbor.setVisited(True) #set the neighbor as visited
                    neighborFound = True
                    stack.append(neighbor)#Push the neighbor onto the LIFO
                    output += str(neighbor.getName()) + " "    
                else:
                    neighborPath = neighborPath.getNext()
            if neighborFound == False:
                stack.pop()#Pop the peeked node if no unvisited neighbors were found

        self.resetVisited() #After traversal has completed, reset the visited status for each node
        return output
                


    def resetVisited(self):
        for i in range(0, self.numNodes):
            self.nodeList[i].setVisited(False)


    def minTree(self, name):
        """Accepts a starting node as a parameter and returns a string showing a minimum spanning tree with the starting node as root. This algorithm should use the depthFirst traversal."""
        stack  = []
        startNode = self.nodeList[self.findNode(name)]
        startNode.setVisited(True)
        stack.append(startNode)
        output = f"{name}: "
    
        while stack:
            peekNode = stack[-1]
            neighborPath = peekNode.getAdjacency() #Get path to neighbor
            neighborFound = False
            if neighborPath == None: #If there was no neighbor
                stack.pop()#Pop the peeked node
            while neighborPath != None and neighborFound == False: #If there was a neighbor
                neighbor = self.nodeList[neighborPath.getEndIndex()]
                if neighbor.getVisited() == False: #if the neighbor hasn't been visited yet
                    neighbor.setVisited(True) #set the neighbor as visited
                    neighborFound = True
                    stack.append(neighbor)#Push the neighbor onto the LIFO
                    output += f"{peekNode.getName()}-{neighbor.getName()} "
                else:
                    neighborPath = neighborPath.getNext()
            if neighborFound == False:
                stack.pop()#Pop the peeked node if no unvisited neighbors were found

        self.resetVisited() #After traversal has completed, reset the visited status for each node
        return output

    def connectTable(self):
        """returns a string. To solve this, remember that a breadth first traversal will report all nodes that can be reached from a given starting node. Using this, consider how to show the connectivity starting from each node in the graph."""
        output = ""
        for i in range(0, self.numNodes):
            output += f"{self.breadthFirst(self.nodeList[i].getName())}\n"
        return output