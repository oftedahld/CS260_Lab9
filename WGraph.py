from collections import deque

class Edge:
    def __init__(self, endIndex=None, startIndex=None, weight=0, next=None):
        self.endIndex = endIndex
        self.startIndex = startIndex
        self.weight = weight
        self.next = next

    def __str__(self):
        if self.endIndex != None:
            endIndex = str(self.endIndex)
        else:
            endIndex = ""
        if self.endIndex != None:
            startIndex = str(self.startIndex)
        else:
            startIndex = ""
        if self.weight != None:
            weight = str(self.weight)
        else:
            startIndex = ""
        strOut = f"[{endIndex}<-{weight}->{startIndex}]"
        return strOut
    
    def getStartIndex(self):
        return self.startIndex

    def setStartIndex(self, value):
        self.startIndex = value

    def getEndIndex(self):
        return self.endIndex

    def setEndIndex(self, value):
        self.endIndex = value
    
    def getWeight(self):
        return self.weight
        
    def setWeight(self, weight):
        self.weight = weight
    
    def getNext(self):
        return self.next
        
    def setNext(self, next):
        self.next = next


class Node:
    def __init__(self, name="", visited="", adjacency=None, distance=0):
        self.name = name
        self.visited = visited
        self.adjacency = adjacency
        self.distance = distance

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

    def setDistance(self, distance):
        self.distance = distance

    def getDistance(self):
        return self.distance

class WGraph:
    def __init__(self):
        """Init for DGraph class"""
        self.size = 20
        self.numNodes = 0
        self.nodeList = [None] * self.size
        self.edgeList = []
        self.adjacencyMatrix = []
        self.highestWeight = 0
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
        
        

    def addEdge(self, left, right, weight):
        """"""
        #print (f"Adding edge with starts = {starts} and ends = {ends}") #TEMP PRINT FOR TESTING        
        if left == right:
            return False
        leftIndex = self.findNode(left)
        rightIndex = self.findNode(right)
        if leftIndex == -1 or rightIndex == -1:
            return False
        self.adjacencyMatrix[leftIndex][rightIndex] = True
        self.adjacencyMatrix[rightIndex][leftIndex] = True
        if weight > self.highestWeight:
            self.highestWeight = weight

        leftEdge = Edge()
        leftEdge.setEndIndex(rightIndex)
        leftEdge.setStartIndex(leftIndex)
        leftEdge.setWeight(weight)
        leftEdge.setNext(self.nodeList[leftIndex].getAdjacency())
        self.nodeList[leftIndex].setAdjacency(leftEdge)
        
        rightEdge = Edge()
        rightEdge.setEndIndex(leftIndex)
        rightEdge.setStartIndex(rightIndex)
        rightEdge.setWeight(weight)
        rightEdge.setNext(self.nodeList[rightIndex].getAdjacency())
        self.nodeList[rightIndex].setAdjacency(rightEdge)

        self.edgeList.append(leftEdge)
        

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
        theList = ""
        for i in range(0, self.numNodes):
            theList += self.nodeList[i].getName()
            theList += " "
        return theList

    def displayAdjacency(self):
        """"""
        buffer = ""
        for i in range (0, self.numNodes):
            buffer += self.nodeList[i].getName()
            buffer +=": "
            ptr = self.nodeList[i].getAdjacency()
            while ptr != None:
                buffer += f"{self.nodeList[ptr.getEndIndex()].getName()}({ptr.getWeight()}) "
                ptr = ptr.getNext()
            buffer += '\n'
        return buffer

    def displayEdges(self):
        """"""
        output  = ""
        for i in self.edgeList:
            startNode  = self.nodeList[i.getStartIndex()].getName()
            weight  = i.getWeight()
            endNode  = self.nodeList[i.getEndIndex()].getName()
            output += f"[{startNode} <-- {weight} --> {endNode}]\n"
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
        output = f"{name} "
    
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
    
    def resetDistance(self):
        for i in range(0, self.numNodes):
            self.nodeList[i].setDistance(0)


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

    def minCostTree(self, name):
        """"""
        edgeList = []
        output = f"{name}: "
        # tempOut = "" #TEMP FOR TESTING ONLY
        startNode = self.nodeList[self.findNode(name)]
        startNode.setVisited(True)
        addEdge = startNode.getAdjacency()
        highestWeight  = self.highestWeight * self.numNodes
        # tempOut += "\nAdding edges from initial node...\n"  #TEMP FOR TESTING ONLY
        while addEdge != None: #Add all edges from the starting node
            edgeList.append(addEdge)

            # #=======================================================================================================
            # #TEMP FOR TESTING ======================================================================================
            # tempOut += "\nCurrent EdgeList:\n"
            # for i in edgeList: #TEMP FOR TESTING
            #     tempOut += f"    [{self.nodeList[i.getStartIndex()].getName()} <- {i.getWeight()} -> {self.nodeList[i.getEndIndex()].getName()}]\n" #TEMP FOR TESTING
            # #TEMP FOR TESTING ======================================================================================
            # #=======================================================================================================
  
            addEdge = addEdge.getNext()
        while edgeList:
            # tempOut += f"Locating shortest edge from list...\n" #TEMP FOR TESTING ONLY
            lowestWeight = highestWeight
            lowestWeightIndex = -1
            for i in range(0, len(edgeList)): #Locates the lowest weight edge currently in the list
                weight = edgeList[i].getWeight()
                # print(f"Edge weight:  {weight}") #TEMP FOR TESTING ONLY
                if weight < lowestWeight:
                    lowestWeight = weight
                    lowestWeightIndex = i
            lowestWeightEdge = edgeList.pop(lowestWeightIndex) #find and remove shortest edge from PQ

            # #=======================================================================================================
            # #TEMP FOR TESTING ======================================================================================
            # tempOut += f"Removing shortest edge from list...[{self.nodeList[lowestWeightEdge.getStartIndex()].getName()} <- {lowestWeightEdge.getWeight()} -> {self.nodeList[lowestWeightEdge.getEndIndex()].getName()}]\n" #TEMP FOR TESTING
            # tempOut += "\nCurrent EdgeList:\n"
            # for i in edgeList: #TEMP FOR TESTING
            #     tempOut += f"    [{self.nodeList[i.getStartIndex()].getName()} <- {i.getWeight()} -> {self.nodeList[i.getEndIndex()].getName()}]\n" #TEMP FOR TESTING
            # #TEMP FOR TESTING ======================================================================================
            # #=======================================================================================================

            currentNode = self.nodeList[lowestWeightEdge.getEndIndex()] #save ending node of that edge as current
            currentNode.setVisited(True) #mark current as visited
            # tempOut += f"Node [{currentNode.getName()}] has now been marked visited...\n" #TEMP FOR TESTING ONLY
            lowestWeightStartNode = self.nodeList[lowestWeightEdge.getStartIndex()].getName() #output this edge 
            lowestWeightEndNode = self.nodeList[lowestWeightEdge.getEndIndex()].getName() #output this edge 
            output += f"{lowestWeightStartNode}-{lowestWeightEndNode} " #output this edge 
            # tempOut += f"Removed edge was added to output...\n" #TEMP FOR TESTING ONLY
            # tempOut += f"Current output: \n" #TEMP FOR TESTING ONLY
            # tempOut += f"   {output}\n" #TEMP FOR TESTING ONLY
            # tempOut += f"\nMoving to new Node [{currentNode.getName()}] to get edges starting from there...\n" #TEMP FOR TESTING ONLY
            newEdge = currentNode.getAdjacency()
            # edgeAddCount = 0 #TEMP FOR TESTING ONLY
            while newEdge != None:
                nodeVisited = self.nodeList[newEdge.getEndIndex()].getVisited()
                if nodeVisited != True:
                    # tempOut += f"       Node [{self.nodeList[newEdge.getEndIndex()].getName()}] has not been visited yet...\n" #TEMP FOR TESTING ONLY
                    newEdgeEndIndex = newEdge.getEndIndex()
                    newEdgeWeight = newEdge.getWeight()
                    addNewEdge = True
                    for i in edgeList:
                        if i.getEndIndex() == newEdgeEndIndex:
                            if i.getWeight() > newEdgeWeight: #New edge is lighter than existing item ending in same node, need to remove old item
                                edgeList.remove(i)
                            else:
                                addNewEdge = False
                    if addNewEdge == True:
                        # tempOut += "\n      Adding new edge....." #TEMP FOR TESTING ONLY
                        # edgeAddCount += 1 #TEMP FOR TESTING ONLY
                        edgeList.append(newEdge)

                    #=======================================================================================================
                    # #TEMP FOR TESTING ======================================================================================
                    # tempOut += "\n      Current EdgeList:\n"
                    # for i in edgeList: #TEMP FOR TESTING
                    #     tempOut += f"       [{self.nodeList[i.getStartIndex()].getName()} <- {i.getWeight()} -> {self.nodeList[i.getEndIndex()].getName()}]\n" #TEMP FOR TESTING
                    # # print(tempOut)
                    # #TEMP FOR TESTING ======================================================================================
                    #=======================================================================================================
                # else:
                #     tempOut += f"       Node [{self.nodeList[newEdge.getEndIndex()].getName()}] has already been visited...\n" #TEMP FOR TESTING ONLY
                newEdge = newEdge.getNext()                
            # tempOut += f"Finished adding {edgeAddCount} new edges from node [{currentNode.getName()}]...\n\n" #TEMP FOR TESTING ONLY
            

        # output += f"\n{tempOut}" #TEMP FOR TESTING ONLY
        self.resetVisited()
        return output

    


    def minCostPaths(self,  name):
        """"""
        output = ""
        maxDistance = self.highestWeight * self.numNodes
        inputSet = []
        startNode = self.nodeList[self.findNode(name)]
        for i in range(0, self.numNodes):
            self.nodeList[i].setDistance(maxDistance)
            inputSet.append(self.nodeList[i])
        startNode.setDistance(0)
        # for i in inputSet: #TEMP FOR TESTING
            # print(f"{i.getName()}({i.getDistance()}) ") #TEMP FOR TESTING
        while inputSet:
            currentMin = maxDistance
            currentMinIndex = -1
            for i in range(0, len(inputSet)): #Get the Node with the smallest distance
                nodeDistance = inputSet[i].getDistance()
                if nodeDistance < currentMin:
                    currentMin = nodeDistance
                    currentMinIndex = i
            currentNode = inputSet[currentMinIndex] #Node with smallest distance value was selected
            inputSet.pop(currentMinIndex)
            # print(f"Current node: {currentNode.getName()}") #TEMP FOR TESTING
            neighborPath = currentNode.getAdjacency() #Find what the first neighbor of that node would be
            while neighborPath != None: #While we haven't run out of sidewalk
                neighbor = self.nodeList[neighborPath.getEndIndex()] #Find out what node is the neighbor
                neighborPathDistance = neighborPath.getWeight() #Find out how long the path is to the neighbor
                neighborDistance = neighbor.getDistance() #Find out what that neighbor's distance value is
                proposedDistance = currentNode.getDistance() + neighborPathDistance #Find out what the distance to the neighbor might be
                # print(f"    Checking neighbor: {neighbor.getName()}") #TEMP FOR TESTING
                # print(f"        Path to neighbor: ({neighborPathDistance})") #TEMP FOR TESTING
                # print(f"        Neighbor existing distance: ({neighborDistance})") #TEMP FOR TESTING
                # print(f"        New neighbor distance might be: ({proposedDistance})") #TEMP FOR TESTING
                if (proposedDistance) < neighborDistance: #If the current distance + the path distance is less than the distance the neighbor has on their own
                    neighbor.setDistance(proposedDistance) #Update the neighbor distance to the proposed distance
                #     print(f"        Neighbor distance update: ({proposedDistance})") #TEMP FOR TESTING
                # print(f"        Moving  to next neighbor...") #TEMP FOR TESTING
                neighborPath = neighborPath.getNext() #Find the next path to check
            # print(f"    All neighbors visited, moving to next node...") #TEMP FOR TESTING

        for i in range(0, self.numNodes):
            if self.nodeList[i] != startNode:
                if self.nodeList[i].getDistance() == maxDistance:
                    distance = "inf"
                else:
                    distance = self.nodeList[i].getDistance()
                output += f"{self.nodeList[i].getName()}({distance}) "


        self.resetVisited()
        self.resetDistance()
        return output