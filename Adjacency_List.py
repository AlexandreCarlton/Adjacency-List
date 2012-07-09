from collections import deque

def vertices_and_edges(file):
    '''Helper function to convert text file into a list of vertices and edges
    Assumes file is of format:
    A
    B
    C
    A B 2
    C A 5
    Listing vertices, and then the edges (with weights if the graph is weighted).'''
    vertices = [vertex.strip() for vertex in open(file, 'rU') if len(vertex.strip().split()) == 1]
    edges = [pair.strip().split() for pair in open(file, 'rU') if len(pair.strip().split()) in (2,3)]
    return vertices, edges


class Adjacency_List(object):
    '''A dictionary where the key is a vertex and the value a list of all the vertices it is adjacent to.
    Includes algorithms such as BFS, DFS, etc.'''

    def __init__(self, vertices, edges, directed=False, weighted=False):#edge_list being a list of tuples (eg. ('A', 'B', 2))
        if not isinstance(directed, bool):
            raise TypeError("Directed must be a boolean value.")
        if not isinstance(weighted, bool):
            raise TypeError("Weighted must be a boolean value.")
               
        self._directed = directed
        self._weighted = weighted
        
        self._adjacency_list = { vertex : [] for vertex in vertices }
        self._edge_list = [tuple(edge[:2]) for edge in edges]
        
        #What if we made self._weighted a dictionary of dictionaries
        #instead of d[(a,b)], you'd have d[a][b]
        #So it's more of an adjacency matrix, and if there is no edge then it returns None.
        self._weight = {} #weight[(a,b)] -> int
        

        #Could probably make an add_edge(A,B) or something to make this easier.               
        for edge in edges:
            if self._weighted and len(edge) != 3:
                raise ValueError('Only two vertices and an edge per tuple.')
            if not self._weighted and len(edge) != 2:
                raise ValueError('Only two vertices per tuple.')

            v1, v2 = edge[:2]
            self._adjacency_list[v1].append(v2)
            self._weight[(v1,v2)] = int(edge[2]) if self._weighted else 1       # An unweighted graph has edges of weight 1.
            
            if not self._directed:                                              # v1 is adjacent to v2 as well if the graph is not directed
                self._adjacency_list[v2].append(v1)
                self._weight[(v2,v1)] = int(edge[2]) if self._weighted else 1 

    ###################
    # Special methods #
    ###################

    def __str__(self):
        '''String representation of the Adjacency List'''
        if self._weighted:
            return '\n'.join(
                ['%s:\t%s' % (vertex, ', '.join(                                        # Each vertex in the adjacency list
                    ['%s: %s' % (joined_vertex, self._weight[(vertex, joined_vertex)])  # Each joined vertex and its weight
                     for joined_vertex in joined_vertices]))
                for vertex, joined_vertices in self._adjacency_list.iteritems()])
        else:
            return '\n'.join(['%s:\t%s' % (vertex, ', '.join(joined_vertices))
                       for vertex, joined_vertices in self._adjacency_list.iteritems()])
    
    def __repr__(self):
        '''"Official" string representation of the Adjacency List'''
        return str(self._adjacency_list) + (('\n' + str(self._weight)) if self._weighted else '') 

    ####################
    # Accessor methods #
    ####################
    
    #Apparently accessor methods are redundant in Python?

    def is_weighted(self):
        '''Returns True if the edges of the graph are weighted; False otherwise.'''
        return self._weighted

    def is_directed(self):
        '''Returns True if the graph is directed; False otherwise.'''
        return self._directed

    def vertices(self):
        '''Returns a list of all vertices in the graph.'''
        return self._adjacency_list.keys()

    def edges(self):
        '''Returns a copy of the original list of tuples fed in.'''
        return self._edge_list
    
    def num_vertices(self):
        '''Returns the number of vertices in the graph.'''
        return len(self._adjacency_list.keys())
    
    def num_edges(self):
        '''Returns the number of edges in the graph.'''
        return len(self._edge_list)

    def weight(v1, v2):
        '''Returns the weight of an edge between two given vertices.'''
        return self._weight[(v1,v2)] if (v1,v2) in self._weight.keys() else None


    ########################
    # Traversal Algorithms #
    ########################
   
    def _XFS(self, is_DFS, root=None):
        '''X first-search: generalised for both Depth- and Breadth-first searches'''
        if not isinstance(is_DFS, bool):
            raise ValueError('Must specify whether search type is DFS or not (hence BFS).')
        deq = deque()
        marked = { vertex : False for vertex in self._adjacency_list.iterkeys() }           # Make all vertices unmarked.

        root = root or self._adjacency_list.keys()[0]
        deq.append(root)
        marked[root] = True
        XFS = []

        while len(deq):
            vertex = deq.pop() if is_DFS else deq.popleft()                                 # Pop off the stack if DFS, or dequeue it if BFS.
            XFS.append(vertex)

            for adjacent_vertex in self._adjacency_list[vertex]:                            # Grab each unmarked adjacent vertex, 
                if not marked[adjacent_vertex]:
                    marked[adjacent_vertex] = True                                          # mark it
                    deq.append(adjacent_vertex)                                             # and shove it into the deque
    
        return XFS

    def BFS(self, root=None):
        '''Returns a traversal of the graph via the Breadth-first search algorithm.
        If no root is given then the first node in the graph is chosen.'''
        return self._XFS(True, root)      


    def DFS(self, root=None):
        '''Returns a traversal of the graph via the Depth-first search algorithm.
        If no root is given then the first node in the graph is chosen.'''
        return self._XFS(False, root)


    #################
    # Minimum paths #
    #################

    def Floyd_Warshall(self):
        '''Returns a dictionary where each pair (v1,v2) returns the length of the smallest path between them.'''
        if any((weight < 0 for weight in self._weight.itervalues())):               # Check if any edges have negative values
            raise ValueError("Graph cannot have negative weights.")


        vertices = self._adjacency_list.keys() 
        path = { (v1,v2): 0 if v1 is v2
                            else self._weight[(v1,v2)] if (v1,v2) in self._weight.keys()
                            else float('inf') 
                     for v1 in vertices
                     for v2 in vertices }
    
        for k in vertices:
            for i in vertices:
                for j in vertices:
                    path[(i,j)] = min(path[(i,j)], path[(i,k)] + path[(k,j)])

        return path
    

    def Dijkstra(self, root=None, target=None):
        '''Find the shortest path between the root and a target.'''

        vertices = self._adjacency_list.keys()
        distance = { vertex : float('inf') for vertex in vertices }                     # Dictionary of all distances from the root to each vertex
        previous = { vertex : None for vertex in vertices }                             # Reference to the previous node in the optimal path from the root

        root = root or self._adjacency_list.keys()[0]
        distance[root] = 0
        
        while len(vertices):
            vertex = min(vertices, key = lambda v: distance[v])                         # Grab the vertex with the smallest distance to the source.
            if target == vertex:                                                        # If a target was specified then we can determine the optimal path 
                curr_vertex = target                                                    # between the source and itself.
                min_path = deque()
                while previous[curr_vertex]:                                            # While the previous vertex in the optimal path exists, 
                    min_path.appendleft(curr_vertex)                                    # smack it to the front of the path.
                    curr_vertex = previous[curr_vertex]                                 # Grab the previous vertex of the vertex we just appended.
                min_path.appendleft(root)
                return list(min_path)                                                   #We don't really need a deque, just append and then reverse
            
            if distance[vertex] == float('inf'):                                        # If all the remaining vertices are detached then we end the algorithm.
                break;

            vertices.remove(vertex)
            
            for adjacent_vertex in self._adjacency_list[vertex]:                        # For each adjacent vertex of the current vertex 
                if adjacent_vertex in vertices:                                         # which hasn't been removed from vertices.
                    alt = distance[vertex] + self._weight[(vertex, adjacent_vertex)]    # Get the distance of the alternate path from the adjacent to the current vertex.
                    if alt < distance[adjacent_vertex]:                                 # If this new distance is smaller,
                        distance[adjacent_vertex] = alt                                 # then set it as the new distance
                        previous[adjacent_vertex] = vertex                              # and set previous (optimal) vertex of adjacent vertex to be the current vertex.

                        i = vertices.index(adjacent_vertex)                             # Shift adjacent_vertex left in vertices 
                        vertices[i], vertices[i-1] = vertices[i-1], vertices[i]

        return distance
