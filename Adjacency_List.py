from collections import deque

def vertices_and_edges(file):
    '''Convert text file into a list of vertices and edges
    Assumes file is of format:
    A
    B
    C
    A B 2
    C A 5
    Listing vertices, and then the edges (with weights if the graph is weighted).'''
    vertices = [vertex.strip() for vertex in open(file, 'rU') if len(vertex.strip().split()) == 1]
    edges = [pair.strip().split() for pair in open(file, 'rU') if len(pair.strip().split()) in (2,3)]
    print vertices
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
        if self._weighted:
            self._weight = {} #weight[(a,b)] -> int
        

        #Could probably make an add_edge(A,B) or something to make this easier.               
        for edge in edges:
            if self._weighted and len(edge) != 3:
                raise ValueError("Only two vertices and an edge per tuple.")
            if not self._weighted and len(edge) != 2:
                raise ValueError("Only two vertices per tuple.")

            v1, v2 = edge[:2]
            self._adjacency_list[v1].append(v2)
            self._weight[(v1,v2)] = int(edge[2]) if self._weighted else 1 #An unweighted graph has edges of weight 1.
            
            #v1 is adjacent to v2 as well if the graph is not directed
            if not self._directed:
                self._adjacency_list[v2].append(v1)
                self._weight[(v2,v1)] = int(edge[2]) if self._weighted else 1 

    ###################
    # Special methods #
    ###################

    def __str__(self):
        '''String representation of the Adjacency List'''
        return '\n'.join(
            ['%s:\t%s' % (vertex, ', '.join(
                ['%s: %s' % (joined_vertex, self._weight[(vertex, joined_vertex)]) 
                 for joined_vertex in joined_vertices]))
            for vertex, joined_vertices in self._adjacency_list.iteritems()]) if self._weighted else '\n'.join(
            ['%s:\t%s' % (vertex, ', '.join(joined_vertices))
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
        '''Returns the weight of an edge between two given vertices (if the graph is weighted).'''
        if not self._weighted:
            raise ValueError("Graph is not weighted.")
        return self._weight[(v1,v2)]


    ########################
    # Traversal Algorithms #
    ########################
   
    def _XFS(self, is_DFS, root=None):
        '''X first-search: generalised for both Depth- and Breadth-first searches'''
        if not isinstance(is_DFS, bool):
            raise ValueError('Must specify whether search type is DFS or not (hence BFS).')
        deq = deque()
        marked = { vertex : False for vertex in self._adjacency_list.iterkeys() }

        root = root or self._adjacency_list.keys()[0]
        deq.append(root)
        marked[root] = True
        XFS = []

        while len(deq):
            vertex = deq.pop() if is_DFS else deq.popleft()
            XFS.append(vertex)

            #Grab each unmarked adjacent vertex and shove it into the deque
            for adjacent_vertex in self._adjacency_list[vertex]:
                if not marked[adjacent_vertex]:
                    marked[adjacent_vertex] = True
                    deq.append(adjacent_vertex)
    
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
        if not self._weighted:
            raise ValueError("Graph must be weighted in order to execute algorithm.")
        if any((weight < 0 for weight in self._weight.itervalues())):
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
        ''' Description goes here '''
        if not self._weighted:
            raise ValueError('Graph must be weighted in order to execute algorithm.')
        
        vertices = self._adjacency_list.keys()
        dist = { vertex : float('inf') for vertex in vertices }
        previous = { vertex : None for vertex in vertices }

        root = root or self._adjacency_list.keys()[0]
        dist[root] = 0

        while len(vertices):
            vertex = min(vertices, key = lambda v: dist[v]) #Grabs key with smallest value
###            if target == vertex: return dist
            
            if dist[vertex] == float('inf'):
                break;

            vertices.remove(vertex)

            for adjacent_vertex in self._adjacency_list[vertex]:
                alt = dist[vertex] + self._weight[(vertex, adjacent_vertex)]
                if alt < dist[adjacent_vertex]:
                    dist[adjacent_vertex] = alt
                    previous[adjacent_vertex] = vertex

                    #Shift adjacent_vertex left
                    i = vertices.index(adjacent_vertex)
                    vertices[i], vertices[i-1] = vertices[i-1], vertices[i]

        return dist
