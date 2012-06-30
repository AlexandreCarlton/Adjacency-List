from collections import deque

def vertices_and_edges(file):
    '''Convert text file into a list of vertices and edges'''
    vertices = [vertex.strip() for vertex in open(file, 'rU') if len(vertex.strip().split()) == 1]
    edges = [pair.strip().split() for pair in open(file, 'rU') if len(pair.strip().split()) in (2,3)]
    print vertices
    return vertices, edges


#PROBLEM: Doesn't allow for isolated vertices (only takes in connected graphs)
#Resolution: Take in a list of vertices as well as a list of edges.
class Adjacency_List(object):
    '''A dictionary where the key is a vertex and the value a list of all the vertices it is adjacent to.
    Includes algorithms such as BFS, DFS, etc.'''

    def __init__(self, vertices, edges, directed=False, weighted=False):#edge_list being a list of tuples (eg. ('A', 'B', 2))
        if not isinstance(directed, bool):
            raise TypeError("Directed must be a boolean value.")
        if not isinstance(weighted, bool):
            raise TypeError("Weighted must be a boolean value.")
               
        self.directed = directed
        self.weighted = weighted
        
        self.adj_list = {vertex : [] for vertex in vertices}
        self.edge_list = edges
        if self.weighted:
            self.weight = {} #weight[(a,b)] -> int


        #Could probably make an add_edge(A,B) or something to make this easier.               
        for edge in self.edge_list:
            if self.weighted and len(edge) != 3:
                raise ValueError("Only two vertices and an edge per tuple.")
            if not self.weighted and len(edge) != 2:
                raise ValueError("Only two vertices per tuple.")

            v1, v2 = edge[:2]
            self.adj_list[v1].append(v2)
            if self.weighted:
                self.weight[(v1,v2)] = int(edge[2])
            
            #v1 is adjacent to v2 as well if the graph is not directed
            if not self.directed:
                self.adj_list[v2].append(v1)
                if self.weighted:
                    self.weight[(v2,v1)] = int(edge[2])


    ###################
    # Special methods #
    ###################

    def __str__(self):
        '''String representation of the Adjacency List'''
        return '\n'.join(
            ['%s:\t%s' % (vertex, ', '.join(
                ['%s: %s' % (joined_vertex, self.weight[(vertex, joined_vertex)]) 
                 for joined_vertex in joined_vertices]))
            for vertex, joined_vertices in self.adj_list.iteritems()]) if self.weighted else '\n'.join(
            ['%s:\t%s' % (vertex, ', '.join(joined_vertices))
             for vertex, joined_vertices in self.adj_list.iteritems()])
    
    def __repr__(self):
        '''"Official" string representation of the Adjacency List'''
        return str(self.adj_list) + (('\n' + str(self.weight)) if self.weighted else '') 


    ####################
    # Accessor methods #
    ####################

    def is_weighted(self):
        '''Returns True if the edges of the graph are weighted; False otherwise.'''
        return self.weighted

    def is_directed(self):
        '''Returns True if the graph is directed; False otherwise.'''
        return self.directed

    def edge_list(self):
        '''Returns a copy of the original list of tuples fed in.'''
        return self.edge_list[:]
    
    def num_edges(self):
        '''Returns the number of edges in the graph.'''
        return len(self.edge_list)

    def num_vertices(self):
        '''Returns the number of vertices in the graph.'''
        return len(self.adj_list.keys())

    def weight(v1, v2):
        '''Returns the weight of an edge between two given vertices (if the graph is weighted).'''
        if not self.weighted:
            raise ValueError("Graph is not weighted.")
        return self.weight[(v1,v2)]


    ########################
    # Traversal Algorithms #
    ########################
   
    def _XFS(self, search_type, root=None):
        '''X first-search: generalised for both Depth- and Breadth-first searches'''
        if search_type not in ('BFS', 'DFS'):
            raise ValueError("Must specify whether the search is DFS or BFS")
        
        deq = deque()
        marked = {}

        for vertex in self.adj_list.iterkeys():
            marked[vertex] = False

        root = root or self.adj_list.keys()[0]
        deq.append(root)
        marked[root] = True
        XFS = []

        while len(deq):
            vertex = deq.pop() if search_type is 'DFS' else deq.popleft()
            XFS.append(vertex)

            #Grab each unmarked adjacent vertex and shove it into the deque
            for joined_vertex in self.adj_list[vertex]:
                if not marked[joined_vertex]:
                    marked[joined_vertex] = True
                    deq.append(joined_vertex)
    
        return XFS

    def BFS(self, root=None):
        '''Returns a traversal of the graph via the Breadth-first search algorithm.
        If no root is given then the first node in the graph is chosen.'''
        return self._XFS('BFS', root)      


    def DFS(self, root=None):
        '''Returns a traversal of the graph via the Depth-first search algorithm.
        If no root is given then the first node in the graph is chosen.'''
        return self._XFS('DFS', root)


    #################
    # Minimum paths #
    #################

    def Floyd_Warshall(self):
        '''Returns a dictionary where each pair (v1,v2) returns the length of the smallest path between them.'''
        if not self.weighted:
            raise ValueError("Graph must be weighted in order to execute algorithm.")
        
        vertices = self.adj_list.keys() 
        path = { (v1,v2): 0 if v1 is v2 
                       else self.weight[(v1,v2)] if (v1,v2) in self.weight.keys() 
                       else float('inf') 
                     for v1 in vertices
                     for v2 in vertices}
    
        for k in vertices:
            for i in vertices:
                for j in vertices:
                    path[(i,j)] = min(path[(i,j)], path[(i,k)] + path[(k,j)])

        return path
    

    def Dijkstra(self, root=None, target=None):
        ''' Description goes here '''
        if not self.weighted:
            raise ValueError('Graph must be weighted in order to execute algorithm.')
        
        dist = { vertex : float('inf') for vertex in self.adj_list.iterkeys() }
        previous = { vertex : None for vertex in self.adj_list.iterkeys() }
        vertices = self.adj_list.keys()

        root = root or self.adj_list.keys()[0]
        dist[root] = 0

        while len(vertices):
            vertex = min(vertices, key = lambda v: dist[v]) #Grabs key with smallest value
###            if target == vertex: return dist
            
            if dist[vertex] == float('inf'):
                break;

            vertices.remove(vertex)

            for adjacent_vertex in self.adj_list[vertex]:
                alt = dist[vertex] + self.weight[(vertex, adjacent_vertex)]
                if alt < dist[adjacent_vertex]:
                    dist[adjacent_vertex] = alt
                    previous[adjacent_vertex] = vertex

                    #Shift adjacent_vertex left
                    i = vertices.index(adjacent_vertex)
                    vertices[i], vertices[i-1] = vertices[i-1], vertices[i]

        return dist
