from collections import deque

def edge_list(file):
    '''Convert text file into a list of tuples'''
    return [pair.strip().split() for pair in open(file, 'rU')]

class Adjacency_List(object):
    '''A dictionary where the key is a vertex and the value a list of all the vertices it is adjacent to.
    Includes algorithms such as BFS, DFS, etc.'''

    def __init__(self, edge_list, directed=False, weighted=False):#edge_list being a list of tuples (eg. ('A', 'B', 2))
        if not isinstance(directed, bool):
            raise TypeError("Directed must be a boolean value.")
        if not isinstance(weighted, bool):
            raise TypeError("Weighted must be a boolean value.")
        
        self.adj_list = {}
        self.edge_list = edge_list
        self.weight = {} #weight[(a,b)] -> int

        self.directed = directed
        self.weighted = weighted
        
        #Could probably make an add_edge(A,B) or something to make this easier.               
        for edge in edge_list:
            if self.weighted and len(edge) is not 3:
                raise ValueError("Only two vertices and an edge per tuple.")
            if not self.weighted and len(edge) is not 2:
                raise ValueError("Only two vertices per tuple.")

            v1, v2 = edge[:2]

            if v1 in self.adj_list.keys():
                self.adj_list[v1].append(v2)
            else:
                self.adj_list[v1] = [v2]
            if self.weighted:
                self.weight[(v1,v2)] = int(edge[2])
            
            #v1 is adjacent to v2 as well if the graph is not directed
            if not self.directed:
                if v2 in self.adj_list.keys():
                    self.adj_list[v2].append(v1)
                else:
                    self.adj_list[v2] = [v1]
                if self.weighted:
                    self.weight[(v2,v1)] = int(edge[2])


    ###################
    # Special methods #
    ###################

    def __str__(self):
        '''String representation of the Adjacency List'''
        if self.weighted:
            return '\n'.join(
                ['%s:\t%s' % (vertex, ', '.join(
                    ['%s: %s' % (joined_vertex, self.weight[(vertex, joined_vertex)]) 
                     for joined_vertex in joined_vertices]))
                 for vertex, joined_vertices in self.adj_list.iteritems()])
    
        return '\n'.join(
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

    def weight(a, b):
        '''Returns the weight of a given edge if the graph is weighted.'''
        if not self.weighted:
            raise ValueError("Graph is not weighted.")
        return self.weight[(a,b)]


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
