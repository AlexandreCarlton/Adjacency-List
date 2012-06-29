from Queue import Queue

def edge_list(file):
    '''Convert text file into a list of tuples'''
    return [pair.strip().split() for pair in open(file, 'rU')]

class Adjacency_List(object):
    '''A dictionary where the key is a vertex and the value a list of all the vertices it is adjacent to.
    Includes algorithms such as BFS, DFS, etc.'''

    def __init__(self, edge_list, directed=False, weighted=False):#edge_list being a list of tuples (eg. (A, B, 2))
        self.adj_list = {}
        if not isinstance(directed, bool):
            raise TypeError("Directed must be a boolean value.")
        if not isinstance(weighted, bool):
            raise TypeError("Weighted mut be a boolean value.")
        self.directed = directed
        self.weighted = weighted
        
        #Could probably make an add_edge(A,B) or something to make this easier.               
        for edge in edge_list:
            if self.weighted:
                if len(edge) is not 3:
                    raise ValueError("Only two vertices and an edge per line.")
                v1, v2 = edge[:2]
                weight = int(edge[2])
            else:
                if len(edge) is not 2:
                    raise ValueError("Only two vertices per line.")
                v1, v2 = edge

            if v1 in self.adj_list.keys():
                self.adj_list[v1].append((v2, weight) if self.weighted else v2)
            else:
                self.adj_list[v1] = [(v2, weight) if self.weighted else v2]

            if not self.directed:
                if v2 in self.adj_list.keys():
                    self.adj_list[v2].append((v1, weight) if self.weighted else v1)
                else:
                    self.adj_list[v2] = [(v1, weight) if self.weighted else v1]
###        print self.adj_list        




    def __str__(self):
        '''String representation of the Adjacency List'''
        if self.weighted:
            return '\n'.join(
                ['%s:\t%s' % (vertex, ', '.join(
                ['%s: %s' % (joined_vertex, weight) for joined_vertex, weight in joined_vertices]))
                 for vertex, joined_vertices in self.adj_list.iteritems()])
    
        return '\n'.join(
            ['%s:\t%s' % (vertex, ', '.join(joined_vertices))
             for vertex, joined_vertices in self.adj_list.iteritems()])
    
    def __repr__(self):
        '''"Official" string representation of the Adjacency List'''
        return str(self.adj_list)


    def is_weighted(self):
        '''Returns True if the edges of the graph are weighted; False otherwise.'''
        return self.weighted

    def is_directed(self):
        '''Returns True if the graph is directed; False otherwise.'''
        return self.directed

    ########################
    # Traversal Algorithms #
    ########################
    
    def BFS(self, root=None):
        '''Returns a traversal of the graph via the Breadth-first search algorithm.
        If no root is given then the first node in the graph is chosen.'''
        
        queue = Queue()
        marked = {}
        
        for vertex in self.adj_list.iterkeys():
            marked[vertex] = False

        #a or b returns a if it is NOT False/None, else returns b
        #a and b returns a if it IS False/None, else returns b
        root = root or self.adj_list.keys()[0]
        queue.put(root)
        marked[root] = True

        BFS = []
        while not queue.empty():
            vertex = queue.get()
            if self.weighted: 
                vertex = vertex[0] 
            BFS.append(vertex)
            
            for joined_vertex in self.adj_list[vertex]:
                if self.weighted:
                    joined_vertex = joined_vertex[0]
                if not marked[joined_vertex]:
                    marked[joined_vertex] = True
                    queue.put(joined_vertex)
        
        return BFS


    def DFS(self, root=None):
        '''Returns a traversal of the graph via the Depth-first search algorithm.
        If no root is given then the first node in the graph is chosen.'''

        stack = []
        marked = {}
        
        for vertex in self.adj_list.iterkeys():
            marked[vertex] = False
        
        root = root or self.adj_list.keys()[0]
        stack.append(root)
        marked[root] = True

        DFS = []
        while len(stack):
            vertex = stack.pop()
            if self.weighted:
                vertex = vertex[0]
            DFS.append(vertex)
            for joined_vertex in self.adj_list[vertex]:
                if self.weighted:
                    joined_vertex = joined_vertex[0]
                if not marked[joined_vertex]:
                    marked[joined_vertex] = True
                    stack.append(joined_vertex)
        
        return DFS


