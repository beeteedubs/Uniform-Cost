import queue

class Vertex:
    def __init__(self, name):
        self.neighborList = []  # list of tuples, where 2nd value is the distance
        self.name = name

    def invert_add_neighbor(self, List_Of_Lists):  # so can add all neighbors at once
        for node in List_Of_Lists:
            # makes G = -G
            if node[1] == 0:
                node[1] = 2
            elif node[1] == 2:
                node[1] = 0
            self.neighborList.append(node)

    def __lt__(self, other):
        return self.name < other.name

    def printNeighbors(self):
        for x in self.neighborList:
            print("(" + x[0].name + "," + str(x[1]) + ")")

    def add_neighbors(self,list_of_lists):
        for node in list_of_lists: # nodes: [Vertex, cost]
            self.neighborList.append(node)
            #node[0].neighborList.append([self,node[1]])
            #above code makes it unnecessary to add same edge 2x


# arg: start and goal vertices
# returns: dictionary indicating order of vertices to explore or False if failed
# display actual cost
def UniformCost(start_vertex, goal_vertex):

    curr_vertex = None

    """ 4 main data structures"""
    dict = {start_vertex.name: None}                                   # key = current index, value = prev index
    frontier = queue.PriorityQueue()            # list of tuples (cost, Vertex), used for heap functions
    frontier_vertices = []                      # list of to-be explored vertices, used for easily checking if vertices are in frontier
    explored = []                               # list of explored Vertices,

    frontier.put([0, start_vertex])
    frontier_vertices.append(start_vertex)

    while True:  # just run until you break cuz of success or failure

        # stop if frontier is empty
        if len(frontier_vertices) <= 0:
            print("No solution found ")
            break

        # pop from frontier and update dict
        tup = frontier.get() # (path cost, Vertex)

        # dict stuff, not part of pseudocode
        curr_vertex = tup[1]
        frontier_vertices.remove((curr_vertex))

        # break if nextVertex is goal state
        if curr_vertex == goal_vertex:
            print("Found solution with a cost of: " + str(tup[0]))
            return dict

        # add node to explored
        explored.append(curr_vertex)

        # add in the neighbors to frontier and visited
        for neighbor in curr_vertex.neighborList:
            # if child is not in explored or frontier
            in_explored = neighbor[0] in explored
            in_frontier = neighbor[0] in frontier_vertices

            # accumulate travel cost
            updated_cost = neighbor[1] + tup[0]

            if in_explored is False and in_frontier is False:
                dict[neighbor[0].name] = curr_vertex.name
                frontier.put([updated_cost, neighbor[0]])
                frontier_vertices.append(neighbor[0])
            # I found a shorter path! update the cost
            elif in_frontier is True:
                for n in frontier.queue:
                    if n[1] == neighbor[0] and updated_cost < n[0]:
                        n[0] = updated_cost
                        dict[neighbor[0].name] = curr_vertex.name
    return dict

def printPath(dict, finish):
    index = finish.name
    path_list = []
    path_list.append(index)
    while True:
        path_list.append(dict[index])
        if dict[dict[index]] == None:
            break
        else:
            index = dict[index]
    path_list.reverse()
    print("The path is: ", end =" ")
    for i in path_list:
        print(i + "-> ", end = " ")
    print()
    return None



def main():

    start = Vertex("start")
    a = Vertex("a")
    b = Vertex("b")
    c = Vertex("c")
    d = Vertex("d")
    finish = Vertex("finish")
    start.add_neighbors([[a,4],[b,6],[c,5]])
    a.add_neighbors([[start,4],[b,2],[d,5],[c,0]])
    b.add_neighbors([[start,6],[a,2],[c,1],[finish,12]])
    c.add_neighbors([[start,5],[a,0],[b,1],[d,4]])
    d.add_neighbors([[a,5],[c,4],[finish,4]])
    finish.add_neighbors([[b,12],[d,4]])

    """
    a = Vertex("a")
    b = Vertex("b")
    c = Vertex("c")
    d = Vertex("d")
    e = Vertex("e")
    f = Vertex("f")
    g = Vertex("g")
    h = Vertex("h")
    i = Vertex("i")
    j = Vertex("j")
    k = Vertex("k")
    l = Vertex("l")
    m = Vertex("m")
    n = Vertex("n")
    o = Vertex("o")
    p = Vertex("p")
    q = Vertex("q")
    r = Vertex("r")
    s = Vertex("s")
    t = Vertex("t")
    u = Vertex("u")
    v = Vertex("v")
    w = Vertex("w")
    x = Vertex("x")
    y = Vertex("y")
    z = Vertex("z")
    finish = Vertex("finish")
    start.invert_add_neighbor([[a, 0], [d, 0]])
    a.invert_add_neighbor([[b, 1], [c, 0]])
    d.invert_add_neighbor([[e, 1], [p, 1]])
    b.invert_add_neighbor([[c, 1], [t, 2]])
    c.invert_add_neighbor([[b, 1], [q, 1]])
    e.invert_add_neighbor([[f, 0], [h, 2]])
    p.invert_add_neighbor([[f, 0], [o, 0]])
    t.invert_add_neighbor([[s, 0], [x, 1]])
    q.invert_add_neighbor([[s, 0], [o, 1]])
    f.invert_add_neighbor([[p, 0], [g, 2]])
    h.invert_add_neighbor([[g, 0], [i, 2]])
    g.invert_add_neighbor([[h, 0], [m, 1]])
    i.invert_add_neighbor([[y, 1]])
    j.invert_add_neighbor([[k, 0], [i, 1]])
    k.invert_add_neighbor([[j, 0], [y, 1]])
    l.invert_add_neighbor([[r, 0], [j, 1]])
    m.invert_add_neighbor([[n, 1], [l, 0]])
    n.invert_add_neighbor([[m, 1], [r, 0]])
    o.invert_add_neighbor([[q, 1], [g, 1]])
    r.invert_add_neighbor([[l, 0], [u, 0]])
    s.invert_add_neighbor([[t, 0], [v, 0]])
    t.invert_add_neighbor([[s, 0], [x, 1]])
    u.invert_add_neighbor([[k, 0], [v, 1]])
    v.invert_add_neighbor([[u, 1], [w, 1]])
    w.invert_add_neighbor([[z, 0], [x, 1]])
    x.invert_add_neighbor([[w, 1], [z, 1]])
    y.invert_add_neighbor([[finish, 0]])
    z.invert_add_neighbor([[finish, 1]])
    """
    # UniformCost(start, finish)
    printPath(UniformCost(start, finish), finish)
    """
    list_of_tupes = [(0, a), (1, b), (0, c), (0, d)]
    ff = heapq.heappop(list_of_tupes)"""
main()
