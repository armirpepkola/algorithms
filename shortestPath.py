import heapq

def shortestPath(graph_file):
    #Read the information from the text file to form the graph
    graph, start_node, nodes = readGraph(graph_file)

    #Initialize dist with a distance of infinity for each node
    dist = {node: float('inf') for node in nodes}

    #Run DFS algorithm to check if there are cycles in the graph
    cycle = DFS(graph, start_node)

    #Run Dijkstra's algorithm if there are no cycles / Run Bellman-Ford's algorithm if there are cycles
    if not cycle:
        dist = Dijkstra(graph, start_node, dist)
    else:
        dist = BellmanFord(graph, start_node, dist)

    #Print the shortest path distances
    print(dist)

def readGraph(graph_file):
    with open(graph_file, 'r') as file:
        lines = file.readlines()
        graph = {}
        nodes = set()
        for line in lines[3:]:
            u, v, w = line.split()
            nodes.add(u)
            nodes.add(v)
            if u not in graph:
                graph[u] = []
            graph[u].append((v, int(w)))
        start_node = lines[2].strip()
    return graph, start_node, nodes

def DFS(graph, start_node):
    visited = set()
    return DFS_helper(graph, start_node, visited)

def DFS_helper(graph, node, visited):
    visited.add(node)
    for neighbor, _ in graph[node]:
        if neighbor not in visited:
            if DFS_helper(graph, neighbor, visited):
                return True
        elif neighbor in visited:
            return True
    return False

def Dijkstra(graph, start_node, dist):
    dist[start_node] = 0
    pq = [(0, start_node)]
    while pq:
        _, node = heapq.heappop(pq)
        for neighbor, weight in graph[node]:
            distance = dist[node] + weight
            if distance < dist[neighbor]:
                dist[neighbor] = distance
                heapq.heappush(pq, (distance, neighbor))
    return dist

def BellmanFord(graph, start_node, dist):
    dist[start_node] = 0
    for _ in range(len(graph) - 1):
        for node in graph:
            for neighbor, weight in graph[node]:
                if dist[node] != float('inf') and dist[node] + weight < dist[neighbor]:
                    dist[neighbor] = dist[node] + weight
    return dist

#Call the function
shortestPath('graph.txt')