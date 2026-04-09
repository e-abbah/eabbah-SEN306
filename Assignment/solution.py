# ============================================================
# Assignment 1 – Drone Delivery Navigation: Python Solution
# Algorithms: DFS, BFS, UCS, A*
# ============================================================

import heapq
from collections import deque

graph = {
    'A': {'B': 2, 'C': 5, 'D': 1},
    'B': {'A': 2, 'D': 2, 'E': 3},
    'C': {'A': 5, 'D': 2, 'F': 3},
    'D': {'A': 1, 'B': 2, 'C': 2, 'E': 1, 'F': 4},
    'E': {'B': 3, 'D': 1, 'G': 2},
    'F': {'C': 3, 'D': 4, 'G': 1},
    'G': {'E': 2, 'F': 1, 'H': 3},
    'H': {'G': 3}
}

heuristic = {
    'A': 7, 'B': 6, 'C': 6, 'D': 4,
    'E': 2, 'F': 2, 'G': 1, 'H': 0
}

start = 'A'
goal  = 'H'

# ----------------------------------------------------------
# 1. DFS – Depth-First Search (no edge costs)
# ----------------------------------------------------------
def dfs(graph, start, goal):
    stack = [(start, [start])]
    visited = set()
    nodes_expanded = 0

    while stack:
        node, path = stack.pop()
        if node in visited:
            continue
        visited.add(node)
        nodes_expanded += 1

        if node == goal:
            return path, len(path) - 1, nodes_expanded

        for neighbour in reversed(list(graph[node])):
            if neighbour not in visited:
                stack.append((neighbour, path + [neighbour]))

    return None, None, nodes_expanded

# ----------------------------------------------------------
# 2. BFS – Breadth-First Search (no edge costs)
# ----------------------------------------------------------
def bfs(graph, start, goal):
    queue = deque([(start, [start])])
    visited = {start}
    nodes_expanded = 0

    while queue:
        node, path = queue.popleft()
        nodes_expanded += 1

        if node == goal:
            return path, len(path) - 1, nodes_expanded

        for neighbour in graph[node]:
            if neighbour not in visited:
                visited.add(neighbour)
                queue.append((neighbour, path + [neighbour]))

    return None, None, nodes_expanded

# ----------------------------------------------------------
# 3. UCS – Uniform Cost Search (uses edge costs)
# ----------------------------------------------------------
def ucs(graph, start, goal):
    # heap entries: (cost, node, path)
    heap = [(0, start, [start])]
    visited = {}
    nodes_expanded = 0

    while heap:
        cost, node, path = heapq.heappop(heap)

        if node in visited and visited[node] <= cost:
            continue
        visited[node] = cost
        nodes_expanded += 1

        if node == goal:
            return path, cost, nodes_expanded

        for neighbour, weight in graph[node].items():
            new_cost = cost + weight
            if neighbour not in visited or visited[neighbour] > new_cost:
                heapq.heappush(heap, (new_cost, neighbour, path + [neighbour]))

    return None, None, nodes_expanded

# ----------------------------------------------------------
# 4. A* Search (uses edge costs + heuristic)
# ----------------------------------------------------------
def astar(graph, start, goal, heuristic):
    # heap entries: (f, g, node, path)
    heap = [(heuristic[start], 0, start, [start])]
    visited = {}
    nodes_expanded = 0

    while heap:
        f, g, node, path = heapq.heappop(heap)

        if node in visited and visited[node] <= g:
            continue
        visited[node] = g
        nodes_expanded += 1

        if node == goal:
            return path, g, nodes_expanded

        for neighbour, weight in graph[node].items():
            new_g = g + weight
            new_f = new_g + heuristic[neighbour]
            if neighbour not in visited or visited[neighbour] > new_g:
                heapq.heappush(heap, (new_f, new_g, neighbour, path + [neighbour]))

    return None, None, nodes_expanded

# ----------------------------------------------------------
# 5. OPTIONAL – Battery-constrained UCS
# ----------------------------------------------------------
def ucs_battery(graph, start, goal, max_battery):
    heap = [(0, start, [start])]
    visited = {}
    nodes_expanded = 0

    while heap:
        cost, node, path = heapq.heappop(heap)

        if node in visited and visited[node] <= cost:
            continue
        visited[node] = cost
        nodes_expanded += 1

        if node == goal:
            return path, cost, nodes_expanded

        for neighbour, weight in graph[node].items():
            new_cost = cost + weight
            if new_cost <= max_battery and (neighbour not in visited or visited[neighbour] > new_cost):
                heapq.heappush(heap, (new_cost, neighbour, path + [neighbour]))

    return None, None, nodes_expanded

# ----------------------------------------------------------
# Run and print results
# ----------------------------------------------------------
print("=" * 55)
print("   DRONE DELIVERY NAVIGATION – SEARCH ALGORITHMS")
print("=" * 55)

dfs_path,  dfs_steps,  dfs_exp  = dfs(graph, start, goal)
bfs_path,  bfs_steps,  bfs_exp  = bfs(graph, start, goal)
ucs_path,  ucs_cost,   ucs_exp  = ucs(graph, start, goal)
ast_path,  ast_cost,   ast_exp  = astar(graph, start, goal, heuristic)

results = [
    ("DFS",  dfs_path,  f"{dfs_steps} steps",  dfs_exp),
    ("BFS",  bfs_path,  f"{bfs_steps} steps",  bfs_exp),
    ("UCS",  ucs_path,  f"cost = {ucs_cost}",  ucs_exp),
    ("A*",   ast_path,  f"cost = {ast_cost}",  ast_exp),
]

for name, path, metric, expanded in results:
    print(f"\n{name}")
    print(f"  Path          : {' → '.join(path)}")
    print(f"  {metric}")
    print(f"  Nodes expanded: {expanded}")

# Battery constraint demo
print("\n" + "=" * 55)
print("OPTIONAL: Battery-Constrained UCS (max = 7)")
print("=" * 55)
b_path, b_cost, b_exp = ucs_battery(graph, start, goal, max_battery=7)
if b_path:
    print(f"  Path          : {' → '.join(b_path)}")
    print(f"  cost = {b_cost}")
    print(f"  Nodes expanded: {b_exp}")
else:
    print("  No path found within battery limit of 7.")

print("\nBattery-Constrained UCS (max = 10)")
b_path, b_cost, b_exp = ucs_battery(graph, start, goal, max_battery=10)
if b_path:
    print(f"  Path          : {' → '.join(b_path)}")
    print(f"  cost = {b_cost}")
    print(f"  Nodes expanded: {b_exp}")
else:
   print("  No path found within battery limit of 10.")



