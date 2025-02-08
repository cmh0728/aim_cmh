from collections import deque

def bfs(start, graph, visited):
    queue = deque([start])
    # print(queue)
    visited[start] = True
    
    while queue:
        node = queue.popleft()
        print(node, end=" ")  # 방문한 노드 출력

        for neighbor in graph[node]:
            if not visited[neighbor]:
                queue.append(neighbor)
                visited[neighbor] = True

# 그래프 초기화
graph = {
    1: [2, 3],
    2: [1, 4, 5],
    3: [1, 6],
    4: [2],
    5: [2],
    6: [3]
}
visited = {i: False for i in range(1, 7)}

print(visited)

print("BFS 탐색 순서:", end=" ")
bfs(1, graph, visited)
