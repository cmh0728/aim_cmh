def dfs_stack(start, graph):
    stack = [start]
    visited = {i: False for i in graph}
    
    while stack:
        node = stack.pop()
        if not visited[node]:
            visited[node] = True
            print(node, end=" ")  # 방문한 노드 출력

            for neighbor in reversed(graph[node]):
                if not visited[neighbor]:
                    stack.append(neighbor)

# 그래프 초기화
graph = {
    1: [2, 3],
    2: [1, 4, 5],
    3: [1, 6],
    4: [2],
    5: [2],
    6: [3]
}

print("DFS 탐색 순서:", end=" ")
dfs_stack(1, graph)
