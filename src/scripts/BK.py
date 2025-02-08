import sys
input = sys.stdin.readline

from collections import deque

def bfs(sr,sy,visit):
    answer = mat[sr][sy]
    visit[sr][sy] = True
    qu = deque([sr])
    
    while qu:
        qu.popleft()
        if visit[sr+1][sr-1]:

        



n, m = map(int, input().split())

# 올바른 2D 리스트 초기화 , 0,0 에서 n-1,m-1까지 최단경로 
mat = [[0] * m for _ in range(n)]  

for i in range(n):
    row = input().rstrip() # 개행 문자 제거
    for j in range(len(row)):
        mat[i][j] = int(row[j])  # 숫자로 변환

# print(mat)
# 그래프로 만든 다음에 검토하는게 빠를까?
visit = [[False] * m for _ in range(n)] 
bfs(0,0,visit)

