import sys
input = sys.stdin.readline

def dfs():
    if len(s) == m:  # 리스트의 길이가 다 찼다면 리턴 
        print(' '.join(map(str,s)))
        return
    else:
        for i in range(1,n+1): # 1부터 n까지 순회 
            if visited[i]: #방문한 적이 있다면 
                continue
            else : #방문한 적이 없다면 
                s.append(i)
                visited[i] = True
                dfs()
                s.pop()
                # print(s)
                # print(visited)
                visited[i] = False

n, m = map(int,input().split())

s = []
visited = [False] * (n+1)


# 1부터 n까지의 자연수 중에서 중복 없이 m개를 고른 순열 

dfs()

# --> 이해했으니까 cpp로 구현해보기 