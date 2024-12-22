import numpy as np
import matplotlib.pyplot as plt
import heapq

# 맵 크기 설정
GRID_SIZE = 50
START = (0, 0)  # 시작점
GOAL = (49, 49)  # 목표점

# 장애물 위치 설정 (1은 장애물, 0은 자유 공간)
grid = np.zeros((GRID_SIZE, GRID_SIZE))

# 복잡한 미로 만들기 (여기서는 임의로 장애물을 설정)
np.random.seed(42)
grid[1:5, 1:30] = 1
grid[10:40, 15:20] = 1
grid[30:35, 30:50] = 1
grid[5:10, 30:40] = 1
grid[40:50, 5:25] = 1

# A* 알고리즘에서 사용할 함수들
def heuristic(a, b):
    # 맨하탄 거리 (Heuristic)
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(grid, start, goal):
    # A* 알고리즘 구현
    open_list = []
    heapq.heappush(open_list, (0 + heuristic(start, goal), 0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    while open_list:
        _, current_g, current = heapq.heappop(open_list)
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path
        
        neighbors = [(current[0]+1, current[1]), (current[0]-1, current[1]),
                    (current[0], current[1]+1), (current[0], current[1]-1)]
        
        for neighbor in neighbors:
            if 0 <= neighbor[0] < GRID_SIZE and 0 <= neighbor[1] < GRID_SIZE:
                if grid[neighbor[0], neighbor[1]] == 1:  # 장애물인 경우
                    continue
                
                tentative_g_score = current_g + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score[neighbor], tentative_g_score, neighbor))
    
    return None  # 경로를 찾지 못한 경우

# A* 알고리즘 실행
path = a_star(grid, START, GOAL)

# 경로 시각화
def plot_grid(grid, path):
    plt.imshow(grid, cmap='Greys', origin='lower')
    if path:
        path_x, path_y = zip(*path)
        plt.plot(path_y, path_x, color='red', linewidth=2, marker='o', markersize=5)
    
    plt.scatter(START[1], START[0], color='green', marker='*', s=100, label='Start')
    plt.scatter(GOAL[1], GOAL[0], color='blue', marker='*', s=100, label='Goal')
    plt.legend()
    plt.title("A* Algorithm Path in Complex Maze")
    plt.show()

plot_grid(grid, path)
