import cv2
import matplotlib.pyplot as plt
import numpy as np
import heapq

# 이미지 불러오기
image = cv2.imread(r'C:\Users\s\Desktop\10.png')

if image is None:
    print("이미지를 불러올 수 없습니다.")
    exit()

# 그레이스케일로 변환
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 이진화
_, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

# 모폴로지 연산 (선 굵게)
kernel = np.ones((3, 3), np.uint8)
morphed = cv2.morphologyEx(thresh, cv2.MORPH_DILATE, kernel)

# 시작점과 목표점 설정 (y, x 형식)
start = (100, 200)  # (y, x)
goal = (600, 600)  # (y, x)

# 휴리스틱 함수 (맨해튼 거리)
def heuristic(a, b):
    return abs(b[0] - a[0]) + abs(b[1] - a[1])

# A* 알고리즘 수정 부분
def astar(array, start, goal):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        current = heapq.heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)
            return data[::-1], gscore  # 경로와 gscore 반환

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)

            if 0 <= neighbor[0] < array.shape[0] and 0 <= neighbor[1] < array.shape[1]:
                # 검정색(0)만 이동 가능
                if array[neighbor[0], neighbor[1]] != 0:  # 흰색(255)은 벽
                    continue
            else:
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if tentative_g_score < gscore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))

    return None, None  # 경로 없으면 None 반환

# A* 알고리즘 실행
path, gscore = astar(morphed, start, goal)

# 경로 시각화
output_image = cv2.cvtColor(morphed, cv2.COLOR_GRAY2BGR)  # 이진 이미지를 컬러로 변환
if path:
    for i in range(len(path) - 1):
        # 경로 그리기
        cv2.line(output_image, (path[i][1], path[i][0]), (path[i + 1][1], path[i + 1][0]), (0, 0, 255), 2)

        # 코스트 값 표시 (간격을 두고 표시)
        if i % 50 == 0:  # 경로의 50번째 점마다 코스트 값 표시 (간격을 더 크게)
            mid_point = ((path[i][1] + path[i + 1][1]) // 2, (path[i][0] + path[i + 1][0]) // 2)
            cost_value = gscore.get(path[i], 0)  # 해당 지점의 gscore 값을 가져옴
            cost_text = f"{cost_value}"  # gscore 값을 표시
            # 글씨가 빨간선에서 겹치지 않도록 좀 더 위쪽 또는 아래쪽으로 위치
            offset = (0, -15)  # 글씨를 경로에서 조금 떨어지게 하기 위해 -15만큼 y축으로 오프셋
            cv2.putText(output_image, cost_text, (mid_point[0] + offset[0], mid_point[1] + offset[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)  # 노란색으로 설정
else:
    print("경로를 찾을 수 없습니다.")

# 출발점과 목표점 표시
cv2.circle(output_image, (start[1], start[0]), 10, (255, 0, 0), -1)  # 출발점을 파란색 원으로 표시
cv2.circle(output_image, (goal[1], goal[0]), 10, (0, 255, 0), -1)  # 목표점을 초록색 원으로 표시

# 결과 출력
plt.figure(figsize=(8, 8))
output_rgb = cv2.cvtColor(output_image, cv2.COLOR_BGR2RGB)
plt.imshow(output_rgb)
plt.axis('off')
plt.show()
