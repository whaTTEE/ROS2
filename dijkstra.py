import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import heapq



class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)  # /cmd_vel 퍼블리셔 생성
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_orientation = None
        self.get_logger().info("로봇 컨트롤러 노드가 시작되었습니다.")

    def odom_callback(self, msg):
        """
        오도메트리 데이터를 수신하여 로봇의 방향 정보를 추적
        """
        self.current_orientation = msg.pose.pose.orientation

    def stop(self):
        """
        로봇을 멈추게 함.
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.get_logger().info("로봇이 멈춥니다.")
        self.publisher.publish(twist)

    def move_straight(self, duration=1.0, speed=0.2):
        """
        로봇을 직진시킴.
        """
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0

        self.get_logger().info(f"로봇이 {duration}초 동안 직진합니다.")
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        current_time = start_time

        while current_time - start_time < duration:
            self.publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
            current_time = self.get_clock().now().seconds_nanoseconds()[0]

        self.stop()

    def rotate(self, target_angle):
        """
        오도메트리를 이용해 로봇을 목표 각도로 회전시킴.
        :param target_angle: 회전할 각도 (도 단위)
        """
        if self.current_orientation is None:
            self.get_logger().info("오도메트리 데이터가 없습니다.")
            return

        angular_speed = 0.5  # rad/s
        current_angle = self.quaternion_to_euler(self.current_orientation)
        target_angle_rad = math.radians(target_angle)

        # 목표 각도와 현재 각도의 차이 계산
        angle_diff = target_angle_rad - current_angle
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # -pi to pi 범위로 조정

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed if angle_diff > 0 else -angular_speed

        self.get_logger().info(f"로봇이 {math.degrees(angle_diff)}도 회전합니다.")

        while abs(angle_diff) > 0.1:  # 목표 각도에 충분히 근접할 때까지 회전
            self.publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
            current_angle = self.quaternion_to_euler(self.current_orientation)
            angle_diff = target_angle_rad - current_angle
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        self.stop()

    def quaternion_to_euler(self, orientation):
        """
        Quaternion을 Euler 각도로 변환
        """
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw  # yaw (z 축 회전)

    def move_along_path(self, path):
        """
        주어진 경로를 따라 이동.
        """
        self.get_logger().info(f"계산된 경로: {path}")
        for i in range(len(path) - 1):
            current = path[i]
            next_position = path[i + 1]

            # 방향 회전
            rotation_angle = self.calculate_rotation(current, next_position)
            self.rotate(rotation_angle)

            # 직진
            self.move_straight(duration=1.0)

    def calculate_rotation(self, current, next_position):
        """
        현재 위치와 다음 위치를 기반으로 회전 각도를 계산.
        """
        dx, dy = next_position[0] - current[0], next_position[1] - current[1]
        if dx > 0:  # 동쪽
            return 90
        elif dx < 0:  # 서쪽
            return -90
        elif dy > 0:  # 북쪽
            return 0
        elif dy < 0:  # 남쪽
            return 180
        return 0

    def dijkstra(self, grid, start, goal):
        """
        다익스트라 알고리즘을 사용하여 최단 경로를 찾습니다.
        :param grid: 맵 (2D 배열)
        :param start: 시작 위치 (x, y)
        :param goal: 목표 위치 (x, y)
        :return: 최단 경로 (목표 지점까지의 좌표 목록)
        """
        rows, cols = len(grid), len(grid[0])
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 동, 남, 서, 북
        queue = [(0, start)]  # (거리, 위치) 큐
        distances = {start: 0}  # 시작 지점의 거리는 0
        previous = {start: None}  # 각 지점의 이전 지점 (경로 추적용)

        while queue:
            current_distance, current = heapq.heappop(queue)

            # 목표 지점에 도달한 경우
            if current == goal:
                path = []
                while current is not None:
                    path.append(current)
                    current = previous[current]
                return path[::-1]  # 경로를 뒤집어서 반환

            # 인접한 지점들 검사
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor[0]][neighbor[1]] != 1:
                    new_distance = current_distance + 1
                    if neighbor not in distances or new_distance < distances[neighbor]:
                        distances[neighbor] = new_distance
                        heapq.heappush(queue, (new_distance, neighbor))
                        previous[neighbor] = current

        return []  # 목표 지점에 도달할 수 없으면 빈 경로 반환


def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()

    # 맵을 정의 (1은 장애물, 0은 통로)
    grid = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 0, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 0, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 0, 1, 1, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 1, 0, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0]
    ]

    start = (0, 0)  # 시작 지점
    goal = (8, 8)   # 목표 지점

    # 다익스트라 알고리즘을 사용하여 최단 경로 계산
    path = robot_controller.dijkstra(grid, start, goal)

    if path:
        robot_controller.get_logger().info(f"최단 경로: {path}")
        robot_controller.move_along_path(path)
    else:
        robot_controller.get_logger().info("목표 지점에 도달할 수 없습니다.")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
