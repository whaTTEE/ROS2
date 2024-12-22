import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Obstacle Avoidance Node has started.")
        self.twist = Twist()

    def scan_callback(self, msg):
        """
        라이다 데이터를 처리하여 장애물을 회피합니다.
        """
        # 장애물 감지 범위 설정 (로봇 앞쪽 중심)
        min_distance_front = min(min(msg.ranges[0:10]), min(msg.ranges[-10:]))  # 정면
        min_distance_left = min(msg.ranges[30:90])  # 왼쪽
        min_distance_right = min(msg.ranges[-90:-30])  # 오른쪽

        self.get_logger().info(f"Front: {min_distance_front:.2f}, Left: {min_distance_left:.2f}, Right: {min_distance_right:.2f}")

        # 충돌 임계값 설정 (더 보수적으로 설정)
        obstacle_distance_threshold = 0.3  # 0.3m로 설정하여 장애물에 더 민감하게 반응
        side_obstacle_threshold = 0.5  # 측면 장애물 감지 임계값 (0.5m)

        # 장애물 회피 로직
        if min_distance_front < obstacle_distance_threshold:
            # 장애물이 정면에 있으면 회전
            self.twist.linear.x = 0.0
            # 장애물이 왼쪽보다 오른쪽에 더 가까우면 오른쪽으로 빠르게 회전
            if min_distance_left > min_distance_right:
                self.twist.angular.z = 2.0  # 왼쪽으로 빠르게 회전
            else:
                self.twist.angular.z = -2.0  # 오른쪽으로 빠르게 회전
        elif min_distance_left < side_obstacle_threshold and min_distance_right < side_obstacle_threshold:
            # 양 측면에 장애물이 가까워지면 벽에서 멀어지도록 회전
            self.twist.linear.x = 0.1  # 전진 속도를 낮추고
            if min_distance_left < min_distance_right:
                self.twist.angular.z = 0.5  # 오른쪽으로 회전하여 왼쪽 벽에서 멀어짐
            else:
                self.twist.angular.z = -0.5  # 왼쪽으로 회전하여 오른쪽 벽에서 멀어짐
        elif min_distance_left < side_obstacle_threshold:
            # 왼쪽 벽이 가까운 경우 오른쪽으로 회전하여 멀어지도록
            self.twist.linear.x = 0.1
            self.twist.angular.z = -0.5  # 오른쪽으로 회전
        elif min_distance_right < side_obstacle_threshold:
            # 오른쪽 벽이 가까운 경우 왼쪽으로 회전하여 멀어지도록
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0.5  # 왼쪽으로 회전
        else:
            # 장애물이 없으면 직진
            self.twist.linear.x = 0.1  # 전진 속도를 더 낮춤
            self.twist.angular.z = 0.0

        self.cmd_vel_publisher.publish(self.twist)

    def stop_robot(self):
        """
        로봇을 정지합니다.
        """
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping the robot...")
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
