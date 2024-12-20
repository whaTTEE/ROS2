import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class DistanceSensor(Node):
    def __init__(self):
        super().__init__('distance_sensor')
        # '/scan' 토픽을 구독합니다. (LaserScan 메시지 사용)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # 일반적으로 라이다의 토픽 이름은 '/scan'입니다.
            self.scan_callback,
            10)
        self.subscription  # prevent unused variable warning

    def scan_callback(self, msg):
        # msg.ranges는 레이저 스캔 데이터에서 거리 값을 담고 있는 리스트입니다.
        # 전방 구간의 인덱스 범위 (예: 0도 ~ 30도 사이)
        # 0도를 기준으로 전방의 범위를 선택합니다.
        front_range = msg.ranges[0:30]  # 전방 범위 데이터 (0도에서 30도 사이)

        # 전방의 최소 거리를 출력 (물체와의 최소 거리)
        min_distance = min(front_range)
        
        # 물체와의 최소 거리가 너무 가까운 값이 나오지 않도록 필터링
        if min_distance == float('Inf'):
            min_distance = None
        
        if min_distance:
            self.get_logger().info(f'Front object distance: {min_distance} meters')
        else:
            self.get_logger().info('No object detected in front.')

def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화

    distance_sensor = DistanceSensor()  # DistanceSensor 객체 생성

    rclpy.spin(distance_sensor)  # 노드가 실행되도록 대기

    rclpy.shutdown()  # 종료 시 정리

if __name__ == '__main__':
    main()
