import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class MapSubscriber(Node):
    def __init__(self):
        super().__init__('map_subscriber')
        # /map 토픽을 구독하여 맵 데이터를 받음
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.get_logger().info("맵 구독 노드가 시작되었습니다.")

    def map_callback(self, msg):
        """
        /map 토픽에서 받은 OccupancyGrid 메시지 처리
        """
        self.get_logger().info(f"맵 데이터의 크기: {msg.info.width}x{msg.info.height}")
        self.print_map(msg.data, msg.info.width, msg.info.height)

    def print_map(self, data, width, height):
        """
        맵 데이터를 2D 형태로 출력
        """
        for i in range(height):
            row = data[i * width:(i + 1) * width]  # 한 행을 잘라서 가져옴
            row_str = ''.join(['1' if cell == 100 else '0' for cell in row])  # 1은 장애물, 0은 비장애물
            self.get_logger().info(row_str)


def main(args=None):
    rclpy.init(args=args)

    map_subscriber = MapSubscriber()

    rclpy.spin(map_subscriber)

    map_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
