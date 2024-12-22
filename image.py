import rclpy
from nav2_map_server.map_saver import MapSaver

def save_map():
    # ROS 2 초기화
    rclpy.init()

    # MapSaver 객체 생성
    map_saver = MapSaver()

    # 맵 저장 (파일 이름 및 경로 지정)
    map_saver.save_map("/path/to/map_1.pgm", "/path/to/map_1.yaml")

    # ROS 2 종료
    rclpy.shutdown()

if __name__ == '__main__':
    save_map()
