#!/usr/bin/env python3
import time
import os
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geographic_msgs.msg import GeoPoint
from auspex_msgs.msg import SearchMission, Area, UserCommand

class MissionPub(Node):
    def __init__(self, start_lat, start_lon, target_lat, target_lon):
        super().__init__('search_mission_mock_publisher')
        self.pub = self.create_publisher(SearchMission, 'search_mission', 10)
        self.command_pub = self.create_publisher(UserCommand, 'planner_command', 10)
        self.timer = self.create_timer(1.0, self.publish_mission)  # 1 Hz

        self.start_lat = start_lat
        self.start_lon = start_lon
        self.target_lat = target_lat
        self.target_lon = target_lon

    def publish_mission(self):
        msg = SearchMission()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = '0'

        msg.team_id = 'drone_team'
        msg.mission_status = 'UNPROCESSED'
        msg.platform_class.value = 0

        search_area = Area()
        search_area.type = 1
        search_area.description = 'drone_search_area'
        size_lon = 0.0008
        size_lat = size_lon * 2/3
        search_area.points = [
            GeoPoint(latitude=self.target_lat - size_lat, longitude=self.target_lon - size_lon, altitude=0.0),
            GeoPoint(latitude=self.target_lat + size_lat, longitude=self.target_lon - size_lon, altitude=0.0),
            GeoPoint(latitude=self.target_lat + size_lat, longitude=self.target_lon + size_lon, altitude=0.0),
            GeoPoint(latitude=self.target_lat - size_lat, longitude=self.target_lon + size_lon, altitude=0.0)
        ]
        msg.search_areas = [search_area]

        msg.max_height = 150
        msg.min_height = 15
        msg.desired_ground_dist = 15
        msg.starting_point = GeoPoint(latitude=self.start_lat, longitude=self.start_lon, altitude=0.0)

        for _ in range(3):
            self.pub.publish(msg)
            self.get_logger().info('Mission published.')

        start_msg = UserCommand()
        start_msg.user_command = 4  # USER_START_TEAM
        start_msg.team_id = 'drone_team'
        start_msg.platform_id = ''
        start_msg.planner_id = ''
        start_msg.planner_specs = ''

        for _ in range(3):
            self.command_pub.publish(start_msg)
            self.get_logger().info('Start command published.')

        self.timer.cancel()

def main(argv=None):
    rclpy.init()
    while True:
        try:
            with open("need_help.txt", "r") as f:
                print("Got help signal!")
                parts = f.readline().split(';')
                start_lat = float(parts[0])
                start_lon = float(parts[1])
                target_lat = float(parts[2])
                target_lon = float(parts[3])

            os.remove("need_help.txt")

            node = MissionPub(start_lat, start_lon, target_lat, target_lon)
            rclpy.spin(node)
            node.destroy_node()
            rclpy.shutdown()
            break

        except FileNotFoundError:
            print("Listening...")
            time.sleep(1)
            continue

if __name__ == '__main__':
    main()
