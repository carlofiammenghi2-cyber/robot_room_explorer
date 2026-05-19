#!/usr/bin/env python3
import math
import zmq
import rclpy
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

ANGLE_MIN_DEG = -120.0
ANGLE_MAX_DEG =  120.0

class LidarBridge(Node):
    def __init__(self):
        super().__init__('lidar_bridge')
        self.scan_topic = self.declare_parameter('scan_topic', '/rm0/scan').value
        self.lidar_frame = self.declare_parameter('lidar_frame', 'rm0/lidar_link').value
        self.pub = self.create_publisher(LaserScan, self.scan_topic, 10)

        self.client = RemoteAPIClient()
        self.sim    = self.client.getObject('sim')
        self.client.socket.setsockopt(zmq.RCVTIMEO, 500)

        self._no_data  = 0
        self.create_timer(0.05, self.poll_lidar)
        self.get_logger().info('LidarBridge avviato e sbloccato!')

    def poll_lidar(self):
        try:
            data_str = self.sim.getStringSignal('lidar_data')
            sim_time = self.sim.getSimulationTime()
        except Exception as e:
            self.get_logger().warn(f'RemoteAPI error: {e}')
            return

        if not data_str:
            self._no_data += 1
            if self._no_data % 40 == 1:
                self.get_logger().warn('Nessun segnale lidar_data da CoppeliaSim...')
            return

        self._no_data = 0

        try:
            values = [float(x) for x in data_str.split(',')]
        except Exception as e:
            self.get_logger().warn(f'Parse error: {e}')
            return

        n = len(values)
        if n < 2:
            return

        ranges = list(values)
        ranges.reverse() # Inverte l'array per correggere eventuali scan specchiati

        angle_min       = math.radians(ANGLE_MIN_DEG)
        angle_max       = math.radians(ANGLE_MAX_DEG)
        angle_increment = (angle_max - angle_min) / (n - 1)

        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.lidar_frame
        msg.angle_min       = angle_min
        msg.angle_max       = angle_max
        msg.angle_increment = angle_increment
        msg.range_min       = 0.05
        msg.range_max       = 8.0
        msg.scan_time       = 0.05
        msg.time_increment  = 0.0
        msg.ranges          = ranges

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LidarBridge())

if __name__ == '__main__':
    main()
