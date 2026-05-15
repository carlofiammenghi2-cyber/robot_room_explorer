#!/usr/bin/env python3
import math
import zmq
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


ODOM_FRAME = 'rm0/odom_truth'
BASE_FRAME = 'rm0/base_link_truth'
LIDAR_FRAME = 'rm0/lidar_link'
ODOM_TOPIC = f'/{ODOM_FRAME}'
IMU_TOPIC = '/rm0/imu'
ROBOT_PATH = '/rm0'
LIDAR_PATH = '/rm0/fastHokuyo_ROS'
YAW_OFFSET = math.pi
LIDAR_YAW_OFFSET = 0.0
CMD_VEL_TOPIC = '/rm0/cmd_vel'
CMD_YAW_RATE_SCALE = 1.0
RAW_YAW_RATE_THRESHOLD = 0.02


def normalize_quaternion(quat):
    norm = math.sqrt(sum(value * value for value in quat))
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in quat)


def quaternion_to_matrix(quat):
    x, y, z, w = normalize_quaternion(quat)
    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def coppelia_robot_vector_to_ros(vec):
    # In this scene the robot object's local +Z is forward and +X is up.
    return (vec[2], -vec[1], vec[0])


class CoppeliasimOdom(Node):
    def __init__(self):
        super().__init__('coppeliasim_odom')
        self.odom_frame = self.declare_parameter('odom_frame', ODOM_FRAME).value
        self.base_frame = self.declare_parameter('base_frame', BASE_FRAME).value
        self.lidar_frame = self.declare_parameter('lidar_frame', LIDAR_FRAME).value
        self.yaw_offset = float(self.declare_parameter('yaw_offset', YAW_OFFSET).value)
        self.lidar_yaw_offset = float(
            self.declare_parameter('lidar_yaw_offset', LIDAR_YAW_OFFSET).value)
        self.cmd_yaw_rate_scale = float(
            self.declare_parameter('cmd_yaw_rate_scale', CMD_YAW_RATE_SCALE).value)
        self.raw_yaw_rate_threshold = float(
            self.declare_parameter(
                'raw_yaw_rate_threshold', RAW_YAW_RATE_THRESHOLD).value)
        odom_topic = self.declare_parameter('odom_topic', ODOM_TOPIC).value
        imu_topic = self.declare_parameter('imu_topic', IMU_TOPIC).value
        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', CMD_VEL_TOPIC).value
        robot_path = self.declare_parameter('robot_path', ROBOT_PATH).value
        lidar_path = self.declare_parameter('lidar_path', LIDAR_PATH).value

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.imu_pub = self.create_publisher(Imu, imu_topic, 10)
        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_cb, 10)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.tf_bcast = TransformBroadcaster(self)
        self.static_tf_bcast = StaticTransformBroadcaster(self)
        self.cmd_angular_z = 0.0

        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.client.socket.setsockopt(zmq.RCVTIMEO, 500)
    
        try:
            self.robot_handle = self.sim.getObject(robot_path)
            self.lidar_handle = self.sim.getObject(lidar_path)
            self.get_logger().info('Hardware CoppeliaSim collegato!')
            
            # Calcola e pubblica Offset Lidar
            pos_l = coppelia_robot_vector_to_ros(
                self.sim.getObjectPosition(self.lidar_handle, self.robot_handle))
            self.publish_static_lidar_tf(
                pos_l, quaternion_from_yaw(self.lidar_yaw_offset))
            
        except Exception as e:
            self.get_logger().error(f'Errore Hardware: {e}')
            raise

        self.last_sim_time = self.sim.getSimulationTime()
        self.last_pos = self.sim.getObjectPosition(self.robot_handle, -1)
        init_quat = normalize_quaternion(self.sim.getObjectQuaternion(self.robot_handle, -1))
        self.last_raw_yaw = self.yaw_from_quaternion(init_quat) + self.yaw_offset
        self.current_yaw = self.last_raw_yaw

        self.create_timer(0.05, self.publish_data)
        self.get_logger().info('Odometria CoppeliaSim attiva. SLAM pronto.')

    def cmd_vel_cb(self, msg):
        self.cmd_angular_z = msg.angular.z * self.cmd_yaw_rate_scale

    @staticmethod
    def yaw_from_quaternion(quat):
        rotation = quaternion_to_matrix(quat)
        forward_x = rotation[0][2]
        forward_y = rotation[1][2]
        return math.atan2(forward_y, forward_x)

    @staticmethod
    def shortest_angle_delta(current, previous):
        delta = current - previous
        while delta > math.pi:
            delta -= 2.0 * math.pi
        while delta < -math.pi:
            delta += 2.0 * math.pi
        return delta

    def publish_static_lidar_tf(self, pos, quat):
        t = TransformStamped()
        t.header.stamp = Time().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.lidar_frame
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = pos[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.static_tf_bcast.sendTransform(t)

    def publish_data(self):
        try:
            pos = self.sim.getObjectPosition(self.robot_handle, -1)
            sim_time = self.sim.getSimulationTime()
            quat = normalize_quaternion(self.sim.getObjectQuaternion(self.robot_handle, -1))
        except Exception as e:
            self.get_logger().warn(f'ZMQ error: {e}')
            return

        dt = sim_time - self.last_sim_time
        raw_yaw = self.yaw_from_quaternion(quat) + self.yaw_offset
        raw_yaw_delta = self.shortest_angle_delta(raw_yaw, self.last_raw_yaw)
        raw_yaw_rate = raw_yaw_delta / dt if dt > 1e-6 else 0.0
        if dt > 1e-6 and abs(raw_yaw_rate) > self.raw_yaw_rate_threshold:
            yaw = raw_yaw
        else:
            yaw = self.current_yaw + self.cmd_angular_z * dt
        ros_quat = quaternion_from_yaw(yaw)
        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0
        if dt > 1e-6:
            world_vx = (pos[0] - self.last_pos[0]) / dt
            world_vy = (pos[1] - self.last_pos[1]) / dt
            linear_x = math.cos(yaw) * world_vx + math.sin(yaw) * world_vy
            linear_y = -math.sin(yaw) * world_vx + math.cos(yaw) * world_vy
            angular_z = self.shortest_angle_delta(yaw, self.current_yaw) / dt

        self.last_sim_time = sim_time
        self.last_pos = pos
        self.last_raw_yaw = raw_yaw
        self.current_yaw = yaw

        now_msg = rclpy.time.Time(seconds=sim_time).to_msg()

        # Pubblica CLOCK
        clock_msg = Clock()
        clock_msg.clock = now_msg
        self.clock_pub.publish(clock_msg)

        # IMU
        imu = Imu()
        imu.header.stamp = now_msg
        imu.header.frame_id = self.base_frame
        imu.angular_velocity.z = angular_z
        imu.orientation.x = ros_quat[0]
        imu.orientation.y = ros_quat[1]
        imu.orientation.z = ros_quat[2]
        imu.orientation.w = ros_quat[3]
        self.imu_pub.publish(imu)

        # Odometry
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.position.z = pos[2]
        odom.pose.pose.orientation.x = ros_quat[0]
        odom.pose.pose.orientation.y = ros_quat[1]
        odom.pose.pose.orientation.z = ros_quat[2]
        odom.pose.pose.orientation.w = ros_quat[3]
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.linear.y = linear_y
        odom.twist.twist.angular.z = angular_z
        self.odom_pub.publish(odom)

        # TF
        tf = TransformStamped()
        tf.header.stamp = now_msg
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = pos[0]
        tf.transform.translation.y = pos[1]
        tf.transform.translation.z = pos[2]
        tf.transform.rotation.x = ros_quat[0]
        tf.transform.rotation.y = ros_quat[1]
        tf.transform.rotation.z = ros_quat[2]
        tf.transform.rotation.w = ros_quat[3]
        self.tf_bcast.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CoppeliasimOdom())


if __name__ == '__main__':
    main()
