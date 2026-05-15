#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy
import time
import math

class SubsumptionExplorerNode(Node):
    def __init__(self):
        super().__init__('room_explorer_node')
        self.publisher_ = self.create_publisher(Twist, '/rm0/cmd_vel', 10)

        qos_sensors = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_map = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # Mapping sensori (robot fisico, da Task 4):
        # range_0 → posteriore
        # range_1 → laterale destro
        # range_2 → frontale principale
        # range_3 → laterale sinistro
        self.create_subscription(Range, '/rm0/range_0', self.cb0, qos_sensors)
        self.create_subscription(Range, '/rm0/range_1', self.cb1, qos_sensors)
        self.create_subscription(Range, '/rm0/range_2', self.cb2, qos_sensors)
        self.create_subscription(Range, '/rm0/range_3', self.cb3, qos_sensors)

        self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos_map)
        self.create_subscription(Odometry, '/rm0/odom_truth', self.odom_cb, qos_sensors)

        self.distances = {0: None, 1: None, 2: None, 3: None}
        self.state = 'WAIT_SENSORS'
        self.turn_start_time    = 0.0
        self.turn_direction     = -1.0
        self.reverse_start_time = 0.0

        self.map_msg  = None
        self.robot_x  = 0.0
        self.robot_y  = 0.0
        self.robot_yaw = 0.0

        # ── Soglie ────────────────────────────────────────────────────────────
        self.STOP_DIST   = 0.5   # inizia a girare presto
        self.CLEAR_DIST  = 1.0   # esce da TURN solo quando fronte è davvero libero
        self.STEER_DIST  = 0.45
        self.BACK_SAFE   = 0.25
        self.TURN_SPEED  = 0.1
        self.FWD_SPEED   = 0.1
        self.REV_SPEED   = 0.10
        self.REV_DURATION = 1.5
        self.TIMEOUT_360 = (2 * math.pi / self.TURN_SPEED) + 0.5
        self.FRONTIER_ATTRACTION = 0.4

        self.create_timer(0.1, self.run_logic)
        self.get_logger().info('--- SUBSUMPTION EXPLORER: ATTIVO ---')

        self.SETTLE_TIME = 1.5   # secondi di stop per consolidare SLAM


    # ── Callback sensori ──────────────────────────────────────────────────────
    def cb0(self, msg): self.distances[0] = msg.range  # posteriore
    def cb1(self, msg): self.distances[1] = msg.range  # destra
    def cb2(self, msg): self.distances[2] = msg.range  # frontale
    def cb3(self, msg): self.distances[3] = msg.range  # sinistra

    # ── Callback odometria ────────────────────────────────────────────────────
    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    # ── Callback mappa ────────────────────────────────────────────────────────
    def map_cb(self, msg):
        self.map_msg = msg


    #__________________ Debug _____________________________
    def log_sensors(self):
        def fmt(v): return f'{v:.3f}' if v is not None else '---'
        self.get_logger().info(
        f'[RAW] range_0={fmt(self.distances[0])}  '
        f'range_1={fmt(self.distances[1])}  '
        f'range_2={fmt(self.distances[2])}  '
        f'range_3={fmt(self.distances[3])}'
    )


    
    # ── LIVELLO 1: Frontier exploration dalla mappa SLAM ─────────────────────
    def get_frontier_steering(self):
        """
        Cerca la cella inesplorata (-1) più vicina nella mappa SLAM
        e calcola la sterzata necessaria per puntarci.
        Priorità più bassa — attivo solo quando i sensori non rilevano ostacoli.
        """
        if not self.map_msg:
            return 0.0

        res      = self.map_msg.info.resolution
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        width    = self.map_msg.info.width
        data     = self.map_msg.data

        robot_col = int((self.robot_x - origin_x) / res)
        robot_row = int((self.robot_y - origin_y) / res)

        search_radius = 50
        step = 3
        closest_x = None
        closest_y = None
        min_dist  = float('inf')

        for r in range(max(0, robot_row - search_radius),
                       min(robot_row + search_radius, self.map_msg.info.height), step):
            for c in range(max(0, robot_col - search_radius),
                           min(robot_col + search_radius, width), step):
                if data[r * width + c] == -1:
                    dist = math.hypot(c - robot_col, r - robot_row)
                    if 5 < dist < min_dist:
                        min_dist  = dist
                        closest_x = origin_x + c * res
                        closest_y = origin_y + r * res

        if closest_x is not None:
            target_angle = math.atan2(closest_y - self.robot_y, closest_x - self.robot_x)
            angle_diff   = target_angle - self.robot_yaw
            # Normalizza tra -π e π
            while angle_diff >  math.pi: angle_diff -= 2 * math.pi
            while angle_diff < -math.pi: angle_diff += 2 * math.pi
            steer = max(-self.FRONTIER_ATTRACTION,
                        min( self.FRONTIER_ATTRACTION, angle_diff * 0.8))
            self.get_logger().info(
                f'[FRONTIER] target=({closest_x:.1f},{closest_y:.1f}) '
                f'steer={steer:.2f}',
                throttle_duration_sec=3.0)
            return steer

        self.get_logger().info('[FRONTIER] Nessuna zona inesplorata trovata',
                               throttle_duration_sec=5.0)
        return 0.0

    # ── MOTORE PRINCIPALE ─────────────────────────────────────────────────────
    def run_logic(self):
        msg = Twist()

        # WAIT_SENSORS
        if self.state == 'WAIT_SENSORS':
            if all(d is not None for d in self.distances.values()):
                self.state = 'FORWARD'
                self.get_logger().info('Sensori OK → FORWARD')
            else:
                self.get_logger().info('Attesa sensori...', throttle_duration_sec=2.0)
            return

        d_front = self.distances[0]
        d_right = self.distances[1]
        d_left  = self.distances[3]
        d_rear  = self.distances[2]

        # ── FORWARD ───────────────────────────────────────────────────────────
        if self.state == 'FORWARD':

            # LIVELLO 3: ostacolo frontale → TURN
            if d_front < self.STOP_DIST:
                if d_right >= d_left:
                    self.turn_direction = -self.TURN_SPEED
                    verso = 'DESTRA'
                else:
                    self.turn_direction = +self.TURN_SPEED
                    verso = 'SINISTRA'
                self.get_logger().info(
                    f'STOP! front={d_front:.2f}m '
                    f'dx={d_right:.2f}m sx={d_left:.2f}m → {verso}')
                self.turn_start_time = time.time()
                self.state = 'TURN'

            else:
                # Velocità proporzionale alla distanza dall'ostacolo
                speed_factor = min(1.0, (d_front - self.STOP_DIST) / 0.5)
                msg.linear.x = self.FWD_SPEED * max(0.4, speed_factor)

                # LIVELLO 2: evitamento laterale (sensori — priorità alta)
                if d_right < self.STEER_DIST and d_left >= self.STEER_DIST:
                    msg.angular.z = +0.5   # ostacolo a destra → sterza sinistra
                elif d_left < self.STEER_DIST and d_right >= self.STEER_DIST:
                    msg.angular.z = -0.5   # ostacolo a sinistra → sterza destra

                # LIVELLO 1: frontier exploration dalla mappa (priorità bassa)
                else:
                    msg.angular.z = 0.0

        # ── TURN ──────────────────────────────────────────────────────────────
        elif self.state == 'TURN':
            elapsed = time.time() - self.turn_start_time

            if d_front >= self.CLEAR_DIST:
                self.get_logger().info(
                    f'Via libera! front={d_front:.2f}m dopo {elapsed:.1f}s → FORWARD')
                # ── STOP per consolidare SLAM ──
                stop_msg = Twist()   # tutti i campi a 0
                self.publisher_.publish(stop_msg)
                time.sleep(self.SETTLE_TIME)    
                self.state = 'FORWARD'
                return       # esci subito, non pubblicare altro questo ciclo
            elif elapsed > self.TIMEOUT_360:
                if d_rear >= self.BACK_SAFE:
                    self.get_logger().info('Vicolo cieco → REVERSE')
                    self.reverse_start_time = time.time()
                    self.state = 'REVERSE'
                else:
                    self.get_logger().info('Vicolo cieco + muro dietro → inverto rotazione')
                    self.turn_direction = -self.turn_direction
                    self.turn_start_time = time.time()
            else:
                msg.angular.z = self.turn_direction

        # ── REVERSE ───────────────────────────────────────────────────────────
        elif self.state == 'REVERSE':
            elapsed = time.time() - self.reverse_start_time
            if elapsed < self.REV_DURATION and d_rear >= self.BACK_SAFE:
                msg.linear.x = -self.REV_SPEED
            else:
                self.get_logger().info('Retromarcia OK → riprovo TURN')
                self.turn_direction = -self.turn_direction
                self.turn_start_time = time.time()
                self.state = 'TURN'

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SubsumptionExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher_.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()