#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy
import time
import math

class SubsumptionExplorerNode(Node):
    def __init__(self):
        super().__init__('room_explorer_node')
        
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        
        self.publisher_ = self.create_publisher(Twist, '/rm0/cmd_vel', 10)

        qos_sensors = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_map = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.create_subscription(Range, '/rm0/range_0', self.cb0, qos_sensors)
        self.create_subscription(Range, '/rm0/range_1', self.cb1, qos_sensors)
        self.create_subscription(Range, '/rm0/range_2', self.cb2, qos_sensors)
        self.create_subscription(Range, '/rm0/range_3', self.cb3, qos_sensors)
        self.create_subscription(Odometry, '/rm0/odom_truth', self.odom_cb, qos_sensors)
        self.create_subscription(Path, '/plan', self.plan_cb, qos_sensors)

        self.distances = {0: None, 1: None, 2: None, 3: None}
        self.state = 'WAIT_SENSORS'
        self.start_time = time.time()
        
        self.global_path = []
        self.current_waypoint_idx = 0
        self.robot_x, self.robot_y, self.robot_yaw = 0.0, 0.0, 0.0

        # ── SOGLIE MOLTO CONSERVATIVE PER STABILITÀ ───────────────────────────
        self.STOP_DIST   = 0.4
        self.CLEAR_DIST  = 0.6
        self.TURN_SPEED  = 0.30  # Velocità raddoppiata per test dinamici (x2)
        self.FWD_SPEED   = 0.20  # Velocità raddoppiata per test dinamici (x2)
        self.WAYPOINT_TOLERANCE = 0.4 # Più tolleranza per i punti

        self.create_timer(0.1, self.run_logic)
        self.get_logger().info('--- IBRIDO NAV2 + TOF: STABILIZZAZIONE ATTIVA ---')

    def cb0(self, msg): self.distances[0] = msg.range
    def cb1(self, msg): self.distances[1] = msg.range
    def cb2(self, msg): self.distances[2] = msg.range
    def cb3(self, msg): self.distances[3] = msg.range

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def plan_cb(self, msg):
        self.global_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.current_waypoint_idx = 0
        if self.global_path:
            self.get_logger().info(f'[PLAN] Ricevuto percorso di {len(self.global_path)} punti.')

    def get_path_steering(self):
        if not self.global_path or self.current_waypoint_idx >= len(self.global_path):
            return 0.0, False
        
        tx, ty = self.global_path[self.current_waypoint_idx]
        dist = math.hypot(tx - self.robot_x, ty - self.robot_y)
        
        # Se siamo vicini al punto attuale, saltiamo avanti di 5 punti per fluidità
        if dist < self.WAYPOINT_TOLERANCE:
            self.current_waypoint_idx = min(len(self.global_path)-1, self.current_waypoint_idx + 5)
            tx, ty = self.global_path[self.current_waypoint_idx]

        target_angle = math.atan2(ty - self.robot_y, tx - self.robot_x)
        diff = target_angle - self.robot_yaw
        while diff > math.pi: diff -= 2*math.pi
        while diff < -math.pi: diff += 2*math.pi
        
        # Log di debug ogni 2 secondi
        self.get_logger().info(f'Target: ({tx:.2f}, {ty:.2f}) Dist: {dist:.2f}m Diff: {diff:.2f}rad', throttle_duration_sec=2.0)
        
        # Controllo Proporzionale ridotto (guadagno 0.5)
        steer = max(-0.4, min(0.4, diff * 0.5))
        return steer, True

    def run_logic(self):
        if self.state == 'WAIT_SENSORS':
            received = [i for i, v in self.distances.items() if v is not None]
            if len(received) > 0 or (time.time() - self.start_time > 5.0):
                for i in range(4):
                    if self.distances[i] is None: self.distances[i] = 10.0
                self.state = 'IDLE'
                return
            return

        msg = Twist()
        d_front = self.distances[2] if self.distances[2] is not None else 10.0

        if self.state == 'IDLE':
            if self.global_path:
                self.state = 'FOLLOW_PATH'
            else:
                self.state = 'WANDER'

        if self.state == 'FOLLOW_PATH':
            if d_front <= self.STOP_DIST:
                self.state = 'TURN'
                d_right = self.distances[1] if self.distances[1] is not None else 10.0
                d_left  = self.distances[3] if self.distances[3] is not None else 10.0
                self.turn_dir = -self.TURN_SPEED if d_right >= d_left else self.TURN_SPEED
                self.turn_start = time.time()
                self.get_logger().warn('OSTACOLO! Entro in modalità evitamento.')
            else:
                steer, has_path = self.get_path_steering()
                if has_path:
                    msg.linear.x = self.FWD_SPEED
                    msg.angular.z = steer
                else:
                    self.state = 'IDLE'
                    self.get_logger().info('Percorso completato.')

        elif self.state == 'WANDER':
            # Recuperiamo le distanze dei 3 sensori (destra, frontale, sinistra)
            d_right = self.distances[1] if self.distances[1] is not None else 10.0
            d_left  = self.distances[3] if self.distances[3] is not None else 10.0

            # Ignoriamo valori inferiori a 0.05 (rumore o letture non valide)
            if d_right < 0.05: d_right = 10.0
            if d_front < 0.05: d_front = 10.0
            if d_left < 0.05: d_left = 10.0

            obs_thresh = 0.5  # Soglia ostacolo a 50cm

            if d_front <= obs_thresh or d_right <= obs_thresh or d_left <= obs_thresh:
                # Ostacolo rilevato! Giriamo sul posto verso la direzione con piu' spazio
                if d_right >= d_left:
                    msg.angular.z = -self.TURN_SPEED  # Gira a destra
                else:
                    msg.angular.z = self.TURN_SPEED   # Gira a sinistra
                msg.linear.x = 0.0
                self.get_logger().info(f'Ostacolo rilevato (F:{d_front:.2f}, R:{d_right:.2f}, L:{d_left:.2f}) -> Evito ostacolo', throttle_duration_sec=1.0)
            else:
                # Via libera, andiamo dritto!
                msg.linear.x = self.FWD_SPEED
                msg.angular.z = 0.02 * math.sin(time.time() * 0.5) # Piccola oscillazione per esplorare meglio

        elif self.state == 'TURN':
            if d_front >= self.CLEAR_DIST:
                self.get_logger().info('Via libera, riprendo il cammino.')
                self.state = 'FOLLOW_PATH' if self.global_path else 'WANDER'
            else:
                msg.angular.z = self.turn_dir
                if time.time() - self.turn_start > 4.0:
                    self.state = 'FOLLOW_PATH' if self.global_path else 'WANDER'

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
