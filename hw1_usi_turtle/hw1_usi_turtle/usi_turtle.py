"""
Angry Turtle Controller Node
=============================
Controls turtle1 in turtlesim to:
  1. Draw "USI" on the canvas using waypoints
  2. Chase any intruder (turtle2) that enters within k_chase distance
  3. Kill the intruder (within k_kill) and respawn it at a random position
  4. Use predictive interception (aim ahead of the moving target)

State machine: WRITING -> ANGRY -> RETURNING -> WRITING (loop)
"""

import rclpy
from rclpy.node import Node
import math
import random

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn, SetPen


class AngryTurtleNode(Node):

    def __init__(self):
        super().__init__('angry_turtle')

        # --------------- ROS Parameters ---------------
        self.declare_parameter('k_chase', 3.5)        # distance to trigger chase
        self.declare_parameter('k_kill', 0.5)          # distance to kill offender
        self.declare_parameter('chase_speed', 4.0)     # linear speed while chasing
        self.declare_parameter('write_speed', 2.0)     # linear speed while writing
        self.declare_parameter('intercept_gain', 0.8)  # lookahead gain for Task 4

        # --------------- Internal State ---------------
        self.pose = None              # turtle1 current pose
        self.offender_pose = None     # turtle2 current pose
        self.offender_alive = False
        self.state = 'WRITING'        # WRITING | ANGRY | RETURNING

        # Pen state tracking (avoids spamming the service)
        self._pen_key = None

        # Waypoints that spell "USI" on the turtlesim canvas
        self.waypoints = self._build_waypoints()
        self.current_wp = 0           # index into waypoints
        self.saved_wp = 0             # waypoint to resume after chase

        # --------------- ROS Interfaces ---------------
        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Subscribers
        self.create_subscription(Pose, '/turtle1/pose', self._pose_cb, 10)
        self.create_subscription(Pose, '/turtle2/pose', self._offender_cb, 10)

        # Service clients
        self.cli_kill = self.create_client(Kill, '/kill')
        self.cli_spawn = self.create_client(Spawn, '/spawn')
        self.cli_pen = self.create_client(SetPen, '/turtle1/set_pen')

        # Wait for turtlesim to be ready, then spawn turtle2
        self._spawn_wait_timer = self.create_timer(1.0, self._wait_and_spawn)

        # Main control loop at 20 Hz
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info('AngryTurtle node started -- waiting for turtlesim...')

    # ================================================================
    #                        WAYPOINTS
    # ================================================================

    @staticmethod
    def _build_waypoints():
        """
        Define the sequence of (x, y, pen_down) waypoints to draw "USI".
        pen_down=False means the turtle lifts the pen (moves without drawing).
        """
        pts = []

        # ---- Letter U ----
        pts.append({'x': 2.0, 'y': 8.0, 'pen': False})   # move to top-left
        pts.append({'x': 2.0, 'y': 6.0, 'pen': True})    # draw down
        pts.append({'x': 3.0, 'y': 6.0, 'pen': True})    # draw bottom-right
        pts.append({'x': 3.0, 'y': 8.0, 'pen': True})    # draw up

        # ---- Letter S ----
        pts.append({'x': 5.0, 'y': 8.0, 'pen': False})   # move to top-right of S
        pts.append({'x': 4.0, 'y': 8.0, 'pen': True})    # top bar left
        pts.append({'x': 4.0, 'y': 7.0, 'pen': True})    # middle down
        pts.append({'x': 5.0, 'y': 7.0, 'pen': True})    # middle bar right
        pts.append({'x': 5.0, 'y': 6.0, 'pen': True})    # bottom down
        pts.append({'x': 4.0, 'y': 6.0, 'pen': True})    # bottom bar left

        # ---- Letter I ----
        pts.append({'x': 6.5, 'y': 8.0, 'pen': False})   # move to top
        pts.append({'x': 6.5, 'y': 6.0, 'pen': True})    # draw line down

        return pts

    # ================================================================
    #                      TOPIC CALLBACKS
    # ================================================================

    def _pose_cb(self, msg: Pose):
        self.pose = msg

    def _offender_cb(self, msg: Pose):
        self.offender_pose = msg
        # If we thought the offender was dead but we're getting poses,
        # it means a respawn happened.
        if not self.offender_alive:
            self.offender_alive = True

    # ================================================================
    #                     SERVICE HELPERS
    # ================================================================

    def _set_pen(self, down: bool, color: str = 'white'):
        """Change pen state only when needed (avoids flooding the service)."""
        key = (down, color)
        if self._pen_key == key:
            return
        self._pen_key = key

        req = SetPen.Request()
        if down:
            if color == 'red':
                req.r, req.g, req.b = 255, 50, 50
            else:
                req.r, req.g, req.b = 255, 255, 255
            req.width = 3
            req.off = 0
        else:
            req.off = 1
        self.cli_pen.call_async(req)

    def _wait_and_spawn(self):
        """Called on a timer: spawn turtle2 once the /spawn service is ready."""
        if self.cli_spawn.service_is_ready():
            self._spawn_wait_timer.cancel()
            self._spawn_offender()
        else:
            self.get_logger().info('Waiting for /spawn service...')

    def _spawn_offender(self):
        """Spawn turtle2 at a random location inside the canvas."""
        req = Spawn.Request()
        req.x = random.uniform(1.0, 10.0)
        req.y = random.uniform(1.0, 10.0)
        req.theta = random.uniform(0.0, 2.0 * math.pi)
        req.name = 'turtle2'
        future = self.cli_spawn.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'turtle2 spawned at ({req.x:.1f}, {req.y:.1f})'))

    def _kill_offender(self):
        """Kill turtle2 and schedule a respawn."""
        self.offender_alive = False
        self.offender_pose = None

        req = Kill.Request()
        req.name = 'turtle2'
        future = self.cli_kill.call_async(req)
        future.add_done_callback(self._on_kill_done)

    def _on_kill_done(self, future):
        try:
            future.result()
            self.get_logger().info('turtle2 eliminated! Respawning in 2 s...')
            # One-shot timer to respawn after a short delay
            self._respawn_timer = self.create_timer(2.0, self._do_respawn)
        except Exception as e:
            self.get_logger().error(f'Kill failed: {e}')

    def _do_respawn(self):
        """One-shot callback: cancel the timer and spawn a new offender."""
        self.destroy_timer(self._respawn_timer)
        self._spawn_offender()

    # ================================================================
    #                      NAVIGATION
    # ================================================================

    @staticmethod
    def _dist(x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def _go_to(self, tx, ty, max_speed):
        """
        Compute a Twist to steer turtle1 toward (tx, ty).
        Returns (Twist, remaining_distance).
        """
        vel = Twist()
        dist = self._dist(self.pose.x, self.pose.y, tx, ty)
        desired_angle = math.atan2(ty - self.pose.y, tx - self.pose.x)
        angle_err = math.atan2(
            math.sin(desired_angle - self.pose.theta),
            math.cos(desired_angle - self.pose.theta))

        vel.angular.z = 6.0 * angle_err
        vel.linear.x = min(max_speed, 1.5 * dist)
        return vel, dist

    # ================================================================
    #                      CONTROL LOOP
    # ================================================================

    def _control_loop(self):
        if self.pose is None:
            return

        # Read parameters (allows dynamic reconfiguration)
        k_chase = self.get_parameter('k_chase').value
        k_kill = self.get_parameter('k_kill').value
        chase_speed = self.get_parameter('chase_speed').value
        write_speed = self.get_parameter('write_speed').value
        intercept_gain = self.get_parameter('intercept_gain').value

        vel = Twist()

        # ---------- State transitions ----------
        if self.offender_alive and self.offender_pose is not None:
            d = self._dist(self.pose.x, self.pose.y,
                           self.offender_pose.x, self.offender_pose.y)

            # WRITING -> ANGRY: offender entered the chase radius
            if self.state == 'WRITING' and d < k_chase:
                self.state = 'ANGRY'
                self.saved_wp = self.current_wp
                self._set_pen(down=True, color='red')
                self.get_logger().warn(
                    f'ANGRY! Intruder detected at {d:.2f} m -- chasing!')

            # ANGRY -> RETURNING: offender caught (within kill radius)
            if self.state == 'ANGRY' and d < k_kill:
                self._kill_offender()
                self.state = 'RETURNING'
                self._set_pen(down=False)
                self.get_logger().info('Target eliminated. Returning to work...')

        # ---------- State behaviour ----------
        if self.state == 'ANGRY':
            if self.offender_alive and self.offender_pose is not None:
                # ---- Advanced Task 2: predictive interception ----
                d = self._dist(self.pose.x, self.pose.y,
                               self.offender_pose.x, self.offender_pose.y)
                v_target = self.offender_pose.linear_velocity

                # m = gain * v_target * distance  (lookahead in metres)
                m = intercept_gain * v_target * d

                # Predicted position: offender + m in heading direction
                tx = self.offender_pose.x + m * math.cos(self.offender_pose.theta)
                ty = self.offender_pose.y + m * math.sin(self.offender_pose.theta)

                # Clamp to turtlesim bounds [0.5, 10.5]
                tx = max(0.5, min(10.5, tx))
                ty = max(0.5, min(10.5, ty))

                vel, _ = self._go_to(tx, ty, chase_speed)
            else:
                # Offender disappeared while chasing -- go back
                self.state = 'RETURNING'
                self._set_pen(down=False)

        elif self.state == 'RETURNING':
            if self.saved_wp < len(self.waypoints):
                wp = self.waypoints[self.saved_wp]
                vel, dist = self._go_to(wp['x'], wp['y'], write_speed)
                if dist < 0.15:
                    self.state = 'WRITING'
                    self.current_wp = self.saved_wp
                    self._set_pen(down=False)       # will be set by WRITING state
                    self._pen_key = None             # force pen refresh
                    self.get_logger().info('Resumed writing.')
            else:
                # Edge case: was at the end of waypoints
                self.state = 'WRITING'
                self.current_wp = 0

        elif self.state == 'WRITING':
            # Loop the drawing indefinitely
            if self.current_wp >= len(self.waypoints):
                self.current_wp = 0

            wp = self.waypoints[self.current_wp]

            # Set pen according to waypoint
            if wp['pen']:
                self._set_pen(down=True, color='white')
            else:
                self._set_pen(down=False)

            vel, dist = self._go_to(wp['x'], wp['y'], write_speed)

            if dist < 0.1:
                self.current_wp += 1

        self.vel_pub.publish(vel)


# ================================================================
#                           MAIN
# ================================================================

def main(args=None):
    rclpy.init(args=args)
    node = AngryTurtleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()