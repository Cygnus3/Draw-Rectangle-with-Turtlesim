#!/usr/bin/env python3
"""
rectangle_mover.py  (pose-feedback version)
--------------------------------------------
Uses /turtle1/pose to measure actual distance travelled and angle turned,
so rectangles are geometrically accurate regardless of speed or timer jitter.

Parameters:
  width    float  — long side  (default 3.0 units)
  height   float  — short side (default 2.0 units)
  speed    float  — linear speed in units/sec (default 1.5)
  repeats  int    — rectangles to draw; -1 = infinite (default -1)

Usage:
  ros2 run turtlesim turtlesim_node
  ros2 run my_pkg rectangle_mover

  # Custom size:
  ros2 run my_pkg rectangle_mover --ros-args -p width:=4.0 -p height:=2.5
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


ANGULAR_SPEED = 1.00   # rad/s while turning
TIMER_HZ      = 50    # 50 Hz control loop
TURN_ANGLE    = math.pi / 2          # 90 degrees in radians
ANGLE_TOL     = 0.005               # stop turning when within ~0.005 deg
DIST_TOL      = 0.01                 # stop moving when within 1 cm


def angle_diff(a, b):

    #Shortest signed difference from angle b to angle a, in (-pi, pi].
    d = a - b

    while d >  math.pi: 
        d -= 2 * math.pi
    while d < -math.pi: 
        d += 2 * math.pi
    return d


class RectangleMover(Node):

    def __init__(self):
        super().__init__('rectangle_mover')

        self.declare_parameter('width',   3.0)
        self.declare_parameter('height',  2.0)
        self.declare_parameter('speed',   1.0)
        self.declare_parameter('repeats', 1)

        self.width   = self.get_parameter('width').value
        self.height  = self.get_parameter('height').value
        self.speed   = self.get_parameter('speed').value
        self.repeats = self.get_parameter('repeats').value

        self.side_lengths = [self.width, self.height, self.width, self.height]

        # current pose (updated by subscriber)
        self.pose: Pose | None = None

        # state machine
        self.phase       = 'wait_pose'  # wait until first pose arrives
        self.side_index  = 0   # tracks which rectangle side
        self.loops_done  = 0   # we'll do only 1 loop here
        self.done = False

        # reference values set at the start of each phase
        self.start_x     = 0.0
        self.start_y     = 0.0
        self.start_theta = 0.0
        self.target_dist = 0.0
        self.target_theta= 0.0  # absolute heading we want to reach

        self.pub  = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub  = self.create_subscription(Pose, '/turtle1/pose',self._pose_cb, 10)
        self.timer = self.create_timer(1.0 / TIMER_HZ, self._tick)  

        self.get_logger().info(
            f'Rectangle mover ready — '
            f'width={self.width}, height={self.height}, speed={self.speed}'
        )

    # ------------------------------------------------------------------

    def _pose_cb(self, msg: Pose):
        self.pose = msg
        # transition out of waiting state as soon as first pose arrives

        if self.phase == 'wait_pose':
            self._begin_move()

    def _stop(self):
        self.pub.publish(Twist())

    def _begin_move(self):
        self.phase       = 'move'
        self.start_x     = self.pose.x
        self.start_y     = self.pose.y
        self.target_dist = self.side_lengths[self.side_index]
        self.get_logger().info(
            f'Moving side {self.side_index} — '
            f'distance {self.target_dist:.2f} units'
        )

    def _begin_turn(self):
        self.phase        = 'turn'
        self.start_theta  = self.pose.theta
        # target heading = current heading + 90° (left turn)
        self.target_theta = self.start_theta + TURN_ANGLE
        # normalise to (-pi, pi]
        while self.target_theta >  math.pi: 
            self.target_theta -= 2 * math.pi
        while self.target_theta < -math.pi: 
            self.target_theta += 2 * math.pi

    # ------------------------------------------------------------------

    def _tick(self):
        if self.pose is None or self.done:
            return

        msg = Twist()

        if self.phase == 'move':
            travelled = math.hypot(
                self.pose.x - self.start_x,
                self.pose.y - self.start_y)
            remaining = self.target_dist - travelled

            if remaining > DIST_TOL:
            # remove proportional slowdown — constant speed for straight sides
                msg.linear.x = self.speed
                self.pub.publish(msg)
            else:
                self._stop()
                self.side_index = (self.side_index + 1) % 4
                self._begin_turn()

        elif self.phase == 'turn':
            err = angle_diff(self.target_theta, self.pose.theta)

            if abs(err) > ANGLE_TOL:
                factor = min(1.0, abs(err) / (0.2 * TURN_ANGLE + 1e-6))
                msg.angular.z = math.copysign (max(0.15, ANGULAR_SPEED * factor), err)
                self.pub.publish(msg)
            else:
                self._stop()
                if self.side_index == 0:
                    self.loops_done += 1
                    self.get_logger().info(f'Rectangle {self.loops_done} complete.')
                    if self.repeats != -1 and self.loops_done >= self.repeats:
                        self.get_logger().info('All rectangles complete. Stopping.')
                        self.done = True
                        self.timer.cancel()
                        return
                self._begin_move()


# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = RectangleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()