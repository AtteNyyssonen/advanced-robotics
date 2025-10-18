#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool

def tri_wave(t, T):
    frac = (t % T) / T
    return 1.0 - 4.0 * abs(frac - 0.5)

class Commander(Node):
    def __init__(self):
        super().__init__('tri_y_commander')
        self.pub = self.create_publisher(Float64MultiArray, 'command', 10)
        self.sub = self.create_subscription(Bool, '/aruco_active', self.on_aruco_active, 1)

        # Pose center
        self.x  = 0.8
        self.y0 = 0.0
        self.z  = 0.6
        self.r  = math.pi
        self.p  = 0.0
        self.yw = 0.0
        self.A  = 0.6
        self.T  = 8.0
        self.t0 = time.time()

        self.aruco_active = False
        self.timer = self.create_timer(1.0/100.0, self.tick)

    def on_aruco_active(self, msg: Bool):
        self.aruco_active = msg.data
        state = 'paused' if msg.data else 'running'
        self.get_logger().info(f'Tri-wave {state} (aruco_active={msg.data})')

    def tick(self):
        if self.aruco_active:
            return

        t = time.time() - self.t0
        y = self.y0 + self.A * tri_wave(t, self.T)
        msg = Float64MultiArray()
        msg.data = [self.x, y, self.z, self.r, self.p, self.yw]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = Commander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()