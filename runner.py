#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill
import random, math


class TurtleRunnerNode(Node):

    def __init__(self):
        super().__init__("turtle_runner")
        self.name_ = "turtle1"
        self.position_ = Pose()

        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, f"/{self.name_}/cmd_vel", 10
        )
        self.pose_subscriber_ = self.create_subscription(
            Pose, f"/{self.name_}/pose", self.pose_callback, 10
        )
        self.start()

        self.get_logger().info("turtle_runner has started.")

    def pose_callback(self, pose:Pose):
        self.position_ = pose
        self.get_logger().info(f"{self.name_}:({pose.x}, {pose.y})")

        cmd = Twist()

        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9
        else:
            cmd.linear.x = random.random()*5
            cmd.angular.z = random.random()

        self.get_logger().info(f"Setting {self.name_}'s theta to {cmd.angular.z} and linX to {cmd.linear.x}")

        self.cmd_vel_publisher_.publish(cmd)

    def start(self):
        cmd = Twist()
        cmd.linear.x = random.random()
        cmd.angular.z = random.random()

        self.cmd_vel_publisher_.publish(cmd)


    def spawn(self, x:float, y:float):

        client = self.create_client(Spawn, '/spawn')
        
        
        while not client.wait_for_service(1.0):
            self.get_logger().warn(f"Waiting for {self.name_} to spawn...")

        request = Spawn.Request()
        request.x = float(x)
        request.y = float(y)
        request.theta = 0.0
        request.name = self.name_

        future = client.call_async(request)

        self.start()

    def callback_spawn(self, future):
        pass

    def delete(self):
        client = self.create_client(Kill, '/kill')
        
        while not client.wait_for_service(1.0):
            self.get_logger().warn(f"Waiting for {self.name_} to die...")

        request = Kill.Request()
        request.name = self.name_        

        future = client.call_async(request)

    def callback_delete(self, future):
        pass

def main(args=None):
    rclpy.init(args=args)
    runner = TurtleRunnerNode()
    rclpy.spin(runner)
    rclpy.shutdown()