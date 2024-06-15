#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill
import random, math
from functools import partial

class TurtleCatcherNode(Node):

    def __init__(self, name):
        super().__init__("turtle_catcher")
        self.name_ = name
        self.position_ = Pose()
        self.catcher_name = 'turtle1'
        self.spawn(name, random.randint(2, 10)*1.0, random.randint(2, 10)*1.0, False)

        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, f"/{name}/cmd_vel", 10
        )
        self.pose_subscriber_ = self.create_subscription(
            Pose, f"/{self.catcher_name}/pose", self.pose_callback, 10
        )
        self.pose_self_subscriber_ = self.create_subscription(
            Pose, f"/{name}/pose", self.pose_self_callback, 10
        )
        self.get_logger().info("turtle_catcher has started.")

    
    def pose_callback(self, pose:Pose):
        #self.get_logger().info(f"pose_callback")
        dif_x = self.position_.x - pose.x
        dif_y = self.position_.y - pose.y
        
        if math.sqrt(dif_x**2 + dif_y**2) < 1:
            #self.delete(self.catcher_name)
            #self.catcher_name = chr(random.randint(97, 122))+chr(random.randint(97, 122))
            self.catcher_name = 'turtle1'
            self.spawn(self.catcher_name, random.randint(0, int(self.position_.x))*1.0, 
                       random.randint(0, int(self.position_.y))*1.0, True)

        x1, y1 = self.position_.x, self.position_.y
        x2, y2 = pose.x, pose.y
        self.get_logger().info(f"({x1}, {y1}) and ({x2}, {y2})")

        base = x2-x1
        hyp = math.sqrt((x2-x1)**2 + (y2-y1)**2)

        theta = math.acos(base/hyp)
        self.get_logger().info(f"theta is {theta}")

        cur_theta = self.position_._theta

        if cur_theta < 0:
            cur_theta += 2*math.pi

        self.get_logger().info(f"current theta is {cur_theta}")
        
        if y2-y1 < 0:
            theta = 2*math.pi - theta
            theta -= cur_theta
        else:
            theta -= cur_theta

        cmd = Twist()
        cmd.angular.z = theta
        cmd.linear.x = 1.2
        self.get_logger().info(f"Setting {self.name_}'s theta to {cmd.angular.z} and linX to {cmd.linear.x}")

        self.cmd_vel_publisher_.publish(cmd)

    def pose_self_callback(self, pose:Pose):
        self.get_logger().info("In pose_self_callback")
        self.position_ = pose

    
    def spawn(self, name, x:float, y:float, re_init:bool):

        client = self.create_client(Spawn, '/spawn')
        
        while not client.wait_for_service(1.0):
            self.get_logger().warn(f"Waiting for {name} to spawn...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = 0.0
        request.name = name

        future = client.call_async(request)
        if re_init:
            future.add_done_callback(partial(self.callback_spawn))
        

    def callback_spawn(self, future):
        self.pose_subscriber_ = self.create_subscription(
            Pose, f"/{self.catcher_name}/pose", self.pose_callback, 10
        )

    
    def delete(self, name):
        client = self.create_client(Kill, '/kill')
        
        while not client.wait_for_service(1.0):
            self.get_logger().warn(f"Waiting for {name} to die...")

        request = Kill.Request()
        request.name = name        

        future = client.call_async(request)
        

    def callback_delete(self, future):
        pass
    


def main(args=None):
    rclpy.init(args=args)
    
    catcher = TurtleCatcherNode(name='turtle2')
    
    rclpy.spin(catcher)
    rclpy.shutdown()
