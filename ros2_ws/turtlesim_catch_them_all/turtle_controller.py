#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import math

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.srv import CatchTurtle

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.target_x = None
        self.target_y = None
        self.name = None
        self.catching_turtle = False
        self.subscriber_catch_turtle_ = self.create_subscription(TurtleArray, "/alive_turtles", self.catch_turtle, 10)
        self.subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.control_turtle, 10)
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer_ = self.create_timer(0.01, self.reach_target)
        self.get_logger().info("Turtle Controller has started")
        
    def catch_turtle(self, msg):
        if len(msg.turtles) == 0:
            return
        
        if self.catching_turtle == False:
            self.turtle_distances = []
            for turtle in msg.turtles:
                turtle_distance_x = (self.pose_.x - turtle.x)**2
                turtle_distance_y = (self.pose_.y - turtle.y)**2
                distance = math.sqrt(turtle_distance_x + turtle_distance_y)
                self.turtle_distances.append(distance)
            
            closest_turtle = 0
            turtle_min_distance = self.turtle_distances[0]
            for i, distance in enumerate(self.turtle_distances):
                if distance < turtle_min_distance:
                    closest_turtle = i
                    turtle_min_distance = distance
   
                
            self.name = msg.turtles[closest_turtle].name
            self.target_x = msg.turtles[closest_turtle].x
            self.target_y = msg.turtles[closest_turtle].y

    def control_turtle(self, pose):
        self.pose_ = pose
        
    def reach_target(self):
        if self.target_x is None or self.target_y is None:
            return
        
        self.catching_turtle = True
        
        distance_x = (self.pose_.x - self.target_x)**2
        distance_y = (self.pose_.y - self.target_y)**2
        distance = math.sqrt(distance_x + distance_y)
        
        msg = Twist()

        if distance > 0.5:
            msg.linear.x = 2*distance
            
            goal_theta = math.atan2(self.target_y - self.pose_.y, self.target_x - self.pose_.x)
            diff = goal_theta - self.pose_.theta
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi
            
            msg.angular.z = 6*diff
        else:
            client = self.create_client(CatchTurtle, "/catch_turtle")
            while not client.wait_for_service(1.0):
                self.get_logger().warn("Waiting For Server To Catch Turtle...")
                
            request = CatchTurtle.Request()
            request.turtle_name = self.name
            
            future = client.call_async(request)
            future.add_done_callback(self.handle_catch_response)
            
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        
        self.publisher_.publish(msg)
    
    def handle_catch_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Turtle {self.name} caught succesfully")
            else:
                self.get_logger().warn(f"Failed to catch turtle {self.name}.")
        except Exception as e:
            self.get_logger().error(f"Error catching turtle: {str(e)}")
        finally:
            self.target_x = None
            self.target_y = None
            self.name = None
            self.catching_turtle = False

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
