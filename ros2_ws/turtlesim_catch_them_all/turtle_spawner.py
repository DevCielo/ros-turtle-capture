#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
from functools import partial

from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle

class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.alive_turtles = {}
        self.turtle_counter = 2
        self.timer = self.create_timer(0.5, self.callback_spawn_turtle)
        self.publisher_ = self.create_publisher(TurtleArray, "/alive_turtles", 10)
        self.server_ = self.create_service(CatchTurtle, "/catch_turtle", self.kill_turtle)
        self.kill_client = self.create_client(Kill, "/kill")
        self.get_logger().info("Turtle spawner has been started")
    
    def kill_turtle(self, request, response):
        if request.turtle_name not in self.alive_turtles:
            response.success = False
            return response
        
        request_kill = Kill.Request()
        request_kill.name = request.turtle_name
        
        future = self.kill_client.call_async(request_kill)
        future.add_done_callback(partial(self.kill_response_callback, turtle_name=request.turtle_name))
        response.success = True
        
        return response
    
    def kill_response_callback(self, future, turtle_name):
        try:
            result = future.result()
            if result is not None:
                self.alive_turtles.pop(turtle_name, None)
                self.publish_alive_turtles()
                self.get_logger().info(f"Turtle {turtle_name} killed successfully.")
            else:
                self.get_logger().error(f"Failed to kill turtle {turtle_name}.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")
    
    def callback_spawn_turtle(self):
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        
        client = self.create_client(Spawn, "/spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting For Server Spawn Turtles...")
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.name = "turtle" + str(self.turtle_counter)
        self.turtle_counter += 1
        
        self.alive_turtles[request.name] = {"x": x, "y": y}
        
        self.publish_alive_turtles()
        
        future = client.call_async(request)
    
    def publish_alive_turtles(self):
        msg = TurtleArray()
        for name, coords in self.alive_turtles.items():
            turtle_msg = Turtle()
            turtle_msg.name = name
            turtle_msg.x = coords['x']
            turtle_msg.y = coords['y']
            msg.turtles.append(turtle_msg)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
