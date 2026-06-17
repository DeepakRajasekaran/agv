#!/usr/bin/env python3
# Name:        navigator_node.py
# Author:      Deepak Rajasekaran
# Date:        2026-06-12
# Version:     1.0
# Description: Industrial Standard Topological Navigation Server for AGV

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import heapq

# Define the Topological Graph for the 10m Industrial Track
# Describes nodes (RFID tags), available paths, turn commands, and distance costs
MAP_GRAPH = {
    'TAG_TOP': {
        'TAG_RIGHT': ('STRAIGHT', 7.0),
        'TAG_LEFT': ('TURN_RIGHT', 5.0)  # Shortcut down to the left loop
    },
    'TAG_RIGHT': {
        'TAG_BOT': ('STRAIGHT', 7.0)
    },
    'TAG_BOT': {
        'TAG_LEFT': ('STRAIGHT', 7.0),
        'TAG_RIGHT': ('TURN_RIGHT', 5.0) # Shortcut up to the right loop
    },
    'TAG_LEFT': {
        'TAG_TOP': ('STRAIGHT', 7.0)
    }
}

class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigation_server_node')

        # State
        self.current_goal = None
        self.computed_path = []
        self.last_tag = None

        # Subscribers
        self.sub_goal = self.create_subscription(String, '/nav/goal_id', self.goal_callback, 10)
        self.sub_tag = self.create_subscription(String, '/sensor/tag_id', self.tag_callback, 10)

        # Publishers
        self.pub_command = self.create_publisher(String, '/nav/tag_command', 10)

        self.get_logger().info("Topological Navigation Server Initialized. Awaiting Goal on /nav/goal_id...")

    def goal_callback(self, msg: String):
        target = msg.data
        if target not in MAP_GRAPH:
            self.get_logger().error(f"Invalid goal '{target}'. Not found in map.")
            return

        self.current_goal = target
        
        # If we don't know our current tag, we can't plan yet.
        if self.last_tag is None:
            self.get_logger().warn(f"Goal '{target}' received, but current position is unknown. Waiting for first tag read...")
        else:
            self.compute_path()

    def tag_callback(self, msg: String):
        current_tag = msg.data
        self.get_logger().info(f"Robot reached node: {current_tag}")
        self.last_tag = current_tag

        if self.current_goal is None:
            # No active goal
            return

        if current_tag == self.current_goal:
            self.get_logger().info(f"GOAL REACHED! Halting AGV at {current_tag}.")
            self.pub_command.publish(String(data="STOP"))
            self.current_goal = None
            self.computed_path = []
            return

        # If we have a goal but no path, compute it now
        if not self.computed_path or current_tag not in self.computed_path:
            self.compute_path()

        if self.computed_path:
            # We are currently at `current_tag`, find it in the path to get the next step
            try:
                idx = self.computed_path.index(current_tag)
                if idx + 1 < len(self.computed_path):
                    next_node = self.computed_path[idx + 1]
                    cmd, _ = MAP_GRAPH[current_tag][next_node]
                    
                    self.get_logger().info(f"Routing to next node '{next_node}'. Issuing command: {cmd}")
                    self.pub_command.publish(String(data=cmd))
            except ValueError:
                self.get_logger().error(f"Current tag {current_tag} is not in the computed path {self.computed_path}. Replanning needed.")
                self.compute_path()

    def compute_path(self):
        if not self.last_tag or not self.current_goal:
            return

        start = self.last_tag
        goal = self.current_goal

        if start == goal:
            self.computed_path = [start]
            return

        # Dijkstra's Algorithm
        distances = {node: float('infinity') for node in MAP_GRAPH}
        distances[start] = 0
        priority_queue = [(0, start)]
        previous_nodes = {node: None for node in MAP_GRAPH}

        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)

            if current_distance > distances[current_node]:
                continue

            for neighbor, (cmd, weight) in MAP_GRAPH[current_node].items():
                distance = current_distance + weight

                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous_nodes[neighbor] = current_node
                    heapq.heappush(priority_queue, (distance, neighbor))

        # Reconstruct path
        path = []
        curr = goal
        while curr is not None:
            path.insert(0, curr)
            curr = previous_nodes[curr]

        if path[0] == start:
            self.computed_path = path
            self.get_logger().info(f"Computed Shortest Path: {' -> '.join(self.computed_path)}")
        else:
            self.get_logger().error(f"No path found from {start} to {goal}!")
            self.computed_path = []

def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
