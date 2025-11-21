#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from flask import Flask, request, jsonify
import math
import threading
import sys

app = Flask(__name__)

# Global Navigator Object
navigator = None

def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    Input: yaw in radians
    Output: qx, qy, qz, qw
    """
    qx = 0.0 # Assuming strictly 2D navigation (roll=0)
    qy = 0.0 # Assuming strictly 2D navigation (pitch=0)
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)
    return qx, qy, qz, qw

@app.route('/navigate', methods=['POST'])
def navigate_robot():
    global navigator
    
    # 1. Parse Request Body
    data = request.get_json()
    
    if not data or 'x' not in data or 'y' not in data or 'heading' not in data:
        return jsonify({"error": "Missing x, y, or heading"}), 400

    try:
        target_x = float(data['x'])
        target_y = float(data['y'])
        target_heading = float(data['heading']) # Expecting Radians
    except ValueError:
        return jsonify({"error": "Invalid number format"}), 400

    # 2. Create PoseStamped Message
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # Position
    goal_pose.pose.position.x = target_x
    goal_pose.pose.position.y = target_y
    
    # Orientation (Yaw -> Quaternion)
    # Note: If your heading input is degrees, convert using math.radians(target_heading)
    qx, qy, qz, qw = get_quaternion_from_euler(0, 0, target_heading)
    goal_pose.pose.orientation.x = qx
    goal_pose.pose.orientation.y = qy
    goal_pose.pose.orientation.z = qz
    goal_pose.pose.orientation.w = qw

    # 3. Send Command to Nav2 (Non-blocking)
    navigator.goToPose(goal_pose)

    return jsonify({
        "status": "accepted",
        "target": {
            "x": target_x,
            "y": target_y,
            "heading": target_heading
        }
    }), 202

def main():
    global navigator
    
    # Initialize ROS 2 Logic
    rclpy.init()
    
    # Initialize Navigator
    navigator = BasicNavigator()
    
    # Wait for Nav2 to be fully active before accepting API calls
    print("Waiting for Nav2 to activate...")
    # navigator.waitUntilNav2Active() # Uncomment this if you want to block startup until Nav2 is ready
    print("Nav2 is ready!")

    # Run Flask (turn off debug/reloader to prevent double-initialization of ROS nodes)
    app.run(host='0.0.0.0', port=5000, debug=False)

    # Cleanup when Flask exits
    rclpy.shutdown()

if __name__ == '__main__':
    main()