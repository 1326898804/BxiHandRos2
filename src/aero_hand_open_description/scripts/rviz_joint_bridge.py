#!/usr/bin/env python3
"""
rviz_joint_bridge.py

Subscribe to namespaced `joint_states` (from joint_state_publisher_gui in RViz)
and bridge them to `aero_hand_open_msgs/JointControl` messages so RViz GUI
can be used to control the real hand via existing ROS topics.

Usage:
  # Run RViz via display.launch.py with gui=true
  ros2 launch aero_hand_open_description display.launch.py gui:=true

  # In another terminal run the bridge
  /home/szz/aero-hand-open/ros2/.venv/bin/python scripts/rviz_joint_bridge.py

The script listens to `/left/joint_states` and `/right/joint_states` and
publishes `/left/joint_control` and `/right/joint_control`.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from aero_hand_open_msgs.msg import JointControl

# Expected joint keys in order (16 joints) as defined in JointControl.msg
EXPECTED_JOINT_KEYS = [
    "thumb_cmc_abd",
    "thumb_cmc_flex",
    "thumb_mcp",
    "thumb_ip",
    "index_mcp_flex",
    "index_pip",
    "index_dip",
    "middle_mcp_flex",
    "middle_pip",
    "middle_dip",
    "ring_mcp_flex",
    "ring_pip",
    "ring_dip",
    "pinky_mcp_flex",
    "pinky_pip",
    "pinky_dip",
]

class RVizJointBridge(Node):
    def __init__(self):
        super().__init__("rviz_joint_bridge")
        # Subscribe to namespaced joint_states for left and right
        self.left_sub = self.create_subscription(
            JointState, "/left/joint_states", lambda msg: self.joint_cb(msg, "left"), 10
        )
        self.right_sub = self.create_subscription(
            JointState, "/right/joint_states", lambda msg: self.joint_cb(msg, "right"), 10
        )

        self.left_pub = self.create_publisher(JointControl, "/left/joint_control", 10)
        self.right_pub = self.create_publisher(JointControl, "/right/joint_control", 10)

        self.get_logger().info("RViz joint bridge started: listening on /left|/right/joint_states")

    def joint_cb(self, msg: JointState, side: str):
        # Build target_positions array of length 16 (radians) in EXPECTED_JOINT_KEYS order
        positions = [0.0] * len(EXPECTED_JOINT_KEYS)

        # Try to match names by substring: prefer names that contain the key
        for i, key in enumerate(EXPECTED_JOINT_KEYS):
            matched = False
            for j, name in enumerate(msg.name):
                # match if the expected key is found in the joint name
                if key in name:
                    try:
                        positions[i] = msg.position[j]
                        matched = True
                        break
                    except Exception:
                        pass
            if not matched:
                # Try side-prefixed names, e.g. left_thumb_mcp
                for j, name in enumerate(msg.name):
                    if f"{side}_{key}" in name:
                        try:
                            positions[i] = msg.position[j]
                            matched = True
                            break
                        except Exception:
                            pass
            if not matched:
                # leave default 0.0 and warn once
                # (avoid spamming logger for every missing joint by warning at debug level)
                self.get_logger().debug(f"{side}: could not find joint matching '{key}' in incoming JointState names")

        jc = JointControl()
        jc.header = msg.header
        jc.target_positions = positions

        if side == "left":
            self.left_pub.publish(jc)
        else:
            self.right_pub.publish(jc)

        self.get_logger().debug(f"Bridged {side}/joint_states -> {side}/joint_control (first4: {positions[:4]})")


def main(args=None):
    rclpy.init(args=args)
    node = RVizJointBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
