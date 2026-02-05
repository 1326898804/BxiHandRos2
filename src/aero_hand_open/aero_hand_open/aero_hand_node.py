import rclpy
from rclpy.node import Node
import signal
import sys

from aero_hand_open_msgs.msg import JointControl, ActuatorStates, ActuatorControl

import numpy as np

from aero_open_sdk.aero_hand import AeroHand
from aero_open_sdk.aero_hand_constants import AeroHandConstants


class AeroHandNode(Node):
    def __init__(self):
        super().__init__("aero_hand_node")
        ## Parameters
        self.declare_parameter("right_port", "")
        self.declare_parameter("left_port", "")
        self.declare_parameter("baudrate", 921600)
        self.declare_parameter("feedback_frequency", 100.0)
        self.declare_parameter("control_space", "joint")
        self.declare_parameter("bluetooth", False)

        right_port = self.get_parameter("right_port").value
        left_port = self.get_parameter("left_port").value
        baudrate = self.get_parameter("baudrate").value
        feedback_frequency = self.get_parameter("feedback_frequency").value
        control_space = self.get_parameter("control_space").value
        bluetooth = bool(self.get_parameter("bluetooth").value)
        
        ## Initialize hands and subscribers/publishers based on provided ports
        if right_port != "":
            try:
                self.right_hand = AeroHand(port=right_port, baudrate=baudrate, bluetooth=bluetooth)
            except Exception as e:
                self.get_logger().error(
                    f"Failed to initialize Right hand on port {right_port}: {e}"
                )
                raise e
            if control_space == "joint":
                self.joint_states_sub_right = self.create_subscription(
                    JointControl,
                    "right/joint_control",
                    self.joint_states_right_callback,
                    10,
                )
            elif control_space == "actuator":
                self.actuator_control_sub_right = self.create_subscription(
                    ActuatorControl,
                    "right/actuator_control",
                    self.actuator_control_right_callback,
                    10,
                )
            else:
                self.get_logger().error(f"Invalid control space: {control_space}")
                raise ValueError(
                    f'Invalid control space: {control_space}, expected "joint" or "actuator"'
                )

            self.hand_state_pub_right = self.create_publisher(
                ActuatorStates, "right/actuator_states", 10
            )

        if left_port != "":
            try:
                self.left_hand = AeroHand(port=left_port, baudrate=baudrate, bluetooth=bluetooth)
            except Exception as e:
                self.get_logger().error(
                    f"Failed to initialize Left hand on port {left_port}: {e}"
                )
                raise e
            if control_space == "joint":
                self.joint_states_sub_left = self.create_subscription(
                    JointControl,
                    "left/joint_control",
                    self.joint_states_left_callback,
                    10,
                )
            elif control_space == "actuator":
                self.actuator_control_sub_left = self.create_subscription(
                    ActuatorControl,
                    "left/actuator_control",
                    self.actuator_control_left_callback,
                    10,
                )
            else:
                self.get_logger().error(f"Invalid control space: {control_space}")
                raise ValueError(
                    f'Invalid control space: {control_space}, expected "joint" or "actuator"'
                )

            self.hand_state_pub_left = self.create_publisher(
                ActuatorStates, "left/actuator_states", 10
            )

        ## Atleast one hand should be initialized
        if right_port == "" and left_port == "":
            self.get_logger().error(
                "Both right_port and left_port are None. Please provide at least one port."
            )
            raise ValueError(
                "Both right_port and left_port are None. Please provide at least one port."
            )

        ## Joint Limits
        self.joint_ll = AeroHandConstants.joint_lower_limits
        self.joint_ul = AeroHandConstants.joint_upper_limits
        self.actuator_ll = AeroHandConstants.actuation_lower_limits
        self.actuator_ul = AeroHandConstants.actuation_upper_limits

        self.feedback_timer = self.create_timer(
            1.0 / feedback_frequency, self.feedback_callback
        )

        self.get_logger().info("Aero hand node has been started.")

    def cleanup(self):
        """清理资源"""
        self.get_logger().info("Cleaning up resources...")
        
        if hasattr(self, "right_hand"):
            try:
                self.get_logger().info("Closing right hand connection...")
                self.right_hand.close()
            except Exception as e:
                self.get_logger().error(f"Error closing right hand: {e}")
        
        if hasattr(self, "left_hand"):
            try:
                self.get_logger().info("Closing left hand connection...")
                self.left_hand.close()
            except Exception as e:
                self.get_logger().error(f"Error closing left hand: {e}")
        
        self.get_logger().info("Cleanup complete.")

    def __del__(self):
        """析构函数"""
        self.cleanup()

    def feedback_callback(self):
        if hasattr(self, "right_hand"):
            right_hand_state = ActuatorStates()
            right_hand_state.header.stamp = self.get_clock().now().to_msg()
            right_hand_state.side = "right"
            try:
                right_hand_state.actuations = self.right_hand.get_actuations()
                right_hand_state.actuator_speeds = self.right_hand.get_actuator_speeds()
                right_hand_state.actuator_currents = (
                    self.right_hand.get_actuator_currents()
                )
                right_hand_state.actuator_temperatures = (
                    self.right_hand.get_actuator_temperatures()
                )
            except Exception as e:
                return
            self.hand_state_pub_right.publish(right_hand_state)

        if hasattr(self, "left_hand"):
            left_hand_state = ActuatorStates()
            left_hand_state.header.stamp = self.get_clock().now().to_msg()
            left_hand_state.side = "left"
            try:
                left_hand_state.actuations = self.left_hand.get_actuations()
                left_hand_state.actuator_speeds = self.left_hand.get_actuator_speeds()
                left_hand_state.actuator_currents = (
                    self.left_hand.get_actuator_currents()
                )
                left_hand_state.actuator_temperatures = (
                    self.left_hand.get_actuator_temperatures()
                )
            except Exception as e:
                return
            self.hand_state_pub_left.publish(left_hand_state)

    def joint_states_right_callback(self, msg: JointControl):
        if not hasattr(self, "right_hand"):
            self.get_logger().warn("Right hand is not initialized.")
            return
        if len(msg.target_positions) != 16:
            self.get_logger().warn(
                f"Expected 16 joint positions for right hand, but got {len(msg.target_positions)}."
            )
            return
        try:
            self.get_logger().debug(
                f"right/joint_control received len={len(msg.target_positions)} first_vals={msg.target_positions[:4]}"
            )
        except Exception:
            pass

        joint_values = [np.rad2deg(jv) for jv in msg.target_positions]
        joint_values = np.clip(joint_values, self.joint_ll, self.joint_ul).tolist()
        try:
            self.right_hand.set_joint_positions(joint_values)
            self.get_logger().debug(f"Sent joint positions to right hand: {joint_values[:4]} ...")
        except Exception as e:
            self.get_logger().error(f"Error sending joint positions to right hand: {e}")

    def joint_states_left_callback(self, msg: JointControl):
        if not hasattr(self, "left_hand"):
            self.get_logger().warn("Left hand is not initialized.")
            return
        if len(msg.target_positions) != 16:
            self.get_logger().warn(
                f"Expected 16 joint positions for left hand, but got {len(msg.target_positions)}."
            )
            return
        try:
            self.get_logger().debug(
                f"left/joint_control received len={len(msg.target_positions)} first_vals={msg.target_positions[:4]}"
            )
        except Exception:
            pass

        joint_values = [np.rad2deg(jv) for jv in msg.target_positions]
        joint_values = np.clip(joint_values, self.joint_ll, self.joint_ul).tolist()
        try:
            self.left_hand.set_joint_positions(joint_values)
            self.get_logger().debug(f"Sent joint positions to left hand: {joint_values[:4]} ...")
        except Exception as e:
            self.get_logger().error(f"Error sending joint positions to left hand: {e}")

    def actuator_control_right_callback(self, msg: ActuatorControl):
        if not hasattr(self, "right_hand"):
            self.get_logger().warn("Right hand is not initialized.")
            return
        if len(msg.actuation_positions) != 7:
            self.get_logger().warn(
                f"Expected 7 actuator positions for right hand, but got {len(msg.actuation_positions)}."
            )
            return
        actuation_values = np.clip(
            msg.actuation_positions, self.actuator_ll, self.actuator_ul
        ).tolist()
        try:
            self.get_logger().debug(
                f"right/actuator_control received mode={msg.control_mode} len={len(msg.actuation_positions)} vals={actuation_values}"
            )
        except Exception:
            pass

        try:
            self.right_hand.set_actuations(actuation_values)
        except Exception as e:
            self.get_logger().error(f"Error sending actuation values to right hand: {e}")

    def actuator_control_left_callback(self, msg: ActuatorControl):
        if not hasattr(self, "left_hand"):
            self.get_logger().warn("Left hand is not initialized.")
            return
        if len(msg.actuation_positions) != 7:
            self.get_logger().warn(
                f"Expected 7 actuator positions for left hand, but got {len(msg.actuation_positions)}."
            )
            return
        actuation_values = np.clip(
            msg.actuation_positions, self.actuator_ll, self.actuator_ul
        ).tolist()
        try:
            self.get_logger().debug(
                f"left/actuator_control received mode={msg.control_mode} len={len(msg.actuation_positions)} vals={actuation_values}"
            )
        except Exception:
            pass

        try:
            self.left_hand.set_actuations(actuation_values)
            self.get_logger().debug(f"Sent actuation values to left hand: {actuation_values}")
        except Exception as e:
            self.get_logger().error(f"Error sending actuation values to left hand: {e}")


def main(args=None):
    rclpy.init(args=args)
    aero_hand_node = AeroHandNode()
    
    try:
        rclpy.spin(aero_hand_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 确保清理资源
        aero_hand_node.cleanup()
        aero_hand_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()