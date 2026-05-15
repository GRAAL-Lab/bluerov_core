#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO


class SwitchMonitor(Node):
    """
    A ROS2 node that monitors the state of a switch connected to GPIO pin
    and publishes its state to a boolean topic.
    """

    def __init__(self):
        super().__init__("switch_monitor")

        # Use only GPIO pin 35 for the safety switch.
        self.pin = 35

        try:
            # Initialize GPIO settings.
            GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
            # Set up pin as input (no pull-up resistor)
            GPIO.setup(self.pin, GPIO.IN)

            self.get_logger().info(f"Configured GPIO pin {self.pin} as input")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize GPIO: {e}")
            raise

        # Publisher for the safety switch state.
        self.publisher = self.create_publisher(Bool, "/auv/safety_switch", 10)

        # Timer to periodically read and publish the switch state.
        self.timer = self.create_timer(0.1, self.read_and_publish)

        self.get_logger().info(f"SwitchMonitor node started, monitoring pin {self.pin}")

    def read_and_publish(self):
        """
        Reads the current state of the GPIO pin and publishes it.
        """
        try:
            # Read the digital value from the pin.
            value = GPIO.input(self.pin)

            # Create and publish the message for the safety switch.
            msg = Bool()
            msg.data = bool(value)
            self.publisher.publish(msg)

            self.get_logger().debug(f"Published /auv/safety_switch: {value}")

        except Exception as e:
            self.get_logger().error(f"Error reading GPIO: {e}")

    def destroy_node(self):
        """
        Cleans up GPIO resources before the node is destroyed.
        """
        try:
            GPIO.cleanup([self.pin])
            self.get_logger().info("Cleaned up GPIO pin")
        except Exception as e:
            self.get_logger().error(f"Error during GPIO cleanup: {e}")
        super().destroy_node()


def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    """
    rclpy.init(args=args)
    node = SwitchMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
