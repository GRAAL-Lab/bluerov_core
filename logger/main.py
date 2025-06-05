#!/usr/bin/env python3
import rclpy
from logger.node import LoggerNode
import sys
import signal

def main(args=None):
    node = None
    shutdown_complete = False
    
    def signal_handler(sig, frame):
        nonlocal shutdown_complete
        if shutdown_complete:
            return
            
        print(f"\nSignal {sig} received. Exiting...")
        if node:
            node.get_logger().warn(f"Signal {sig} received. Exiting...")
        
        shutdown_complete = True
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)

    try:
        rclpy.init(args=args)
        
        # Register signal handlers after init but before node creation
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        node = LoggerNode()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('LoggerNode interrupted by user')
    except Exception as e:
        if node:
            node.get_logger().error(f'Unexpected error: {e}')
        else:
            print(f'Error before node creation: {e}')
    finally:
        if not shutdown_complete:
            if node:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

if __name__ == '__main__':
    main()