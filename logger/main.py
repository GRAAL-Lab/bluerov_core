#!/usr/bin/env python3
import rclpy
from logger.node import LoggerNode
import sys
import signal
import threading

def main(args=None):
    node = None
    shutdown_event = threading.Event()
    
    def signal_handler(sig, frame):
        if shutdown_event.is_set():
            print(f"Force exit on signal {sig}")
            sys.exit(1)
            
        print(f"\nReceived signal {sig}. Initiating graceful shutdown...")
        shutdown_event.set()
        
        if node:
            node.get_logger().info(f"Signal {sig} received. Shutting down gracefully...")
            
            # Start cleanup in a separate thread with timeout
            def cleanup_with_timeout():
                try:
                    node.cleanup_resources()
                    print("Cleanup completed successfully")
                except Exception as e:
                    print(f"Error during cleanup: {e}")
                finally:
                    shutdown_event.set()
            
            cleanup_thread = threading.Thread(target=cleanup_with_timeout)
            cleanup_thread.daemon = True
            cleanup_thread.start()
            
            # Wait for cleanup to complete or timeout
            cleanup_thread.join(timeout=10.0)
            
            if cleanup_thread.is_alive():
                print("Cleanup timed out, forcing exit...")
            
        # Force exit after cleanup attempt
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except:
                pass
        sys.exit(0)

    try:
        rclpy.init(args=args)
        
        # Register signal handlers
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        node = LoggerNode()
        
        # Remove the signal handlers from the node since we handle them here
        # This prevents duplicate signal handling
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass  # Signal handler will take care of this
            
    except Exception as e:
        print(f'Error: {e}')
        if node:
            node.get_logger().error(f'Unexpected error: {e}')
    finally:
        if not shutdown_event.is_set():
            try:
                if node:
                    node.cleanup_resources()
                    node.destroy_node()
            except:
                pass
                
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except:
                pass

if __name__ == '__main__':
    main()