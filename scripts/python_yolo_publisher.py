#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import CompressedImage,Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO  # Assuming YOLOv8

from obstacle_tracking_msg.msg import BoundingBox2DArray, BoundingBox2D

import time


class YOLOImageNode(Node):
    def __init__(self):
        super().__init__('img_detect_node')

        # Declare and get list of camera IDs (e.g., [0, 1, 2])
        self.declare_parameter('camera_ids', ['sf/AUV/rgb_camera'])  # Default string array
        self.camera_ids = self.get_parameter('camera_ids').get_parameter_value().string_array_value  # Use string_array_value
        print(f"Camera IDs: {self.camera_ids}")  # Debug print

        # Create a container to hold subscribers, publishers, and bridge for each camera
        self.subscribers_dict = {}  # Renamed from self.subscribers
        self.publishers_dict = {}  # Renamed from self.publishers
        self.bridges_dict = {}  # Renamed from self.bridges
        

        # Load YOLO model (can be a custom model path or "yolov8n.pt" for a small default model)
        self.get_logger().info('Loading YOLO model...')
        self.model = YOLO('/home/lucas/models/rami2.pt') # You can specify a custom model path here
        #self.model = YOLO('/home/graal/graal_ws/obstacle_detection_analysis_tools/models/yolov8n.pt') # You can specify a custom model path here
        self.get_logger().info('YOLO model loaded successfully!')
        self.br = CvBridge()

        # Create subscribers and publishers for each camera
        for cam_id in self.camera_ids:
            self.setup_camera(cam_id)
            
        import torch
        print("Is CUDA available:", torch.cuda.is_available())
        print("Number of GPUs:", torch.cuda.device_count())
        print("Torch device name:", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "No CUDA device")


    def setup_camera(self, cam_id):
        """Set up subscriber, publisher, and bridge for a given camera."""
        image_topic = f'/{cam_id}/image_color'
        annotations_topic = f'/dtc/annotations/{cam_id}'

        print("cam_id = " + str(cam_id))
        print("image_topic = " + str(image_topic))
        print("annotations_topic = " + str(annotations_topic))

        # Subscriber for image topic
        self.subscribers_dict[cam_id] = self.create_subscription(
            Image,
            image_topic,
            lambda msg, cam_id=cam_id: self.image_callback(msg, cam_id),
            10  # QoS profile depth
        )

        # Publisher for annotation topic
        self.publishers_dict[cam_id] = self.create_publisher(
            BoundingBox2DArray,
            annotations_topic,
            10  # QoS profile depth
        )

        # CV bridge for image conversion
        self.bridges_dict[cam_id] = CvBridge()

        self.get_logger().info(f'Set up subscriber and publisher for Camera {cam_id} on {image_topic} and {annotations_topic}.')
    
    def image_callback(self, msg, cam_id):
        """ Callback function to process the incoming image message for a specific camera. """
        self.get_logger().info(f'Received image message from Camera {cam_id}')
        start_time = time.time()  # Start the timer

        try:
            # Convert ROS2 Image message to OpenCV format
            cv_image = self.br.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image message to OpenCV format for Camera {cam_id}: {e}")
            return

        # Verify the cv_image is valid
        if cv_image is None or cv_image.size == 0:
            self.get_logger().error(f"Empty image received from Camera {cam_id}. Skipping processing.")
            return
        
        cv_imageCopy = cv_image.copy()

        # Run YOLO detection on the image
        self.get_logger().info(f'Running YOLO detection on Camera {cam_id}...')
        start_timeYOLO = time.time()  # Start the timer
        results = self.model(cv_image)
        end_timeYOLO = time.time()  # End the timer
        execution_timeYOLO = end_timeYOLO - start_timeYOLO  # Calculate total execution time
        self.get_logger().info(f'Inference time: {execution_timeYOLO:.4f} seconds')

        # Create BoundingBox2DArray message
        box_array_msg = BoundingBox2DArray()
        box_array_msg.header.stamp = msg.header.stamp
        box_array_msg.header.frame_id = f'camera_{cam_id}_frame'

        names = self.model.names
        # Process YOLO results and fill BoundingBox2DArray
        cnt = 0
        for result in results:
            for box in result.boxes:
                # Extract x, y, width, and height from YOLO detection
                x1, y1, x2, y2 = box.xyxy[0]  # YOLO returns (x1, y1, x2, y2) format
                width = x2 - x1
                height = y2 - y1
                x_center = x1 + width / 2
                y_center = y1 + height / 2

                # Draw the bounding box on the image
                color = (0, 255, 0)  # Green color for bounding boxes
                cv2.rectangle(cv_image, 
                (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                label = f"{names[box.cls.item()]}: {box.conf.item():.2f}"
                cv2.putText(cv_image, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 1)
                #cv2.putText(cv_image, label, (int(x1), int(y2) + 20), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 1)

                # Create a BoundingBox2D message and fill it
                bbox = BoundingBox2D()
                bbox.center_x = float(x_center)
                bbox.center_y = float(y_center)
                bbox.size_x = float(width)
                bbox.size_y = float(height)
                bbox.id = cnt
                bbox.conf = box.conf.item()
                bbox.desc = names[box.cls.item()]

                # Add bounding box to array message
                print(f'Camera {cam_id} - {bbox}')
                box_array_msg.boxes.append(bbox)

                cnt = cnt + 1

        # Save the image with bounding boxes to the given path
        tempYoloSaveDir = "/home/graal/exp_results/rami/"
        tempYoloSaveDirIMG = "/home/graal/exp_results/ramiIMG/"
        tempYoloSavePath = tempYoloSaveDir + "sim_dtc_" +  str( time.time()) + ".png"
        tempYoloSavePathIMG = tempYoloSaveDirIMG + "sim_" +  str( time.time()) + ".png"
        try:
            if not cv2.imwrite(tempYoloSavePath, cv_image):
                raise ValueError(f"Failed to save image to {tempYoloSavePath}")
            self.get_logger().info(f"Image saved successfully at {tempYoloSavePath}")
        except Exception as e:
            self.get_logger().error(f"Error saving image for Camera {cam_id}: {e}")
        try:
            if not cv2.imwrite(tempYoloSavePathIMG, cv_imageCopy):
                raise ValueError(f"Failed to save image to {tempYoloSavePath}")
            self.get_logger().info(f"Image saved successfully at {tempYoloSavePath}")
        except Exception as e:
            self.get_logger().error(f"Error saving image for Camera {cam_id}: {e}")

        # Publish the bounding box array message
        self.publishers_dict[cam_id].publish(box_array_msg)
        end_time = time.time()  # End the timer
        execution_time = end_time - start_time  # Calculate total execution time
        self.get_logger().info(f'Execution time for image_callback for Camera {cam_id}: {execution_time:.4f} seconds')

        self.get_logger().info(f'Published {len(box_array_msg.boxes)} bounding boxes for Camera {cam_id}.')
        self.get_logger().info(f'YOLO model loaded successfully on device: {self.model.device}')

        return None


def main(args=None):
    """ Main entry point of the ROS2 node. """
    rclpy.init(args=args)

    node = YOLOImageNode()

    # MultiThreadedExecutor allows parallel processing for each camera's callback
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

