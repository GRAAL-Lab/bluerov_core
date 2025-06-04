import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from auv_core_helper.msg import PoseStamped, MissionStatus
import csv
import os
from datetime import datetime, timezone
from xml.etree.ElementTree import Element, SubElement, tostring
from xml.dom.minidom import parseString
import math

def pose_csv_to_kml(csv_filepath, kml_filepath):
    kml = Element('kml', xmlns="http://www.opengis.net/kml/2.2")
    document = SubElement(kml, 'Document')
    name = SubElement(document, 'name')
    name.text = "Vehicle Navigation Data"

    with open(csv_filepath, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            timestamp_float = float(row['timestamp'])
            dt = datetime.fromtimestamp(timestamp_float, tz=timezone.utc)
            iso_time = dt.isoformat()

            latitude = float(row['latitude'])
            longitude = float(row['longitude'])
            depth = float(row['depth'])
            yaw_deg = float(row['heading'])

            placemark = SubElement(document, 'Placemark')
            timestamp_elem = SubElement(placemark, 'TimeStamp', name='Time')
            SubElement(timestamp_elem, 'Time').text = iso_time

            point = SubElement(placemark, 'Point', name='Position')
            SubElement(point, 'Position').text = f"{longitude},{latitude}"
            
            data1 = SubElement(placemark, 'Data', name='Heading')
            SubElement(data1, 'value').text = f"{yaw_deg}"
            data2 = SubElement(placemark, 'Data', name='Depth')
            SubElement(data2, 'value').text = f"{depth}"

    with open(kml_filepath, 'w') as f:
        pretty = parseString(tostring(kml, 'utf-8')).toprettyxml(indent="  ")
        f.write(pretty)

def status_csv_to_kml(csv_filepath, kml_filepath):
    kml = Element('kml', xmlns="http://www.opengis.net/kml/2.2")
    document = SubElement(kml, 'Document')
    name = SubElement(document, 'name')
    name.text = "Mission Status Events"

    with open(csv_filepath, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            timestamp_float = float(row['timestamp'])
            dt = datetime.fromtimestamp(timestamp_float, tz=timezone.utc)
            iso_time = dt.isoformat()

            placemark = SubElement(document, 'Placemark')
            SubElement(placemark, 'TimeStamp')
            SubElement(placemark.find('TimeStamp'), 'when').text = iso_time

            desc = SubElement(placemark, 'description')
            desc.text = f"Subtask: {row['subtask']}, Event: {row['key_decision']}"

            extdata = SubElement(placemark, 'ExtendedData')
            for key in ['task_benchmark', 'state', 'state_object']:
                data = SubElement(extdata, 'Data', name=key)
                SubElement(data, 'value').text = row.get(key, '')

    with open(kml_filepath, 'w') as f:
        pretty = parseString(tostring(kml, 'utf-8')).toprettyxml(indent="  ")
        f.write(pretty)

class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')

        self.pose_sub = self.create_subscription(PoseStamped, "/auv/global/pose_actual", self.pose_callback, 10)
        #self.status_sub = self.create_subscription(MissionStatus, "/auv/mission/status", self.status_callback, 10)

        log_dir = os.path.expanduser('~/mission_logs')
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.pose_log_path = os.path.join(log_dir, f"vehicle_navigation_data{timestamp}.csv")
        #self.status_log_path = os.path.join(log_dir, f"mission_status_data{timestamp}.csv")

        self.pose_csv = open(self.pose_log_path, 'w', newline='')
        self.pose_writer = csv.writer(self.pose_csv)
        self.pose_writer.writerow(['timestamp', 'latitude', 'longitude', 'heading', 'depth'])

        #self.status_csv = open(self.status_log_path, 'w', newline='')
        #self.status_writer = csv.writer(self.status_csv)
        #self.status_writer.writerow(['timestamp', 'task_benchmark', 'state', 'state_object', 'subtask', 'key_decision'])

        self.latest_pose = None
        self.timer = self.create_timer(1.0, self.log_pose)

        self.get_logger().info(f"Logging vehicle data to {self.pose_log_path}")
        #self.get_logger().info(f"Logging mission status to {self.status_log_path}")

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg

    
    #def status_callback(self, msg: MissionStatus):
        

    def log_pose(self):
        if self.latest_pose is None:
            return

        t = self.latest_pose.header.stamp
        timestamp = t.sec + t.nanosec * 1e-9
        
        yaw_deg = self.latest_pose.yaw * (180.0 / math.pi)

        self.pose_writer.writerow([
            timestamp,
            self.latest_pose.position.latitude,
            self.latest_pose.position.longitude,
            yaw_deg,
            self.latest_pose.depth
        ])
        self.get_logger().info(f"[POSE] Logged at {timestamp:.2f}")
        
        
    def destroy_node(self):
        self.pose_csv.close()
        #self.status_csv.close()

        pose_kml = self.pose_log_path.replace('.csv', '.kml')
        #status_kml = self.status_log_path.replace('.csv', '.kml')

        print(f"Converting CSV logs to KML files...")
        pose_csv_to_kml(self.pose_log_path, pose_kml)
        #status_csv_to_kml(self.status_log_path, status_kml)

        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('LoggerNode stopped by user')
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()
