import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
import json
import torch
from cv_bridge import CvBridge

# Yolov5-Node die Objekte erkennt und diese als JSON published
class YOLOv5Node(Node):
    def __init__(self):
        super().__init__('yolov5_node')
        
        # Subscriber auf das '/image_raw'-Topic, um Bilddaten der Webcam/Kamera zu empfangen
        self.subscription = self.create_subscription(
            Image,
            '/image/undistorted',
            self.listener_callback,
            10) 
        
        # Publisher für das '/detected_objects'-Topic, um erkannte Objekte als JSON bereitzustellen
        self.publisher = self.create_publisher(String, '/detected_objects', 10)
        
        self.bridge = CvBridge()
        
        # Laden des YOLOv5-Modells von ultralytics
        # yolov5s = small Modell
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  

    # Callback-Funktion, wenn eine neue Bildnachricht empfangen wird
    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Objekterkennung YOLOv5
        results = self.model(frame)
        
        detections = []
        for result in results.xyxy[0].cpu().numpy():  
            x1, y1, x2, y2, conf, cls = result  
            detections.append({
                'label': self.model.names[int(cls)],  # Klassifizierungsinformationen
                'bbox': [int(x1), int(y1), int(x2), int(y2)],  # Bounding Box Koordinaten
                'confidence': float(conf)  # Konfidenz der Erkennung
            })

        self.publisher.publish(String(data=json.dumps(detections)))

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv5Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
