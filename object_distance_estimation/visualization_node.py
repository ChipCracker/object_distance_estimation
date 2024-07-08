import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray
import cv2
import numpy as np
from cv_bridge import CvBridge
import json

# VisualizationNode, zeigt die erkannten Objekte mit Label, Accuracy und Entfernung an
# Label = Name des Objekts z.B. Mensch/People (Englische-Label)
# Accuracy = Wie genau ist die Erkennung des Objekts
# Entfernung = die Entfernung des Objekts zum Sensor (Kamera und Lidar-Sensor)
class VisualizationNode(Node):
    def __init__(self):
        super().__init__('image_distance_visualizer_node')
        
        self.bridge = CvBridge()

        # Subscriber für das '/image_raw'-Topic, um Bilder zu empfangen
        self.image_subscription = self.create_subscription(
            Image,
            '/image/undistorted',
            self.image_callback,
            10)  
        
        # Subsriber für das '/detected_objects'-Topic, um erkannte Objekte zu empfangen
        self.detections_subscription = self.create_subscription(
            String,
            '/detected_objects',
            self.detections_callback,
            10)
        
        # Subscriber für das '/fused_depth_data'-Topic, um Tiefendaten zu empfangen
        self.depth_subscription = self.create_subscription(
            Float32MultiArray,
            '/fused_depth_data',
            self.depth_callback,
            10)

        self.current_frame = None
        self.current_detections = []
        self.fused_depth_map = np.zeros((480, 640))  # Auflösung von 640x480

    # Callback-Funktion, wenn neue Bilddaten empfangen werden
    def image_callback(self, msg):
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Detektionen wird auf das Bild gezeichnet
        self.draw_detections()

    # Callback-Funktion, wenn ein neues Objekt erkannt wurde
    def detections_callback(self, msg):
        # Laden der Detektionen aus dem JSON
        self.current_detections = json.loads(msg.data)
        # Detektionen wird auf das Bild gezeichnet
        self.draw_detections()

    # Callback-Funktion, wenn neue Tiefendaten empfangen werden
    def depth_callback(self, msg):
        # Konvertieren der Tiefendaten in Array und Anpassen an die Auflösung
        self.fused_depth_map = np.array(msg.data).reshape((480, 640))

        # Ausgabe der minimalen und maximalen Tiefenwerte --> DEBUG
        # TODO: Entfernen wenn nicht mehr gebraucht wird
        self.get_logger().info(f'Min depth: {np.min(self.fused_depth_map)}, Max depth: {np.max(self.fused_depth_map)}')
        
        # Detektionen wird auf das Bild gezeichnet
        self.draw_detections()

    # Methode zum Zeichnen der Detektionen auf dem Bild/Video
    def draw_detections(self):
        if self.current_frame is None or self.fused_depth_map is None:
            return
        frame = self.current_frame.copy()  # Kopieren des aktuellen Frames
        for detection in self.current_detections:
            bbox = detection['bbox']
            label = detection['label']
            confidence = detection['confidence']
            x1, y1, x2, y2 = bbox

            # Berechnen der Mittelpunkte der Bounding Box
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2

            # Überprüfen, ob die Mittelpunkte innerhalb der Bildgrenzen liegen
            if 0 <= center_x < 640 and 0 <= center_y < 480:

                # Abrufen der Tiefe an der Mittelpunktkoordinate
                distance = self.fused_depth_map[center_y, center_x]
                
                # Ausgabe der Detektionsinformationen zur Information --> DEBUG
                # TODO: Entfernen wenn nicht mehr gebraucht wird
                # self.get_logger().info(f'Detected {label} at ({center_x}, {center_y}) with distance {distance:.2f}m')
            else:
                distance = float('inf') 

            # Zusätzliche Debugging-Informationen --> Informationen die über der Box angezeigt werden
            # TODO: Entfernen wenn nicht mehr gebraucht wird
            # self.get_logger().info(f'BBox: ({x1}, {y1}), ({x2}, {y2}) Center: ({center_x}, {center_y}) Distance: {distance}')

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f'{label}: {confidence:.2f}', 
                        (x1, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.putText(frame, f'Dist: {distance:.2f}m', 
                        (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.imshow('Detections', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
