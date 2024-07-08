import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import torch
import torchvision.transforms as T
from PIL import Image
from std_msgs.msg import Float32MultiArray  # Import Float32MultiArray for publishing depth data
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge, CvBridgeError

class DepthEstimatorNode(Node):
    def __init__(self):
        super().__init__('midas_depth_estimator')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Float32MultiArray, '/depth_data', 10)  # Publish depth data, not image
        self.subscription = self.create_subscription(
            ROSImage, '/image/undistorted', self.image_callback, 10)
        self.model = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model.to(self.device)
        self.model.eval()
        self.transform = T.Compose([
            T.Resize(384),
            T.CenterCrop(384),
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            depth_map = self.estimate_depth(frame)
            depth_array_msg = Float32MultiArray()  # Create a Float32MultiArray message
            depth_array_msg.data = depth_map.flatten().tolist()  # Flatten the depth map and convert to list

            self.publisher.publish(depth_array_msg)  # Publish the depth data
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')

    def estimate_depth(self, frame):
        img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        input_batch = self.transform(img).unsqueeze(0).to(self.device)
        with torch.no_grad():
            prediction = self.model(input_batch)
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=frame.shape[:2],
                mode="bilinear",
                align_corners=False
            ).squeeze()
        return prediction.cpu().numpy()

def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()