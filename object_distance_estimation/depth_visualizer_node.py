import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import torch
import torchvision.transforms as T
from PIL import Image
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge, CvBridgeError

class DepthVisualizerNode(Node):
    def __init__(self):
        super().__init__('midas_depth_estimator')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(ROSImage, '/depth_image', 10)
        self.subscription = self.create_subscription(
            ROSImage, '/image/undistorted', self.image_callback, 10)
        
        #model_type = "DPT_Large"
        #model_type = "DPT_Hybrid"
        model_type = "MiDaS_small"

        self.model = torch.hub.load("intel-isl/MiDaS", model_type)
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
            
            # Normalize the depth map to 0-255 for visualization
            depth_map_normalized = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
            depth_colormap = cv2.applyColorMap(depth_map_normalized.astype(np.uint8), cv2.COLORMAP_MAGMA)
            
            # Publish the depth image
            depth_image = self.bridge.cv2_to_imgmsg(depth_colormap, encoding="bgr8")
            self.publisher.publish(depth_image)
            
            # Display the depth image using OpenCV
            cv2.imshow('Depth Visualization', depth_colormap)
            cv2.waitKey(1)
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
    node = DepthVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()