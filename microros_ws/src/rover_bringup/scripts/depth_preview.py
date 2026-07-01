#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
from transformers import pipeline
from PIL import Image as PILImage
import numpy as np

class AIDepthPreview(Node):
    def __init__(self):
        super().__init__('ai_depth_preview')
        self.bridge = CvBridge()
        
        # Subscribe to RGB
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw_decompressed', 
            self.image_callback,
            10)
            
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
            
        self.get_logger().info("Loading AI Depth Model onto GPU...")
        device = 0 if torch.cuda.is_available() else -1
        self.depth_estimator = pipeline(
            task="depth-estimation", 
            model="depth-anything/Depth-Anything-V2-Small-hf", 
            device=device
        )
        self.get_logger().info("Model loaded successfully!")

        self.pseudo_focal_baseline = 59.3 

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            pil_img = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            
            result = self.depth_estimator(pil_img)
            
            pred_depth = result["predicted_depth"].squeeze().cpu().numpy()
            
            original_h, original_w = cv_image.shape[:2]
            depth_resized = cv2.resize(pred_depth, (original_w, original_h), interpolation=cv2.INTER_LINEAR)
            
            safe_depth = np.clip(depth_resized, a_min=0.001, a_max=None)
            
            metric_depth_meters = self.pseudo_focal_baseline / safe_depth 
            
            metric_depth_meters = np.clip(metric_depth_meters, 0.0, 10.0)
            
            depth_mm = (metric_depth_meters * 1000.0).astype(np.uint16)
            
            depth_msg = self.bridge.cv2_to_imgmsg(depth_mm, encoding="16UC1")
            
            depth_msg.header = msg.header 
            
            self.depth_pub.publish(depth_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AIDepthPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()