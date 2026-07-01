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
            
        # NEW: Publisher for the AI Depth Map
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
            
        self.get_logger().info("Loading AI Depth Model onto GPU...")
        device = 0 if torch.cuda.is_available() else -1
        self.depth_estimator = pipeline(
            task="depth-estimation", 
            model="depth-anything/Depth-Anything-V2-Small-hf", 
            device=device
        )
        self.get_logger().info("Model loaded successfully!")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            pil_img = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            
            result = self.depth_estimator(pil_img)
            
            # --- NEW ROS PUBLISHING LOGIC ---
            # 1. Get raw depth array as floats
            depth_cv = np.array(result["depth"]).astype(np.float32)
            
            # 2. Normalize to 0-1, then scale to an arbitrary "millimeter" range. 
            # (Note: AI monocular depth is relative. We scale it here to an assumed 10 meter max depth)
            fixed_scale = 255.0 / np.max(depth_cv) # Do this ONCE during init, or use a hardcoded max
            
            # Use a static multiplier so 1 meter is always represented by the same pixel intensity
            depth_normalized = np.clip(depth_cv / 10.0, 0.0, 1.0)
            depth_mm = (depth_normalized * 10000).astype(np.uint16)
            
            # 3. Convert to ROS Image message (16-bit unsigned integer)
            depth_msg = self.bridge.cv2_to_imgmsg(depth_mm, encoding="16UC1")
            
            # 4. Copy the exact header/timestamp from the incoming RGB frame! 
            # RTAB-Map will reject the frame if the RGB and Depth timestamps don't match.
            depth_msg.header = msg.header 
            
            self.depth_pub.publish(depth_msg)
            # --------------------------------
            
            # Keep visualizer for debugging
            depth_viz = cv2.applyColorMap(cv2.normalize(depth_cv, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U), cv2.COLORMAP_INFERNO)
            cv2.imshow("Raw Rover Stream", cv_image)
            cv2.imshow("AI Depth Map", depth_viz)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

def main(args=None):
    # ... (Keep main function exactly the same)
    rclpy.init(args=args)
    node = AIDepthPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()