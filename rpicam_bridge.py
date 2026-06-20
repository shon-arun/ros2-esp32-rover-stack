import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import subprocess
import threading

class RpiCamFinalBridge(Node):
    def __init__(self):
        super().__init__('rpicam_bridge')
        self.publisher_ = self.create_publisher(CompressedImage, 'image_raw/compressed', 10)

        # Hardware MJPEG pipeline
        # -n: Headless mode
        # --codec mjpeg: Hardware-encoded, standard JPEG stream
        # NO --vflip/--hflip: Protects the Bayer color matrix
        cmd = [
            'rpicam-vid', '-n', '-t', '0',
            '--width', '1280', '--height', '720',
            '--framerate', '30',
            '--codec', 'mjpeg',
            '-o', '-'
        ]

        self.get_logger().info("Spinning up hardware MJPEG pipeline...")
        self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

        self.capture_thread = threading.Thread(target=self.stream_reader, daemon=True)
        self.capture_thread.start()

    def stream_reader(self):
        byte_buffer = b''
        while rclpy.ok():
            chunk = self.process.stdout.read(4096)
            if not chunk: continue
            byte_buffer += chunk

            # Find JPEG boundaries
            a = byte_buffer.find(b'\xff\xd8') # Start of Image
            b = byte_buffer.find(b'\xff\xd9') # End of Image

            if a != -1 and b != -1 and a < b:
                jpg = byte_buffer[a:b+2]
                byte_buffer = byte_buffer[b+2:]

                msg = CompressedImage()
                msg.header.frame_id = "camera_link"
                msg.format = "jpeg"
                msg.data = jpg
                self.publisher_.publish(msg)
            elif a > b and b != -1:
                # Flush invalid bytes
                byte_buffer = byte_buffer[a:]

def main(args=None):
    rclpy.init(args=args)
    node = RpiCamFinalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.process.terminate()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
