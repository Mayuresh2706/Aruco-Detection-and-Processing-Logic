import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import time
import numpy as np

class ArucoSimplePub(Node):
    def __init__(self):
        super().__init__('aruco_simple_pub')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'target_pixels', 10)

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret: return

        corners, ids, _ = self.detector.detectMarkers(frame)
        
        if ids is not None:
            for i in range(len(ids)):
                pixel_data = corners[i][0].flatten().tolist() 
                pixel_data.append(float(ids[i][0]))
                self.publish_message(pixel_data)


    def publish_message(self, data_list):
        msg = Float32MultiArray()
        msg.data = data_list
        self.publisher_.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    node = ArucoSimplePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()