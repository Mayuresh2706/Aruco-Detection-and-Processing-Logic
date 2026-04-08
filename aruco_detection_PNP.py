import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np

class ArucoSub_Pub(Node):
    def __init__(self):
        super().__init__('ArucoSub_Pub')
        self.subscription = self.create_subscription(
            Float32MultiArray, 'target_pixels', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(PoseStamped, 'target_3d', 10)
        
        # Calibration data(need to change)(Assume 320x240 dimensions)
        self.mtx = np.array([
            [282.5, 0.0, 160.0],
            [0.0, 282.5, 120.0],
            [0.0, 0.0, 1.0]], dtype=np.float32)
        
        # Constant(need to change)
        self.dist = np.zeros((5, 1), dtype=np.float32)

        #Measured with ruler(CHANGE!!!!!)
        self.marker_size = 0.2

        self.obj_points = np.array([
        [-self.marker_size/2,  self.marker_size/2, 0],
        [ self.marker_size/2,  self.marker_size/2, 0], 
        [ self.marker_size/2, -self.marker_size/2, 0], 
        [-self.marker_size/2, -self.marker_size/2, 0] ], dtype=np.float32)

    def listener_callback(self, msg):
        data = msg.data
        if len(data) < 9: 
            return
        
        corners = np.array(data[:8]).reshape((4, 1, 2)).astype(np.float32)
        marker_id = int(data[8])


        success, rvec, tvec = cv2.solvePnP(
            self.obj_points, corners, self.mtx, self.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        
        if success:
            self.publish_pose(tvec, rvec,marker_id)
            print(f"Marker id = {marker_id}")
    
    def publish_pose(self, tvec, rvec,id):
        pose_msg = PoseStamped()

        pose_msg.header.frame_id = "camera_link"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
<<<<<<< HEAD
        pose_msg.pose.position.x = float(tvec[2][0])  
        pose_msg.pose.position.y = -float(tvec[0][0])  
        pose_msg.pose.position.z = -float(tvec[1][0])  

=======
        pose_msg.pose.position.x = float(tvec[0][0])
        pose_msg.pose.position.y = float(tvec[1][0])
        pose_msg.pose.position.z = float(tvec[2][0])
>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a

        pose_msg.pose.orientation.x = float(rvec[0][0])
        pose_msg.pose.orientation.y = float(rvec[1][0])
        pose_msg.pose.orientation.z = float(rvec[2][0])
        pose_msg.pose.orientation.w = float(id)

        self.publisher_.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoSub_Pub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()