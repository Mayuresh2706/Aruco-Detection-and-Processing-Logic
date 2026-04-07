# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Bool, String
# from geometry_msgs.msg import PoseStamped 
# from nav2_msgs.action import NavigateToPose
# from rclpy.action import ActionClient
# import math 
# from tf2_ros import Buffer, TransformListener
# from tf2_geometry_msgs import do_transform_pose

# class MissionManager(Node):
#     def __init__(self):
#         super().__init__('mission_manager')
#         self.target_id = 1
#         self.state = "SEARCHING" 
        
#         self.history_x = []
#         self.history_y = [] 
#         self.filter_size = 3

#         self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=5))
#         self.tf_listener = TransformListener(self.tf_buffer, self)
        
#         self.exp_active_pub = self.create_publisher(Bool, '/explorer_active', 10)
#         self.task_active_pub = self.create_publisher(Bool, '/task_a_active', 10)
        
#         # MUST BE PoseStamped
#         self.create_subscription(PoseStamped, 'target_3d', self.aruco_callback, 10)
#         self.create_subscription(String, 'task_status', self.task_status_cb, 10)
        
#         self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
#         self.get_logger().info("Mission Manager Initialized - SEARCHING")

#     def aruco_callback(self, msg):
#         marker_id = int(msg.pose.orientation.w)
        
#         if self.state == "SEARCHING" and marker_id == self.target_id:
#             try:
#                 # Precision lookup using the message's own timestamp
#                 t = self.tf_buffer.lookup_transform(
#                     'map', 
#                     msg.header.frame_id, 
#                     rclpy.time.Time(),    
#                     timeout=rclpy.duration.Duration(seconds=0.1)
#                 )

#                 # Transform the .pose part of the PoseStamped
#                 p_map = do_transform_pose(msg.pose, t)

#                 self.history_x.append(p_map.position.x)
#                 self.history_y.append(p_map.position.y)

#                 if len(self.history_x) >= self.filter_size:
#                     avg_x = sum(self.history_x) / self.filter_size
#                     avg_y = sum(self.history_y) / self.filter_size
#                     self.get_logger().info(f"TARGET {self.target_id} LOCKED. Approaching...")
#                     self.start_approach(avg_x, avg_y)

#             except Exception as e:
#                 self.get_logger().warn(f"TF sync failed: {e}")

#     def start_approach(self, target_x, target_y):
#         self.state = "APPROACHING"
#         self.exp_active_pub.publish(Bool(data=False))
        
#         try:
#             t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
#             robot_x = t.transform.translation.x
#             robot_y = t.transform.translation.y
            
#             dx = robot_x - target_x
#             dy = robot_y - target_y
#             dist = math.sqrt(dx**2 + dy**2)

#             if dist < 0.1: return

#             goal_x = target_x + (0.2 * dx / dist)
#             goal_y = target_y + (0.2 * dy / dist)
#             angle = math.atan2(target_y - goal_y, target_x - goal_x)

#             goal_msg = NavigateToPose.Goal()
#             goal_msg.pose.header.frame_id = 'map'
#             goal_msg.pose.pose.position.x = goal_x
#             goal_msg.pose.pose.position.y = goal_y
#             goal_msg.pose.pose.orientation.z = math.sin(angle / 2.0)    
#             goal_msg.pose.pose.orientation.w = math.cos(angle / 2.0)

#             self.nav_client.wait_for_server()
#             self.nav_client.send_goal_async(goal_msg).add_done_callback(self.nav_response_cb)
            
#             self.get_logger().error(f"🎯 Target in map frame: ({target_x:.2f}, {target_y:.2f})")
#             self.get_logger().error(f"🤖 Robot in map frame: ({robot_x:.2f}, {robot_y:.2f})")
#             self.get_logger().error(f"📍 Approach goal: ({goal_x:.2f}, {goal_y:.2f})")
        
#         except Exception as e:
#             self.get_logger().error(f"Approach setup failed: {e}")
#             self.reset_to_explore()

#     def nav_response_cb(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error("Goal Rejected")
#             self.reset_to_explore()
#             return
#         goal_handle.get_result_async().add_done_callback(self.nav_finished_cb)

#     def nav_finished_cb(self, future):
#         self.get_logger().info("Approach Complete. Docking...")
#         self.state = "DOCKING"
#         self.task_active_pub.publish(Bool(data=True))

#     def task_status_cb(self, msg):
#         if msg.data == "SUCCESS" and self.state == "DOCKING":
#             self.get_logger().info("Docked. Resuming...")
#             self.reset_to_explore()

#     def reset_to_explore(self):
#         self.state = "SEARCHING"
#         self.history_x.clear()
#         self.history_y.clear()
#         self.task_active_pub.publish(Bool(data=False))
#         self.exp_active_pub.publish(Bool(data=True))

# def main():
#     rclpy.init()
#     node = MissionManager()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped 
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math 
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.target_id = 1
        self.state = "SEARCHING" 
        
        self.history_x = []
        self.history_y = [] 
        self.filter_size = 3  # reduced from 5

        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=5))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.exp_active_pub = self.create_publisher(Bool, '/explorer_active', 10)
        self.task_active_pub = self.create_publisher(Bool, '/task_a_active', 10)
        
        self.initial_pose_pub = self.create_publisher(
    PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_timer(1.0, self.publish_initial_pose_once)
        self.initial_pose_published = False

        self.create_subscription(PoseStamped, 'target_3d', self.aruco_callback, 10)
        self.create_subscription(String, 'task_status', self.task_status_cb, 10)
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.last_detection_time = None
        self.create_timer(0.1, self.check_detection_timeout)

        self.get_logger().info("Mission Manager Initialized - SEARCHING")

    def check_detection_timeout(self):
        if self.last_detection_time is None:
            return
        elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        if elapsed > 1.0: 
            if len(self.history_x) > 0:
                self.get_logger().warn("Marker lost, resetting history")
            self.history_x.clear()
            self.history_y.clear()
            self.last_detection_time = None

    def aruco_callback(self, msg):
        marker_id = int(msg.pose.orientation.w)
        
        if self.state != "SEARCHING" or marker_id != self.target_id:
            return

        self.last_detection_time = self.get_clock().now()

        try:
            t = self.tf_buffer.lookup_transform(
                'map', 
                msg.header.frame_id, 
                rclpy.time.Time(),  # use latest available transform
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            p_map = do_transform_pose(msg.pose, t)

            self.history_x.append(p_map.position.x)
            self.history_y.append(p_map.position.y)

            self.get_logger().info(f"Marker seen ({len(self.history_x)}/{self.filter_size})")

            if len(self.history_x) >= self.filter_size:
                avg_x = sum(self.history_x) / len(self.history_x)
                avg_y = sum(self.history_y) / len(self.history_y)
                
                # Clear history immediately so we don't re-trigger
                self.history_x.clear()
                self.history_y.clear()
                self.last_detection_time = None

                self.get_logger().info(f"TARGET {self.target_id} LOCKED at ({avg_x:.2f}, {avg_y:.2f}). Approaching...")
                self.start_approach(avg_x, avg_y)

        except Exception as e:
            self.get_logger().warn(f"TF sync failed: {e}")

    def start_approach(self, target_x, target_y):
        self.state = "APPROACHING"
        self.exp_active_pub.publish(Bool(data=False))
        
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = t.transform.translation.x
            robot_y = t.transform.translation.y
            
            dx = robot_x - target_x
            dy = robot_y - target_y
            dist = math.sqrt(dx**2 + dy**2)

            self.get_logger().error(f"🤖 Robot: ({robot_x:.2f}, {robot_y:.2f})")
            self.get_logger().error(f"🎯 Target: ({target_x:.2f}, {target_y:.2f})")
            self.get_logger().error(f"📏 Distance: {dist:.2f}m")

            if dist < 0.1:
                self.get_logger().warn("Target too close, going straight to docking")
                self.state = "DOCKING"
                self.task_active_pub.publish(Bool(data=True))
                return

            # Place goal 0.35m from target toward robot
            goal_x = target_x + (0.35 * dx / dist)
            goal_y = target_y + (0.35 * dy / dist)
            angle = math.atan2(target_y - goal_y, target_x - goal_x)

            self.get_logger().error(f"📍 Approach goal: ({goal_x:.2f}, {goal_y:.2f}), angle: {math.degrees(angle):.1f}deg")

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = goal_x
            goal_msg.pose.pose.position.y = goal_y
            goal_msg.pose.pose.orientation.z = math.sin(angle / 2.0)    
            goal_msg.pose.pose.orientation.w = math.cos(angle / 2.0)

            self.nav_client.wait_for_server()
            self.nav_client.send_goal_async(goal_msg).add_done_callback(self.nav_response_cb)
            
        except Exception as e:
            self.get_logger().error(f"Approach setup failed: {e}")
            self.reset_to_explore()

    def nav_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal Rejected - resetting to explore")
            self.reset_to_explore()
            return
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.nav_finished_cb)

    def nav_finished_cb(self, future):
        result = future.result()
        status = result.status
        if status == 4:  # SUCCEEDED
            self.get_logger().info("Approach complete. Starting docking...")
            self.state = "DOCKING"
            self.task_active_pub.publish(Bool(data=True))
        else:
            self.get_logger().warn(f"Approach failed with status {status}, resetting...")
            self.reset_to_explore()

    def task_status_cb(self, msg):
        if msg.data == "SUCCESS" and self.state == "DOCKING":
            self.get_logger().info("Docking complete. Resuming exploration...")
            self.reset_to_explore()

    def reset_to_explore(self):
        self.state = "SEARCHING"
        self.history_x.clear()
        self.history_y.clear()
        self.last_detection_time = None
        self.task_active_pub.publish(Bool(data=False))
        self.exp_active_pub.publish(Bool(data=True))
        self.get_logger().info("Back to SEARCHING")

    def publish_initial_pose_once(self):
        if self.initial_pose_published: 
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        self.initial_pose_pub.publish(msg)
        self.initial_pose_published = True
        self.get_logger().info("Initial pose published")
def main():
    rclpy.init()
    node = MissionManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()