<<<<<<< HEAD
# # import rclpy
# # from rclpy.node import Node
# # from std_msgs.msg import Bool, String
# # from geometry_msgs.msg import PoseStamped 
# # from nav2_msgs.action import NavigateToPose
# # from rclpy.action import ActionClient
# # import math 
# # from tf2_ros import Buffer, TransformListener
# # from tf2_geometry_msgs import do_transform_pose

# # class MissionManager(Node):
# #     def __init__(self):
# #         super().__init__('mission_manager')
# #         self.target_id = 1
# #         self.state = "SEARCHING" 
        
# #         self.history_x = []
# #         self.history_y = [] 
# #         self.filter_size = 3

# #         self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=5))
# #         self.tf_listener = TransformListener(self.tf_buffer, self)
        
# #         self.exp_active_pub = self.create_publisher(Bool, '/explorer_active', 10)
# #         self.task_active_pub = self.create_publisher(Bool, '/task_a_active', 10)
        
# #         # MUST BE PoseStamped
# #         self.create_subscription(PoseStamped, 'target_3d', self.aruco_callback, 10)
# #         self.create_subscription(String, 'task_status', self.task_status_cb, 10)
        
# #         self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
# #         self.get_logger().info("Mission Manager Initialized - SEARCHING")

# #     def aruco_callback(self, msg):
# #         marker_id = int(msg.pose.orientation.w)
        
# #         if self.state == "SEARCHING" and marker_id == self.target_id:
# #             try:
# #                 # Precision lookup using the message's own timestamp
# #                 t = self.tf_buffer.lookup_transform(
# #                     'map', 
# #                     msg.header.frame_id, 
# #                     rclpy.time.Time(),    
# #                     timeout=rclpy.duration.Duration(seconds=0.1)
# #                 )

# #                 # Transform the .pose part of the PoseStamped
# #                 p_map = do_transform_pose(msg.pose, t)

# #                 self.history_x.append(p_map.position.x)
# #                 self.history_y.append(p_map.position.y)

# #                 if len(self.history_x) >= self.filter_size:
# #                     avg_x = sum(self.history_x) / self.filter_size
# #                     avg_y = sum(self.history_y) / self.filter_size
# #                     self.get_logger().info(f"TARGET {self.target_id} LOCKED. Approaching...")
# #                     self.start_approach(avg_x, avg_y)

# #             except Exception as e:
# #                 self.get_logger().warn(f"TF sync failed: {e}")

# #     def start_approach(self, target_x, target_y):
# #         self.state = "APPROACHING"
# #         self.exp_active_pub.publish(Bool(data=False))
        
# #         try:
# #             t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
# #             robot_x = t.transform.translation.x
# #             robot_y = t.transform.translation.y
            
# #             dx = robot_x - target_x
# #             dy = robot_y - target_y
# #             dist = math.sqrt(dx**2 + dy**2)

# #             if dist < 0.1: return

# #             goal_x = target_x + (0.2 * dx / dist)
# #             goal_y = target_y + (0.2 * dy / dist)
# #             angle = math.atan2(target_y - goal_y, target_x - goal_x)

# #             goal_msg = NavigateToPose.Goal()
# #             goal_msg.pose.header.frame_id = 'map'
# #             goal_msg.pose.pose.position.x = goal_x
# #             goal_msg.pose.pose.position.y = goal_y
# #             goal_msg.pose.pose.orientation.z = math.sin(angle / 2.0)    
# #             goal_msg.pose.pose.orientation.w = math.cos(angle / 2.0)

# #             self.nav_client.wait_for_server()
# #             self.nav_client.send_goal_async(goal_msg).add_done_callback(self.nav_response_cb)
            
# #             self.get_logger().error(f"🎯 Target in map frame: ({target_x:.2f}, {target_y:.2f})")
# #             self.get_logger().error(f"🤖 Robot in map frame: ({robot_x:.2f}, {robot_y:.2f})")
# #             self.get_logger().error(f"📍 Approach goal: ({goal_x:.2f}, {goal_y:.2f})")
        
# #         except Exception as e:
# #             self.get_logger().error(f"Approach setup failed: {e}")
# #             self.reset_to_explore()

# #     def nav_response_cb(self, future):
# #         goal_handle = future.result()
# #         if not goal_handle.accepted:
# #             self.get_logger().error("Goal Rejected")
# #             self.reset_to_explore()
# #             return
# #         goal_handle.get_result_async().add_done_callback(self.nav_finished_cb)

# #     def nav_finished_cb(self, future):
# #         self.get_logger().info("Approach Complete. Docking...")
# #         self.state = "DOCKING"
# #         self.task_active_pub.publish(Bool(data=True))

# #     def task_status_cb(self, msg):
# #         if msg.data == "SUCCESS" and self.state == "DOCKING":
# #             self.get_logger().info("Docked. Resuming...")
# #             self.reset_to_explore()

# #     def reset_to_explore(self):
# #         self.state = "SEARCHING"
# #         self.history_x.clear()
# #         self.history_y.clear()
# #         self.task_active_pub.publish(Bool(data=False))
# #         self.exp_active_pub.publish(Bool(data=True))

# # def main():
# #     rclpy.init()
# #     node = MissionManager()
# #     rclpy.spin(node)
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()




# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Bool, String
# from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped 
=======
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Bool, String
# from geometry_msgs.msg import PoseStamped
>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a
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
        
<<<<<<< HEAD
#         self.history_x = []
#         self.history_y = [] 
#         self.filter_size = 3  # reduced from 5

#         self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=5))
=======
#         self.detection_count = 0
#         self.history_x = []
#         self.history_z = []
#         self.filter_size = 10 

#         self.tf_buffer = Buffer()
>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a
#         self.tf_listener = TransformListener(self.tf_buffer, self)
        
#         self.exp_active_pub = self.create_publisher(Bool, '/explorer_active', 10)
#         self.task_active_pub = self.create_publisher(Bool, '/task_a_active', 10)
        
<<<<<<< HEAD
#         self.initial_pose_pub = self.create_publisher(
#     PoseWithCovarianceStamped, '/initialpose', 10)
#         self.create_timer(1.0, self.publish_initial_pose_once)
#         self.initial_pose_published = False

#         self.create_subscription(PoseStamped, 'target_3d', self.aruco_callback, 10)
#         self.create_subscription(String, 'task_status', self.task_status_cb, 10)
        
#         self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

#         self.last_detection_time = None
#         self.create_timer(0.1, self.check_detection_timeout)

#         self.get_logger().info("Mission Manager Initialized - SEARCHING")

#     def check_detection_timeout(self):
#         if self.last_detection_time is None:
#             return
#         elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
#         if elapsed > 1.0: 
#             if len(self.history_x) > 0:
#                 self.get_logger().warn("Marker lost, resetting history")
#             self.history_x.clear()
#             self.history_y.clear()
#             self.last_detection_time = None

#     def aruco_callback(self, msg):
#         marker_id = int(msg.pose.orientation.w)
        
#         if self.state != "SEARCHING" or marker_id != self.target_id:
#             return

#         self.last_detection_time = self.get_clock().now()

#         try:
#             t = self.tf_buffer.lookup_transform(
#                 'map', 
#                 msg.header.frame_id, 
#                 rclpy.time.Time(),  # use latest available transform
#                 timeout=rclpy.duration.Duration(seconds=0.1)
#             )

#             p_map = do_transform_pose(msg.pose, t)

#             self.history_x.append(p_map.position.x)
#             self.history_y.append(p_map.position.y)

#             self.get_logger().info(f"Marker seen ({len(self.history_x)}/{self.filter_size})")

#             if len(self.history_x) >= self.filter_size:
#                 avg_x = sum(self.history_x) / len(self.history_x)
#                 avg_y = sum(self.history_y) / len(self.history_y)
                
#                 # Clear history immediately so we don't re-trigger
#                 self.history_x.clear()
#                 self.history_y.clear()
#                 self.last_detection_time = None

#                 self.get_logger().info(f"TARGET {self.target_id} LOCKED at ({avg_x:.2f}, {avg_y:.2f}). Approaching...")
#                 self.start_approach(avg_x, avg_y)

#         except Exception as e:
#             self.get_logger().warn(f"TF sync failed: {e}")
=======
#         self.create_subscription(PoseStamped, 'target_3d', self.aruco_callback, 10)
        
#         self.create_subscription(String, 'task_status', self.task_status_cb, 10)
        
#         self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
#         self.get_logger().info("Mission Manager Initialized - SEARCHING")

#     def aruco_callback(self, msg):
#         marker_id = int(msg.orientation.w)
        
#         if self.state == "SEARCHING" and marker_id == self.target_id:
      
#             try:
#                 t = self.tf_buffer.lookup_transform(
#                     'map', 
#                     msg.header.frame_id, 
#                     msg.header.stamp,    
#                     timeout=rclpy.duration.Duration(seconds=0.1)
#                 )

#                 p_map = do_transform_pose(msg.pose, t)

#                 self.history_x.append(p_map.position.x)
#                 self.history_y.append(p_map.position.y)

#                 if len(self.history_x) >= self.filter_size:
#                     avg_x = sum(self.history_x) / len(self.history_x)
#                     avg_y = sum(self.history_y) / len(self.history_y)
                    
#                     self.get_logger().info(f"TARGET {self.target_id} LOCKED. Precision Match.")
#                     self.start_approach(avg_x, avg_y)

#             except Exception as e:
#                 self.get_logger().warn(f"TF sync failed: {e}")
>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a

#     def start_approach(self, target_x, target_y):
#         self.state = "APPROACHING"
#         self.exp_active_pub.publish(Bool(data=False))
        
<<<<<<< HEAD
#         try:
#             t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
#             robot_x = t.transform.translation.x
#             robot_y = t.transform.translation.y
            
=======
#         # Get current robot position in map to calculate approach vector
#         try:
#             now = rclpy.time.Time()
#             t = self.tf_buffer.lookup_transform('map', 'base_link', now, 
#                                                 timeout=rclpy.duration.Duration(seconds=0.5))
#             robot_x = t.transform.translation.x
#             robot_y = t.transform.translation.y
            
#             # Vector from target to robot (the approach direction)
>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a
#             dx = robot_x - target_x
#             dy = robot_y - target_y
#             dist = math.sqrt(dx**2 + dy**2)

<<<<<<< HEAD
#             self.get_logger().error(f"🤖 Robot: ({robot_x:.2f}, {robot_y:.2f})")
#             self.get_logger().error(f"🎯 Target: ({target_x:.2f}, {target_y:.2f})")
#             self.get_logger().error(f"📏 Distance: {dist:.2f}m")

#             if dist < 0.1:
#                 self.get_logger().warn("Target too close, going straight to docking")
#                 self.state = "DOCKING"
#                 self.task_active_pub.publish(Bool(data=True))
#                 return

#             # Place goal 0.35m from target toward robot
#             goal_x = target_x + (0.35 * dx / dist)
#             goal_y = target_y + (0.35 * dy / dist)
#             angle = math.atan2(target_y - goal_y, target_x - goal_x)

#             self.get_logger().error(f"📍 Approach goal: ({goal_x:.2f}, {goal_y:.2f}), angle: {math.degrees(angle):.1f}deg")

#             goal_msg = NavigateToPose.Goal()
#             goal_msg.pose.header.frame_id = 'map'
#             goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
#             goal_msg.pose.pose.position.x = goal_x
#             goal_msg.pose.pose.position.y = goal_y
#             goal_msg.pose.pose.orientation.z = math.sin(angle / 2.0)    
#             goal_msg.pose.pose.orientation.w = math.cos(angle / 2.0)

#             self.nav_client.wait_for_server()
#             self.nav_client.send_goal_async(goal_msg).add_done_callback(self.nav_response_cb)
            
#         except Exception as e:
#             self.get_logger().error(f"Approach setup failed: {e}")
#             self.reset_to_explore()

#     def nav_response_cb(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error("Goal Rejected - resetting to explore")
#             self.reset_to_explore()
#             return
#         self._goal_handle = goal_handle
#         goal_handle.get_result_async().add_done_callback(self.nav_finished_cb)

#     def nav_finished_cb(self, future):
#         result = future.result()
#         status = result.status
#         if status == 4:  # SUCCEEDED
#             self.nav_client.cancel_all_goals()
#             self.get_logger().info("Approach complete. Starting docking...")
#             self.state = "DOCKING"
#             self.task_active_pub.publish(Bool(data=True))
#             self.create_timer(1.0, self.activate_task_a, oneshot=True)
#         else:
#             self.get_logger().warn(f"Approach failed with status {status}, resetting...")
#             self.reset_to_explore()

#     def task_status_cb(self, msg):
#         if msg.data == "SUCCESS" and self.state == "DOCKING":
#             self.get_logger().info("Docking complete. Resuming exploration...")
#             self.reset_to_explore()

#     def reset_to_explore(self):
#         self.state = "SEARCHING"
#         self.history_x.clear()
#         self.history_y.clear()
#         self.last_detection_time = None
#         self.task_active_pub.publish(Bool(data=False))
#         self.exp_active_pub.publish(Bool(data=True))
#         self.get_logger().info("Back to SEARCHING")

#     def publish_initial_pose_once(self):
#         if self.initial_pose_published: 
#             return
#         msg = PoseWithCovarianceStamped()
#         msg.header.frame_id = 'map'
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.pose.pose.orientation.w = 1.0
#         msg.pose.covariance[0] = 0.25
#         msg.pose.covariance[7] = 0.25
#         msg.pose.covariance[35] = 0.07
#         self.initial_pose_pub.publish(msg)
#         self.initial_pose_published = True
#         self.get_logger().info("Initial pose published")
=======
#             # Safety: Don't divide by zero if you're already there
#             if dist < 0.1:
#                 self.get_logger().warn("Too close to ArUco to calculate approach.")
#                 return

#             # Calculate point 40cm away from target toward the robot
#             goal_x = target_x + (0.4 * dx / dist)
#             goal_y = target_y + (0.4 * dy / dist)
            
#             # Angle to face the ArUco
#             angle = math.atan2(target_y - goal_y, target_x - goal_x)

#             # Send Goal... (rest of your Nav2 logic)
            
#         except Exception as e:
#             self.get_logger().error(f"Final approach calculation failed: {e}")

#     def nav_response_cb(self, future):
#             goal_handle = future.result()
#             if not goal_handle.accepted:
#                 self.get_logger().error("Approach Goal Rejected! Resuming Search.")
#                 self.reset_to_explore()
#                 return
            
#             result_future = goal_handle.get_result_async()
#             result_future.add_done_callback(self.nav_finished_cb)

#     def nav_finished_cb(self, future):
#         self.get_logger().info("Approach Complete. Handing control to Task A Controller.")
#         self.state = "DOCKING"
        
#         self.task_active_pub.publish(Bool(data=True))

#     def task_status_cb(self, msg):
#         if msg.data == "SUCCESS" and self.state == "DOCKING":
#             self.get_logger().info("Task A confirmed Success. Resuming Exploration.")
#             self.reset_to_explore()

#     def reset_to_explore(self):
#         self.task_active_pub.publish(Bool(data=False))
#         self.exp_active_pub.publish(Bool(data=True))
        
#         self.state = "SEARCHING"
#         self.detection_count = 0
#         self.history_x.clear()
#         self.history_z.clear()

>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a
# def main():
#     rclpy.init()
#     node = MissionManager()
#     rclpy.spin(node)
#     rclpy.shutdown()

<<<<<<< HEAD
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
 
 
=======









import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped # UPDATED
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math 
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a
class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.target_id = 1
<<<<<<< HEAD
        self.state = "SEARCHING"
 
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=5))
        self.tf_listener = TransformListener(self.tf_buffer, self)
 
        self.exp_active_pub = self.create_publisher(Bool, '/explorer_active', 10)
        self.task_active_pub = self.create_publisher(Bool, '/task_a_active', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
 
        self.create_subscription(PoseStamped, 'target_3d', self.aruco_callback, 10)
        self.create_subscription(String, 'task_status', self.task_status_cb, 10)
 
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
 
        self.target_x_map = None
        self.target_y_map = None
        self._goal_handle = None
        self.approach_in_progress = False
 
        self.initial_pose_published = False
        self.create_timer(1.0, self.publish_initial_pose_once)
 
        self.get_logger().info("Mission Manager Initialized - SEARCHING")
 
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
 
    def aruco_callback(self, msg):
        marker_id = int(msg.pose.orientation.w)
 
        if self.state != "SEARCHING" or marker_id != self.target_id:
            return
 
        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
 
            p_map = do_transform_pose(msg.pose, t)
            new_x = p_map.position.x
            new_y = p_map.position.y
 
            if self.target_x_map is None:
                self.state = "APPROACHING"
                self.get_logger().info(f"First detection! Target at ({new_x:.2f}, {new_y:.2f}). Moving in...")
                self.target_x_map = new_x
                self.target_y_map = new_y
                self.exp_active_pub.publish(Bool(data=False))
                self.start_approach(new_x, new_y)
 
            else:
                # Subsequent detections — update if target shifted significantly
                dist_change = math.sqrt(
                    (new_x - self.target_x_map)**2 +
                    (new_y - self.target_y_map)**2
                )
 
                if dist_change > 0.15:  # target moved >15cm → re-route
                    self.get_logger().info(f"Target updated ({dist_change:.2f}m shift) → re-routing")
                    self.target_x_map = new_x
                    self.target_y_map = new_y
 
                    # Cancel current nav goal and send new one
                    if self._goal_handle is not None:
                        self._goal_handle.cancel_goal_async()
                        self._goal_handle = None
                    self.approach_in_progress = False
                    self.start_approach(new_x, new_y)
 
        except Exception as e:
            self.get_logger().warn(f"TF sync failed: {e}")
 
    def start_approach(self, target_x, target_y):
        if self.approach_in_progress:
            return
        
        self.state = "APPROACHING"
 
=======
        self.state = "SEARCHING" 
        
        self.history_x = []
        self.history_y = [] 
        self.filter_size = 5

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.exp_active_pub = self.create_publisher(Bool, '/explorer_active', 10)
        self.task_active_pub = self.create_publisher(Bool, '/task_a_active', 10)
        
        # MUST BE PoseStamped
        self.create_subscription(PoseStamped, 'target_3d', self.aruco_callback, 10)
        self.create_subscription(String, 'task_status', self.task_status_cb, 10)
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Mission Manager Initialized - SEARCHING")

    def aruco_callback(self, msg):
        # marker_id stored in orientation.w hack
        marker_id = int(msg.pose.orientation.w)
        
        if self.state == "SEARCHING" and marker_id == self.target_id:
            try:
                # Precision lookup using the message's own timestamp
                t = self.tf_buffer.lookup_transform(
                    'map', 
                    msg.header.frame_id, 
                    msg.header.stamp,    
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )

                # Transform the .pose part of the PoseStamped
                p_map = do_transform_pose(msg.pose, t)

                self.history_x.append(p_map.position.x)
                self.history_y.append(p_map.position.y)

                if len(self.history_x) >= self.filter_size:
                    avg_x = sum(self.history_x) / self.filter_size
                    avg_y = sum(self.history_y) / self.filter_size
                    self.get_logger().info(f"TARGET {self.target_id} LOCKED. Approaching...")
                    self.start_approach(avg_x, avg_y)

            except Exception as e:
                self.get_logger().warn(f"TF sync failed: {e}")

    def start_approach(self, target_x, target_y):
        self.state = "APPROACHING"
        self.exp_active_pub.publish(Bool(data=False))
        
>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = t.transform.translation.x
            robot_y = t.transform.translation.y
<<<<<<< HEAD
 
            dx = robot_x - target_x
            dy = robot_y - target_y
            dist = math.sqrt(dx**2 + dy**2)
 
            self.get_logger().error(f"🤖 Robot: ({robot_x:.2f}, {robot_y:.2f})")
            self.get_logger().error(f"🎯 Target: ({target_x:.2f}, {target_y:.2f})")
            self.get_logger().error(f"📏 Distance: {dist:.2f}m")
 
            if dist < 0.1:
                self.get_logger().warn("Already close enough, skipping approach")
                self.reset_to_explore()
                return
 
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
 
            self.approach_in_progress = True
            self.nav_client.wait_for_server()
            self.nav_client.send_goal_async(goal_msg).add_done_callback(self.nav_response_cb)
 
        except Exception as e:
            self.get_logger().error(f"Approach setup failed: {e}")
            self.reset_to_explore()
 
    def nav_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal Rejected - resetting to explore")
            self.approach_in_progress = False
            self.reset_to_explore()
            return
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.nav_finished_cb)
 
    def nav_finished_cb(self, future):
        self.approach_in_progress = False
        self._goal_handle = None
        result = future.result()
        status = result.status
 
        if status != 4:
            self.get_logger().warn(f"Approach failed with status {status}, resetting...")
            self.reset_to_explore()
            return
 
        # Verify robot actually moved close enough to target
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = t.transform.translation.x
            robot_y = t.transform.translation.y
            dist_to_target = math.sqrt(
                (robot_x - self.target_x_map)**2 +
                (robot_y - self.target_y_map)**2
            )
            self.get_logger().error(f"📏 Distance to target after approach: {dist_to_target:.2f}m")
 
            if dist_to_target > 1.0:
                self.get_logger().warn(f"Nav2 faked success ({dist_to_target:.2f}m away). Retrying...")
                self.start_approach(self.target_x_map, self.target_y_map)
                return
 
        except Exception as e:
            self.get_logger().warn(f"Could not verify position: {e}")
 
        self.get_logger().info("Approach complete. Starting docking...")
        self.state = "DOCKING"
        self.task_active_pub.publish(Bool(data=True))
 
    def task_status_cb(self, msg):
        if msg.data == "SUCCESS" and self.state == "DOCKING":
            self.get_logger().info("Docking complete. Resuming exploration...")
            self.reset_to_explore()
 
    def reset_to_explore(self):
        self.state = "SEARCHING"
        self.target_x_map = None
        self.target_y_map = None
        self._goal_handle = None
        self.approach_in_progress = False
        self.task_active_pub.publish(Bool(data=False))
        self.exp_active_pub.publish(Bool(data=True))
        self.get_logger().info("Back to SEARCHING")
 
 
=======
            
            dx = robot_x - target_x
            dy = robot_y - target_y
            dist = math.sqrt(dx**2 + dy**2)

            if dist < 0.1: return

            goal_x = target_x + (0.4 * dx / dist)
            goal_y = target_y + (0.4 * dy / dist)
            angle = math.atan2(target_y - goal_y, target_x - goal_x)

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
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
            self.get_logger().error("Goal Rejected")
            self.reset_to_explore()
            return
        goal_handle.get_result_async().add_done_callback(self.nav_finished_cb)

    def nav_finished_cb(self, future):
        self.get_logger().info("Approach Complete. Docking...")
        self.state = "DOCKING"
        self.task_active_pub.publish(Bool(data=True))

    def task_status_cb(self, msg):
        if msg.data == "SUCCESS" and self.state == "DOCKING":
            self.get_logger().info("Docked. Resuming...")
            self.reset_to_explore()

    def reset_to_explore(self):
        self.state = "SEARCHING"
        self.history_x.clear()
        self.history_y.clear()
        self.task_active_pub.publish(Bool(data=False))
        self.exp_active_pub.publish(Bool(data=True))

>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a
def main():
    rclpy.init()
    node = MissionManager()
    rclpy.spin(node)
    rclpy.shutdown()
<<<<<<< HEAD
 
 
if __name__ == '__main__':
    main()
 
=======

if __name__ == '__main__':
    main()
>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a
