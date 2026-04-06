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
        
#         self.detection_count = 0
#         self.history_x = []
#         self.history_z = []
#         self.filter_size = 10 

#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
        
#         self.exp_active_pub = self.create_publisher(Bool, '/explorer_active', 10)
#         self.task_active_pub = self.create_publisher(Bool, '/task_a_active', 10)
        
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

#     def start_approach(self, target_x, target_y):
#         self.state = "APPROACHING"
#         self.exp_active_pub.publish(Bool(data=False))
        
#         # Get current robot position in map to calculate approach vector
#         try:
#             now = rclpy.time.Time()
#             t = self.tf_buffer.lookup_transform('map', 'base_link', now, 
#                                                 timeout=rclpy.duration.Duration(seconds=0.5))
#             robot_x = t.transform.translation.x
#             robot_y = t.transform.translation.y
            
#             # Vector from target to robot (the approach direction)
#             dx = robot_x - target_x
#             dy = robot_y - target_y
#             dist = math.sqrt(dx**2 + dy**2)

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

# def main():
#     rclpy.init()
#     node = MissionManager()
#     rclpy.spin(node)
#     rclpy.shutdown()










import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped # UPDATED
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
        
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = t.transform.translation.x
            robot_y = t.transform.translation.y
            
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

def main():
    rclpy.init()
    node = MissionManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
