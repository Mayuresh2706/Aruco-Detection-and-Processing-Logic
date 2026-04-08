<<<<<<< HEAD
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, PoseStamped
# from nav_msgs.msg import Odometry
# import math
# from std_msgs.msg import Bool, String

# class Task_A_Controller(Node):
#     def __init__(self):
#         super().__init__('Task_A_Controller')
#         self.status_pub = self.create_publisher(String, 'task_status', 10)
#         self.active_sub = self.create_subscription(Bool, '/task_a_active', self.active_cb, 10)
#         self.create_subscription(PoseStamped, 'target_3d', self.task_a, 10)
#         self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
#         self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

#         self.drive_timer = self.create_timer(0.05, self.drive_callback)

#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.start_x = 0.0
#         self.start_y = 0.0
#         self.start_yaw = 0.0
#         self.current_yaw = 0.0

#         self.distance_to_travel = 0.0
#         self.angle_to_turn = 0.0
#         self.state = 'idle'

#         # Tuneables
#         self.stop_dist   = 0.07   # 7cm from aruco marker
#         self.side_offset = 0.15   # 15cm sideways to actual target (to the right)
#         self.x_threshold = 0.02   # lateral alignment tolerance (2cm)
#         self.lin_speed   = 0.04   # 4cm/s
#         self.ang_speed   = 0.2    # rad/s

#         # State machine phases for rotation
#         # 0 = aligning face-on to aruco
#         # 1 = turning RIGHT 90° toward target
#         # 2 = turning LEFT 90° to face target after offset drive
#         self.rotation_phase = 0

#         self.is_active = False

#     def active_cb(self, msg):
#         self.is_active = msg.data
#         if msg.data == True and not self.is_active:
#             self.is_active = True
#             self.state = 'idle'

#         elif msg.data == False:
#             self.is_active = False
#             self.stop_robot()

#     def drive_callback(self):
#         if not self.is_active:
#             self.state = 'idle'
#             return

#         if self.state == 'rotating':
#             yaw_traveled = self._angle_diff(self.current_yaw, self.start_yaw)

#             if abs(yaw_traveled) >= abs(self.angle_to_turn):
#                 if self.rotation_phase == 0:
#                     # Finished aligning face-on → now drive straight to 7cm from marker
#                     self.get_logger().info("Aligned to marker! Driving in...")
#                     self.rotation_phase = 0
#                     self.state = 'driving_to_marker'
#                     self.start_x = self.current_x
#                     self.start_y = self.current_y

#                 elif self.rotation_phase == 1:
#                     # Finished turning RIGHT 90° → drive sideways offset
#                     self.get_logger().info("Turned right! Driving offset...")
#                     self.rotation_phase = 0
#                     self.state = 'driving_offset'
#                     self.start_x = self.current_x
#                     self.start_y = self.current_y

#                 elif self.rotation_phase == 2:
#                     # Finished turning LEFT 90° → facing target, FIRE
#                     self.get_logger().info("Facing target!")
#                     self.rotation_phase = 0
#                     self.fire()
#             else:
#                 cmd = Twist()
#                 cmd.angular.z = -self.ang_speed if self.angle_to_turn > 0 else self.ang_speed
#                 self.cmd_pub.publish(cmd)

#         elif self.state == 'driving_to_marker':
#             dist_traveled = math.sqrt(
#                 (self.current_x - self.start_x)**2 +
#                 (self.current_y - self.start_y)**2
#             )
#             if dist_traveled >= self.distance_to_travel:
#                 self.get_logger().info(f"Reached marker! Turning right 90°")
#                 self.rotation_phase = 1
#                 self.angle_to_turn = math.pi / 2   # positive = turn right
#                 self.state = 'rotating'
#                 self.start_yaw = self.current_yaw
#             else:
#                 cmd = Twist()
#                 cmd.linear.x = self.lin_speed
#                 self.cmd_pub.publish(cmd)

#         elif self.state == 'driving_offset':
#             dist_traveled = math.sqrt(
#                 (self.current_x - self.start_x)**2 +
#                 (self.current_y - self.start_y)**2
#             )
#             if dist_traveled >= self.side_offset:
#                 self.get_logger().info("Offset reached! Turning left 90°")
#                 self.rotation_phase = 2
#                 self.angle_to_turn = -math.pi / 2  # negative = turn left
#                 self.state = 'rotating'
#                 self.start_yaw = self.current_yaw
#             else:
#                 cmd = Twist()
#                 cmd.linear.x = self.lin_speed
#                 self.cmd_pub.publish(cmd)

#     def odom_callback(self, msg):
#         self.current_x = msg.pose.pose.position.x
#         self.current_y = msg.pose.pose.position.y
#         q = msg.pose.pose.orientation
#         siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#         self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

#     def task_a(self, msg):
#         if not self.is_active: return
#         if self.state != 'idle': return

#         marker_id = int(msg.pose.orientation.w)
#         if marker_id != 1: return

#         marker_x = msg.pose.position.x  
#         marker_z = msg.pose.position.z   

#         # Too far away — wait for nav2 to get closer
#         if marker_z > 0.5:
#             self.get_logger().info(f"Marker too far ({marker_z:.2f}m), waiting...")
#             return

#         # Too far to the side
#         if abs(marker_x) > 0.3:
#             self.get_logger().info(f"Marker too far sideways ({marker_x:.2f}m), waiting...")
#             return

#         err_x = marker_x  # we want marker centered (err = 0)

#         if abs(err_x) > self.x_threshold:
#             # Need to rotate to center the marker
#             self.angle_to_turn = math.atan2(err_x, marker_z)
#             self.distance_to_travel = max(0.0, marker_z - self.stop_dist)
#             self.start_yaw = self.current_yaw
#             self.rotation_phase = 0
#             self.state = 'rotating'
#             self.get_logger().info(f"Aligning: rotating {math.degrees(self.angle_to_turn):.1f}°, then driving {self.distance_to_travel:.3f}m")
#         else:
#             # Already aligned, just drive in
#             self.distance_to_travel = max(0.0, marker_z - self.stop_dist)
#             self.start_x = self.current_x
#             self.start_y = self.current_y
#             self.state = 'driving_to_marker'
#             self.get_logger().info(f"Already aligned. Driving {self.distance_to_travel:.3f}m")

#     def fire(self):
#         self.stop_robot()
#         self.get_logger().info("🔥 FIRE 🔥")
#         status_msg = String()
#         status_msg.data = "SUCCESS"
#         self.status_pub.publish(status_msg)

#     def stop_robot(self):
#         self.state = 'idle'
#         self.rotation_phase = 0
#         self.cmd_pub.publish(Twist())

#     def _angle_diff(self, current, start):
#         diff = current - start
#         while diff > math.pi:
#             diff -= 2 * math.pi
#         while diff < -math.pi:
#             diff += 2 * math.pi
#         return diff

# def main(args=None):
#     rclpy.init(args=args)
#     node = Task_A_Controller()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Manual Shutdown")
#     finally:
#         node.cmd_pub.publish(Twist())
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
from std_msgs.msg import Bool, String


class Task_A_Controller(Node):
    def __init__(self):
        super().__init__('task_a_node')
        self.status_pub = self.create_publisher(String, 'task_status', 10)
        self.active_sub = self.create_subscription(Bool, '/task_a_active', self.active_cb, 10)
        self.create_subscription(PoseStamped, 'target_3d', self.task_a, 10)
=======
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import math
from std_msgs.msg import Bool,String

class Task_A_Controller(Node):
    def __init__(self):
        super().__init__('Task_A_Controller')
        self.status_pub = self.create_publisher(String, 'task_status', 10)
        self.active_sub = self.create_subscription(Bool, '/task_a_active', self.active_cb, 10)
        self.create_subscription(Pose, 'target_3d', self.task_a, 10)
>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.drive_timer = self.create_timer(0.05, self.drive_callback)

        self.current_x = 0.0
        self.current_y = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.current_yaw = 0.0

        self.distance_to_travel = 0.0
        self.angle_to_turn = 0.0
        self.state = 'idle'

<<<<<<< HEAD
        # Tuneables
        self.stop_dist   = 0.07   # 7cm from aruco marker
        self.side_offset = 0.15   # 15cm sideways to actual target (to the right)
        self.x_threshold = 0.02   # lateral alignment tolerance (2cm)
        self.lin_speed   = 0.04   # 4cm/s
        self.ang_speed   = 0.2    # rad/s

        # rotation_phase:
        # 0 = aligning face-on to aruco
        # 1 = turning RIGHT 90° toward target
        # 2 = turning LEFT 90° to face target after offset drive
        self.rotation_phase = 0
=======
        self.target_x = 0.10       # X offset of target from marker (for yaw)
        self.target_z = 0.07       # This is linear x in world system
        self.z_threshold = 0.02   # 2cm lateral tolerance
        self.x_threshold = 0.02   # ~5 degree tolerance
        self.lin_speed = 0.04      # 4cm/s
        self.ang_speed = 0.2      # rad/s

        # 0 = aligning to target, 1 = turning 90, 2 = straightening -90
        self.rotation_phase = 0
        self.turn_90_direction = 1
>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a

        self.is_active = False

    def active_cb(self, msg):
        self.is_active = msg.data
<<<<<<< HEAD
        if not self.is_active:
            self.stop_robot()

    def drive_callback(self):
        if not self.is_active:
            self.state = 'idle'
            return

=======
    
    def drive_callback(self):
>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a
        if self.state == 'rotating':
            yaw_traveled = self._angle_diff(self.current_yaw, self.start_yaw)

            if abs(yaw_traveled) >= abs(self.angle_to_turn):
<<<<<<< HEAD
                if self.rotation_phase == 0:
                    self.get_logger().info("Aligned to marker! Driving in...")
                    self.state = 'driving_to_marker'
                    self.start_x = self.current_x
                    self.start_y = self.current_y

                elif self.rotation_phase == 1:
                    self.get_logger().info("Turned right! Driving offset...")
                    self.rotation_phase = 0
                    self.state = 'driving_offset'
                    self.start_x = self.current_x
                    self.start_y = self.current_y

                elif self.rotation_phase == 2:
                    self.get_logger().info("Facing target!")
                    self.rotation_phase = 0
                    self.fire()
=======
                if (self.rotation_phase == 0):
                    self.get_logger().info("Aligned! Turning 90")
                    self.rotation_phase = 1
                    self.angle_to_turn = self.turn_90_direction * math.pi / 2
                    self.start_yaw = self.current_yaw

                elif (self.rotation_phase == 1):
                    self.get_logger().info("Aligned! Turning 90")
                    self.rotation_phase = 0
                    self.state = 'driving'
                    self.start_x = self.current_x
                    self.start_y = self.current_y
                
                elif self.rotation_phase == 2:
                    self.get_logger().info("Ready to Fire")
                    self.rotation_phase = 0
                    self.stop_robot()
                
>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a
            else:
                cmd = Twist()
                cmd.angular.z = -self.ang_speed if self.angle_to_turn > 0 else self.ang_speed
                self.cmd_pub.publish(cmd)

<<<<<<< HEAD
        elif self.state == 'driving_to_marker':
=======
        elif self.state == 'driving':
>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a
            dist_traveled = math.sqrt(
                (self.current_x - self.start_x)**2 +
                (self.current_y - self.start_y)**2
            )
            if dist_traveled >= self.distance_to_travel:
<<<<<<< HEAD
                self.get_logger().info("Reached marker! Turning right 90°")
                self.rotation_phase = 1
                self.angle_to_turn = math.pi / 2   # turn right
                self.state = 'rotating'
                self.start_yaw = self.current_yaw
=======
                self.rotation_phase = 2
                self.angle_to_turn = -self.turn_90_direction * math.pi / 2
                self.state = 'rotating'
                self.get_logger().info("Aligning")
                self.start_yaw = self.current_yaw
            
>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a
            else:
                cmd = Twist()
                cmd.linear.x = self.lin_speed
                self.cmd_pub.publish(cmd)

<<<<<<< HEAD
        elif self.state == 'driving_offset':
            dist_traveled = math.sqrt(
                (self.current_x - self.start_x)**2 +
                (self.current_y - self.start_y)**2
            )
            if dist_traveled >= self.side_offset:
                self.get_logger().info("Offset reached! Turning left 90°")
                self.rotation_phase = 2
                self.angle_to_turn = -math.pi / 2  # turn left
                self.state = 'rotating'
                self.start_yaw = self.current_yaw
            else:
                cmd = Twist()
                cmd.linear.x = self.lin_speed
                self.cmd_pub.publish(cmd)
=======
>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
<<<<<<< HEAD
=======
        
        #Quart to Yaw(just a formula)
>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def task_a(self, msg):
<<<<<<< HEAD
        self.get_logger().error(f"task_a called, is_active={self.is_active}, state={self.state}")
        if not self.is_active: return
        if self.state != 'idle': return

        marker_id = int(msg.pose.orientation.w)
        if marker_id != 1: return

        # Corrected axis mapping: x=depth(forward), y=lateral
        marker_z = msg.pose.position.x   # forward depth
        marker_x = msg.pose.position.y   # lateral offset

        if marker_z > 0.5:
            self.get_logger().info(f"Marker too far ({marker_z:.2f}m), waiting...")
            return

        if abs(marker_x) > 0.3:
            self.get_logger().info(f"Marker too far sideways ({marker_x:.2f}m), waiting...")
            return

        if abs(marker_x) > self.x_threshold:
            self.angle_to_turn = math.atan2(marker_x, marker_z)
            self.distance_to_travel = max(0.0, marker_z - self.stop_dist)
            self.start_yaw = self.current_yaw
            self.rotation_phase = 0
            self.state = 'rotating'
            self.get_logger().info(f"Aligning: {math.degrees(self.angle_to_turn):.1f}°, then driving {self.distance_to_travel:.3f}m")
        else:
            self.distance_to_travel = max(0.0, marker_z - self.stop_dist)
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.state = 'driving_to_marker'
            self.get_logger().info(f"Already aligned. Driving {self.distance_to_travel:.3f}m")

    def fire(self):
        self.stop_robot()
        self.get_logger().info("🔥 FIRE 🔥")
        status_msg = String()
        status_msg.data = "SUCCESS"
        self.status_pub.publish(status_msg)

    def stop_robot(self):
        self.state = 'idle'
        self.rotation_phase = 0
        self.cmd_pub.publish(Twist())

    def _angle_diff(self, current, start):
        diff = current - start
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff
=======
        marker_id = int(msg.orientation.w)
        if not self.is_active: return
        rvec_yaw = msg.orientation.y
        marker_x = msg.position.x
        marker_z = msg.position.z
        err_x = marker_x - self.target_x
        
        if marker_id != 1:
            return
        
        if self.state != 'idle':
            return
        
        #Ensure that we are not too far away
        if marker_z > 0.5: 
            return

        if abs(marker_x) > 0.3:
            return
        
       

        if abs(err_x) > self.x_threshold:
            self.angle_to_turn = math.atan2(err_x, marker_z)
            self.distance_to_travel = max(0.0,marker_z - self.target_z)
            self.start_yaw = self.current_yaw
            self.state = 'rotating'
            self.get_logger().info(f"Starting rotation: {math.degrees(self.angle_to_turn):.1f} deg")
        
        else:
            self.distance_to_travel = marker_z - self.target_z
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.state = 'driving'
            self.get_logger().info(f"Already aligned. Driving {self.distance_to_travel:.3f}m")

    def _angle_diff(self,current,start):
        # Handles wrapping of angles
        diff = current - start
        while diff > math.pi:  
            diff -= 2 * math.pi
        while diff < -math.pi: 
            diff += 2 * math.pi
        return diff


    def stop_robot(self):
        self.state = 'idle'
        self.cmd_pub.publish(Twist())
        self.get_logger().info("SUCCESS: 7cm Target Reached via Odom")
        status_msg = String()
        status_msg.data = "SUCCESS"
        self.status_pub.publish(status_msg)
>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a


def main(args=None):
    rclpy.init(args=args)
    node = Task_A_Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Manual Shutdown")
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

<<<<<<< HEAD

=======
>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a
if __name__ == '__main__':
    main()