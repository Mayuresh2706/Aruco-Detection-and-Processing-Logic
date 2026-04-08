import rclpy
from rclpy.node import Node
<<<<<<< HEAD
from geometry_msgs.msg import Twist, PoseStamped
=======
from geometry_msgs.msg import Twist, Pose
>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a
from nav_msgs.msg import Odometry
import math

class Task_B_Controller(Node):
    def __init__(self):
        super().__init__('Task_B_Controller')
<<<<<<< HEAD
        self.create_subscription(PoseStamped, 'target_3d', self.marker_callback, 10)
=======
        self.create_subscription(Pose, 'target_3d', self.marker_callback, 10)
>>>>>>> 498805b792fca7ca60c6d3e6a1cd38b616040c7a
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.drive_timer = self.create_timer(0.05, self.drive_callback)

        # Odom constants
        self.current_x = 0.0
        self.current_y = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.current_yaw = 0.0

        self.distance_to_travel = 0.0
        self.angle_to_turn = 0.0
        self.state = 'idle'

        self.target_x = 0.10       # X offset of target from marker (for yaw)
        self.target_z = 0.07       # This is linear x in world system
        self.z_threshold = 0.02   # 2cm lateral tolerance
        self.x_threshold = 0.02   # ~5 degree tolerance
        self.lin_speed = 0.04      # 4cm/s
        self.ang_speed = 0.2      # rad/s

        # 0 = aligning to target, 1 = turning 90, 2 = straightening -90
        self.rotation_phase = 0
        self.turn_90_direction = 1

        self.last_marker_x = None
        self.last_marker_time = None
        self.zero_crossings = []
        self.period_samples = []
        self.period = None
        self.shots_fired = 0

    def drive_callback(self):
        if self.state == 'rotating':
            yaw_traveled = self._angle_diff(self.current_yaw, self.start_yaw)

            if abs(yaw_traveled) >= abs(self.angle_to_turn):
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
                    self.get_logger().info("Tracking target")
                    self.state = 'tracking'
                    self.rotation_phase = 0
                
            else:
                cmd = Twist()
                cmd.angular.z = -self.ang_speed if self.angle_to_turn > 0 else self.ang_speed
                self.cmd_pub.publish(cmd)

        elif self.state == 'driving':
            dist_traveled = math.sqrt(
                (self.current_x - self.start_x)**2 +
                (self.current_y - self.start_y)**2
            )
            if dist_traveled >= self.distance_to_travel:
                self.rotation_phase = 2
                self.angle_to_turn = -self.turn_90_direction * math.pi / 2
                self.state = 'rotating'
                self.get_logger().info("Aligning")
                self.start_yaw = self.current_yaw
            
            else:
                cmd = Twist()
                cmd.linear.x = self.lin_speed
                self.cmd_pub.publish(cmd)

        elif self.state == 'tracking':
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)


    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        #Quart to Yaw(just a formula)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def marker_callback(self, msg):
        marker_id = int(msg.orientation.w)


        if marker_id == 0 and self.state == 'tracking':
            self.task_b(msg)
            return

        rvec_yaw = msg.orientation.y
        
        if marker_id != 1:
            return
        
        if self.state != 'idle':
            return
        
        marker_x = msg.position.x
        marker_z = msg.position.z
        #Ensure that we are not too far away
        if marker_z > 0.5: 
            return

        if abs(marker_x) > 0.3:
            return
        
        
        err_x = marker_x - self.target_x

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

    def task_b(self,msg):
        marker_x = msg.position.x
        now = self.get_clock().now().nanoseconds / 1e9

        if self.last_marker_x is not None:

            #Detect a zero crossing
            if (self.last_marker_x * marker_x < 0): 
                if not self.zero_crossings or (now - self.zero_crossings[-1] > 0.1):
                    self.zero_crossings.append(now)
                

            if len(self.zero_crossings) >= 3:
                self.period = self.zero_crossings[-1] - self.zero_crossings[-3]
                current_cycle_period = self.zero_crossings[-1] - self.zero_crossings[-3]
                self.period_samples.append(current_cycle_period)
            
            if len(self.period_samples) == 3:
                self.period = sum(self.period_samples) / len(self.period_samples)
            
            if (self.last_marker_x * marker_x < 0):
                if self.period is not None and self.period > 0 and self.shots_fired < 3:
                    self.get_logger().info(f"!!! SHOOTING !!! (Target at Zero, Shot {self.shots_fired + 1})")
                    self.shots_fired += 1
            
                elif (self.shots_fired == 3):
                    self.stop_robot()
                

        self.last_marker_x = marker_x
        self.last_marker_time = now


    
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


def main(args=None):
    rclpy.init(args=args)
    node = Task_B_Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Manual Shutdown")
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()