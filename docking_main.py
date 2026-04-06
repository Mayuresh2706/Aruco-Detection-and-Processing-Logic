import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        
        self.state = "SEARCHING"
        self.detection_count = 0
    
        self.exp_active_pub = self.create_publisher(Bool, '/explorer_active', 10)
        self.task_active_pub = self.create_publisher(Bool, '/task_a_active', 10)
        
        self.create_subscription(Pose, 'target_3d', self.aruco_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def aruco_callback(self, msg):
        marker_id = int(msg.orientation.w)
        if self.state == "SEARCHING" and marker_id == 1:
            self.detection_count += 1
            if self.detection_count >= 10:
                self.get_logger().info("!!! TARGET LOCKED !!! Switching to Nav2 Approach.")
                self.start_approach(msg)

    def start_approach(self, msg):
        self.state = "APPROACHING"
        self.exp_active_pub.publish(Bool(data=False))
        
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'base_link' 
        goal.pose.pose.position.x = msg.position.x
        goal.pose.pose.position.z = max(0.0, msg.position.z - 0.4) # Stop 40cm early
        goal.pose.pose.orientation.w = 1.0

        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal).add_done_callback(self.nav_done)

    def nav_done(self, future):
        self.get_logger().info("Approach Finished. Handing to Task A.")
        self.state = "DOCKING"
        self.task_active_pub.publish(Bool(data=True)) 
        
        self.create_timer(20.0, self.reset_to_explore)

    def reset_to_explore(self):
        self.get_logger().info("Mission Cycle Complete. Resuming Search.")
        self.task_active_pub.publish(Bool(data=False))
        self.exp_active_pub.publish(Bool(data=True))
        self.state = "SEARCHING"
        self.detection_count = 0

def main():
    rclpy.init()
    rclpy.spin(MissionManager())