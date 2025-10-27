import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np


class ElbowVelCmdNode(Node):
    def __init__(self):
        super().__init__('elbow_vel_cmd_node')
        
        # Create subscriber for vel_cmd
        self.status_subscriber = self.create_subscription(
            Float64,
            'target_angle',
            self.status_callback,
            10
        )

        # Publisher for vel_cmd
        self.vel_cmd_publisher = self.create_publisher(
            Float64,
            'vel_cmd',
            10
        )
        
        # Control parameters (same as ElbowMuscleBrain)
        self.current_time = 0.0
        self.switch_interval = 2.0
        self.target_th = np.deg2rad(130)
        self.ctrl_period = 0.02
        
        # Create timer for periodic velocity command publishing
        self.timer = self.create_timer(self.ctrl_period, self.control_loop)
        
        self.get_logger().info('Elbow Vel Cmd Publisher Node started')

    def _update_desired_angle(self):
        # Motion Task - same logic as in ElbowMuscleBrain
        if self.current_time % (2 * self.switch_interval) < self.switch_interval:
            self.target_th = np.deg2rad(130)
        else:
            self.target_th = np.deg2rad(10)

    def control_logic(self):
        # Same logic as in ElbowMuscleBrain.control_logic()
        if self.target_th > np.deg2rad(90):  # Task에 따라서(flexion, extension) 토크를 주는 간단한 동작
            velocity = -1.2
        else:
            velocity = 1.2
        
        return velocity

    def status_callback(self, msg):
        # Update target angle from status message
        self.target_th = msg.data
        self.get_logger().info(f'Received target_angle: {np.rad2deg(self.target_th):.1f}°')

    def control_loop(self):
        # Update desired angle based on time
        #self._update_desired_angle()
        
        # Get velocity command
        vel_cmd = self.control_logic()
        
        # Create and publish message
        msg = Float64()
        msg.data = vel_cmd
        
        self.vel_cmd_publisher.publish(msg)
        
        # Update time
        self.current_time += self.ctrl_period
        
        # Optional: Log the command
        self.get_logger().info(f'Publishing vel_cmd: {vel_cmd:.2f}, target_angle: {np.rad2deg(self.target_th):.1f}°')


def main(args=None):
    rclpy.init(args=args)
    
    vel_cmd_publisher_node = ElbowVelCmdNode()
    
    try:
        rclpy.spin(vel_cmd_publisher_node)
    except KeyboardInterrupt:
        vel_cmd_publisher_node.get_logger().info('Elbow Vel Cmd Publisher interrupted by user')
    finally:
        vel_cmd_publisher_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
