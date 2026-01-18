import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
import numpy as np


class Upperlimb_1DOF:
    """API class for controlling the upper limb exoskeleton"""

    def __init__(self, node=None):
        self.node = node
        self.rep_count = 0
        self.freq = 60
        self.velocity_cmd = 0.0
        self.DELTA_TIME = 0.02  # Control period (20ms)

    def init(self, rep_count=20, freq=60):
        """Initialize the upperlimb control parameters"""
        self.rep_count = rep_count
        self.freq = freq
        if self.node:
            self.node.get_logger().info(f'Upperlimb_1DOF initialized: rep_count={rep_count}, freq={freq}')

    def get_target_angle(self):
        """Get the current target angle in degrees"""
        if self.node:
            return np.rad2deg(self.node.target_th)
        return 0.0

    def set_velocity(self, velocity):
        """Set the velocity command"""
        self.velocity_cmd = velocity

    def get_velocity_cmd(self):
        """Get the current velocity command"""
        return self.velocity_cmd

    def get_velocity(self):
        """Get the current sensor velocity"""
        if self.node:
            return self.node.sensor_velocity
        return 0.0

    def get_previous_velocity(self):
        """Get the previous sensor velocity"""
        if self.node:
            return self.node.previous_velocity
        return 0.0

    def set_previous_velocity(self, velocity):
        """Set the previous sensor velocity"""
        if self.node:
            self.node.previous_velocity = velocity

    def get_force(self):
        """Get the current sensor force"""
        if self.node:
            return self.node.sensor_force
        return 0.0

    def get_position(self):
        """Get the current sensor position"""
        if self.node:
            return self.node.sensor_position
        return 0.0

    def get_moment(self):
        """Get the moment of inertia (관성 계수)"""
        return 0.0  # Default value, can be configured

    def get_spring(self):
        """Get the spring coefficient (spring 계수)"""
        return 0.0  # Default value, can be configured

    def get_damping(self):
        """Get the damping coefficient (damping 계수)"""
        return 100.0  # Default value, same as in elbow_vel_cmd control_logic

    @property
    def target_th(self):
        """Get the target angle in radians"""
        if self.node:
            return self.node.target_th
        return 0.0


class ElbowVelCmdNode(Node):
    def __init__(self):
        super().__init__('elbow_vel_cmd_node')

        # Create subscriber for vel_cmd
        self.status_subscriber = self.create_subscription(
            Float64MultiArray,
            'status',
            self.status_callback,
            10
        )

        # Publisher for commands
        self.vel_cmd_publisher = self.create_publisher(
            Float64,
            'vel_cmd',
            10
        )

        self.torque_cmd_publisher = self.create_publisher(
            Float64,
            'torque_cmd',
            10
        )

        self.position_cmd_publisher = self.create_publisher(
            Float64,
            'position_cmd',
            10
        )

        # Control parameters (same as ElbowMuscleBrain)
        self.current_time = 0.0
        self.switch_interval = 2.0
        self.target_th = np.deg2rad(130)
        self.ctrl_period = 0.02

        self.sensor_position = 0.0 # modified_1208
        self.sensor_velocity = 0.0 # modified_1208
        self.sensor_force = 0.0 # modified_1208
        self.previous_velocity = 0.0 # new_1208
        self.target_ang = 0.0 # new_1208

        # Initialize Upperlimb_1DOF API and load task
        self.upperlimb = Upperlimb_1DOF(node=self)
        self._load_task_module()

        # Create timer for periodic velocity command publishing
        self.timer = self.create_timer(self.ctrl_period, self.control_loop)

        self.get_logger().info('Elbow Vel Cmd Publisher Node started with task integration')

    def _load_task_module(self):
        """Load task module and call its setup function"""
        try:
            from . import task
            self.task_module = task
            # Set the upperlimb instance for task module
            task.upperlimb = self.upperlimb
            # Call setup function
            task.setup()
            self.get_logger().info('task module loaded and setup() called successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load task module: {e}')
            self.task_module = None

    def _update_desired_angle(self):
        # Motion Task - same logic as in ElbowMuscleBrain
        if self.current_time % (2 * self.switch_interval) < self.switch_interval:
            self.target_th = np.deg2rad(130)
        else:
            self.target_th = np.deg2rad(5)

    def control_logic(self):
        
        DELTA_TIME = self.ctrl_period # new_1208
        m = 0 # new_1208
        c = 100 # new_1208
        k = 0 # new_1208

        # ################# Task 1 (Stiff) #################
        # if self.target_th > np.deg2rad(90):  # Task에 따라서(flexion, extension) 토크를 주는 간단한 동작
        #     velocity = -1.2  # flexion
        # else:
        #     velocity = 1.2  # extension

        # ################# Task 2 (Compliant) new_1208 #################
        # delta_velocity = self.sensor_velocity - self.previous_velocity
        # acceleration = delta_velocity / DELTA_TIME
        # acceleration = max(min(acceleration, 500.0), -500.0)
        # self.previous_velocity = self.sensor_velocity
        # delta_force = self.sensor_force * 1000 / 9.8   # dimension change

        # if self.target_th > np.deg2rad(90):  # Task에 따라서(flexion, extension) 토크를 주는 간단한 동작
        #     velocity = -0.6 + (delta_force - m * acceleration - k * (self.sensor_position - self.target_th)) / c # flexion
        # else:
        #     velocity = 1.2  # extension

        ################# Task 3 (Resistive) new_1208   #################
        delta_velocity = self.sensor_velocity - self.previous_velocity
        acceleration = delta_velocity / DELTA_TIME
        acceleration = max(min(acceleration, 500.0), -500.0)
        self.previous_velocity = self.sensor_velocity
        delta_force = self.sensor_force * 1000 / 9.8    # dimension change

        if self.target_th > np.deg2rad(90):  # Task에 따라서(flexion, extension) 토크를 주는 간단한 동작
            velocity = 0.2 + (delta_force - m * acceleration - k * (self.sensor_position - self.target_th)) / c # flexion
        else:
            velocity = 1.2  # extension



        return velocity

    def status_callback(self, msg):
        # Update target angle from status message
        self.current_time, self.target_ang, self.sensor_position, self.sensor_velocity, self.sensor_force = msg.data # modified_1210
        self.get_logger().info(f'Received target angle: {np.rad2deg(self.target_ang):.1f}°, position: {self.sensor_position:.2f}, velocity: {self.sensor_velocity:.2f}, force: {self.sensor_force:.2f}')

    def control_loop(self):
        # Update desired angle based on time
        self._update_desired_angle()
        # self.target_th = self.target_ang # new_1208 temporary

        if self.task_module: # If task module is loaded
            try:
                self.task_module.loop()
                # Get velocity command from upperlimb API
                vel_cmd = self.upperlimb.get_velocity_cmd()
                self.previous_velocity = self.sensor_velocity # new_1208
            except Exception as e:
                self.get_logger().error(f'Error in task.loop(): {e}')
                # Fallback to original control logic
                vel_cmd = self.control_logic()
        else:
            # Fallback to original control logic if task not loaded
            vel_cmd = self.control_logic()

        # Create and publish message
        msg = Float64()
        msg.data = vel_cmd

        self.vel_cmd_publisher.publish(msg)

        # Update time
        # self.current_time += self.ctrl_period

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
