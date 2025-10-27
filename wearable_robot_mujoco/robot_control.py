#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class UpperLimbNode(Node):
    def __init__(self):
        super().__init__('wearable_robot_upper_limb_controller')
        
        # loop time
        self.DELTA_TIME = 0.0125  # 80Hz

        # running status
        self.selected_task = 0
        self.is_running = False
        self.is_sending_upper_limb_state = False
        
        # Parameters
        self.declare_parameter('robot_weight', 200.0)  # g
        self.declare_parameter('l1', 0.18)  # m
        self.declare_parameter('l2', 0.30)  # m
        #joe
        self.declare_parameter('repeat', 39) # start from zero, total 20 flexion and extention
        #self.declare_parameter('repeat', 5) # start from zero, total 3 flexion and extention
        self.declare_parameter('delay_time', 1000)  # ms
        
        # Control parameters
        self.m = 0.5
        self.c = 10
        self.k = 0
        self.k2 = 1
        self.a = -90/2
        
        # State variables
        self.current_time = 0.0
        self.direction = -1
        self.flag = 2
        self.r = -1
        self.start = 0
        self.theta = 0.0

        # Initialize Dynamixel
        self.previous_velocity = 0.0
       
        # Publishers
        self.upper_limb_state_pub = self.create_publisher(UpperLimbState, 'upper_limb_status', 10)
        
        # Subscribers
        self.loadcell_value = None
        self.loadcell_sub = self.create_subscription(Float32, 'load_cell_weight', self.loadcell_callback, 10)

        # Services
        self.command_service = self.create_service(UpperLimbCommand, 'upper_limb_command', self.handle_command)
        
        # Timer for control loop (80Hz)
        self.timer = self.create_timer(self.DELTA_TIME, self.control_loop)

        # Motor controller
        self.dxl = DynamixelController(self.get_logger())

        # Reste hardware errors
        dynamixel_reboot_tries = 0
        while(self.dxl.get_hardware_error_status() != 0 and dynamixel_reboot_tries < 5):
            self.get_logger().info("Try to reset hardware errors of dynamixel motor")
            self.dxl.reboot()
            time.sleep(3)
            dynamixel_reboot_tries += 1

        self.dxl.reboot()
        self.reset_motor_position()


    def __del__(self):
        self.stop()
        self.dxl.close()
        self.get_logger().info("__del__ Port closed")

    def reset_motor_position(self):
        self.dxl.ping() # ping to check if the motor is connected

        self.dxl.disable_torque() # Disable Dynamixel Torque
        self.dxl.set_operating_mode(OP_CURRENT_BASED_POSITION) # Set operating mode to current-based position for repositioning
        self.dxl.enable_torque() # Enable Dynamixel Torque
 
        self.dxl.write_control_table(ADDR_PROFILE_VELOCITY, 30)
        self.dxl.set_goal_position(170.0)

        # reposition motor to 170 degree
        current_position = self.dxl.get_present_position()
        while abs(current_position - 170.0) > DXL_MOVING_STATUS_THRESHOLD: # Wait until the 170 goal position is reached
            current_position = self.dxl.get_present_position()
            time.sleep(0.1)

        time.sleep(1.0)
        self.dxl.write_control_table(ADDR_PROFILE_VELOCITY, 60)
        time.sleep(2.0)
        self.get_logger().info("Motor reposition is complete")

        #self.dxl.disable_torque() # Disable Dynamixel Torque
        #self.dxl.set_operating_mode(OP_VELOCITY) # Set operating mode to velocity
        #self.dxl.enable_torque() # Enable Dynamixel Torque

    #  def start(self):
    #      self.timer.reset()
    #      self.timer.start()
    #      self.dxl.enable_torque()
    #      self.get_logger().info('Starting...')
    
    def stop(self):
        self.timer.cancel()
        self.dxl.disable_torque()
        self.get_logger().info('Stopping...')

    def control_loop(self):
        if not self.is_running:
            self.dxl.set_goal_velocity(0)
            #self.dxl.disable_torque()
            if self.is_sending_upper_limb_state:
                current_position = self.dxl.get_present_position()
                loadcell_value = self.read_loadcell()  # Implement according to your HX711 interface
                current = self.dxl.get_present_current()
                self.publish_state(current_position, loadcell_value, current)
            return

        if self.r > self.get_parameter('repeat').value:
            self.get_logger().info(f'Task {self.selected_task} completed successfully')
            self.dxl.set_goal_velocity(0)
            #  self.stop()
            self.is_running = False
            self.is_sending_upper_limb_state = False
            time.sleep(1.0)
            self.dxl.disable_torque()
            return

        # Read sensors
        current_position = self.dxl.get_present_position()
        current_velocity = self.dxl.get_present_velocity() * 6  # 6 From KNU's firmware
        loadcell_value = self.read_loadcell()  # Implement according to your HX711 interface
        current = self.dxl.get_present_current()
        #  self.get_logger().info(f'direction: {self.direction}, velocity: {current_velocity}, positiion: {current_position}, theta: {self.theta}, current: {current}')
       
        # Calculate control
        self.current_time += 12.5
        
        if current_position >= 165 and self.direction == -1:
            self.direction = 1
            self.r += 1
            self.current_time = 0
        elif current_position <= 75 and self.direction == 1:
            self.direction = -1
            self.r += 1
            self.current_time = 0
            
        if self.direction > 0:
            self.theta = self.a * self.current_time / 1000 + 165
        else:
            self.theta = -self.a * self.current_time / 1000 + 75
            
        # Impedance control
        delta_velocity = current_velocity - self.previous_velocity
        acceleration = delta_velocity / self.DELTA_TIME
        acceleration = max(min(acceleration, 500.0), -500.0) # TODO: Need to check the limit value
        self.get_logger().info(f'acceleration: {acceleration}, velocity: {current_velocity}, delta_velocity: {delta_velocity}')
        self.previous_velocity = current_velocity

        delta_force = loadcell_value + 50 - 300
        velocityT = self.calculate_velocity(current_position, acceleration, delta_force, self.theta)
        self.dxl.set_goal_velocity(velocityT / 6) # 6 From KNU's firmware
        #  self.dxl.set_goal_velocity(velocityT)

        self.publish_state(current_position, loadcell_value, current)
    
    #joe Modify the speed on the flextion position at the Task 1    
    def calculate_velocity(self, position, acceleration, delta_force, theta):
        velocity = self.a * self.direction + (delta_force - self.m * acceleration - self.k * (position - theta)) / self.c
        if self.selected_task == 1 and self.direction == 1: # TASK 1 with Flextion
            velocity = -20 * self.direction + (delta_force - self.m * acceleration - self.k * (position - theta)) / self.c
        if self.selected_task == 2:  # TAhhSK 2 with Flextion
            velocity = -40 * self.direction + (delta_force - self.m * acceleration - self.k * (position - theta)) / self.c

        return max(min(velocity, 70.0), -70.0)
        
    def publish_state(self, position, loadcell_value, current):
        state = 'Flexion' if self.direction == 1 else 'Extension'

        state_msg = UpperLimbState()
        state_msg.state = state
        if self.r == -1:
            state_msg.repeat = 0
        else:
            state_msg.repeat = self.r
        state_msg.weight = loadcell_value
        state_msg.angle = position
        state_msg.current = float(current)

        # State : Flexion , Repeat : 6 , Weight : 475.40g , Elbow Angle : 125.22° , Current : -304.0000mA
        # State : Extension , Repeat : 5 , Weight : -92.22g , Elbow Angle : 151.98° , Current : 46.0000mA
        self.get_logger().info(f'State : {state}, Repeat : {self.r}, Weight : {loadcell_value:.2f}g, Elbow Angle : {position:.2f}°, Current : {current:.4f}mA')

        self.upper_limb_state_pub.publish(state_msg)
        
    def loadcell_callback(self, msg):
        self.loadcell_value = msg.data

    def read_loadcell(self):
        if self.loadcell_value is not None:
            return self.loadcell_value
        return 0.0  # TODO need error handling

    def handle_command(self, request, response):
        commands = {UpperLimbCommand.Request.COMMAND_START_STOP: 'COMMAND_START_STOP',
                    UpperLimbCommand.Request.COMMAND_RESET: 'COMMAND_RESET'}

        if request.command > UpperLimbCommand.Request.COMMAND_RESET:
            self.get_logger().error(f"Invalid command: {request.command}")
            response.success = False
            response.message = "Invalid command"
            return response
        self.get_logger().info(f"Received command: Task {request.task}, {commands[request.command]}")

        self.selected_task = request.task

        # Set control parameters by task
        if request.task == UpperLimbCommand.Request.TASK_1:
            self.m = 0.1
            self.c = 50
        elif request.task == UpperLimbCommand.Request.TASK_2:
            self.m = 0.1
            self.c = 15
        elif request.task == UpperLimbCommand.Request.TASK_3:
            self.c = 10000

        # Set control running status and reset variables
        if request.command == UpperLimbCommand.Request.COMMAND_START_STOP:
            self.is_running = not self.is_running
            if self.is_running:
                self.dxl.disable_torque() # Disable Dynamixel Torque
                self.dxl.set_operating_mode(OP_VELOCITY) # Set operating mode to velocity
                self.dxl.enable_torque()
        elif request.command == UpperLimbCommand.Request.COMMAND_RESET:
            if self.is_running:
                self.is_running = False
                time.sleep(2.0)
            self.is_sending_upper_limb_state = True
            self.current_time = 0.0
            self.direction = -1
            self.flag = 2
            self.r = -1
            self.start = 0
            self.theta = 0.0
            self.reset_motor_position()
        
        response.success = True
        response.message = "Command received"
        return response

