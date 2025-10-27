import rclpy
from rclpy.node import Node
import mujoco
import mujoco.viewer
import imageio
import time
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
from elbow_control import ElbowMuscleBrain
from ament_index_python.packages import get_package_share_directory

import rclpy
from std_msgs.msg import Float64
#
class SimulationNode(Node):
    def __init__(self):
        super().__init__('simulation_node')

        # Create subscriber for vel_cmd
        self.vel_cmd_subscriber = self.create_subscription(
            Float64,
            'vel_cmd',
            self.vel_cmd_callback,
            10
        )

        # Create publisher for status (target_angle)
        self.status_publisher = self.create_publisher(
            Float64,
            'target_angle',
            10
        )

        # Get XML path from ROS2 package
        share_dir = get_package_share_directory("wearable_robot_mujoco")
        xml_path_ = os.path.join(share_dir, "myoelbow_1dof6muscles_1dofexo.xml")
        self.env = ElbowMuscleBrain("M1", kp=25.0, kd=3.0, xml_path=xml_path_)

        # Start MuJoCo Viewer
        self.get_logger().info("MuJoCo Viewer Starts...")
        self.viewer = mujoco.viewer.launch_passive(self.env.model, self.env.data, show_left_ui=False, show_right_ui=False)
        self.renderer = mujoco.Renderer(self.env.model, height=480, width=640)
        self.frames = []

        # Camera position/rotation
        self.viewer.cam.azimuth = 0  
        self.viewer.cam.elevation = 0  
        self.viewer.cam.distance = 2 
        self.viewer.cam.lookat[1] = 0
        self.viewer.cam.lookat[2] = 1.0

        self.cam = mujoco.MjvCamera()
        self.cam.azimuth = 0  
        self.cam.elevation = -20   
        self.cam.distance = 2       
        self.cam.lookat[:] = np.array([0.0, -0.5, 1.4])

        self.MAX_STEPS = 400 * 10
        self.step_count = 0
        self.infos = []

        # Create timer for simulation loop
        self.timer = self.create_timer(0.01, self.simulation_step)

    def vel_cmd_callback(self, msg):
        self.get_logger().info(f"Received vel_cmd: {msg.data}")
        self.vel_cmd = msg.data
        self.env.set_velocity_command(self.vel_cmd)

    def simulation_step(self):
        if not self.viewer.is_running() or self.step_count >= self.MAX_STEPS:
            self.cleanup_and_shutdown()
            return

        # Env step process
        info = self.env.step()
        self.infos.append(info)

        self.renderer.update_scene(self.env.data, camera=self.cam)
        pixels = self.renderer.render()
        self.frames.append(np.copy(pixels))

        self.step_count += 1

        # MuJoCo Viewer update
        self.viewer.sync()

        # Publish target angle and current angle
        self.publish_status(info)

    def publish_status(self, info):
        target_angle = info["target_angle"]
        msg = Float64()
        msg.data = target_angle
        self.status_publisher.publish(msg)

    def cleanup_and_shutdown(self):
        self.env.close()
        self.renderer.close()
        self.get_logger().info("MuJoCo Viewer ended.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    simulation_node = SimulationNode()
    
    try:
        rclpy.spin(simulation_node)
    except KeyboardInterrupt:
        simulation_node.get_logger().info('Simulation interrupted by user')
    finally:
        simulation_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

# # Save mp4
# imageio.mimsave("simulation.mp4", frames, fps=30)
# print("Video saved: simulation.mp4")

# # --- Drawing plots ---
# time_steps = list(range(len(infos)))

# # Extraction
# target_angles  = [info["target_angle"] for info in infos]
# current_angles = [info["current_angle"] for info in infos]
# torques        = [info["input"]         for info in infos]
# actuations     = [info["actuation"]     for info in infos]

# TRIlong_vals   = [info["TRIlong"]       for info in infos]
# TRIlat_vals    = [info["TRIlat"]        for info in infos]
# TRImed_vals    = [info["TRImed"]        for info in infos]
# BIClong_vals   = [info["BIClong"]       for info in infos]
# BICshort_vals  = [info["BICshort"]      for info in infos]
# BRA_vals       = [info["BRA"]           for info in infos]

# plt.figure(figsize=(12, 10))

# # 1. Target Angle vs. Current Angle
# plt.subplot(4, 1, 1)
# plt.plot(time_steps, target_angles, label="Target Angle", color='red')
# plt.plot(time_steps, current_angles, label="Current Angle", color='blue')
# plt.xlabel("Time Step")
# plt.ylabel("Angle (deg)")
# plt.title("Target vs Current Angle")
# plt.legend()
# plt.grid()

# # 2. Torque vs Actuation
# plt.subplot(4, 1, 2)
# plt.plot(time_steps, torques, label="Target Torque", color='black')
# plt.plot(time_steps, actuations, label="Actual Actuation", color='green')
# plt.xlabel("Time Step")
# plt.ylabel("Torque")
# plt.title("Torque Over Time")
# plt.legend()
# plt.grid()

# # 3. Triceps Activations
# plt.subplot(4, 1, 3)
# plt.plot(time_steps, TRIlong_vals, label="TRIlong", color='orange')
# plt.plot(time_steps, TRIlat_vals, label="TRIlat", color='tomato')
# plt.plot(time_steps, TRImed_vals, label="TRImed", color='gold')
# plt.xlabel("Time Step")
# plt.ylabel("Activation")
# plt.title("Triceps Muscle Activations")
# plt.ylim(0, 1)  # Y축 고정
# plt.legend()
# plt.grid()

# # 4. Biceps Activations
# plt.subplot(4, 1, 4)
# plt.plot(time_steps, BIClong_vals, label="BIClong", color='cyan')
# plt.plot(time_steps, BICshort_vals, label="BICshort", color='dodgerblue')
# plt.plot(time_steps, BRA_vals, label="BRA", color='blueviolet')
# plt.xlabel("Time Step")
# plt.ylabel("Activation")
# plt.title("Biceps Muscle Activations")
# plt.ylim(0, 1)  # Y축 고정
# plt.legend()
# plt.grid()

# plt.tight_layout()
# plt.show()
