import mujoco
import mujoco.viewer
import imageio
import time
import numpy as np
import matplotlib.pyplot as plt
from ElbowControl import ElbowMuscleBrain

# Env generation
env = ElbowMuscleBrain("M1", kp=25.0, kd=3.0, xml_path="myoelbow_1dof6muscles_1dofexo.xml")

# Start MuJoCo Viewer
print("MuJoCo Viewer Starts...")
viewer = mujoco.viewer.launch_passive(env.model, env.data, show_left_ui=False, show_right_ui=False)
renderer = mujoco.Renderer(env.model, height=480, width=640)
frames = []

# Camera position/rotation
viewer.cam.azimuth = 0  
viewer.cam.elevation = 0  
viewer.cam.distance = 2 
viewer.cam.lookat[1] = 0
viewer.cam.lookat[2] = 1.0

cam = mujoco.MjvCamera()
cam.azimuth = 0  
cam.elevation = -20   
cam.distance = 2       
cam.lookat[:] = np.array([0.0, -0.5, 1.4]) 

MAX_STEPS = 400 * 10
step_count = 0

# Info list
infos = []

# flexion-extension simulation
while viewer.is_running() and step_count < MAX_STEPS:
    # env step process
    info = env.step()
    infos.append(info)  # save info

    renderer.update_scene(env.data, camera=cam)
    pixels = renderer.render()
    frames.append(np.copy(pixels))  # save frame

    step_count += 1

    # MuJoCo Viewer update
    viewer.sync()

    # speed control
    time.sleep(0.01)

env.close()
renderer.close()
print("MuJoCo Viewer ended.")

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
