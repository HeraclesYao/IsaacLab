# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Simplified script to verify ROS2 communication with Unitree G1 robot."""

import argparse
import time
import traceback

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="ROS2 communication verification for hand or robot.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default="Isaac-PickPlace-G1-InspireFTP-Abs-v0", help="Name of the task.")
parser.add_argument("--device_name", type=str, default="ros2_gloves", help="ROS2 device name in configuration.")  # 修改设备名称

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch the simulator
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch

from isaaclab.envs import ManagerBasedEnv
from isaaclab.devices import create_teleop_device


def main():
    """Main function for ROS2 communication verification."""
    print("=" * 60)
    print("ROS2 Communication Verification for Unitree G1 Robot")
    print("=" * 60)
    
    # Import the simplified configuration
    try:
        if args_cli.task == "Isaac-PickPlace-G1-InspireFTP-Abs-v0":
            from isaaclab_tasks.manager_based.manipulation.pick_place.pickplace_ros2_verify_env_cfg import PickPlaceG1ROS2VerifyEnvCfg
            env_cfg = PickPlaceG1ROS2VerifyEnvCfg()
        elif args_cli.task == "Isaac-Rokae-v0":
            from isaaclab_tasks.manager_based.rokae.rokae_env_cfg import RokaeEnvCfg
            env_cfg = RokaeEnvCfg()
        elif args_cli.task == "Isaac-Linkerhand-v0":
            from isaaclab_tasks.manager_based.linkerhand.linkerhand_env_cfg import LinkerhandEnvCfg
            env_cfg = LinkerhandEnvCfg()
        else:
            raise ValueError(f"Unsupported task: {args_cli.task}")
        
        env_cfg.scene.num_envs = args_cli.num_envs
        print("✓ Loaded simplified configuration")
    except ImportError as e:
        print(f"✗ Failed to load configuration: {e}")
        traceback.print_exc()
        return

    # Create environment
    try:
        env = ManagerBasedEnv(env_cfg)
        print("✓ Created environment")
    except Exception as e:
        print(f"✗ Failed to create environment: {e}")
        traceback.print_exc()
        return

    # Define teleoperation callbacks
    def reset_callback():
        """Reset the simulation."""
        print("Resetting simulation...")
        env.reset()
        print("✓ Simulation reset")

    def start_callback():
        """Start teleoperation."""
        print("Starting teleoperation...")

    def stop_callback():
        """Stop teleoperation."""
        print("Stopping teleoperation...")

    teleoperation_callbacks = {
        "RESET": reset_callback,
        "START": start_callback,
        "STOP": stop_callback,
    }

    # Create ROS2 teleoperation device
    try:
        teleop_device = create_teleop_device(
            args_cli.device_name,
            env_cfg.teleop_devices.devices,
            teleoperation_callbacks
        )
        print(f"✓ Created ROS2 teleoperation device: {teleop_device}")
    except Exception as e:
        print(f"✗ Failed to create ROS2 device: {e}")
        print("Available devices:", list(env_cfg.teleop_devices.devices.keys()))
        traceback.print_exc()
        return

    # Reset environment
    env.reset()
    teleop_device.reset()

    # Get the idle action from configuration
    idle_action = env_cfg.idle_action.to(env.device).unsqueeze(0)  # Shape: (1, action_dim)

    # Simulation loop
    sim_dt = env.step_dt
    episode_count = 0
    episode_steps = 0
    max_episode_steps = 5000  # Very long episode for testing

    print("\n" + "=" * 60)
    print("Starting ROS2 Communication Verification")
    print("=" * 60)
    print("Make sure your ROS2 hand tracking nodes are running!")
    print("Waiting for joint data on topics:")
    print(f"  - Left hand: {teleop_device.cfg.left_hand_topic}")
    print(f"  - Right hand: {teleop_device.cfg.right_hand_topic}")
    print("Press Ctrl+C to stop")
    print("=" * 60)

    last_message_time = time.time()
    message_count = {"left": 0, "right": 0}

    while simulation_app.is_running():
        try:
            # Get commands from ROS2 device
            commands = teleop_device.advance()
            
            # Use idle action if no commands received
            if commands is None:
                commands = idle_action
            
            # Apply commands to environment
            # In Isaac Lab's new ManagerBasedEnv, step() returns only 2 values: (obs_buf, extras)
            obs, extras = env.step(commands)
            episode_steps += 1
            
            # Count messages (check if we actually received ROS2 data)
            if commands is not idle_action:
                if hasattr(teleop_device, '_left_hand_data') and teleop_device._left_hand_data is not None:
                    message_count["left"] += 1
                if hasattr(teleop_device, '_right_hand_data') and teleop_device._right_hand_data is not None:
                    message_count["right"] += 1
            
            # Print status every 2 seconds
            current_time = time.time()
            if current_time - last_message_time > 2.0:
                status_msg = f"Status: Left={message_count['left']} messages, Right={message_count['right']} messages, Steps={episode_steps}"
                if commands is idle_action:
                    status_msg += " (Using idle action - waiting for ROS2 data)"
                print(status_msg)
                last_message_time = current_time
            
            # Reset if episode is done (unlikely in verification mode)
            # Check for episode completion from extras
            # terminated = extras.get("log", {}).get("terminated", False)
            # truncated = episode_steps >= max_episode_steps
            # if terminated or truncated:
            #     env.reset()
            #     episode_count += 1
            #     episode_steps = 0
            #     print(f"Episode {episode_count} completed")
        
        except KeyboardInterrupt:
            print("\nInterrupted by user")
            break
        except Exception as e:
            print(f"Error in simulation loop: {e}")
            traceback.print_exc()
            break

    # Cleanup
    print("\nCleaning up...")
    env.close()
    print("ROS2 communication verification completed.")
    print(f"Final message count: Left={message_count['left']}, Right={message_count['right']}")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Error in main: {e}")
        traceback.print_exc()
    finally:
        simulation_app.close()