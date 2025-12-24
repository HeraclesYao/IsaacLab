# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to run teleoperation with ROS2 Bridge device for Isaac Lab environments."""

import argparse
import time
from collections.abc import Callable

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Teleoperation with ROS2 Bridge device for Isaac Lab environments.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default="Isaac-PickPlace-G1-InspireFTP-Abs-v0", help="Name of the task.")
parser.add_argument("--device_name", type=str, default="ros2_bridge_gloves", help="ROS2 Bridge device name in configuration.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch the simulator
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch
import traceback

from isaaclab.envs import ManagerBasedEnv
from isaaclab.devices import create_teleop_device


def main():
    """Main function for teleoperation with ROS2 Bridge device."""
    # Import task configuration
    try:
        if args_cli.task == "Isaac-PickPlace-G1-InspireFTP-Abs-v0":
            from isaaclab_tasks.manager_based.manipulation.pick_place.pickplace_unitree_g1_inspire_hand_env_cfg import (
                PickPlaceG1InspireFTPEnvCfg,
            )
            
            env_cfg = PickPlaceG1InspireFTPEnvCfg()
            env_cfg.scene.num_envs = args_cli.num_envs
            
            # Ensure ROS2 Bridge device is configured
            if args_cli.device_name not in env_cfg.teleop_devices.devices:
                print(f"Device '{args_cli.device_name}' not found in configuration.")
                print("Available devices:", list(env_cfg.teleop_devices.devices.keys()))
                return
                
    except ImportError as e:
        print(f"Task configuration not found: {e}")
        print("Using default environment.")
        return

    # Create environment
    env = ManagerBasedEnv(env_cfg)

    # Define teleoperation callbacks
    def reset_callback():
        """Reset the simulation."""
        print("Resetting simulation...")
        env.reset()

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

    # Create ROS2 Bridge teleoperation device
    try:
        teleop_device = create_teleop_device(
            args_cli.device_name,
            env_cfg.teleop_devices.devices,
            teleoperation_callbacks
        )
        print(f"Created ROS2 Bridge teleoperation device: {teleop_device}")
    except Exception as e:
        print(f"Failed to create ROS2 Bridge device: {e}")
        print("Available devices:", list(env_cfg.teleop_devices.devices.keys()))
        return

    # Simulation loop
    sim_dt = env.step_dt
    episode_count = 0
    episode_steps = 0
    max_episode_steps = 1000

    print("Starting ROS2 Bridge teleoperation loop...")
    print("Make sure your ROS2 hand tracking nodes are running!")
    print("Waiting for joint data on topics:")
    print(f"  - Left hand: {teleop_device.cfg.left_hand_topic}")
    print(f"  - Right hand: {teleop_device.cfg.right_hand_topic}")
    print("")
    print("IMPORTANT: Make sure Isaac Sim ROS2 Bridge extension is enabled!")
    print("You can enable it in Isaac Sim GUI: Extensions -> ROS2 Bridge")

    while simulation_app.is_running():
        try:
            # Get commands from ROS2 Bridge device
            commands = teleop_device.advance()
            
            if commands is not None:
                # Step the environment
                obs, reward, terminated, truncated, info = env.step(commands)
                episode_steps += 1
                
                # Reset if episode is done
                if terminated or truncated or episode_steps >= max_episode_steps:
                    env.reset()
                    episode_count += 1
                    episode_steps = 0
                    print(f"Episode {episode_count} completed")
            
            # Small delay to prevent excessive CPU usage
            time.sleep(0.001)
            
        except KeyboardInterrupt:
            print("Interrupted by user")
            break
        except Exception as e:
            print(f"Error in simulation loop: {e}")
            traceback.print_exc()
            break

    # Cleanup
    env.close()
    print("ROS2 Bridge teleoperation completed.")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Error in main: {e}")
        traceback.print_exc()
    finally:
        simulation_app.close()