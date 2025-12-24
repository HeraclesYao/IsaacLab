# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""ROS2 Bridge device for teleoperation using Isaac Sim's built-in ROS2 Bridge."""
from __future__ import annotations

import torch
import time
import omni.graph.core as og
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

from isaaclab.devices.device_base import DeviceBase, DeviceCfg
from isaaclab.devices.retargeter_base import RetargeterBase


class ROS2BridgeDevice(DeviceBase):
    """A ROS2 Bridge-based teleoperation device using Isaac Sim's built-in ROS2 Bridge.
    
    This device leverages Isaac Sim's official ROS2 Bridge to receive JointState messages
    and convert them into control commands for Isaac Lab environments.
    """

    def __init__(self, cfg: ROS2BridgeDeviceCfg, retargeters: Optional[List[RetargeterBase]] = None):
        """Initialize the ROS2 Bridge teleoperation device.
        
        Args:
            cfg: Configuration object for ROS2 Bridge device settings.
            retargeters: Optional list of retargeters for command transformation.
        """
        super().__init__(retargeters)
        
        self.cfg = cfg
        self._callbacks: Dict[str, Any] = {}
        
        # Store latest joint data
        self._left_hand_data: Optional[torch.Tensor] = None
        self._right_hand_data: Optional[torch.Tensor] = None
        self._last_message_time: Dict[str, float] = {
            "left": 0.0,
            "right": 0.0
        }
        
        # ROS2 graph nodes
        self._left_hand_node = None
        self._right_hand_node = None
        self._status_node = None
        
        # Initialize ROS2 Bridge
        self._init_ros2_bridge()

    def _init_ros2_bridge(self):
        """Initialize Isaac Sim's ROS2 Bridge using OmniGraph nodes."""
        try:
            # Method 1: Try to import the C++ extension module directly
            try:
                # Correct import for Isaac Sim 5.1.0 ROS2 Bridge
                from isaacsim.ros2.bridge import _ros2_bridge
                
                # Acquire the ROS2 Bridge interface
                self._ros2_bridge = _ros2_bridge.acquire_ros2_bridge_interface()
                
                # Check if bridge is available
                if self._ros2_bridge.get_startup_status() is False:
                    print("ROS2 Bridge startup failed")
                    return
                
                print("Isaac Sim ROS2 Bridge is available. Setting up OmniGraph nodes...")
                
                # Create OmniGraph nodes for ROS2 subscribers
                # This is the correct way to use ROS2 Bridge in Isaac Sim
                self._create_ros2_subscriber_nodes()
                
                print(f"ROS2 Bridge device initialized. Subscribed to:")
                print(f"  - Left hand: {self.cfg.left_hand_topic}")
                print(f"  - Right hand: {self.cfg.right_hand_topic}")
                
            except ImportError as e:
                # Method 2: Try using extension manager API
                try:
                    from isaacsim.core.utils.extensions import get_extension_manager
                    ext_manager = get_extension_manager()
                    if ext_manager.is_extension_enabled("isaacsim.ros2.bridge"):
                        # Load the extension plugin
                        import carb
                        carb.get_framework().load_plugins(
                            loaded_file_wildcards=["isaacsim.ros2.bridge.plugin"],
                            search_paths=[ext_manager.get_extension_path("isaacsim.ros2.bridge")]
                        )
                        
                        # Now try importing again
                        from isaacsim.ros2.bridge import _ros2_bridge
                        self._ros2_bridge = _ros2_bridge.acquire_ros2_bridge_interface()
                        
                        if self._ros2_bridge.get_startup_status() is False:
                            print("ROS2 Bridge startup failed")
                            return
                            
                        print("Isaac Sim ROS2 Bridge is available. Setting up OmniGraph nodes...")
                        
                        # Create OmniGraph nodes for ROS2 subscribers
                        self._create_ros2_subscriber_nodes()
                        
                        print(f"ROS2 Bridge device initialized. Subscribed to:")
                        print(f"  - Left hand: {self.cfg.left_hand_topic}")
                        print(f"  - Right hand: {self.cfg.right_hand_topic}")
                    else:
                        raise ImportError("ROS2 Bridge extension is not enabled")
                        
                except ImportError as e2:
                    print(f"Alternative import methods failed: {e2}")
                    raise ImportError("Failed to import ROS2 Bridge components using any method")
            
        except ImportError as e:
            print(f"Failed to import Isaac Sim ROS2 Bridge components: {e}")
            print("Please make sure ROS2 Bridge extension is enabled in Isaac Sim.")
            print("You can enable it in Isaac Sim GUI: Extensions -> ROS2 Bridge")
        except Exception as e:
            print(f"Failed to initialize ROS2 Bridge: {e}")

    def _create_ros2_subscriber_nodes(self):
        """Create OmniGraph nodes for ROS2 subscribers."""
        try:
            # Create left hand subscriber node - FIXED: use ROS2SubscribeJointState instead of ROS2Subscriber
            (graph, nodes, _, _) = og.Controller.edit(
                {"graph_path": "/ROS2BridgeDevice/LeftHandSubscriber", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("LeftHandSubscriber", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("LeftHandSubscriber.inputs:topicName", self.cfg.left_hand_topic),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "LeftHandSubscriber.inputs:execIn"),
                    ],
                },
            )
            self._left_hand_node = nodes[-1]
            
            # Create right hand subscriber node - FIXED: use ROS2SubscribeJointState instead of ROS2Subscriber
            (graph, nodes, _, _) = og.Controller.edit(
                {"graph_path": "/ROS2BridgeDevice/RightHandSubscriber", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("RightHandSubscriber", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("RightHandSubscriber.inputs:topicName", self.cfg.right_hand_topic),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "RightHandSubscriber.inputs:execIn"),
                    ],
                },
            )
            self._right_hand_node = nodes[-1]
            
            print("ROS2 subscriber nodes created successfully")
            
        except Exception as e:
            print(f"Failed to create ROS2 subscriber nodes: {e}")

    def _get_joint_data_from_node(self, node):
        """Extract joint position data from ROS2 subscriber node."""
        try:
            if node is None:
                return None
                
            # Get the output data from the node - FIXED: use positionCommand for ROS2SubscribeJointState
            position_command_attr = og.Controller.attribute("outputs:positionCommand", node)
            if position_command_attr is not None:
                position_command = position_command_attr.get()
                if position_command is not None and len(position_command) > 0:
                    # FIXED: Return as 2D tensor with shape (1, num_joints)
                    return torch.tensor(position_command, dtype=torch.float32).unsqueeze(0)
                    
        except Exception as e:
            print(f"Error getting joint data from node: {e}")
            
        return None

    def _left_hand_callback(self, msg):
        """Callback for left hand joint state messages."""
        try:
            # Convert JointState to tensor
            if hasattr(msg, 'position') and msg.position:
                self._left_hand_data = torch.tensor(msg.position, dtype=torch.float32)
                self._last_message_time["left"] = time.time()
                
        except Exception as e:
            print(f"Error processing left hand data: {e}")

    def _right_hand_callback(self, msg):
        """Callback for right hand joint state messages."""
        try:
            # Convert JointState to tensor
            if hasattr(msg, 'position') and msg.position:
                self._right_hand_data = torch.tensor(msg.position, dtype=torch.float32)
                self._last_message_time["right"] = time.time()
                
        except Exception as e:
            print(f"Error processing right hand data: {e}")

    def _check_message_timeout(self):
        """Check if messages are too old and reset data if needed."""
        current_time = time.time()
        
        if (current_time - self._last_message_time["left"]) > self.cfg.message_timeout:
            self._left_hand_data = None
            
        if (current_time - self._last_message_time["right"]) > self.cfg.message_timeout:
            self._right_hand_data = None

    def reset(self):
        """Reset the device state."""
        self._left_hand_data = None
        self._right_hand_data = None
        self._last_message_time = {"left": 0.0, "right": 0.0}
        print("ROS2 Bridge device reset")

    def add_callback(self, key: str, func: Any):
        """Add callback function for teleoperation events.
        
        Args:
            key: Event name (e.g., "START", "STOP", "RESET")
            func: Callback function to be called when event occurs.
        """
        self._callbacks[key] = func

    def _get_raw_data(self) -> Dict[str, Optional[torch.Tensor]]:
        """Get raw joint data from ROS2 Bridge subscribers.
        
        Returns:
            Dictionary containing left and right hand joint data.
        """
        self._check_message_timeout()
        
        # Get data from OmniGraph nodes if available
        if self._left_hand_node is not None:
            left_data = self._get_joint_data_from_node(self._left_hand_node)
            if left_data is not None:
                self._left_hand_data = left_data
                self._last_message_time["left"] = time.time()
                
        if self._right_hand_node is not None:
            right_data = self._get_joint_data_from_node(self._right_hand_node)
            if right_data is not None:
                self._right_hand_data = right_data
                self._last_message_time["right"] = time.time()
        
        return {
            "left_hand": self._left_hand_data,
            "right_hand": self._right_hand_data
        }

    def advance(self) -> torch.Tensor:
        """Get the latest commands from the ROS2 Bridge device.
        
        Returns:
            torch.Tensor: Joint commands with shape (1, 38) matching Unitree G1 robot.
        """
        raw_data = self._get_raw_data()
        
        # Check if we have valid data from both hands
        if (raw_data["left_hand"] is not None and 
            raw_data["right_hand"] is not None):
            
            # Each hand has 7 joints from ROS message
            # We need to map this to the robot's 38 joints (14 arm + 24 hand)
            left_arm_data = raw_data["left_hand"]  # Shape: (1, 7) - left arm joints
            right_arm_data = raw_data["right_hand"]  # Shape: (1, 7) - right arm joints
            
            # Create full 38-dimensional control command
            # Structure: [left_arm(7), right_arm(7), left_hand(12), right_hand(12)]
            full_command = torch.zeros(1, 38, dtype=torch.float32)
            
            # Map left arm joints (positions 0-6)
            if left_arm_data.shape[1] >= 7:
                full_command[0, 0:7] = left_arm_data[0, 0:7]
            
            # Map right arm joints (positions 7-13)  
            if right_arm_data.shape[1] >= 7:
                full_command[0, 7:14] = right_arm_data[0, 0:7]
            
            # For hand joints (positions 14-37), set to default positions (0.0)
            # These will be controlled separately once you update your ROS message
            
            return full_command
            
        else:
            # Return zeros with correct shape (1, 38) when no data is available
            return torch.zeros(1, 38)

    def __del__(self):
        """Clean up ROS2 Bridge resources."""
        if hasattr(self, '_ros2_bridge'):
            try:
                self._ros2_bridge.cleanup()
            except:
                pass
        # Clean up OmniGraph nodes
        if self._left_hand_node is not None:
            try:
                og.Controller.delete(self._left_hand_node)
            except:
                pass
        if self._right_hand_node is not None:
            try:
                og.Controller.delete(self._right_hand_node)
            except:
                pass


@dataclass
class ROS2BridgeDeviceCfg(DeviceCfg):
    """Configuration for ROS2 Bridge teleoperation device."""

    # ROS2 topic names for left and right hand joint states
    left_hand_topic: str = "/cb_left_hand_control_cmd"
    right_hand_topic: str = "/cb_right_hand_control_cmd"
    
    # Timeout for receiving messages (seconds)
    message_timeout: float = 1.0
    
    # Whether to use Isaac Sim's built-in ROS2 Bridge
    use_isaacsim_bridge: bool = True
    
    # Set the concrete device class - FIXED: set to ROS2BridgeDevice class
    class_type: type[DeviceBase] = ROS2BridgeDevice