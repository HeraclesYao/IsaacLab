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
        self._dual_arm_data: Optional[torch.Tensor] = None  # 新增：双臂数据
        self._last_message_time: Dict[str, float] = {
            "left": 0.0,
            "right": 0.0,
            "dual_arm": 0.0  # 新增：双臂数据时间戳
        }
        
        # ROS2 graph nodes
        self._left_hand_node = None
        self._right_hand_node = None
        self._dual_arm_node = None  # 新增：双臂控制节点
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
                if self.cfg.dual_arm_topic:  # 新增：打印双臂控制topic
                    print(f"  - Dual arm: {self.cfg.dual_arm_topic}")
                
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
                        if self.cfg.dual_arm_topic:  # 新增：打印双臂控制topic
                            print(f"  - Dual arm: {self.cfg.dual_arm_topic}")
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
            
            # 新增：创建双臂控制subscriber节点
            if self.cfg.dual_arm_topic:
                (graph, nodes, _, _) = og.Controller.edit(
                    {"graph_path": "/ROS2BridgeDevice/DualArmSubscriber", "evaluator_name": "execution"},
                    {
                        og.Controller.Keys.CREATE_NODES: [
                            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                            ("DualArmSubscriber", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                        ],
                        og.Controller.Keys.SET_VALUES: [
                            ("DualArmSubscriber.inputs:topicName", self.cfg.dual_arm_topic),
                        ],
                        og.Controller.Keys.CONNECT: [
                            ("OnPlaybackTick.outputs:tick", "DualArmSubscriber.inputs:execIn"),
                        ],
                    },
                )
                self._dual_arm_node = nodes[-1]
            
            print("ROS2 subscriber nodes created successfully")
            
        except Exception as e:
            print(f"Failed to create ROS2 subscriber nodes: {e}")

    def _get_joint_data_from_node(self, node, node_name="unknown"):
        """Extract joint position data from ROS2 subscriber node with detailed debugging."""
        try:
            if node is None:
                print(f"DEBUG: {node_name} node is None")
                return None
                
            # Get the output data from the node - FIXED: use positionCommand for ROS2SubscribeJointState
            position_command_attr = og.Controller.attribute("outputs:positionCommand", node)
            if position_command_attr is not None:
                position_command = position_command_attr.get()
                if position_command is not None and len(position_command) > 0:
                    # FIXED: Return as 2D tensor with shape (1, num_joints)
                    tensor_data = torch.tensor(position_command, dtype=torch.float32).unsqueeze(0)
                    print(f"DEBUG: {node_name} data received - shape: {tensor_data.shape}, values: {tensor_data}")
                    return tensor_data
                else:
                    print(f"DEBUG: {node_name} position command is None or empty")
            else:
                print(f"DEBUG: {node_name} positionCommand attribute not found")
                
        except Exception as e:
            print(f"Error getting joint data from {node_name} node: {e}")
            
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
            
        if (current_time - self._last_message_time["dual_arm"]) > self.cfg.message_timeout:
            self._dual_arm_data = None

    def reset(self):
        """Reset the device state."""
        self._left_hand_data = None
        self._right_hand_data = None
        self._dual_arm_data = None  # 新增：重置双臂数据
        self._last_message_time = {"left": 0.0, "right": 0.0, "dual_arm": 0.0}
        print("ROS2 Bridge device reset")

    def add_callback(self, key: str, func: Any):
        """Add callback function for teleoperation events.
        
        Args:
            key: Event name (e.g., "START", "STOP", "RESET")
            func: Callback function to be called when event occurs.
        """
        self._callbacks[key] = func

    def _get_raw_data(self) -> torch.Tensor:
        """Get raw joint data from ROS2 Bridge subscribers."""
        self._check_message_timeout()

        print(f"DEBUG: === Starting data collection cycle ===")
        
        # 优先获取双臂控制数据（14个关节）
        dual_arm_data = None
        if self._dual_arm_node is not None:
            dual_arm_data = self._get_joint_data_from_node(self._dual_arm_node, "dual_arm")
            if dual_arm_data is not None:
                self._dual_arm_data = dual_arm_data
                self._last_message_time["dual_arm"] = time.time()
                print(f"DEBUG: Dual arm data stored with shape: {dual_arm_data.shape}")
            else:
                print("DEBUG: No dual arm data received from node - checking topic status")
        else:
            print("DEBUG: Dual arm node is None - topic may not be configured")

        # 获取左手数据（25个关节）
        left_data = None
        if self._left_hand_node is not None:
            left_data = self._get_joint_data_from_node(self._left_hand_node, "left_hand")
            if left_data is not None:
                self._left_hand_data = left_data
                self._last_message_time["left"] = time.time()
                print(f"DEBUG: Left hand data stored with shape: {left_data.shape}")

        # 获取右手数据（25个关节）
        right_data = None
        if self._right_hand_node is not None:
            right_data = self._get_joint_data_from_node(self._right_hand_node, "right_hand")
            if right_data is not None:
                self._right_hand_data = right_data
                self._last_message_time["right"] = time.time()
                print(f"DEBUG: Right hand data stored with shape: {right_data.shape}")

        # 合并数据：双臂14个关节 + 手部50个关节 = 总共64个关节
        combined_data_parts = []
        
        # 添加双臂数据（前14个关节）
        if self._dual_arm_data is not None:
            if self._dual_arm_data.shape[1] >= 14:
                arm_data = self._dual_arm_data[:, :14]
                combined_data_parts.append(arm_data)
                print(f"DEBUG: Using dual arm data with shape: {arm_data.shape}, values: {arm_data}")
            else:
                padding = torch.zeros(1, 14 - self._dual_arm_data.shape[1], dtype=torch.float32)
                arm_data = torch.cat([self._dual_arm_data, padding], dim=1)
                combined_data_parts.append(arm_data)
                print(f"DEBUG: Padded dual arm data to shape: {arm_data.shape}")
        else:
            arm_data = torch.zeros(1, 14, dtype=torch.float32)
            combined_data_parts.append(arm_data)
            print("DEBUG: Using zero padding for dual arm data")

        # 添加手部数据（后50个关节）
        if self._left_hand_data is not None and self._right_hand_data is not None:
            hand_data = torch.cat([self._left_hand_data, self._right_hand_data], dim=1)
            combined_data_parts.append(hand_data)
            print(f"DEBUG: Combined hand data with shape: {hand_data.shape}")
        elif self._left_hand_data is not None:
            right_zeros = torch.zeros_like(self._left_hand_data)
            hand_data = torch.cat([self._left_hand_data, right_zeros], dim=1)
            combined_data_parts.append(hand_data)
            print(f"DEBUG: Left hand only with shape: {hand_data.shape}")
        elif self._right_hand_data is not None:
            left_zeros = torch.zeros_like(self._right_hand_data)
            hand_data = torch.cat([left_zeros, self._right_hand_data], dim=1)
            combined_data_parts.append(hand_data)
            print(f"DEBUG: Right hand only with shape: {hand_data.shape}")
        else:
            hand_data = torch.zeros(1, 50, dtype=torch.float32)
            combined_data_parts.append(hand_data)
            print("DEBUG: Using zero padding for hand data")
        
        # 合并所有数据
        combined_data = torch.cat(combined_data_parts, dim=1)
        
        # 确保最终数据有64个关节
        if combined_data.shape[1] < 64:
            padding = torch.zeros(1, 64 - combined_data.shape[1], dtype=torch.float32)
            combined_data = torch.cat([combined_data, padding], dim=1)
            print(f"DEBUG: Final padding to 64 joints: {combined_data.shape}")
        
        print(f"DEBUG: Final combined data shape: {combined_data.shape}, values: {combined_data}")
        print(f"DEBUG: === Data collection cycle completed ===\n")
        return combined_data

    def advance(self) -> torch.Tensor:
        """Get the latest commands from the ROS2 Bridge device.
        
        Returns:
            torch.Tensor: Joint commands after retargeting
        """
        # Use the base class advance() method which automatically calls retargeters
        # This will call _get_raw_data() and then apply retargeting
        return super().advance()

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
    dual_arm_topic: str = "/cb_arm_control_cmd"  # 新增：双臂控制topic
    
    # Timeout for receiving messages (seconds)
    message_timeout: float = 1.0
    
    # Whether to use Isaac Sim's built-in ROS2 Bridge
    use_isaacsim_bridge: bool = True
    
    # Set the concrete device class - FIXED: set to ROS2BridgeDevice class
    class_type: type[DeviceBase] = ROS2BridgeDevice