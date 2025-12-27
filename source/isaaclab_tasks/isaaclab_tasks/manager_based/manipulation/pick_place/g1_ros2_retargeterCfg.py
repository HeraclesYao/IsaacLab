# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Retargeter for converting 0-255 hand tracking data to linkerhand joint angles."""
from __future__ import annotations
import torch
from dataclasses import dataclass
from isaaclab.devices.retargeter_base import RetargeterBase, RetargeterCfg

class G1Retargeter(RetargeterBase):
    """Retargeter that converts 0-255 hand tracking data to linkerhand joint angles."""
    
    def __init__(self, cfg: G1RetargeterCfg):
        """Initialize the retargeter."""
        super().__init__(cfg)
        self.cfg = cfg
        
        # 不再需要缩放因子，因为输入已经是弧度
        self._call_count = 0
        
        print(f"G1Retargeter initialized:")
        print(f"  Input: 50 joint angles in radians (25 left + 25 right)")
        print(f"  Output: 38 G1 robot joint angles in radians")

    def retarget(self, data: torch.Tensor) -> torch.Tensor:
        """Convert input joint angles in radians to G1 robot joint angles.
        
        Args:
            data: Input tensor with shape (batch_size, 50) containing joint angles in radians
                  从ROS接收的50个关节角度数据（25左手 + 25右手，弧度）
            
        Returns:
            Tensor with shape (batch_size, 38) containing G1 robot joint angles in radians
        """
        if data is None:
            print("Input tensor is None")
            # 返回中间位置作为默认值
            return torch.zeros((1, 38))
        
        # 创建38个关节的输出张量
        joint_angles = torch.zeros((1, 38))
        
        # 检查输入数据维度
        if data.shape[1] < 50:
            print(f"Warning: Expected 50 input joint angles (25 left + 25 right), got {data.shape[1]}")
            # 使用默认值填充
            default_data = torch.cat([data, torch.zeros((1, 50 - data.shape[1]))], dim=1)
            input_data = default_data
        else:
            input_data = data
        
        # 分离左手和右手数据
        left_hand_data = input_data[0, 0:25]  # 前25个是左手数据
        right_hand_data = input_data[0, 25:50]  # 后25个是右手数据
        
        # 根据udexreal_free.py的映射关系，但直接使用弧度值而不是0-255转换
        # 右手映射（基于RightHand.joint_update方法的比例系数）- 使用右手数据
        # 拇指
        joint_angles[0, 23] = right_hand_data[16]   # R_thumb_proximal_yaw_joint (侧摆)
        joint_angles[0, 33] = right_hand_data[17]   # R_thumb_proximal_pitch_joint (旋转)
        joint_angles[0, 35] = right_hand_data[18]    # R_thumb_intermediate_joint (根部关节)
        joint_angles[0, 37] = right_hand_data[19]    # R_thumb_distal_joint (远端关节)
        
        # 右手食指
        joint_angles[0, 19] = right_hand_data[2]           # R_index_proximal_joint
        joint_angles[0, 29] = right_hand_data[3]           # R_index_intermediate_joint
        
        # 右手小指
        joint_angles[0, 21] = right_hand_data[6]          # R_pinky_proximal_joint
        joint_angles[0, 31] = right_hand_data[7]          # R_pinky_intermediate_joint
        
        # 右手中指
        joint_angles[0, 20] = right_hand_data[10]          # R_middle_proximal_joint
        joint_angles[0, 30] = right_hand_data[11]          # R_middle_intermediate_joint
        
        # 右手无名指
        joint_angles[0, 22] = right_hand_data[14]          # R_ring_proximal_joint
        joint_angles[0, 32] = right_hand_data[15]          # R_ring_intermediate_joint
        
        # 左手映射（基于LeftHand.joint_update方法的比例系数）- 使用左手数据
        # 拇指
        joint_angles[0, 18] = left_hand_data[16]          # L_thumb_proximal_yaw_joint (侧摆)
        joint_angles[0, 28] = left_hand_data[17]          # L_thumb_proximal_pitch_joint (旋转)
        joint_angles[0, 34] = left_hand_data[18]           # L_thumb_intermediate_joint (根部关节)
        joint_angles[0, 36] = left_hand_data[19]           # L_thumb_distal_joint (远端关节)
        
        # 左手食指 - 修复：移除负号，与右手一致
        joint_angles[0, 14] = left_hand_data[2]           # L_index_proximal_joint
        joint_angles[0, 24] = left_hand_data[3]           # L_index_intermediate_joint
        
        # 左手小指 - 修复：移除负号，与右手一致
        joint_angles[0, 16] = left_hand_data[6]          # L_pinky_proximal_joint
        joint_angles[0, 26] = left_hand_data[7]          # L_pinky_intermediate_joint
        
        # 左手中指 - 修复：移除负号，与右手一致
        joint_angles[0, 15] = left_hand_data[10]          # L_middle_proximal_joint
        joint_angles[0, 25] = left_hand_data[11]          # L_middle_intermediate_joint
        
        # 左手无名指 - 修复：移除负号，与右手一致
        joint_angles[0, 17] = left_hand_data[14]          # L_ring_proximal_joint
        joint_angles[0, 27] = left_hand_data[15]          # L_ring_intermediate_joint
        
        # 手臂关节保持默认位置（0弧度）
        # 右手臂关节索引 0-6
        # 左手臂关节索引 7-13
        for i in range(14):
            joint_angles[0, i] = 0.0

        # 增加调用计数器
        self._call_count += 1
        
        # 打印调试信息（每100次调用打印一次）
        if self._call_count % 100 == 0:
            print(f"Retargeter调用次数: {self._call_count}")
            print(f"左手数据范围: {torch.min(left_hand_data)} - {torch.max(left_hand_data)}")
            print(f"右手数据范围: {torch.min(right_hand_data)} - {torch.max(right_hand_data)}")
            print(f"输出关节角度范围: {torch.min(joint_angles)} - {torch.max(joint_angles)}")
            print(f"右手拇指关节: {joint_angles[0, 23]:.3f}, {joint_angles[0, 33]:.3f}, {joint_angles[0, 35]:.3f}, {joint_angles[0, 37]:.3f}")
            print(f"左手拇指关节: {joint_angles[0, 18]:.3f}, {joint_angles[0, 28]:.3f}, {joint_angles[0, 34]:.3f}, {joint_angles[0, 36]:.3f}")

        return joint_angles

    def get_requirements(self) -> list[RetargeterBase.Requirement]:
        """Return the list of required data features."""
        return [RetargeterBase.Requirement.HAND_TRACKING]


@dataclass
class G1RetargeterCfg(RetargeterCfg):
    """Configuration for G1 robot data retargeter."""
    
    # 关节角度范围（弧度）- 38个关节的默认范围
    joint_limits_rad = (
        # 下限（弧度）- 38个关节的默认下限
        torch.tensor([-3.14] * 38),  # 约-180度
        # 上限（弧度）- 38个关节的默认上限
        torch.tensor([3.14] * 38)    # 约180度
    )
    
    # 输入数据范围（弧度）- 50个关节角度（25左手 + 25右手）
    input_range = (
        # 下限
        torch.tensor([-3.14] * 50),  # 约-180度
        # 上限  
        torch.tensor([3.14] * 50)    # 约180度
    )
    
    # 正确设置retargeter_type
    retargeter_type: type[RetargeterBase] = G1Retargeter
    
    # 关节角度范围（弧度）
    joint_limits_rad = (
        # 下限（弧度）
        torch.tensor([0.0, 0.0, 0.0, 0.0, -0.784, 0.0, 0.0]),
        # 上限（弧度）  
        torch.tensor([1.325, 1.325, 1.325, 1.325, 0.784, 1.569, 0.527])
    )
    
    # 输入数据范围（0-255）
    input_range = (
        # 下限
        torch.tensor([84.0, 0.0, 66.0, 0.0, 66, 78.0, 0.0]),
        # 上限  
        torch.tensor([255, 255, 255, 255, 255, 255, 130])
        )
    
    # retargeter_type: type[RetargeterBase] = None  # 将在类定义后设置

    # 正确设置retargeter_type
    retargeter_type: type[RetargeterBase] = G1Retargeter