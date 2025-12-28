# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Retargeter for converting 0-255 hand tracking data to linkerhand joint angles."""
from __future__ import annotations
import torch
from dataclasses import dataclass
from isaaclab.devices.retargeter_base import RetargeterBase, RetargeterCfg

class RokaeRetargeter(RetargeterBase):
    """Retargeter that converts 0-255 hand tracking data to linkerhand joint angles."""
    
    def __init__(self, cfg: RokaeRetargeterCfg):
        """Initialize the retargeter."""
        super().__init__(cfg)
        self.cfg = cfg
        
        # 不再需要缩放因子，因为输入已经是弧度
        self._call_count = 0
        
        print(f"RokaeRetargeter initialized:")
        print(f"  Input: 64 joint angles in radians (14 dual arm + 25 left hand + 25 right hand)")
        print(f"  Output: 38 Rokae robot joint angles in radians")

    def retarget(self, data: torch.Tensor) -> torch.Tensor:
        """Convert input joint angles in radians to Rokae robot joint angles.
        
        Args:
            data: Input tensor with shape (batch_size, 64) containing joint angles in radians
                  从ROS接收的64个关节角度数据：
                  - 前14个：双臂数据（前7个左臂，后7个右臂）
                  - 后50个：手部数据（25左手 + 25右手，弧度）
            
        Returns:
            Tensor with shape (batch_size, 38) containing Rokae robot joint angles in radians
        """
        if data is None:
            print("Input tensor is None")
            # 返回中间位置作为默认值
            return torch.zeros((1, 38))
        
        # 创建38个关节的输出张量
        # joint_angles = torch.zeros((1, 38))
        joint_angles = torch.zeros((1, 14))
        
        # 检查输入数据维度
        if data.shape[1] < 64:
            print(f"Warning: Expected 64 input joint angles (14 dual arm + 50 hands), got {data.shape[1]}")
            # 使用默认值填充
            default_data = torch.cat([data, torch.zeros((1, 64 - data.shape[1]))], dim=1)
            input_data = default_data
        else:
            input_data = data
        
        # 分离数据：前14个是双臂数据，后50个是手部数据
        dual_arm_data = input_data[0, 0:14]    # 前14个：双臂数据（前7左臂，后7右臂）
        hand_data = input_data[0, 14:64]       # 后50个：手部数据（25左手 + 25右手）
        
        # 分离手部数据
        left_hand_data = hand_data[0:25]       # 前25个是左手数据
        right_hand_data = hand_data[25:50]     # 后25个是右手数据
        
        # joint_angles[0, 0] = -0.8  # 前7个是左臂数据
        # joint_angles[0, 1] = 0.8  # 前7个是左臂数据
        # joint_angles[0, 2] = -0.4  # 前7个是左臂数据
        # joint_angles[0, 3] = -0.4  # 前7个是左臂数据
        # joint_angles[0, 4] = -0.8  # 前7个是左臂数据
        # joint_angles[0, 5] = -0.8  # 前7个是左臂数据
        # joint_angles[0, 6] = -0.8  # 前7个是左臂数据
        # joint_angles[0, 7] = -0.8  # 前7个是左臂数据
        # joint_angles[0, 8] = -0.8  # 前7个是左臂数据
        # # 使用双臂数据控制手臂关节（前7个左臂，后7个右臂）
        # # 左手臂关节索引
        left_arm_indices = [0,2,4,6,8,10,12]
        left_arm_negate = [1,-1,1,-1,1,-1,1]  # 左臂数据需要取反
        for i in range(7):
            joint_angles[0, left_arm_indices[i]] = dual_arm_data[i] * left_arm_negate[i]  # 前7个是左臂数据
        
        # # # 右手臂关节索引
        right_arm_indices = [1,3,5,7,9,11,13]
        right_arm_negate = [1,1,1,1,1,1,1]  # 右臂数据需要取反
        for i in range(7):
            joint_angles[0, right_arm_indices[i]] = dual_arm_data[7 + i] * right_arm_negate[i]  # 后7个是右臂数据

        # # 根据udexreal_free.py的映射关系，但直接使用弧度值而不是0-255转换
        # # 右手映射（基于RightHand.joint_update方法的比例系数）- 使用右手数据
        # # 拇指
        # joint_angles[0, 23] = right_hand_data[16]   # R_thumb_proximal_yaw_joint (侧摆)
        # joint_angles[0, 33] = right_hand_data[17]   # R_thumb_proximal_pitch_joint (旋转)
        # joint_angles[0, 35] = right_hand_data[18]    # R_thumb_intermediate_joint (根部关节)
        # joint_angles[0, 37] = right_hand_data[19]    # R_thumb_distal_joint (远端关节)
        
        # # 右手食指
        # joint_angles[0, 19] = right_hand_data[2]           # R_index_proximal_joint
        # joint_angles[0, 29] = right_hand_data[3]           # R_index_intermediate_joint
        
        # # 右手小指
        # joint_angles[0, 21] = right_hand_data[6]          # R_pinky_proximal_joint
        # joint_angles[0, 31] = right_hand_data[7]          # R_pinky_intermediate_joint
        
        # # 右手中指
        # joint_angles[0, 20] = right_hand_data[10]          # R_middle_proximal_joint
        # joint_angles[0, 30] = right_hand_data[11]          # R_middle_intermediate_joint
        
        # # 右手无名指
        # joint_angles[0, 22] = right_hand_data[14]          # R_ring_proximal_joint
        # joint_angles[0, 32] = right_hand_data[15]          # R_ring_intermediate_joint
        
        # # 左手映射（基于LeftHand.joint_update方法的比例系数）- 使用左手数据
        # # 拇指
        # joint_angles[0, 18] = left_hand_data[16]          # L_thumb_proximal_yaw_joint (侧摆)
        # joint_angles[0, 28] = left_hand_data[17]          # L_thumb_proximal_pitch_joint (旋转)
        # joint_angles[0, 34] = left_hand_data[18]           # L_thumb_intermediate_joint (根部关节)
        # joint_angles[0, 36] = left_hand_data[19]           # L_thumb_distal_joint (远端关节)
        
        # # 左手食指 - 修复：移除负号，与右手一致
        # joint_angles[0, 14] = left_hand_data[2]           # L_index_proximal_joint
        # joint_angles[0, 24] = left_hand_data[3]           # L_index_intermediate_joint
        
        # # 左手小指 - 修复：移除负号，与右手一致
        # joint_angles[0, 16] = left_hand_data[6]          # L_pinky_proximal_joint
        # joint_angles[0, 26] = left_hand_data[7]          # L_pinky_intermediate_joint
        
        # # 左手中指 - 修复：移除负号，与右手一致
        # joint_angles[0, 15] = left_hand_data[10]          # L_middle_proximal_joint
        # joint_angles[0, 25] = left_hand_data[11]          # L_middle_intermediate_joint
        
        # # 左手无名指 - 修复：移除负号，与右手一致
        # joint_angles[0, 17] = left_hand_data[14]          # L_ring_proximal_joint
        # joint_angles[0, 27] = left_hand_data[15]          # L_ring_intermediate_joint
        
        # 增加调用计数器
        self._call_count += 1
        
        # 打印调试信息（每100次调用打印一次）
        if self._call_count % 100 == 0:
            print(f"Retargeter调用次数: {self._call_count}")
            print(f"左臂数据范围: {torch.min(dual_arm_data[0:7])} - {torch.max(dual_arm_data[0:7])}")
            print(f"右臂数据范围: {torch.min(dual_arm_data[7:14])} - {torch.max(dual_arm_data[7:14])}")
            # print(f"左手数据范围: {torch.min(left_hand_data)} - {torch.max(left_hand_data)}")
            # print(f"右手数据范围: {torch.min(right_hand_data)} - {torch.max(right_hand_data)}")
            print(f"输出关节角度范围: {torch.min(joint_angles)} - {torch.max(joint_angles)}")

        return joint_angles

    def get_requirements(self) -> list[RetargeterBase.Requirement]:
        """Return the list of required data features."""
        return [RetargeterBase.Requirement.HAND_TRACKING]


@dataclass
class RokaeRetargeterCfg(RetargeterCfg):
    """Configuration for Rokae robot data retargeter."""
    
    # 关节角度范围（弧度）- 38个关节的默认范围
    joint_limits_rad = (
        # 下限（弧度）- 38个关节的默认下限
        torch.tensor([-3.14] * 38),  # 约-180度
        # 上限（弧度）- 38个关节的默认上限
        torch.tensor([3.14] * 38)    # 约180度
    )
    
    # 输入数据范围（弧度）- 64个关节角度（14双臂 + 50手部）
    input_range = (
        # 下限
        torch.tensor([-3.14] * 64),  # 约-180度
        # 上限  
        torch.tensor([3.14] * 64)    # 约180度
    )
    
    # 正确设置retargeter_type
    retargeter_type: type[RetargeterBase] = RokaeRetargeter
    
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
    retargeter_type: type[RetargeterBase] = RokaeRetargeter