# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Retargeter for converting 0-255 hand tracking data to linkerhand joint angles."""
from __future__ import annotations
import torch
from dataclasses import dataclass
from isaaclab.devices.retargeter_base import RetargeterBase, RetargeterCfg

class LinkerhandRetargeter(RetargeterBase):
    """Retargeter that converts 0-255 hand tracking data to linkerhand joint angles."""
    
    def __init__(self, cfg: LinkerhandRetargeterCfg):
        """Initialize the retargeter."""
        super().__init__(cfg)
        self.cfg = cfg
        
        # 计算缩放因子
        self._input_min = cfg.input_range[0]
        self._input_max = cfg.input_range[1]
        self._joint_min = cfg.joint_limits_rad[0]
        self._joint_max = cfg.joint_limits_rad[1]

        self._call_count = 0
        
        # 缩放因子：将0-255映射到关节角度范围
        self._scale_factor = (self._joint_max - self._joint_min) / (self._input_max - self._input_min)
        
        print(f"LinkerhandRetargeter initialized:")
        print(f"  Input range: {self._input_min} to {self._input_max}")
        print(f"  Joint range: {self._joint_min} to {self._joint_max}")
        print(f"  Scale factor: {self._scale_factor}")

    def retarget(self, data: torch.Tensor) -> torch.Tensor:
        """Convert 0-255 input data to joint angles in radians.
        
        Args:
            data: Input tensor with shape (batch_size, 7) containing values in range [0, 255]
            
        Returns:
            Tensor with shape (batch_size, 7) containing joint angles in radians
        """
        if data is None:
            print("Input tensor is None")
            # 返回中间位置作为默认值
            return (self._joint_min + self._joint_max) / 2.0
        
        # 确保数据在正确范围内
        data_clamped = torch.clamp(data, self._input_min, self._input_max)
        
        # 修正数据映射关系：ROS2话题的255对应关节角度0，ROS2话题的0对应关节最大角度
        # 公式: joint_angle = joint_min + (input_max - data) * scale_factor
        joint_angles = self._joint_min + (self._input_max - data_clamped) * self._scale_factor
        
        # 确保输出在关节限制范围内
        joint_angles = torch.clamp(joint_angles, self._joint_min, self._joint_max)

        joint_angles = torch.zeros((1, 17))
        # joint_angles = torch.zeros((1, 7))


        # 增加调用计数器
        self._call_count += 1
        
        # 每100次调用为一个循环周期
        cycle_step = self._call_count % 4000  # 4个手指 * 100步
        
        # 定义手指的关节索引（17个关节）
        # 主动关节：0=食指MCP, 1=中指MCP, 2=小指MCP, 3=无名指MCP, 4=拇指CMC_roll, 5=拇指CMC_yaw, 6=拇指CMC_pitch
        # 从动关节：7=拇指MCP, 8=拇指IP, 9=食指PIP, 10=食指DIP, 11=中指PIP, 12=中指DIP, 
        #           13=无名指PIP, 14=无名指DIP, 15=小指PIP, 16=小指DIP
        
        # 手指动作循环
        if cycle_step < 1000:
            # 阶段1：食指动作（弯曲→保持→伸直）
            finger_step = cycle_step
            if finger_step < 250:
                # 弯曲过程：0-25步，从0到1.325弧度
                finger_bend = 1.325 * (finger_step / 25.0)
                action_name = "弯曲食指"
            elif finger_step < 500:
                # 保持弯曲：25-50步，保持1.325弧度
                finger_bend = 1.325
                action_name = "保持食指弯曲"
            elif finger_step < 750:
                # 伸直过程：50-75步，从1.325到0弧度
                finger_bend = 1.325 * (1.0 - (finger_step - 75) / 25.0)
                action_name = "伸直食指"
            else:
                # 伸直过程：75-100步，保持0弧度
                finger_bend = 0
                action_name = "伸直食指"

            # finger_bend = 1.325 * min(cycle_step / 25.0, 1.0)  # 0-1的弯曲程度
            joint_angles[0, 0] = finger_bend  # 食指MCP
            joint_angles[0, 7] = 1.18 * finger_bend    # 食指PIP
            joint_angles[0, 8] = 0.7493 * finger_bend   # 食指DIP
            action_name = "弯曲食指"
            
        elif cycle_step < 2000:
            # 阶段2：中指动作
            finger_step = cycle_step - 1000
            if finger_step < 250:
                finger_bend = 1.325 * (finger_step / 25.0)
                action_name = "弯曲中指"
            elif finger_step < 500:
                finger_bend = 1.325
                action_name = "保持中指弯曲"
            elif finger_step < 750:
                finger_bend = 1.325 * (1.0 - (finger_step - 75) / 25.0)
                action_name = "保持中指弯曲"
            else:
                finger_bend = 0
                action_name = "伸直中指"
            # finger_bend = 1.325 * min((cycle_step - 100) / 25.0, 1.0)
            joint_angles[0, 1] = finger_bend  # 中指MCP
            joint_angles[0, 9] = 1.18 * finger_bend   # 中指PIP
            joint_angles[0, 10] = 0.7493 * finger_bend   # 中指DIP
            action_name = "弯曲中指"
            
        elif cycle_step < 3000:
            # 阶段3：弯曲无名指（先伸直中指）
            finger_step = cycle_step - 2000
            if finger_step < 250:
                finger_bend = 1.325 * (finger_step / 25.0)
                action_name = "弯曲无名指"
            elif finger_step < 500:
                finger_bend = 1.325
                action_name = "保持无名指弯曲"
            elif finger_step < 750:
                finger_bend = 1.325 * (1.0 - (finger_step - 75) / 25.0)
                action_name = "伸直无名指"
            else:
                finger_bend = 0
                action_name = "伸直无名指"
            # finger_bend = 1.325 * min((cycle_step - 200) / 25.0, 1.0)
            joint_angles[0, 3] = finger_bend  # 无名指MCP
            joint_angles[0, 13] = 1.18 * finger_bend   # 无名指PIP
            joint_angles[0, 14] = 0.7493 * finger_bend   # 无名指DIP
            action_name = "弯曲无名指"
            
        else:
            # 阶段4：弯曲小指（先伸直无名指）
            finger_step = cycle_step - 3000
            if finger_step < 250:
                finger_bend = 1.325 * (finger_step / 25.0)
                action_name = "弯曲小指"
            elif finger_step < 500:
                finger_bend = 1.325
                action_name = "保持小指弯曲"
            elif finger_step < 750:
                finger_bend = 1.325 * (1.0 - (finger_step - 75) / 25.0)
                action_name = "伸直小指"
            else:
                finger_bend = 0
                action_name = "伸直小指"
            # finger_bend = 1.325 * min((cycle_step - 300) / 25.0, 1.0)
            joint_angles[0, 2] = finger_bend  # 小指MCP
            joint_angles[0, 11] = 1.18 * finger_bend   # 小指PIP
            joint_angles[0, 12] = 0.7493 * finger_bend   # 小指DIP
            action_name = "弯曲小指"
        
        # 保持拇指在中间位置
        # 控制拇指弯曲程度（0.0-0.5285弧度）
        # thumb_bend = 0.2  # 示例：轻微弯曲
        thumb_bend = 0.0  # 示例：轻微弯曲
        joint_angles[0, 4] = 0.0   # 拇指CMC_roll
        joint_angles[0, 5] = 0.0   # 拇指CMC_yaw
        joint_angles[0, 6] = thumb_bend   # 拇指CMC_pitch
        # mimic关节会自动计算：
        thumb_mcp = thumb_bend * 2.24
        thumb_ip = thumb_bend * 2.38
        joint_angles[0, 15] = thumb_mcp   # 拇指MCP（mimic关节，自动跟随）
        joint_angles[0, 16] = thumb_ip   # 拇指IP（mimic关节，自动跟随）


        # 打印当前动作信息（每10次调用打印一次）
        # if self._call_count % 10 == 0:
        #     print(f"动作循环: 步骤 {cycle_step}/400, 当前动作: {action_name}")
        #     print(f"关节角度: {joint_angles}")

        print(f"Retargeting data: {data_clamped} to joint angles: {joint_angles}")
        return joint_angles

    def get_requirements(self) -> list[RetargeterBase.Requirement]:
        """Return the list of required data features."""
        return [RetargeterBase.Requirement.HAND_TRACKING]


# 设置配置类的retargeter_type
# LinkerhandRetargeterCfg.retargeter_type = LinkerhandRetargeter
@dataclass
class LinkerhandRetargeterCfg(RetargeterCfg):
    """Configuration for linkerhand data retargeter."""
    
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
    retargeter_type: type[RetargeterBase] = LinkerhandRetargeter
