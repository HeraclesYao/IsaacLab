# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""HoloLens2 SE3 teleoperation device."""

import numpy as np
import torch
from collections.abc import Callable
from typing import Optional

from isaaclab.devices.device_base import DeviceBase
from isaaclab.devices.device_base import DeviceCfg

# 导入HoloLens2适配器
import sys
sys.path.append('/home/yaoyh/workspace/IsaacLab/hl2ss')
sys.path.append("/home/yaoyh/workspace/IsaacLab/hl2ss/viewer/")
from isaaclab_adapter import HoloLens2DeviceAdapter
import hl2ss
import hl2ss_lnm
import hl2ss_mp
from isaaclab_tasks.manager_based.manipulation.pick_place.pickplace_unitree_g1_inspire_hand_env_cfg import PickPlaceG1InspireFTPEnvCfg

class Se3HoloLens2Cfg(DeviceCfg):
    """Configuration for HoloLens2 SE3 device."""

    def __init__(self):
        super().__init__(cls=Se3HoloLens2)
        self.host = "192.168.11.121"  # HoloLens2默认IP


class Se3HoloLens2(DeviceBase):
    """HoloLens2 SE3 teleoperation device."""

    def __init__(self, cfg: Se3HoloLens2Cfg):
        super().__init__(cfg)
        self._cfg = cfg
        
        # 创建HoloLens2适配器
        self.adapter = HoloLens2DeviceAdapter(host=cfg.host)
        
        # G1任务需要的38维动作向量
        # 结构: [2个末端执行器(各6维) + 24个手部关节 + 8个零空间关节]
        self.action_dim = 38
        
        # 固定肩部和肘部角度为90度 (约1.57弧度)
        self.fixed_shoulder_elbow_angles = np.radians(90.0)
        
        # 默认手部关节角度 (半开状态)
        self.default_hand_joints = np.array([0.5] * 24)
        
        # 默认零空间关节角度
        self.default_nullspace_joints = np.array([0.0] * 8)

    def open(self) -> bool:
        """打开HoloLens2连接."""
        return self.adapter.open()

    def close(self) -> None:
        """关闭连接."""
        self.adapter.close()

    def reset(self) -> None:
        """重置设备状态."""
        self.adapter.reset()

    def _convert_hand_to_g1_action(self, hand_data: dict) -> Optional[torch.Tensor]:
        """将HoloLens2手部数据转换为G1的38维动作."""
        if not hand_data:
            return None
            
        # 获取右手数据 (假设使用右手控制)
        right_hand = hand_data.get(DeviceBase.TrackingTarget.HAND_RIGHT)
        if not right_hand:
            return None
            
        # 获取手腕关节位置
        wrist_pos = right_hand.get("WRIST")
        if wrist_pos is None:
            return None
            
        # 提取手腕位置 (前3个元素)
        wrist_position = wrist_pos[:3]
        
        # 构建38维动作向量
        g1_action = np.zeros(self.action_dim)
        
        # 1. 第一个末端执行器 (手腕位置和姿态)
        # 使用手腕位置，姿态使用默认值
        g1_action[0:3] = wrist_position  # 位置
        g1_action[3:6] = [0.0, 0.0, 0.0]  # 姿态 (欧拉角)
        
        # 2. 第二个末端执行器 (固定值)
        g1_action[6:12] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # 3. 手部关节 (24个)
        # 简化的映射：根据手指位置计算关节角度
        hand_joints = self._calculate_hand_joints(right_hand)
        g1_action[12:36] = hand_joints
        
        # 4. 零空间关节 (8个)
        # 固定肩部和肘部为90度
        nullspace_joints = np.zeros(8)
        nullspace_joints[0] = self.fixed_shoulder_elbow_angles  # 肩部
        nullspace_joints[1] = self.fixed_shoulder_elbow_angles  # 肘部
        g1_action[36:38] = nullspace_joints[:2]  # 只使用前2个零空间关节
        
        return torch.tensor(g1_action, dtype=torch.float32)

    def _calculate_hand_joints(self, hand_data: dict) -> np.ndarray:
        """根据手部数据计算G1手部关节角度."""
        hand_joints = self.default_hand_joints.copy()
        
        try:
            # 简化的手指弯曲计算
            # 使用指尖和手掌的距离来估计手指弯曲程度
            
            # 获取关键关节位置
            palm_pos = hand_data.get("PALM")
            thumb_tip = hand_data.get("THUMB_TIP")
            index_tip = hand_data.get("INDEX_TIP")
            middle_tip = hand_data.get("MIDDLE_TIP")
            ring_tip = hand_data.get("RING_TIP")
            little_tip = hand_data.get("LITTLE_TIP")
            
            if palm_pos is not None:
                palm_pos = palm_pos[:3]
                
                # 计算每个手指的弯曲程度
                finger_bends = []
                
                for tip_joint in [thumb_tip, index_tip, middle_tip, ring_tip, little_tip]:
                    if tip_joint is not None:
                        tip_pos = tip_joint[:3]
                        distance = np.linalg.norm(tip_pos - palm_pos)
                        # 简化的弯曲计算：距离越小，弯曲越大
                        bend = max(0.0, min(1.0, 1.0 - distance / 0.2))
                        finger_bends.append(bend)
                    else:
                        finger_bends.append(0.5)  # 默认值
                
                # 映射到G1手部关节
                # G1手部关节结构: 大拇指2个，其他手指各3个
                joint_index = 0
                
                # 大拇指 (2个关节)
                hand_joints[joint_index:joint_index+2] = [finger_bends[0] * 0.8, finger_bends[0] * 1.2]
                joint_index += 2
                
                # 其他手指 (各3个关节)
                for bend in finger_bends[1:]:
                    hand_joints[joint_index:joint_index+3] = [bend * 0.6, bend * 0.8, bend * 1.0]
                    joint_index += 3
                    
        except Exception as e:
            print(f"手部关节计算错误: {e}")
            # 使用默认值
            
        return hand_joints

    def advance(self) -> Optional[torch.Tensor]:
        """获取最新的HoloLens2数据并转换为G1动作."""
        try:
            # 获取HoloLens2手部数据
            hand_data = self.adapter.advance()
            
            # 转换为G1动作
            g1_action = self._convert_hand_to_g1_action(hand_data)
            
            return g1_action
            
        except Exception as e:
            print(f"HoloLens2数据获取错误: {e}")
            return None

    def add_callback(self, key: str, callback: Callable[[], None]) -> None:
        """添加回调函数 (简化实现)."""
        # HoloLens2设备暂时不支持键盘回调
        pass

    def __str__(self) -> str:
        """返回设备描述."""
        return f"HoloLens2 SE3 Device (host: {self._cfg.host})"