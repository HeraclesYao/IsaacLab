# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

from . import mdp
from isaaclab.envs.mdp.actions.actions_cfg import JointPositionActionCfg

# 导入新创建的retargeter
from .linkerhand_retargeter import LinkerhandRetargeterCfg

##
# Pre-defined configs
##

from isaaclab.devices.ros2_bridge_device import ROS2BridgeDeviceCfg
from isaaclab.devices import DevicesCfg
from isaaclab_assets.robots.unitree import G1_INSPIRE_FTP_CFG
from isaaclab_assets.robots.linkhand import LINKHAND_CFG

##
# Scene definition
##
convert_mimic_joints_to_normal_joints: bool = False

@configclass
class LinkerhandSceneCfg(InteractiveSceneCfg):
    """Configuration for a linkerhand scene."""

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    # 右手机器人（先专注于单个机器人进行测试）
    robot: ArticulationCfg = LINKHAND_CFG.replace(
        prim_path="/World/envs/env_.*/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 1.0),  # 居中位置
            rot=(1.0, 0.0, 0.0, 0.0),
            joint_pos={
                # 使用精确的关节名称，避免正则表达式冲突
                "thumb_cmc_roll": 0.0,
                "thumb_cmc_yaw": 0.0,
                "thumb_cmc_pitch": 0.0,
                "thumb_mcp": 0.0,
                "thumb_ip": 0.0,
                "index_mcp_pitch": 0.0,
                "index_pip": 0.0,
                "index_dip": 0.0,
                "middle_mcp_pitch": 0.0,
                "middle_pip": 0.0,
                "middle_dip": 0.0,
                "ring_mcp_pitch": 0.0,
                "ring_pip": 0.0,
                "ring_dip": 0.0,
                "pinky_mcp_pitch": 0.0,
                "pinky_pip": 0.0,
                "pinky_dip": 0.0,
            },
            joint_vel={".*": 0.0},
        ),
    )

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )


##
# MDP settings
##


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    # 使用JointPositionActionCfg进行位置控制
    joint_position = JointPositionActionCfg(
        asset_name="robot", 
        joint_names=[
            "index_mcp_pitch", 
            "middle_mcp_pitch", 
            "pinky_mcp_pitch", 
            "ring_mcp_pitch", 
            "thumb_cmc_roll",
            "thumb_cmc_yaw",
            "thumb_cmc_pitch",
            "index_pip",
            "index_dip",
            "middle_pip",
            "middle_dip",
            "pinky_pip",
            "pinky_dip",
            "ring_pip",
            "ring_dip",
            "thumb_mcp",
            "thumb_ip",
        ],
        scale=1.0  # 使用完整的缩放
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""
    print("重置关节位置")
    # 重置拇指关节位置
    reset_thumb_position = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", 
                                        joint_names=[
                                            "index_mcp_pitch", 
                                            "middle_mcp_pitch", 
                                            "pinky_mcp_pitch", 
                                            "ring_mcp_pitch", 
                                            "thumb_cmc_roll",
                                            "thumb_cmc_yaw",
                                            "thumb_cmc_pitch",
                                            "index_pip",
                                            "index_dip",
                                            "middle_pip",
                                            "middle_dip",
                                            "pinky_pip",
                                            "pinky_dip",
                                            "ring_pip",
                                            "ring_dip",
                                            "thumb_mcp",
                                            "thumb_ip",
                                        ]
        ),
            # "position_range": (-2.0, 2.0),  # 较小的位置范围
            # "velocity_range": (-10.0, 10.0),  # 较小的速度范围
            "position_range": (0.0, 0.0),  # 禁用位置随机化
            "velocity_range": (0.0, 0.0),  # 禁用速度随机化
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # (1) Constant running reward
    alive = RewTerm(func=mdp.is_alive, weight=1.0)
    # (2) 保持关节在中间位置的奖励
    joint_pos_center = RewTerm(
        func=mdp.joint_pos_target_l2,
        weight=-0.1,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
            "index_mcp_pitch", 
            "middle_mcp_pitch", 
            "pinky_mcp_pitch", 
            "ring_mcp_pitch", 
            "thumb_cmc_roll",
            "thumb_cmc_yaw",
            "thumb_cmc_pitch",
            "index_pip",
            "index_dip",
            "middle_pip",
            "middle_dip",
            "pinky_pip",
            "pinky_dip",
            "ring_pip",
            "ring_dip",
            "thumb_mcp",
            "thumb_ip",
        ]), "target": 0.0},
    )
    # (3) 降低关节速度的奖励
    joint_vel = RewTerm(
        func=mdp.joint_vel_l1,
        weight=-0.01,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
            "index_mcp_pitch", 
            "middle_mcp_pitch", 
            "pinky_mcp_pitch", 
            "ring_mcp_pitch", 
            "thumb_cmc_roll",
            "thumb_cmc_yaw",
            "thumb_cmc_pitch",
            "index_pip",
            "index_dip",
            "middle_pip",
            "middle_dip",
            "pinky_pip",
            "pinky_dip",
            "ring_pip",
            "ring_dip",
            "thumb_mcp",
            "thumb_ip",            
        ])},
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    # (1) Time out
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    # (2) 关节超出安全范围
    joint_out_of_bounds = DoneTerm(
        func=mdp.joint_pos_out_of_manual_limit,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[
            "index_mcp_pitch", 
            "middle_mcp_pitch", 
            "pinky_mcp_pitch", 
            "ring_mcp_pitch", 
            "thumb_cmc_roll",
            "thumb_cmc_yaw",
            "thumb_cmc_pitch"
        ]), "bounds": (-2.0, 2.0)},
    )


##
# Environment configuration
##


@configclass
class LinkerhandEnvCfg(ManagerBasedRLEnvCfg):
    # Scene settings
    scene: LinkerhandSceneCfg = LinkerhandSceneCfg(num_envs=1, env_spacing=4.0)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    
    # 添加teleop_devices配置，包含数据转换器
    teleop_devices: DevicesCfg = DevicesCfg(
        devices={
            "ros2_gloves": ROS2BridgeDeviceCfg(
                left_hand_topic="/cb_left_hand_control_cmd",
                right_hand_topic="/cb_right_hand_control_cmd", 
                message_timeout=1.0,
                use_isaacsim_bridge=True,
                # 添加数据转换器
                retargeters=[LinkerhandRetargeterCfg()]
            ),
        }
    )
    
    # 添加idle_action配置（初始位置）
    # idle_action = torch.tensor([0, 0, 0, 0, 0.0, 0, 0])  # 7个关节的中间位置
    idle_action = torch.tensor([0, 0, 0, 0, 0, 0, 0, 
                                0, 0, 0, 0, 0, 0, 0, 
                                0, 0, 0])  # 17个关节的中间位置

    # Post initialization
    def __post_init__(self) -> None:
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.episode_length_s = 7
        # viewer settings
        self.viewer.eye = (8.0, 0.0, 5.0)
        # simulation settings
        self.sim.dt = 1 / 120
        self.sim.render_interval = self.decimation