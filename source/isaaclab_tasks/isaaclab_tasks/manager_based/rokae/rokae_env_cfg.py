# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math
import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.devices.device_base import DevicesCfg
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs.mdp.actions.actions_cfg import JointPositionActionCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

from . import mdp

##
# Pre-defined configs
##

# from isaaclab_assets.robots.cartpole import CARTPOLE_CFG  # isort:skip
from isaaclab.devices.ros2_bridge_device import ROS2BridgeDeviceCfg

from isaaclab_assets.robots.rokae import ROKAE_WHEEL_ROBOT_CFG  # isort: skip
from .rokae_ros2_retargeterCfg import RokaeRetargeterCfg


##
# Scene definition
##


@configclass
class RokaeSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    # robot
    robot: ArticulationCfg = ROKAE_WHEEL_ROBOT_CFG.replace(
        prim_path="/World/envs/env_.*/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0, 0, 1.0),
            rot=(0.7071, 0, 0, 0.7071),
            joint_pos={
                # 底盘轮子初始位置
                "chassis_left_Joint": 0.0,
                "chassis_right_Joint": 0.0,
                "Trunk_Joint1": 0.0,
                "Trunk_Joint2": 0.0,
                "Trunk_Joint3": 0.0,
                "Trunk_Joint4": 0.0,
                "Head_Joint1": 0.0,
                "Head_Joint2": 0.0,
                # 左臂初始位置 (AR5_5_07L)
                "AR5_5_07L_joint_1": 0.0,
                "AR5_5_07L_joint_2": 80.0 * math.pi / 180,  # 修改为80度
                "AR5_5_07L_joint_3": -90.0 * math.pi / 180,  # 修改为-90度
                "AR5_5_07L_joint_4": 0.0,
                "AR5_5_07L_joint_5": 0.0,
                "AR5_5_07L_joint_6": 0.0,
                "AR5_5_07L_joint_7": 0.0,
                # 右臂初始位置 (AR5_5_07R)
                "AR5_5_07R_joint_1": 0.0,
                "AR5_5_07R_joint_2": 80.0 * math.pi / 180,  # 修改为80度
                "AR5_5_07R_joint_3": 90.0 * math.pi / 180,  # 修改为90度
                "AR5_5_07R_joint_4": 0.0,
                "AR5_5_07R_joint_5": 0.0,
                "AR5_5_07R_joint_6": 0.0,
                "AR5_5_07R_joint_7": 0.0,
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
    """Action  specifications using joint position control."""

    joint_pos_cfg = JointPositionActionCfg(
        asset_name="robot",
        joint_names=[
            # Arm joints (14 joints total for both arms)
            "AR5_5_07L_joint_1",     ## 0  负前，正后
            "AR5_5_07R_joint_1",     ## 1  负后，正前
            "AR5_5_07L_joint_2",    ## 2  负外，正内
            "AR5_5_07R_joint_2",    ## 3  负外，正内
            "AR5_5_07L_joint_3",     ## 4  负外，正内
            "AR5_5_07R_joint_3",     ## 5  负内，正外
            "AR5_5_07L_joint_4",     ## 6  负后，正前
            "AR5_5_07R_joint_4",     ## 7  负后，正前
            "AR5_5_07L_joint_5",     ## 8  负内，正外
            "AR5_5_07R_joint_5",     ## 9  负内，正外
            "AR5_5_07L_joint_6",     ## 10  负外，正内
            "AR5_5_07R_joint_6",     ## 11  负外，正内
            "AR5_5_07L_joint_7",     ## 12  负前，正后
            "AR5_5_07R_joint_7",     ## 13  负前，正后
            # # Hand joints (24 joints total for both hands)
            # "L_index_proximal_joint",               # index 14 左手食指
            # "L_middle_proximal_joint",               # index 15 左手中指
            # "L_pinky_proximal_joint",               # index 16 左手小指
            # "L_ring_proximal_joint",               # index 17 左手无名指
            # "L_thumb_proximal_yaw_joint",               # index 18 左手拇指
            # "R_index_proximal_joint",               # index 19 右手食指
            # "R_middle_proximal_joint",               # index 20 右手中指
            # "R_pinky_proximal_joint",               # index 21 右手小指
            # "R_ring_proximal_joint",               # index 22 右手无名指
            # "R_thumb_proximal_yaw_joint",               # index 23 右手拇指
            # "L_index_intermediate_joint",               # index 24 左手食指
            # "L_middle_intermediate_joint",               # index 25 左手中指
            # "L_pinky_intermediate_joint",               # index 26 左手小指
            # "L_ring_intermediate_joint",               # index 27 左手无名指
            # "L_thumb_proximal_pitch_joint",               # index 28 左手拇指
            # "R_index_intermediate_joint",               # index 29 右手食指
            # "R_middle_intermediate_joint",               # index 30 右手中指
            # "R_pinky_intermediate_joint",               # index 31 右手小指
            # "R_ring_intermediate_joint",               # index 32 右手无名指
            # "R_thumb_proximal_pitch_joint",               # index 33 右手拇指
            # "L_thumb_intermediate_joint",               # index 34
            # "R_thumb_intermediate_joint",               # index 35
            # "L_thumb_distal_joint",               # index 36
            # "R_thumb_distal_joint",               # index 37
        ],
        scale=1.0,
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

    # reset
    reset_cart_position = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["chassis_left_Joint", "chassis_right_Joint"]),
            "position_range": (-1.0, 1.0),
            "velocity_range": (-0.5, 0.5),
        },
    )

    reset_pole_position = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["AR5_5_07L_joint_2", "AR5_5_07R_joint_2"]),
            "position_range": (-0.25 * math.pi, 0.25 * math.pi),
            "velocity_range": (-0.25 * math.pi, 0.25 * math.pi),
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # (1) Constant running reward
    alive = RewTerm(func=mdp.is_alive, weight=1.0)
    # (2) Failure penalty
    terminating = RewTerm(func=mdp.is_terminated, weight=-2.0)
    # (3) Primary task: keep pole upright
    pole_pos = RewTerm(
        func=mdp.joint_pos_target_l2,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["AR5_5_07L_joint_2", "AR5_5_07R_joint_2"]), "target": 0.0},
    )
    # (4) Shaping tasks: lower cart velocity
    cart_vel = RewTerm(
        func=mdp.joint_vel_l1,
        weight=-0.01,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["chassis_left_Joint", "chassis_right_Joint"])},
    )
    # (5) Shaping tasks: lower pole angular velocity
    pole_vel = RewTerm(
        func=mdp.joint_vel_l1,
        weight=-0.005,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["AR5_5_07L_joint_2", "AR5_5_07R_joint_2"])},
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    # (1) Time out
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    # (2) Cart out of bounds
    cart_out_of_bounds = DoneTerm(
        func=mdp.joint_pos_out_of_manual_limit,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["chassis_left_Joint", "chassis_right_Joint"]), "bounds": (-3.0, 3.0)},
    )


##
# Environment configuration
##


@configclass
class RokaeEnvCfg(ManagerBasedRLEnvCfg):
    # Scene settings
    scene: RokaeSceneCfg = RokaeSceneCfg(num_envs=4096, env_spacing=4.0)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    # Idle action to hold robot in default pose (all joints at 0.0)
    idle_action = torch.zeros(14)  # 14 arm joints (7 left + 7 right)

    # Post initialization
    def __post_init__(self) -> None:
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.episode_length_s = 5
        # viewer settings
        self.viewer.eye = (8.0, 0.0, 5.0)
        # simulation settings
        self.sim.dt = 1 / 120
        self.sim.render_interval = self.decimation
        # Configure ROS2 device for teleoperation (use ROS2BridgeDeviceCfg)
        # Remove the retargeters configuration to avoid DexRetargeting compatibility issues
        self.teleop_devices = DevicesCfg(
            devices={
                "ros2_gloves": ROS2BridgeDeviceCfg(
                    left_hand_topic="/cb_left_hand_control_cmd",
                    right_hand_topic="/cb_right_hand_control_cmd",
                    dual_arm_topic="/cb_arm_control_cmd",  # 新增：双臂控制topic
                    message_timeout=1.0,
                    use_isaacsim_bridge=True,
                    retargeters=[RokaeRetargeterCfg()]
                ),
            },
        )