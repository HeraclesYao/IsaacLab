# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Simplified configuration for ROS2 communication verification with Unitree G1 robot."""

import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.devices.device_base import DevicesCfg
# Remove the problematic import
# from isaaclab.devices.openxr.retargeters.humanoid.unitree.inspire.g1_upper_body_retargeter import UnitreeG1RetargeterCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs.mdp.actions.actions_cfg import JointPositionActionCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

# Import the correct ROS2 device - use ROS2BridgeDeviceCfg from ros2_bridge_device
from isaaclab.devices.ros2_bridge_device import ROS2BridgeDeviceCfg

from isaaclab_assets.robots.unitree import G1_INSPIRE_FTP_CFG  # isort: skip


##
# Simplified Scene definition
##
@configclass
class SimpleG1SceneCfg(InteractiveSceneCfg):
    """Simplified scene configuration for ROS2 verification."""

    # Humanoid robot
    robot: ArticulationCfg = G1_INSPIRE_FTP_CFG.replace(
        prim_path="/World/envs/env_.*/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0, 0, 1.0),
            rot=(0.7071, 0, 0, 0.7071),
            joint_pos={
                # right-arm
                "right_shoulder_pitch_joint": 0.0,
                "right_shoulder_roll_joint": 0.0,
                "right_shoulder_yaw_joint": 0.0,
                "right_elbow_joint": 0.0,
                "right_wrist_yaw_joint": 0.0,
                "right_wrist_roll_joint": 0.0,
                "right_wrist_pitch_joint": 0.0,
                # left-arm
                "left_shoulder_pitch_joint": 0.0,
                "left_shoulder_roll_joint": 0.0,
                "left_shoulder_yaw_joint": 0.0,
                "left_elbow_joint": 0.0,
                "left_wrist_yaw_joint": 0.0,
                "left_wrist_roll_joint": 0.0,
                "left_wrist_pitch_joint": 0.0,
                # -- left/right hand
                ".*_thumb_.*": 0.0,
                ".*_index_.*": 0.0,
                ".*_middle_.*": 0.0,
                ".*_ring_.*": 0.0,
                ".*_pinky_.*": 0.0,
            },
            joint_vel={".*": 0.0},
        ),
    )

    # Ground plane
    ground = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        spawn=GroundPlaneCfg(),
    )

    # Simple light
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )


##
# Simplified MDP settings
##
@configclass
class SimpleActionsCfg:
    """Simple action specifications using joint position control."""

    joint_pos_cfg = JointPositionActionCfg(
        asset_name="robot",
        joint_names=[
            # Arm joints (14 joints total for both arms)
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_yaw_joint",
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_yaw_joint",
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint",
            # Hand joints (24 joints total for both hands)
            "L_index_proximal_joint",
            "L_middle_proximal_joint",
            "L_pinky_proximal_joint",
            "L_ring_proximal_joint",
            "L_thumb_proximal_yaw_joint",
            "R_index_proximal_joint",
            "R_middle_proximal_joint",
            "R_pinky_proximal_joint",
            "R_ring_proximal_joint",
            "R_thumb_proximal_yaw_joint",
            "L_index_intermediate_joint",
            "L_middle_intermediate_joint",
            "L_pinky_intermediate_joint",
            "L_ring_intermediate_joint",
            "L_thumb_proximal_pitch_joint",
            "R_index_intermediate_joint",
            "R_middle_intermediate_joint",
            "R_pinky_intermediate_joint",
            "R_ring_intermediate_joint",
            "R_thumb_proximal_pitch_joint",
            "L_thumb_intermediate_joint",
            "R_thumb_intermediate_joint",
            "L_thumb_distal_joint",
            "R_thumb_distal_joint",
        ],
        scale=1.0,
    )


@configclass
class SimpleObservationsCfg:
    """Simple observation specifications for ROS2 verification."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        robot_joint_pos = ObsTerm(
            func=lambda env, asset_cfg: env.scene[asset_cfg.name].data.joint_pos[:, asset_cfg.joint_ids],
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"])},
        )
        
        robot_root_pos = ObsTerm(func=lambda env, asset_cfg: env.scene[asset_cfg.name].data.root_pos_w, 
                                params={"asset_cfg": SceneEntityCfg("robot")})
        
        robot_root_rot = ObsTerm(func=lambda env, asset_cfg: env.scene[asset_cfg.name].data.root_quat_w, 
                                params={"asset_cfg": SceneEntityCfg("robot")})

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class SimpleTerminationsCfg:
    """Simple termination terms."""

    time_out = DoneTerm(func=lambda env: env.episode_length_buf >= env.max_episode_length, time_out=True)


@configclass
class SimpleEventCfg:
    """Simple configuration for events."""

    # Fix the lambda function to accept both env and env_ids parameters
    reset_all = EventTerm(func=lambda env, env_ids: env.scene.reset(env_ids), mode="reset")


@configclass
class PickPlaceG1ROS2VerifyEnvCfg(ManagerBasedRLEnvCfg):
    """Simplified configuration for ROS2 communication verification."""

    # Scene settings
    scene: SimpleG1SceneCfg = SimpleG1SceneCfg(num_envs=1, env_spacing=2.5, replicate_physics=True)
    # Basic settings
    observations: SimpleObservationsCfg = SimpleObservationsCfg()
    actions: SimpleActionsCfg = SimpleActionsCfg()
    # MDP settings
    terminations: SimpleTerminationsCfg = SimpleTerminationsCfg()
    events = SimpleEventCfg()

    # Unused managers
    commands = None
    rewards = None
    curriculum = None

    # Idle action to hold robot in default pose
    idle_action = torch.zeros(38)  # 14 arm joints + 24 hand joints

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 6
        self.episode_length_s = 60.0  # Longer episode for testing
        # simulation settings
        self.sim.dt = 1 / 120  # 120Hz
        self.sim.render_interval = 2

        # Configure ROS2 device for teleoperation (use ROS2BridgeDeviceCfg)
        # Remove the retargeters configuration to avoid DexRetargeting compatibility issues
        self.teleop_devices = DevicesCfg(
            devices={
                "ros2_gloves": ROS2BridgeDeviceCfg(
                    left_hand_topic="/cb_left_hand_control_cmd",
                    right_hand_topic="/cb_right_hand_control_cmd",
                    message_timeout=1.0,
                    use_isaacsim_bridge=True,
                    # Remove retargeters configuration
                    # retargeters=[
                    #     UnitreeG1RetargeterCfg(
                    #         enable_visualization=True,
                    #         num_open_xr_hand_joints=2 * 26,
                    #         sim_device=self.sim.device,
                    #         hand_joint_names=self.actions.joint_pos_cfg.joint_names[-24:],  # Last 24 are hand joints
                    #     ),
                    # ],
                ),
            },
        )