# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for Linkhand dexterous hand robot."""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##

LINKHAND_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        # usd_path="/home/yaoyh-4090/IsaacLab/assets/robots/linkhand/o7_left/linkerhand_o7_left/linkerhand_o7_left.usd",  # 使用单个左手的USD文件
        usd_path="/home/yaoyh-4090/IsaacLab/assets/robots/linkhand/linkerhand_l7_left_V1/linkerhand_l7_left_V1.usd",  # 使用单个左手的USD文件
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=1.0,
            angular_damping=1.0,
            max_linear_velocity=10.0,
            max_angular_velocity=10.0,
            max_depenetration_velocity=2.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,  # 灵巧手需要自碰撞检测
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=1,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.0),  # 初始位置
        # rot=(1.0, 0.0, 0.0, 0.0),  # 初始旋转
        rot = (0.0, 0.0, 0.0, 1.0),
        joint_pos={
            # 使用精确的关节名称匹配，避免正则表达式冲突
            "index_mcp_pitch": 0.0,
            "index_pip": 0.0,
            "index_dip": 0.0,
            "middle_mcp_pitch": 0.0,
            "middle_pip": 0.0,
            "middle_dip": 0.0,
            "pinky_mcp_pitch": 0.0,
            "pinky_pip": 0.0,
            "pinky_dip": 0.0,
            "ring_mcp_pitch": 0.0,
            "ring_pip": 0.0,
            "ring_dip": 0.0,
            "thumb_cmc_roll": 0.0,
            "thumb_cmc_yaw": 0.0,
            "thumb_cmc_pitch": 0.0,
            "thumb_mcp": 0.0,
            "thumb_ip": 0.0,
        },
        joint_vel={".*": 0.0},  # 所有关节初始速度为0
    ),
    soft_joint_pos_limit_factor=0.9,  # 软关节位置限制因子
    actuators={
        "fingers": ImplicitActuatorCfg(
            joint_names_expr=[
                # "thumb_cmc_roll", "thumb_cmc_yaw", "thumb_cmc_pitch", "thumb_mcp", "thumb_ip",
                # "index_mcp_pitch", "index_pip", "index_dip",
                # "middle_mcp_pitch", "middle_pip", "middle_dip",
                # "ring_mcp_pitch", "ring_pip", "ring_dip",
                # "pinky_mcp_pitch", "pinky_pip", "pinky_dip"
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
            effort_limit=100.0,  # 手指关节力矩限制
            velocity_limit=100.0,  # 手指关节速度限制
            stiffness=200.0,     # 刚度
            damping=1.0,        # 阻尼
        ),
        # 为mimic关节添加零刚度控制器，避免控制器干扰
        # "mimic_joints": ImplicitActuatorCfg(
        #     joint_names_expr=[
        #         "index_pip", "index_dip",
        #         "middle_pip", "middle_dip", 
        #         "pinky_pip", "pinky_dip",
        #         "ring_pip", "ring_dip",
        #         "thumb_mcp", "thumb_ip"
        #     ],
        #     effort_limit=0.0,
        #     velocity_limit=0.0,
        #     stiffness=0.0,  # 关键：设置刚度为零，让USD mimic关系生效
        #     damping=0.0,
        # ),
    },
)
"""Configuration for Linkhand dexterous hand robot."""