# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for Rokae robots.

The following configurations are available:

* :obj:`ROKAE_WHEEL_ROBOT_CFG`: Rokae wheeled robot with dual AR5 arms
* :obj:`ROKAE_WHEEL_ROBOT_FIXED_BASE_CFG`: Rokae wheeled robot with fixed base for manipulation tasks

Reference: Rokae robot model files in /home/yaoyh-4090/IsaacLab/assets/Rokae/
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg, ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg


##
# Configuration
##

ROKAE_WHEEL_ROBOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/yaoyh-4090/IsaacLab/assets/Rokae/Rokae_Robot_l10.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.3),
        joint_pos={
            # 底盘轮子初始位置
            "chassis_left_Joint": 0.0,
            "chassis_right_Joint": 0.0,
            # 左臂初始位置 (AR5_5_07L)
            "AR5_5_07L_joint_1": 0.0,
            "AR5_5_07L_joint_2": 0.0,
            "AR5_5_07L_joint_3": 0.0,
            "AR5_5_07L_joint_4": 0.0,
            "AR5_5_07L_joint_5": 0.0,
            "AR5_5_07L_joint_6": 0.0,
            "AR5_5_07L_joint_7": 0.0,
            # 右臂初始位置 (AR5_5_07R)
            "AR5_5_07R_joint_1": 0.0,
            "AR5_5_07R_joint_2": 0.0,
            "AR5_5_07R_joint_3": 0.0,
            "AR5_5_07R_joint_4": 0.0,
            "AR5_5_07R_joint_5": 0.0,
            "AR5_5_07R_joint_6": 0.0,
            "AR5_5_07R_joint_7": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        # 底盘轮子驱动器
        "chassis_wheels": DCMotorCfg(
            joint_names_expr=["chassis_left_Joint", "chassis_right_Joint"],
            effort_limit=50.0,
            saturation_effort=100.0,  # 添加必需的saturation_effort参数
            velocity_limit=10.0,
            stiffness=100.0,
            damping=5.0,
            armature=0.01,
        ),
        # 左臂驱动器
        "left_arm": ImplicitActuatorCfg(
            joint_names_expr=["AR5_5_07L_joint_.*"],
            effort_limit={
                "AR5_5_07L_joint_1": 108.0,
                "AR5_5_07L_joint_2": 108.0,
                "AR5_5_07L_joint_3": 66.0,
                "AR5_5_07L_joint_4": 66.0,
                "AR5_5_07L_joint_5": 19.0,
                "AR5_5_07L_joint_6": 19.0,
                "AR5_5_07L_joint_7": 19.0,
            },
            velocity_limit={
                "AR5_5_07L_joint_1": 3.49,
                "AR5_5_07L_joint_2": 3.49,
                "AR5_5_07L_joint_3": 4.71,
                "AR5_5_07L_joint_4": 5.24,
                "AR5_5_07L_joint_5": 4.19,
                "AR5_5_07L_joint_6": 5.24,
                "AR5_5_07L_joint_7": 5.24,
            },
            stiffness=1000.0,
            damping=10.0,
            armature=0.001,
        ),
        # 右臂驱动器
        "right_arm": ImplicitActuatorCfg(
            joint_names_expr=["AR5_5_07R_joint_.*"],
            effort_limit={
                "AR5_5_07R_joint_1": 108.0,
                "AR5_5_07R_joint_2": 108.0,
                "AR5_5_07R_joint_3": 66.0,
                "AR5_5_07R_joint_4": 66.0,
                "AR5_5_07R_joint_5": 19.0,
                "AR5_5_07R_joint_6": 19.0,
                "AR5_5_07R_joint_7": 19.0,
            },
            velocity_limit={
                "AR5_5_07R_joint_1": 3.49,
                "AR5_5_07R_joint_2": 3.49,
                "AR5_5_07R_joint_3": 4.71,
                "AR5_5_07R_joint_4": 5.24,
                "AR5_5_07R_joint_5": 4.19,
                "AR5_5_07R_joint_6": 5.24,
                "AR5_5_07R_joint_7": 5.24,
            },
            stiffness=1000.0,
            damping=10.0,
            armature=0.001,
        ),
    },
)
"""Configuration for the Rokae wheeled robot with dual AR5 arms.

This configuration sets up a Rokae wheeled robot equipped with two AR5 robotic arms.
The robot features a mobile base with two wheels and dual 7-DOF arms for manipulation tasks.

Key features:
- Mobile base with differential drive wheels
- Dual 7-DOF AR5 robotic arms
- Configurable for both mobile and fixed base scenarios
- Optimized actuator parameters based on URDF specifications

Joint specifications (from URDF):
- AR5_5_07L/R_joint_1: Base joint, effort=108Nm, velocity=3.49rad/s, limits=±3.1067rad
- AR5_5_07L/R_joint_2: Shoulder joint, effort=108Nm, velocity=3.49rad/s, limits=±2.0944rad  
- AR5_5_07L/R_joint_3: Elbow joint, effort=66Nm, velocity=4.71rad/s, limits=±3.1067rad
- AR5_5_07L/R_joint_4: Wrist pitch, effort=66Nm, velocity=5.24rad/s, limits=-1.0472 to 2.5307rad
- AR5_5_07L/R_joint_5: Wrist roll, effort=19Nm, velocity=4.19rad/s, limits=±3.1067rad
- AR5_5_07L/R_joint_6: Wrist yaw, effort=19Nm, velocity=5.24rad/s, limits=±3.1067rad
- AR5_5_07L/R_joint_7: End effector, effort=19Nm, velocity=5.24rad/s, limits=±3.1067rad

Usage examples:
    # For mobile manipulation scenarios
    mobile_cfg = ROKAE_WHEEL_ROBOT_CFG.copy()
    
    # For fixed base manipulation (upper body only)
    fixed_cfg = ROKAE_WHEEL_ROBOT_FIXED_BASE_CFG.copy()
"""


ROKAE_WHEEL_ROBOT_FIXED_BASE_CFG = ROKAE_WHEEL_ROBOT_CFG.copy()
ROKAE_WHEEL_ROBOT_FIXED_BASE_CFG.spawn.articulation_props.fix_root_link = True
ROKAE_WHEEL_ROBOT_FIXED_BASE_CFG.init_state.pos = (0.0, 0.0, 0.5)
"""Configuration for the Rokae wheeled robot with fixed base.

This configuration fixes the robot base for manipulation-only tasks,
providing better stability for precise arm movements.

Key features:
- Fixed base for stable manipulation
- Same dual AR5 arm configuration as mobile version
- Optimized for upper body manipulation tasks
"""


# 如果需要与手部集成，可以添加手部配置
# 假设使用linkerhand手部
ROKAE_WHEEL_ROBOT_WITH_HANDS_CFG = ROKAE_WHEEL_ROBOT_CFG.copy()
ROKAE_WHEEL_ROBOT_WITH_HANDS_CFG.actuators["left_hand"] = ImplicitActuatorCfg(
    joint_names_expr=[".*_index_.*", ".*_middle_.*", ".*_thumb_.*", ".*_ring_.*", ".*_pinky_.*"],
    effort_limit=30.0,
    velocity_limit=10.0,
    stiffness=20.0,
    damping=2.0,
    armature=0.001,
)
ROKAE_WHEEL_ROBOT_WITH_HANDS_CFG.actuators["right_hand"] = ImplicitActuatorCfg(
    joint_names_expr=[".*_index_.*", ".*_middle_.*", ".*_thumb_.*", ".*_ring_.*", ".*_pinky_.*"],
    effort_limit=30.0,
    velocity_limit=10.0,
    stiffness=20.0,
    damping=2.0,
    armature=0.001,
)
"""Configuration for the Rokae wheeled robot with dual hands.

This configuration extends the basic robot with hand attachments for grasping tasks.
Note: This requires proper hand USD files and attachment configuration.
"""