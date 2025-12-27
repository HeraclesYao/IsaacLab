#------------------------------------------------------------------------------
# HoloLens2控制G1机器人遥操作脚本
# 基于Isaac-PickPlace-G1-InspireFTP-Abs-v0任务
#------------------------------------------------------------------------------

"""Launch Isaac Sim Simulator first."""

import argparse
import numpy as np
import torch
import gymnasium as gym

from isaaclab.app import AppLauncher

# 添加命令行参数
parser = argparse.ArgumentParser(description="HoloLens2控制G1机器人遥操作")
parser.add_argument("--num_envs", type=int, default=1, help="环境数量")
parser.add_argument("--task", type=str, default="Isaac-PickPlace-G1-InspireFTP-Abs-v0", 
                   help="任务名称")
parser.add_argument("--hololens_host", type=str, default="192.168.11.121",
                   help="HoloLens2设备IP地址")
parser.add_argument("--enable_visualization", action="store_true", default=True,
                   help="启用可视化")

# 添加AppLauncher参数
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# 启动Omniverse应用
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""以下是主要实现"""
import sys
sys.path.append("/home/yaoyh/workspace/IsaacLab/hl2ss/")
sys.path.append("/home/yaoyh/workspace/IsaacLab/hl2ss/viewer/")
import hl2ss
import hl2ss_lnm
import hl2ss_mp
from isaaclab_adapter import HoloLens2DeviceAdapter

from isaaclab.devices.openxr.retargeters.humanoid.unitree.inspire.g1_upper_body_retargeter import (
    UnitreeG1Retargeter, UnitreeG1RetargeterCfg
)
# 移除有问题的导入
# from isaaclab_tasks.utils import parse_env_cfg

# 添加G1 PickPlace任务模块的导入，确保任务被正确注册
import isaaclab_tasks.manager_based.manipulation.pick_place.pickplace_unitree_g1_inspire_hand_env_cfg
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_tasks.manager_based.manipulation.pick_place.pickplace_unitree_g1_inspire_hand_env_cfg import PickPlaceG1InspireFTPEnvCfg
import os
import tempfile
from pathlib import Path
from isaaclab_assets.robots.unitree import G1_29DOF_CFG
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR, ISAAC_NUCLEUS_DIR


class HoloLens2G1Teleoperation:
    """HoloLens2控制G1机器人的遥操作类"""
    
    def __init__(self, task_name: str, hololens_host: str, num_envs: int = 1):
        self.task_name = task_name
        self.hololens_host = hololens_host
        self.num_envs = num_envs
        
        # 在创建环境之前，先打印所有已注册的任务，用于调试
        print("所有已注册的任务:", gym.envs.registry.keys())
        
        # 创建环境配置
        env_cfg = PickPlaceG1InspireFTPEnvCfg()
        
        # 根据命令行参数调整配置
        env_cfg.scene.num_envs = num_envs
        
        # 使用本地G1模型文件，避免临时生成网格文件的问题
        # self._use_local_g1_model(env_cfg)
        env_cfg.actions.pink_ik_cfg.controller.urdf_path = "/home/yaoyh/workspace/IsaacLab/assets/G1_models/g1_29dof_inspire_hand/g1_29dof_inspire_hand.urdf"
        env_cfg.actions.pink_ik_cfg.controller.mesh_path = "/home/yaoyh/workspace/IsaacLab/assets/G1_models/g1_29dof_inspire_hand/meshes/"
        
        # 直接实例化环境，不使用gym.make()
        self.env = ManagerBasedRLEnv(cfg=env_cfg)
        
        # 创建HoloLens2适配器
        self.hololens_adapter = HoloLens2DeviceAdapter(host=hololens_host)
        
        # 创建G1重定向器 - 需要从环境配置中获取手部关节名称
        # 由于我们不再使用parse_env_cfg，需要手动设置手部关节名称
        hand_joint_names = [
            # 左手关节名称
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_pitch_joint", "left_wrist_yaw_joint", "left_wrist_pitch_joint",
            # 左手手指关节
            "left_thumb_1_joint", "left_thumb_2_joint", "left_thumb_3_joint",
            "left_index_1_joint", "left_index_2_joint", "left_index_3_joint",
            "left_middle_1_joint", "left_middle_2_joint", "left_middle_3_joint",
            "left_ring_1_joint", "left_ring_2_joint", "left_ring_3_joint",
            "left_pinky_1_joint", "left_pinky_2_joint", "left_pinky_3_joint",
            # 右手关节名称
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_pitch_joint", "right_wrist_yaw_joint", "right_wrist_pitch_joint",
            # 右手手指关节
            "right_thumb_1_joint", "right_thumb_2_joint", "right_thumb_3_joint",
            "right_index_1_joint", "right_index_2_joint", "right_index_3_joint",
            "right_middle_1_joint", "right_middle_2_joint", "right_middle_3_joint",
            "right_ring_1_joint", "right_ring_2_joint", "right_ring_3_joint",
            "right_pinky_1_joint", "right_pinky_2_joint", "right_pinky_3_joint"
        ]
        
        self.retargeter_cfg = UnitreeG1RetargeterCfg(
            enable_visualization=True,
            num_open_xr_hand_joints=2 * 26,  # 两只手各26个关节
            sim_device=torch.device("cpu"),
            hand_joint_names=hand_joint_names
        )
        self.retargeter = UnitreeG1Retargeter(self.retargeter_cfg)
        
        # 控制状态
        self.teleoperation_active = True
        self.should_reset = False
        
    def connect_hololens(self) -> bool:
        """连接HoloLens2设备"""
        return self.hololens_adapter.open()
    
    def start_teleoperation(self):
        """开始遥操作"""
        self.teleoperation_active = True
        print("遥操作已激活")
    
    def stop_teleoperation(self):
        """停止遥操作"""
        self.teleoperation_active = False
        print("遥操作已停止")
    
    def reset_environment(self):
        """重置环境"""
        self.should_reset = True
        print("环境重置已触发")
    
    def run(self):
        """运行遥操作主循环"""
        # 连接HoloLens2
        if not self.connect_hololens():
            print("无法连接HoloLens2，退出")
            return
        
        # 重置环境
        self.env.reset()
        print("环境初始化完成")
        
        print("HoloLens2 G1遥操作已启动")
        print("左手控制G1左手，右手控制G1右手")
        print("按Ctrl+C退出")
        
        try:
            while simulation_app.is_running():
                with torch.inference_mode():
                    # 获取HoloLens2数据
                    hololens_data = self.hololens_adapter.advance()
                    
                    if hololens_data and self.teleoperation_active:
                        # 使用重定向器转换为机器人控制命令
                        robot_commands = self.retargeter.forward(hololens_data)
                        
                        if robot_commands is not None:
                            # 应用控制命令到环境
                            actions = torch.tensor(robot_commands).repeat(self.env.num_envs, 1)
                            obs, reward, terminated, truncated, info = self.env.step(actions)
                    
                    # 处理环境重置
                    if self.should_reset:
                        self.env.reset()
                        self.should_reset = False
                        print("环境重置完成")
                    
                    # 渲染场景
                    self.env.sim.render()
                        
        except KeyboardInterrupt:
            print("\n用户中断操作")
        except Exception as e:
            print(f"运行时错误: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        self.hololens_adapter.close()
        self.env.close()
        print("资源清理完成")

    def _use_local_g1_model(self, env_cfg):
        """使用本地G1模型文件，避免临时生成网格文件的问题"""
        print("使用本地G1模型文件...")
        
        # 检查本地G1模型文件是否存在
        g1_usd_paths = [
            # 用户下载的G1模型文件路径
            "/home/yaoyh/workspace/IsaacLab/assets/G1_models/g1_29dof_with_inspire_rev_1_0.usd",
            # 其他可能的本地路径
            f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/G1/g1_29dof_with_inspire_rev_1_0.usd",
            f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/G1/g1.usd",
            f"{ISAAC_NUCLEUS_DIR}/Robots/Unitree/G1/g1.usd",
            f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/G1/g1_minimal.usd"
        ]
        
        local_g1_path = None
        for path in g1_usd_paths:
            if os.path.exists(path):
                local_g1_path = path
                print(f"找到本地G1模型文件: {path}")
                break
        
        if local_g1_path is None:
            print("未找到本地G1模型文件，尝试从Nucleus服务器下载...")
            # 尝试使用G1_29DOF_CFG中的路径
            try:
                g1_cfg = G1_29DOF_CFG
                local_g1_path = g1_cfg.spawn.usd_path
                print(f"使用G1配置中的模型路径: {local_g1_path}")
            except Exception as e:
                print(f"获取G1模型路径失败: {e}")
                # 回退到默认行为
                self._fix_mesh_paths(env_cfg)
                return
        
        # 更新环境配置中的机器人模型路径
        if hasattr(env_cfg, 'scene') and hasattr(env_cfg.scene, 'robot'):
            env_cfg.scene.robot.spawn.usd_path = local_g1_path
            print(f"已更新机器人模型路径为: {local_g1_path}")
        
        # 禁用Pink IK控制器的网格加载，使用无网格模式
        if hasattr(env_cfg.actions, 'pink_ik_cfg') and hasattr(env_cfg.actions.pink_ik_cfg, 'controller'):
            controller_cfg = env_cfg.actions.pink_ik_cfg.controller
            controller_cfg.mesh_path = None  # 禁用网格加载
            print("已禁用Pink IK控制器的网格加载，使用无网格模式")
            
            # 如果可能，提供URDF文件的本地路径
            # 检查是否有预先生成的URDF文件
            urdf_search_paths = [
                "/home/yaoyh/workspace/IsaacLab/assets/G1_models/",
                "/home/yaoyh/workspace/IsaacLab/source/isaaclab_assets/data/Robots/Unitree/G1/",
                "/home/yaoyh/.local/share/ov/pkg/isaac_sim-2023.1.1/isaaclab/data/Robots/Unitree/G1/",
                tempfile.gettempdir() + "/urdf/"
            ]
            
            for urdf_dir in urdf_search_paths:
                if os.path.exists(urdf_dir):
                    urdf_files = [f for f in os.listdir(urdf_dir) if f.endswith('.urdf')]
                    if urdf_files:
                        urdf_path = os.path.join(urdf_dir, urdf_files[0])
                        controller_cfg.urdf_path = urdf_path
                        print(f"使用预生成的URDF文件: {urdf_path}")
                        break

    def _fix_mesh_paths(self, env_cfg):
        """修复网格文件路径问题（作为备选方案）"""
        print("使用备选方案：修复网格路径...")
        
        # 检查临时目录中的网格文件
        temp_dir = tempfile.gettempdir()
        mesh_dir = os.path.join(temp_dir, "meshes")
        
        # 强制重新生成URDF和网格文件
        if hasattr(env_cfg, 'scene') and hasattr(env_cfg.scene, 'robot'):
            import isaaclab.controllers.utils as ControllerUtils
            
            print("强制重新生成URDF和网格文件...")
            try:
                # 确保临时目录存在
                os.makedirs(env_cfg.temp_urdf_dir, exist_ok=True)
                
                # 重新转换USD到URDF
                temp_urdf_output_path, temp_urdf_meshes_output_path = ControllerUtils.convert_usd_to_urdf(
                    env_cfg.scene.robot.spawn.usd_path, 
                    env_cfg.temp_urdf_dir, 
                    force_conversion=True
                )
                
                # 更新Pink IK控制器的配置
                if hasattr(env_cfg.actions, 'pink_ik_cfg') and hasattr(env_cfg.actions.pink_ik_cfg, 'controller'):
                    controller_cfg = env_cfg.actions.pink_ik_cfg.controller
                    controller_cfg.urdf_path = temp_urdf_output_path
                    controller_cfg.mesh_path = temp_urdf_meshes_output_path
                    
                    print(f"已更新URDF路径: {temp_urdf_output_path}")
                    print(f"已更新网格路径: {temp_urdf_meshes_output_path}")
                    
                    # 确保网格路径存在
                    if os.path.exists(temp_urdf_meshes_output_path):
                        print("新的网格路径存在")
                    else:
                        print("新的网格路径不存在，尝试创建目录")
                        os.makedirs(temp_urdf_meshes_output_path, exist_ok=True)
                        
            except Exception as e:
                print(f"重新生成URDF时出错: {e}")
                # 回退到不使用网格的配置
                if hasattr(env_cfg.actions, 'pink_ik_cfg') and hasattr(env_cfg.actions.pink_ik_cfg, 'controller'):
                    controller_cfg = env_cfg.actions.pink_ik_cfg.controller
                    controller_cfg.mesh_path = None
                    print("已禁用网格加载，使用无网格模式")

        # 添加网格路径到Pinocchio的搜索路径
        if os.path.exists(mesh_dir):
            # 设置环境变量，让Pinocchio能够找到网格文件
            os.environ['PINOCCHIO_MODEL_PATH'] = mesh_dir + ':' + os.environ.get('PINOCCHIO_MODEL_PATH', '')
            print(f"已设置PINOCCHIO_MODEL_PATH环境变量: {mesh_dir}")


def main():
    """主函数"""
    print("task name is %s",args_cli.task)
    teleop = HoloLens2G1Teleoperation(
        task_name=args_cli.task,
        hololens_host=args_cli.hololens_host,
        num_envs=args_cli.num_envs
    )
    
    teleop.run()


if __name__ == "__main__":
    main()
    simulation_app.close()