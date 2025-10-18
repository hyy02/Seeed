#!/usr/bin/env python
"""
最简单的机器人位置监控脚本
每5秒打印一次当前关节位置
"""

import time


def monitor_robot_positions(interval=5):
    """
    监控机器人位置
    
    Args:
        interval: 打印间隔（秒）
    """
    print("正在初始化...")

    # 方法1: 尝试使用已有的机器人实例
    try:
        # 如果你已经有一个运行中的机器人对象，可以直接传入
        from lerobot.robots.xlerobot import XLerobot, XLerobotConfig
        robot_config = XLerobotConfig(id='xlerobot')
        robot = XLerobot(robot_config)
        robot.connect()
        
        
        print("✓ 机器人已连接")
        print(f"开始监控（每{interval}秒刷新）")
        print("按 Ctrl+C 退出\n")
        
        counter = 0
        while True:
            try:
                counter += 1
                print(f"\n{'='*70}")
                print(f"读取 #{counter} - {time.strftime('%Y-%m-%d %H:%M:%S')}")
                print('='*70)
                
                # 获取观测
                obs = robot.get_observation()
                
                # 打印左臂
                print("\n左臂:")
                for joint in ["shoulder_pan", "shoulder_lift", "elbow_flex", 
                             "wrist_flex", "wrist_roll", "gripper"]:
                    key = f"left_arm_{joint}.pos"
                    if key in obs:
                        print(f"  {joint:15s}: {obs[key]:7.4f}")
                
                # 打印右臂
                print("\n右臂:")
                for joint in ["shoulder_pan", "shoulder_lift", "elbow_flex",
                             "wrist_flex", "wrist_roll", "gripper"]:
                    key = f"right_arm_{joint}.pos"
                    if key in obs:
                        print(f"  {joint:15s}: {obs[key]:7.4f}")
                
                # 打印头部
                print("\n头部:")
                for i in [1, 2]:
                    key = f"head_motor_{i}.pos"
                    if key in obs:
                        print(f"  motor_{i:1d}       : {obs[key]:7.4f}")
                
                print()
                time.sleep(interval)
                
            except KeyboardInterrupt:
                print("\n\n正在退出...")
                robot.disconnect()
                break
            except Exception as e:
                print(f"读取错误: {e}")
                time.sleep(interval)
    
    except ImportError:
        print("方法1失败，尝试方法2...")
        monitor_with_bus(interval)
    except Exception as e:
        print(f"初始化失败: {e}")
        print("\n尝试使用电机总线直接读取...")
        monitor_with_bus(interval)
    finally:
        if 'robot' in locals():
            try:
                robot.disconnect()
                print("已断开连接")
            except:
                pass


def monitor_with_bus(interval=5):
    """使用电机总线直接监控"""
    import sys
    from pathlib import Path
    
    sys.path.insert(0, str(Path(__file__).parent / "src"))
    
    from lerobot.motors.dynamixel import DynamixelMotorsBus
    
    # 配置所有电机
    all_motors = {
        # 左臂
        "L_shoulder_pan": (1, "x_series"),
        "L_shoulder_lift": (2, "x_series"),
        "L_elbow_flex": (3, "x_series"),
        "L_wrist_flex": (4, "x_series"),
        "L_wrist_roll": (5, "x_series"),
        "L_gripper": (6, "x_series"),
        
        # 右臂
        "R_shoulder_pan": (11, "x_series"),
        "R_shoulder_lift": (12, "x_series"),
        "R_elbow_flex": (13, "x_series"),
        "R_wrist_flex": (14, "x_series"),
        "R_wrist_roll": (15, "x_series"),
        "R_gripper": (16, "x_series"),
    }
    
    # 尝试常见端口
    ports = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0"]
    
    bus = None
    for port in ports:
        try:
            print(f"尝试连接 {port}...")
            bus = DynamixelMotorsBus(port=port, motors=all_motors)
            bus.connect()
            print(f"✓ 已连接到 {port}")
            break
        except:
            continue
    
    if bus is None:
        print("无法连接到任何端口！")
        return
    
    try:
        print(f"开始监控（每{interval}秒刷新）")
        print("按 Ctrl+C 退出\n")
        
        counter = 0
        while True:
            counter += 1
            print(f"\n{'='*70}")
            print(f"读取 #{counter} - {time.strftime('%H:%M:%S')}")
            print('='*70)
            
            positions = bus.read("Present_Position")
            
            print("\n左臂:")
            for name in ["L_shoulder_pan", "L_shoulder_lift", "L_elbow_flex",
                        "L_wrist_flex", "L_wrist_roll", "L_gripper"]:
                if name in positions:
                    print(f"  {name:15s}: {positions[name]:7.4f}")
            
            print("\n右臂:")
            for name in ["R_shoulder_pan", "R_shoulder_lift", "R_elbow_flex",
                        "R_wrist_flex", "R_wrist_roll", "R_gripper"]:
                if name in positions:
                    print(f"  {name:15s}: {positions[name]:7.4f}")
            
            time.sleep(interval)
            
    except KeyboardInterrupt:
        print("\n\n正在退出...")
    finally:
        bus.disconnect()
        print("已断开连接")


if __name__ == "__main__":
    import sys
    
    # 可以通过命令行参数设置间隔
    interval = 5
    if len(sys.argv) > 1:
        try:
            interval = float(sys.argv[1])
        except:
            print(f"无效的间隔参数，使用默认值 {interval} 秒")
    
    print(f"机器人位置监控脚本")
    print(f"刷新间隔: {interval} 秒")
    print()
    
    monitor_robot_positions(interval)


# """
# 简单的 XLerobot 位置控制函数
# 直接操作电机总线，类似 gym_manipulator.py 中的实现
# """

# from collections import deque
# import numpy as np
# import torch
# import time


# # def reset_follower_position(robot, target_position, steps=150, delay=0.015):
# #     """
# #     让机器人平滑移动到目标位置
    
# #     Args:
# #         robot: 机器人对象 (需要有 bus 属性)
# #         target_position: 目标位置数组 [left_arm_joints..., right_arm_joints...]
# #         steps: 轨迹步数 (默认50)
# #         delay: 每步延迟时间 (默认15ms)
# #     """
# #     # 读取当前位置
    
# #     left_current_position_dict = robot.bus1.sync_read("Present_Position")
# #     right_current_position_dict = robot.bus2.sync_read("Present_Position")
# #     left_current_position = np.array(
# #         [left_current_position_dict[name] for name in left_current_position_dict], dtype=np.float32
# #     )
# #     right_current_position = np.array(
# #         [right_current_position_dict[name] for name in right_current_position_dict], dtype=np.float32
# #     )
# #     left_target_position, right_target_position = target_position[0:6], target_position[6:12]
# #     # 生成平滑轨迹
# #     left_trajectory = torch.from_numpy(
# #         np.linspace(left_current_position, np.concatenate((left_target_position, left_current_position[-2:])), steps)
# #     )
# #     right_trajectory = torch.from_numpy(
# #         np.linspace(right_current_position, np.concatenate((right_target_position, right_current_position[-3:])), steps)
# #     )
    
# #     print("Moving to target position...")
    
# #     # 执行轨迹
# #     for left_pose, right_pose in zip(left_trajectory, right_trajectory):
# #         left_action_dict = dict(zip(left_current_position_dict, left_pose, strict=False))
# #         right_action_dict = dict(zip(right_current_position_dict, right_pose, strict=False))
# #         robot.bus1.sync_write("Goal_Position", left_action_dict)
# #         robot.bus2.sync_write("Goal_Position", right_action_dict)
# #         time.sleep(delay)  # 使用 time.sleep 替代 busy_wait
    
# #     print("Reached target position!")



# import numpy as np
# import torch
# import time

# GLOBAL_BACK_GOAL = np.array([-99.1578, -61.1670, 42.1723, 82.5829, 5.7592, 0.8043, 99.8230, -61.0739, 41.0242, 82.1322, -0.6326, 0.4778])
# GLOBAL_OPEN_GOAL = np.array([-103.1578, -61.1670, 42.1723, 82.5829, 5.7592, 30, 103.8230, -61.0739, 41.0242, 82.1322, -0.6326, 30])

# def reset_follower_position(robot, target_position, steps=50, delay=0.015, start_position=None):
#     """
#     让机器人平滑移动到目标位置，生成可记录的动作序列
    
#     Args:
#         robot: 机器人对象 (需要有 bus1, bus2 属性)
#         target_position: 目标位置数组 [left_arm_joints..., right_arm_joints...]
#         steps: 轨迹步数 (默认150)
#         delay: 每步延迟时间 (默认15ms)
    
#     Returns:
#         list: 动作序列，每个元素是一个动作字典
#     """
#     # 读取当前位置
#     left_current_position_dict = robot.bus1.sync_read("Present_Position")
#     # 将键名统一为 '<motor_name>.pos' 形式，便于与其他代码中的键名一致
#     right_current_position_dict = robot.bus2.sync_read("Present_Position")
#     if start_position is not None:
#         left_current_position, right_current_position = start_position[0:6], start_position[6:12]
#     else:
#         left_current_position = np.array(
#             [left_current_position_dict[name] for name in left_current_position_dict], dtype=np.float32
#         )
#         right_current_position = np.array(
#             [right_current_position_dict[name] for name in right_current_position_dict], dtype=np.float32
#         )
    
#     # 分离左右臂目标位置
#     left_target_position, right_target_position = target_position[0:6], target_position[6:12]
    
#     # 生成平滑轨迹 (保持最后几个关节不变)
#     if start_position is None:
#         left_trajectory = torch.from_numpy(
#             np.linspace(left_current_position, np.concatenate((left_target_position, left_current_position[-2:])), steps)
#         )
#         right_trajectory = torch.from_numpy(
#             np.linspace(right_current_position[:-3], right_target_position, steps)
#         )
#     else:
#         left_trajectory = torch.from_numpy(
#             np.linspace(left_current_position, left_target_position, steps)
#         )
#         right_trajectory = torch.from_numpy(
#             np.linspace(right_current_position, right_target_position, steps)
#         )
    
#     # 生成动作序列
#     action_sequence = []
#     left_current_position_dict = {f"{k}.pos" for k in left_current_position_dict}
#     left_current_position_dict = [
#         'left_arm_shoulder_pan.pos',
#         'left_arm_shoulder_lift.pos',
#         'left_arm_elbow_flex.pos',
#         'left_arm_wrist_flex.pos',
#         'left_arm_wrist_roll.pos',
#         'left_arm_gripper.pos',
#     ]
#     right_current_position_dict = [
#         'right_arm_shoulder_pan.pos',
#         'right_arm_shoulder_lift.pos',
#         'right_arm_elbow_flex.pos',
#         'right_arm_wrist_flex.pos',
#         'right_arm_wrist_roll.pos',
#         'right_arm_gripper.pos'
#     ]
#     if start_position is None:
#         left_current_position_dict += ['head_motor_1.pos', 'head_motor_2.pos']
#     for left_pose, right_pose in zip(left_trajectory, right_trajectory):
#         left_action_dict = dict(zip(left_current_position_dict, left_pose, strict=False))
#         right_action_dict = dict(zip(right_current_position_dict, right_pose, strict=False))
#         if start_position is None:
#             head_motor_dict = {}
#         else:
#             head_motor_dict = {
#                 'head_motor_1.pos': 8.982,
#                 'head_motor_2.pos': 28.8703,
#             }
#         base_action_dict = {
#             "x.vel": 0,
#             "y.vel": 0,
#             "theta.vel": 0,
#         }
        
#         # 合并为完整的动作字典
#         action_dict = {**left_action_dict, **head_motor_dict, **right_action_dict, **base_action_dict}
#         action_sequence.append(action_dict)
    
#     return action_sequence

# def queue_reset_actions(action_queue, robot, target_position, steps=50, start_position=None):
#     """
#     将重置动作序列加入队列
    
#     Args:
#         action_queue: 动作队列
#         robot: 机器人对象
#         target_position: 目标位置
#         steps: 轨迹步数
#     """
#     action_sequence = reset_follower_position(robot, target_position, steps, start_position=start_position)
#     action_queue.extend(action_sequence)

# # 使用示例
# if __name__ == "__main__":
#     from lerobot.robots.xlerobot import XLerobot, XLerobotConfig
#     robot_config = XLerobotConfig(id='xlerobot')
#     robot = XLerobot(robot_config)
#     robot.connect()
#     # 您的目标位置数据

#     action_queue = deque()
#     target_position = GLOBAL_BACK_GOAL
#     queue_reset_actions(action_queue, robot, target_position)
#     target_position = GLOBAL_OPEN_GOAL
#     queue_reset_actions(action_queue, robot, target_position, steps=20, start_position=GLOBAL_BACK_GOAL)
#     target_position = np.zeros(12)
#     queue_reset_actions(action_queue, robot, target_position,start_position=GLOBAL_OPEN_GOAL)
#                 # sychronize the target position
#     for action in action_queue:
#         robot.send_action(action)
#         time.sleep(0.02)
#     print(f"\n目标位置:")
#     time.sleep(3)
#     robot.disconnect()