import logging
import time
import numpy as np
import torch
from dataclasses import asdict, dataclass
from pathlib import Path
from pprint import pformat
from collections import deque
from enum import Enum, auto

from lerobot.cameras import (  # noqa: F401
    CameraConfig,  # noqa: F401
)
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig  # noqa: F401
from lerobot.configs import parser
from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.image_writer import safe_stop_image_writer
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import build_dataset_frame, hw_to_dataset_features
from lerobot.datasets.video_utils import VideoEncodingManager
from lerobot.policies.factory import make_policy
from lerobot.policies.pretrained import PreTrainedPolicy
from lerobot.robots import (  # noqa: F401
    Robot,
    RobotConfig,
    bi_so100_follower,
    hope_jr,
    koch_follower,
    make_robot_from_config,
    so100_follower,
    so101_follower,
    xlerobot,
)
from lerobot.teleoperators import (  # noqa: F401
    Teleoperator,
    TeleoperatorConfig,
    bi_so100_leader,
    homunculus,
    koch_leader,
    make_teleoperator_from_config,
    so100_leader,
    so101_leader,
    xlerobot_vr,
)
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop
from lerobot.teleoperators.xlerobot_vr.xlerobot_vr import XLerobotVRTeleop, init_vr_listener
from lerobot.utils.control_utils import (
    init_keyboard_listener,
    is_headless,
    predict_action,
    sanity_check_dataset_name,
    sanity_check_dataset_robot_compatibility,
)
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import (
    get_safe_torch_device,
    init_logging,
    log_say,
)
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

# 定义状态枚举
class RobotState(Enum):
    NAVIGATION = auto()    # 导航状态
    OPERATION = auto()     # 操作状态
    END = auto()           # 结束状态

GLOBAL_BACK_GOAL = np.array([-99.1578, -67.1670, 21.1723, 99.1829, 5.7592, 0.8043, 99.8230, -67.0739, 21.0242, 99.1322, -0.6326, 0.4778])
GLOBAL_OPEN_GOAL = np.array([-99.1578, -67.1670, 21.1723, 99.1829, 5.7592, 36, 99.8230, -67.0739, 21.0242, 99.1322, -0.6326, 36])
def reset_follower_position(robot, target_position, steps=50, delay=0.015, start_position=None):
    """
    让机器人平滑移动到目标位置，生成可记录的动作序列
    
    Args:
        robot: 机器人对象 (需要有 bus1, bus2 属性)
        target_position: 目标位置数组 [left_arm_joints..., right_arm_joints...]
        steps: 轨迹步数 (默认150)
        delay: 每步延迟时间 (默认15ms)
    
    Returns:
        list: 动作序列，每个元素是一个动作字典
    """
    # 读取当前位置
    left_current_position_dict = robot.bus1.sync_read("Present_Position")
    # 将键名统一为 '<motor_name>.pos' 形式，便于与其他代码中的键名一致
    right_current_position_dict = robot.bus2.sync_read("Present_Position")
    if start_position is not None:
        left_current_position, right_current_position = start_position[0:6], start_position[6:12]
    else:
        left_current_position = np.array(
            [left_current_position_dict[name] for name in left_current_position_dict], dtype=np.float32
        )
        right_current_position = np.array(
            [right_current_position_dict[name] for name in right_current_position_dict], dtype=np.float32
        )
    
    # 分离左右臂目标位置
    left_target_position, right_target_position = target_position[0:6], target_position[6:12]
    
    # 生成平滑轨迹 (保持最后几个关节不变)
    if start_position is None:
        left_trajectory = torch.from_numpy(
            np.linspace(left_current_position, np.concatenate((left_target_position, left_current_position[-2:])), steps)
        )
        right_trajectory = torch.from_numpy(
            np.linspace(right_current_position[:-3], right_target_position, steps)
        )
    else:
        left_trajectory = torch.from_numpy(
            np.linspace(left_current_position, left_target_position, steps)
        )
        right_trajectory = torch.from_numpy(
            np.linspace(right_current_position, right_target_position, steps)
        )
    
    # 生成动作序列
    action_sequence = []
    left_current_position_dict = [
        'left_arm_shoulder_pan.pos',
        'left_arm_shoulder_lift.pos',
        'left_arm_elbow_flex.pos',
        'left_arm_wrist_flex.pos',
        'left_arm_wrist_roll.pos',
        'left_arm_gripper.pos',
    ]
    right_current_position_dict = [
        'right_arm_shoulder_pan.pos',
        'right_arm_shoulder_lift.pos',
        'right_arm_elbow_flex.pos',
        'right_arm_wrist_flex.pos',
        'right_arm_wrist_roll.pos',
        'right_arm_gripper.pos'
    ]
    if start_position is None:
        left_current_position_dict += ['head_motor_1.pos', 'head_motor_2.pos']
    for left_pose, right_pose in zip(left_trajectory, right_trajectory):
        left_action_dict = dict(zip(left_current_position_dict, left_pose, strict=False))
        right_action_dict = dict(zip(right_current_position_dict, right_pose, strict=False))
        if start_position is None:
            head_motor_dict = {}
        else:
            head_motor_dict = {
                'head_motor_1.pos': 8.982,
                'head_motor_2.pos': 28.8703,
            }
        base_action_dict = {
            "x.vel": 0,
            "y.vel": 0,
            "theta.vel": 0,
        }
        
        # 合并为完整的动作字典
        action_dict = {**left_action_dict, **head_motor_dict, **right_action_dict, **base_action_dict}
        action_sequence.append(action_dict)
    
    return action_sequence

def queue_reset_actions(action_queue, robot, target_position, steps=50, start_position=None):
    """
    将重置动作序列加入队列
    
    Args:
        action_queue: 动作队列
        robot: 机器人对象
        target_position: 目标位置
        steps: 轨迹步数
    """
    action_sequence = reset_follower_position(robot, target_position, steps, start_position=start_position)
    action_queue.extend(action_sequence)
    logging.info(f"Queued {len(action_sequence)} reset actions")


class StateMachine:
    """状态机管理机器人的三个状态：导航、操作、结束"""
    
    def __init__(self, initial_state=RobotState.NAVIGATION, state_check_interval=300):
        self.current_state = initial_state
        self.state_check_interval = state_check_interval
        self.step_count = 0
        self.state_transition_log = []
        self.operation_count = 0  # 记录进入操作状态的次数
        self.max_operation_cycles = 3  # 最大操作循环次数
        
    def update(self, observation, action_queue, robot, teleop):
        """更新状态机，每state_check_interval步检查状态转换"""
        self.step_count += 1
        
        # 每state_check_interval步检查状态转换
        if self.step_count % self.state_check_interval == 0:
            new_state = self._evaluate_state_transition(observation, action_queue, robot, teleop)
            if new_state != self.current_state:
                self._transition_to_state(new_state, robot, teleop)
                
        return self.current_state
    
    def _evaluate_state_transition(self, observation, action_queue, robot, teleop):
        """评估状态转换条件"""
        
        if self.current_state == RobotState.NAVIGATION:
            # 导航状态转换条件：如果动作队列为空且机器人处于稳定位置，转到操作状态
            if len(action_queue) == 0 and self._is_robot_stable(observation):
                return RobotState.OPERATION
                
        elif self.current_state == RobotState.OPERATION:
            # 操作状态转换条件：如果操作完成，转回导航状态或结束状态
            if self._is_operation_complete(observation):
                self.operation_count += 1
                logging.info(f"操作完成，第 {self.operation_count} 次操作循环")
                
                # 如果达到最大操作循环次数，转到结束状态
                if self.operation_count >= self.max_operation_cycles:
                    return RobotState.END
                else:
                    # 否则转回导航状态进行下一次操作
                    return RobotState.NAVIGATION
                    
        elif self.current_state == RobotState.END:
            # 结束状态不进行转换
            return RobotState.END
                
        return self.current_state  # 保持当前状态
    
    def _transition_to_state(self, new_state, robot, teleop):
        """执行状态转换"""
        old_state = self.current_state
        self.current_state = new_state
        
        # 记录状态转换
        transition_msg = f"状态转换: {old_state.name} -> {new_state.name}"
        self.state_transition_log.append(transition_msg)
        logging.info(transition_msg)
        log_say(transition_msg)
        
        # 执行状态特定的初始化操作
        if new_state == RobotState.NAVIGATION:
            self._enter_navigation_state(robot, teleop)
        elif new_state == RobotState.OPERATION:
            self._enter_operation_state(robot, teleop)
        elif new_state == RobotState.END:
            self._enter_end_state(robot, teleop)
    
    def _enter_navigation_state(self, robot, teleop):
        """进入导航状态的初始化操作"""
        logging.info("进入导航状态：机器人将移动到目标位置")
        # 可以在这里添加导航相关的初始化代码
        
    def _enter_operation_state(self, robot, teleop):
        """进入操作状态的初始化操作"""
        logging.info(f"进入操作状态：执行第 {self.operation_count + 1} 次具体任务操作")
        # 可以在这里添加操作相关的初始化代码
    
    def _enter_end_state(self, robot, teleop):
        """进入结束状态的初始化操作"""
        logging.info("进入结束状态：完成所有操作任务")
        log_say("任务完成，所有操作循环已结束")
        # 可以在这里添加结束相关的清理代码
    
    def _is_robot_stable(self, observation):
        """检查机器人是否处于稳定状态"""
        # 示例：检查关节速度是否接近零
        try:
            velocity_threshold = 0.1
            velocities = []
            
            # 从观测数据中提取速度信息
            for key in observation.keys():
                if 'vel' in key or 'velocity' in key:
                    velocities.append(abs(observation[key]))
            
            if velocities:
                max_velocity = max(velocities)
                return max_velocity < velocity_threshold
                
        except Exception as e:
            logging.warning(f"检查机器人稳定性时出错: {e}")
            
        return True  # 默认返回稳定
    
    def _is_operation_complete(self, observation):
        """检查操作是否完成"""
        # 示例：检查操作任务是否完成
        try:
            # 检查夹爪位置来判断操作是否完成
            gripper_positions = []
            for key in observation.keys():
                if 'gripper' in key and '.pos' in key:
                    gripper_positions.append(observation[key])
            
            if gripper_positions:
                # 如果夹爪完全闭合或完全打开，认为操作完成
                avg_gripper_pos = sum(gripper_positions) / len(gripper_positions)
                return avg_gripper_pos < 10 or avg_gripper_pos > 30
                
        except Exception as e:
            logging.warning(f"检查操作完成状态时出错: {e}")
            
        return False  # 默认返回未完成
    
    def get_current_state_info(self):
        """获取当前状态信息"""
        return {
            "current_state": self.current_state.name,
            "step_count": self.step_count,
            "operation_count": self.operation_count,
            "max_operation_cycles": self.max_operation_cycles,
            "transitions": self.state_transition_log[-5:]  # 最近5次状态转换
        }


@dataclass
class DatasetRecordConfig:
    # Dataset identifier. By convention it should match '{hf_username}/{dataset_name}' (e.g. `lerobot/test`).
    repo_id: str
    # A short but accurate description of the task performed during the recording (e.g. "Pick the Lego block and drop it in the box on the right.")
    single_task: str
    # Root directory where the dataset will be stored (e.g. 'dataset/path').
    root: str | Path | None = None
    # Limit the frames per second.
    fps: int = 30
    # Number of seconds for data recording for each episode.
    episode_time_s: int | float = 60
    # Number of seconds for resetting the environment after each episode.
    reset_time_s: int | float = 10
    # Number of episodes to record.
    num_episodes: int = 50
    # Encode frames in the dataset into video
    video: bool = True
    # Upload dataset to Hugging Face hub.
    push_to_hub: bool = False
    # Upload on private repository on the Hugging Face hub.
    private: bool = False
    # Add tags to your dataset on the hub.
    tags: list[str] | None = None
    # Number of subprocesses handling the saving of frames as PNG. Set to 0 to use threads only;
    # set to ≥1 to use subprocesses, each using threads to write images. The best number of processes
    # and threads depends on your system. We recommend 4 threads per camera with 0 processes.
    # If fps is unstable, adjust the thread count. If still unstable, try using 1 or more subprocesses.
    num_image_writer_processes: int = 0
    # Number of threads writing the frames as png images on disk, per camera.
    # Too many threads might cause unstable teleoperation fps due to main thread being blocked.
    # Not enough threads might cause low camera fps.
    num_image_writer_threads_per_camera: int = 4
    # Number of episodes to record before batch encoding videos
    # Set to 1 for immediate encoding (default behavior), or higher for batched encoding
    video_encoding_batch_size: int = 1

    def __post_init__(self):
        if self.single_task is None:
            raise ValueError("You need to provide a task as argument in `single_task`.")


@dataclass
class RecordConfig:
    robot: RobotConfig
    dataset: DatasetRecordConfig
    # Whether to control the robot with a teleoperator
    teleop: TeleoperatorConfig | None = None
    # Whether to control the robot with a policy
    policy: PreTrainedConfig | None = None
    # Display all cameras on screen
    display_data: bool = False
    # Use vocal synthesis to read events.
    play_sounds: bool = True
    # Resume recording on an existing dataset.
    resume: bool = False

    def __post_init__(self):
        # HACK: We parse again the cli args here to get the pretrained path if there was one.
        policy_path = parser.get_path_arg("policy")
        if policy_path:
            cli_overrides = parser.get_cli_overrides("policy")
            self.policy = PreTrainedConfig.from_pretrained(policy_path, cli_overrides=cli_overrides)
            self.policy.pretrained_path = policy_path

        if self.teleop is None and self.policy is None:
            raise ValueError("Choose a policy, a teleoperator or both to control the robot")

    @classmethod
    def __get_path_fields__(cls) -> list[str]:
        """This enables the parser to load config from the policy using `--policy.path=local/dir`"""
        return ["policy"]


@safe_stop_image_writer
def record_loop(
    robot: Robot,
    events: dict,
    fps: int,
    dataset: LeRobotDataset | None = None,
    teleop: Teleoperator | list[Teleoperator] | None = None,
    policy: PreTrainedPolicy | None = None,
    control_time_s: int | None = None,
    single_task: str | None = None,
    display_data: bool = False,
    action_queue: deque | None = None,
    state_machine: StateMachine | None = None,  # 添加状态机参数
):
    if dataset is not None and dataset.fps != fps:
        raise ValueError(f"The dataset fps should be equal to requested fps ({dataset.fps} != {fps}).")

    teleop_arm = teleop_keyboard = None
    if isinstance(teleop, list):
        teleop_keyboard = next((t for t in teleop if isinstance(t, KeyboardTeleop)), None)
        teleop_arm = next(
            (
                t
                for t in teleop
                if isinstance(
                    t,
                    (
                        so100_leader.SO100Leader,
                        so101_leader.SO101Leader,
                        koch_leader.KochLeader,
                    ),
                ),
            ),
            None,
        )

        if not (teleop_arm and teleop_keyboard and len(teleop) == 2 and robot.name == "lekiwi_client"):
            raise ValueError(
                "For multi-teleop, the list must contain exactly one KeyboardTeleop and one arm teleoperator. Currently only supported for LeKiwi robot."
            )

    # if policy is given it needs cleaning up
    if policy is not None:
        policy.reset()

    # 初始化动作队列
    if action_queue is None:
        action_queue = deque()
        
    # 初始化状态机（如果未提供）
    if state_machine is None:
        state_machine = StateMachine(state_check_interval=300)

    timestamp = 0
    start_episode_t = time.perf_counter()
    while timestamp < control_time_s:
        start_loop_t = time.perf_counter()

        current_events = teleop.get_vr_events()
        events.update(current_events)

        # 检查是否到达结束状态，如果是则提前退出
        if state_machine.current_state == RobotState.END:
            logging.info("到达结束状态，提前退出录制循环")
            break

        if events["exit_early"]:
            events["exit_early"] = False
            log_say("Exit early")
            time.sleep(1)
            break

        try:
            observation = robot.get_observation()
        except TimeoutError as e:
            logging.warning(f"Camera timeout: {e}. Skipping this frame.")
            continue  # 跳过当前帧，继续循环

        # 更新状态机
        current_state = state_machine.update(observation, action_queue, robot, teleop)
        
        # 根据当前状态执行相应的行为
        state_info = state_machine.get_current_state_info()
        if dataset is not None:
            # 将状态信息添加到观测数据中
            observation["current_state"] = current_state.value
            observation["state_step_count"] = state_machine.step_count
            observation["operation_count"] = state_machine.operation_count

        if policy is not None or dataset is not None:
            observation_frame = build_dataset_frame(dataset.features, observation, prefix="observation")
        
        # 如果队列里有action直接执行, 不采集action
        if action_queue:
            action = action_queue.popleft()
            if action == {}: # flag for the reset
                action = teleop.move_to_zero_position(robot)

        elif policy is not None:
            action_values = predict_action(
                observation_frame,
                policy,
                get_safe_torch_device(policy.config.device),
                policy.config.use_amp,
                task=single_task,
                robot_type=robot.robot_type,
            )
            action = {key: action_values[i].item() for i, key in enumerate(robot.action_features)}
        elif policy is None and isinstance(teleop, Teleoperator):
            action = teleop.get_action(observation, robot)
        elif policy is None and isinstance(teleop, list):
            # TODO(pepijn, steven): clean the record loop for use of multiple robots (possibly with pipeline)
            arm_action = teleop_arm.get_action()
            arm_action = {f"arm_{k}": v for k, v in arm_action.items()}

            keyboard_action = teleop_keyboard.get_action()
            base_action = robot._from_keyboard_to_base_action(keyboard_action)

            action = {**arm_action, **base_action} if len(base_action) > 0 else arm_action
        else:
            logging.info(
                "No policy or teleoperator provided, skipping action generation."
                "This is likely to happen when resetting the environment without a teleop device."
                "The robot won't be at its rest position at the start of the next episode."
            )
            continue

        # Action can eventually be clipped using `max_relative_target`,
        # so action actually sent is saved in the dataset.

        if events["reset_position"]:
            logging.info("Rest to the zero position of robot")
            log_say("Reset position")
            action = teleop.move_to_zero_position(robot)
            teleop.vr_event_handler.events['reset_position'] = False
            events["reset_position"] = False  # 重置事件状态
        elif events["back_position"]:
            logging.info("Back to the backet position of robot")
            log_say("Back backet position")
            if len(action_queue) < 10:  # 如果队列里有很多动作就不加入复位动作
                # 放置松手,和复位一体化
                queue_reset_actions(action_queue, robot, GLOBAL_BACK_GOAL, steps=30)
                queue_reset_actions(action_queue, robot, GLOBAL_OPEN_GOAL, steps=10, start_position=GLOBAL_BACK_GOAL)
                queue_reset_actions(action_queue, robot, np.zeros(12), steps=30, start_position=GLOBAL_OPEN_GOAL)
                action_queue.append({}) # reset to zero flag
            teleop.vr_event_handler.events['back_position'] = False
            events["back_position"] = False  # 返回事件状态
            
        sent_action = robot.send_action(action)

        if dataset is not None:
            action_frame = build_dataset_frame(dataset.features, sent_action, prefix="action")
            frame = {**observation_frame, **action_frame, "task": single_task}
            dataset.add_frame(frame)

        if display_data:
            log_rerun_data(observation, action)
            # 在可视化中显示当前状态
            if "current_state" in observation:
                logging.debug(f"当前状态: {RobotState(observation['current_state']).name}")

        dt_s = time.perf_counter() - start_loop_t
        busy_wait(1 / fps - dt_s)

        timestamp = time.perf_counter() - start_episode_t


@parser.wrap()
def record(cfg: RecordConfig) -> LeRobotDataset:
    init_logging()
    logging.info(pformat(asdict(cfg)))
    if cfg.display_data:
        init_rerun(session_name="recording")

    robot = make_robot_from_config(cfg.robot)
    teleop = make_teleoperator_from_config(cfg.teleop) if cfg.teleop is not None else None

    action_features = hw_to_dataset_features(robot.action_features, "action", cfg.dataset.video)
    obs_features = hw_to_dataset_features(robot.observation_features, "observation", cfg.dataset.video)
    dataset_features = {**action_features, **obs_features}

    if cfg.resume:
        dataset = LeRobotDataset(
            cfg.dataset.repo_id,
            root=cfg.dataset.root,
            batch_encoding_size=cfg.dataset.video_encoding_batch_size,
        )

        if hasattr(robot, "cameras") and len(robot.cameras) > 0:
            dataset.start_image_writer(
                num_processes=cfg.dataset.num_image_writer_processes,
                num_threads=cfg.dataset.num_image_writer_threads_per_camera * len(robot.cameras),
            )
        sanity_check_dataset_robot_compatibility(dataset, robot, cfg.dataset.fps, dataset_features)
    else:
        # Create empty dataset or load existing saved episodes
        sanity_check_dataset_name(cfg.dataset.repo_id, cfg.policy)
        dataset = LeRobotDataset.create(
            cfg.dataset.repo_id,
            cfg.dataset.fps,
            root=cfg.dataset.root,
            robot_type=robot.name,
            features=dataset_features,
            use_videos=cfg.dataset.video,
            image_writer_processes=cfg.dataset.num_image_writer_processes,
            image_writer_threads=cfg.dataset.num_image_writer_threads_per_camera * len(robot.cameras),
            batch_encoding_size=cfg.dataset.video_encoding_batch_size,
        )

    # Load pretrained policy
    policy = None if cfg.policy is None else make_policy(cfg.policy, ds_meta=dataset.meta)

    robot.connect()
    if teleop is not None:
        teleop.connect(robot=robot)
        teleop.send_feedback()

    # 根据teleop类型选择合适的事件监听器
    if isinstance(teleop, XLerobotVRTeleop):
        # 使用VR事件监听器
        listener, events = init_vr_listener(teleop)
        logging.info("🎮 使用VR左手柄控制录制状态")
    else:
        # 使用传统键盘监听器
        listener, events = init_keyboard_listener()
        logging.info("⌨️ 使用键盘控制录制状态")

    # 初始化状态机
    state_machine = StateMachine(state_check_interval=300)
    logging.info("🚀 状态机已初始化，每300步检查状态转换")

    with VideoEncodingManager(dataset):
        recorded_episodes = 0
        while recorded_episodes < cfg.dataset.num_episodes and not events["stop_recording"]:
            log_say(f"Recording episode {dataset.num_episodes}", cfg.play_sounds)
            time.sleep(3)
            record_loop(
                robot=robot,
                events=events,
                fps=cfg.dataset.fps,
                teleop=teleop,
                policy=policy,
                dataset=dataset,
                control_time_s=cfg.dataset.episode_time_s,
                single_task=cfg.dataset.single_task,
                display_data=cfg.display_data,
                state_machine=state_machine,  # 传递状态机
            )

            # 检查是否到达结束状态
            if state_machine.current_state == RobotState.END:
                logging.info("任务完成，提前结束录制")
                break

            # Execute a few seconds without recording to give time to manually reset the environment
            # Skip reset for the last episode to be recorded
            if not events["stop_recording"] and (
                (recorded_episodes < cfg.dataset.num_episodes - 1) or events["rerecord_episode"]
            ):
                log_say("Reset environment", cfg.play_sounds)
                record_loop(
                    robot=robot,
                    events=events,
                    fps=cfg.dataset.fps,
                    teleop=teleop,
                    control_time_s=cfg.dataset.reset_time_s,
                    single_task=cfg.dataset.single_task,
                    display_data=cfg.display_data,
                    dataset=None,
                    state_machine=state_machine,  # 传递状态机
                )

            if events["rerecord_episode"]:
                log_say("Delete Again record", cfg.play_sounds)
                events["rerecord_episode"] = False
                events["exit_early"] = False
                teleop.vr_event_handler.events['rerecord_episode'] = False

                dataset.clear_episode_buffer()
                time.sleep(10)
                continue

            log_say("Saving Episode", cfg.play_sounds, blocking=True)

            dataset.save_episode()
            recorded_episodes += 1

    log_say("Stop recording", cfg.play_sounds, blocking=True)

    robot.disconnect()
    if teleop is not None:
        teleop.disconnect()

    if not is_headless() and listener is not None:
        listener.stop()

    if cfg.dataset.push_to_hub:
        dataset.push_to_hub(tags=cfg.dataset.tags, private=cfg.dataset.private)

    log_say("Exiting", cfg.play_sounds)
    return dataset


def main():
    record()


if __name__ == "__main__":
    main()
