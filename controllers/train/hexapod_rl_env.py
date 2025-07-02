# hexapod_rl_env.py
import numpy as np
from gym import Env
from gym.spaces import Box
from controller import Supervisor

class HexapodRLEnv(Env):
    def __init__(self):
        # 初始化Supervisor（必须用Supervisor才能重置环境）
        self.supervisor = Supervisor()
        self.timestep = int(self.supervisor.getBasicTimeStep())
        
        # 初始化所有18个关节（6条腿×3关节/腿）
        leg_positions = ['MR', 'ML', 'BR', 'BL', 'FR', 'FL']  # 根据URDF命名
        joint_types = ['COXA', 'FEMUR', 'TIBIA']
        
        self.joints = []
        for leg in leg_positions:
            for joint_type in joint_types:
                joint_name = f"M_{leg}_{joint_type}"  # 如 "M_MR_COXA"
                joint = self.supervisor.getDevice(joint_name)
                joint.getPositionSensor().enable(self.timestep)
                self.joints.append(joint)
        
        # 初始化IMU（根据URDF中的base_link_imu_joint）
        self.imu = self.supervisor.getDevice("imu")
        self.imu.enable(self.timestep)
        
        # 定义动作空间（控制18个关节的位置，范围[-1,1]映射到[-π/2, π/2]）
        self.action_space = Box(low=-1.0, high=1.0, shape=(18,))
        
        # 定义观测空间（18关节角度 + 18关节速度 + 3IMU数据）
        self.observation_space = Box(low=-np.inf, high=np.inf, shape=(39,))
        
        # 记录初始位置（用于计算位移奖励）
        self.initial_position = self.supervisor.getSelf().getPosition()
        
        # 调试计数器
        self.step_count = 0

    def reset(self):
        # 重置仿真（Supervisor特有功能）
        self.supervisor.simulationResetPhysics()
        self.supervisor.step(self.timestep)
        
        # 重新获取初始位置
        self.initial_position = self.supervisor.getSelf().getPosition()
        self.step_count = 0
        
        # 返回初始观测
        return self._get_obs()

    def step(self, action):
        # 应用动作到关节（归一化到实际范围）
        for i, joint in enumerate(self.joints):
            clipped_action = np.clip(action[i], -1, 1)
            joint.setPosition(clipped_action * np.pi/2)  # 限制到±90度
        
        # 推进仿真
        self.supervisor.step(self.timestep)
        
        # 获取观测
        obs = self._get_obs()
        
        # 计算奖励
        reward = self._compute_reward()
        
        # 检查终止条件
        done = self._check_done()
        
        # 调试输出（每100步打印一次）
        self.step_count += 1
        if self.step_count % 100 == 0:
            self._debug_status()
        
        return obs, reward, done, {}

    def _get_obs(self):
        # 获取关节角度和角速度
        joint_pos = [j.getPositionSensor().getValue() for j in self.joints]
        joint_vel = [j.getVelocity() for j in self.joints]
        
        # 获取IMU数据并修正朝向（根据你的机器人Y轴朝前）
        roll, pitch, yaw = self.imu.getRollPitchYaw()


        
        corrected_pitch = pitch  # 根据测试可能需要调整符号
        corrected_roll = roll     # 根据测试可能需要调整符号
        
        return np.concatenate([
            joint_pos,
            joint_vel,
            [corrected_roll, corrected_pitch, yaw]
        ])

    def _compute_reward(self):
        # 1. 前进奖励（鼓励 Y 轴移动，你的机器人前方是 Y+）
        current_pos = self.supervisor.getSelf().getPosition()
        forward_reward = 10.0 * (current_pos[1] - self.initial_position[1])
        
        # 2. 存活奖励（每步存活 +0.1，避免过早终止）
        survival_reward = 0.1
        
        # 3. 稳定性惩罚（限制机身倾斜）
        roll, pitch, _ = self.imu.getRollPitchYaw()
        stability_penalty = 1.0 * (abs(pitch) + abs(roll))
        
        # 4. 关节运动平滑惩罚（防止抽搐）
        joint_vel = np.array([j.getVelocity() for j in self.joints])
        smoothness_penalty = 0.01 * np.sum(np.square(joint_vel))
        
        # 综合奖励
        reward = forward_reward + survival_reward - stability_penalty - smoothness_penalty
        return reward

    def _check_done(self):
        # 1. 检查机身倾斜（放宽阈值）
        roll, pitch, _ = self.imu.getRollPitchYaw()
        if abs(pitch) > 1.0 or abs(roll) > 1.0:  # 约 57 度
            return True
        
        # 2. 检查是否超时（增加步数限制）
        if self.step_count >= 5000:  # 允许更长的训练片段
            return True
        
        return False

    def _debug_status(self):
        # 打印调试信息
        current_pos = self.supervisor.getSelf().getPosition()
        displacement = current_pos[1] - self.initial_position[1]
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        
        print(f"Step: {self.step_count} | "
              f"Displacement: {displacement:.2f}m | "
              f"Roll: {np.degrees(roll):.1f}° | "
              f"Pitch: {np.degrees(pitch):.1f}° | "
              f"Avg joint vel: {np.mean([j.getVelocity() for j in self.joints]):.2f} rad/s")

    def close(self):
        # 清理资源
        pass