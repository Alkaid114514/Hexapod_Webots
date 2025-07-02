# train.py（需放在Webots项目的controllers/rl_train目录下）
from hexapod_rl_env import HexapodRLEnv
from stable_baselines3 import PPO
from controller import Supervisor  # 现在可以正常导入

def main():
    env = HexapodRLEnv()
    model = PPO(
    "MlpPolicy",
    env,
    n_steps=4096,    # 增加采样步数
    batch_size=128,  # 减小批大小
    gamma=0.99,      # 增加远期奖励权重
    gae_lambda=0.95, # 平衡偏差和方差
    ent_coef=0.01,   # 鼓励探索
    verbose=1
    )
    model.learn(total_timesteps=100000)
    model.save("hexapod_ppo")

if __name__ == "__main__":
    main()