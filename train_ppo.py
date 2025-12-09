#!/usr/bin/env python3
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor
import pandas as pd
import os
from simple_env import DisasterSimpleEnv

def make_env():
    env = DisasterSimpleEnv(debug=False, max_steps=1000)
    env = Monitor(env, filename="results_ppo_monitor.csv")
    return env

env = make_vec_env(make_env, n_envs=1)

model = PPO(
    "MlpPolicy",
    env,
    verbose=1,
    device="cuda",     # or "cpu"
    learning_rate=3e-4,
    n_steps=512,
    batch_size=64,
    gamma=0.99,
)

model.learn(total_timesteps=50_000, progress_bar=True)
model.save("ppo_disaster_robot.zip")

# Convert monitor log to clean CSV
df = pd.read_csv("results_ppo_monitor.csv", comment="#", header=None)
df.columns = ["r", "l", "t"]
df["episode"] = range(1, len(df) + 1)
df.rename(columns={"r": "total_reward", "l": "steps"}, inplace=True)
df["avg_step_reward"] = df["total_reward"] / df["steps"]
df.to_csv("results_ppo.csv", index=False)
print("\n✅ PPO metrics saved to results_ppo.csv")

env.close()
