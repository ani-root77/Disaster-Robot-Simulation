import numpy as np
import csv
from simple_env import DisasterSimpleEnv

env = DisasterSimpleEnv(debug=False, max_steps=1000)
n_actions = env.action_space.n

BINS = [0.2, 0.4, 0.8, 1.5, 3.0, 6.0]
def discretize(obs):
    idxs = [int(np.digitize(v, BINS)) for v in obs]
    return idxs[0] + 10*idxs[1] + 100*idxs[2] + 1000*idxs[3]

Q = {}
def get_Q(s):
    if s not in Q:
        Q[s] = np.zeros(n_actions, dtype=np.float32)
    return Q[s]

eps, alpha, gamma = 0.1, 0.3, 0.95
episodes = 100

# CSV logging
csv_file = open("results_qlearning.csv", "w", newline="")
writer = csv.writer(csv_file)
writer.writerow(["episode", "steps", "total_reward", "collision", "goal_reached", "avg_step_reward"])

for ep in range(episodes):
    obs, _ = env.reset()
    s = discretize(obs)
    done = False
    total_reward, steps = 0, 0
    collided = reached = False

    while not done:
        if np.random.rand() < eps:
            a = np.random.randint(n_actions)
        else:
            a = int(np.argmax(get_Q(s)))

        obs2, r, done, info = env.step(a)
        s2 = discretize(obs2)
        q, q_next = get_Q(s), get_Q(s2)
        q[a] += alpha * (r + gamma * np.max(q_next) * (1 - done) - q[a])

        total_reward += r
        steps += 1
        s = s2
        collided = collided or info.get("collision", False)
        reached = reached or info.get("reached_goal", False)

    avg_step_reward = total_reward / max(steps, 1)
    writer.writerow([ep + 1, steps, total_reward, collided, reached, avg_step_reward])
    print(f"Episode {ep+1}/{episodes} reward={total_reward:.1f} steps={steps}")

csv_file.close()
env.close()
