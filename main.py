from maze10x10 import Maze
from rl_brain import QLearningTable
import warnings
warnings.filterwarnings('ignore')


def update():
    for episode in range(10000):
        observation = env.reset()
        i = 0

        while True:
            env.render()

            action = RL.choose_action(str(observation))
            observation_, reward, done = env.step(action)

            RL.learn(str(observation), action, reward, str(observation_,))

            observation = observation_
            i += 1
            if done:
                break
            print(f"count: {i}")

    print('game over')
    env.destroy()


if __name__ == '__main__':
    env = Maze()
    RL = QLearningTable(actions=list(range(env.n_actions)))
    env.after(5, update)
    env.mainloop()
