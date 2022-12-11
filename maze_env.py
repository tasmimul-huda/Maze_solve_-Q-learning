import numpy as np
import time
import sys
import tkinter as tk
from config import Config


config = Config()
UNIT = config.UNIT
MAZE_H = config.MAZE_H
MAZE_W = config.MAZE_W


class Maze(tk.Tk, object):
    def __init__(self):
        super(Maze, self).__init__()
        self.action_space = config.action_space
        self.n_actions = len(self.action_space)
        self.title('maze')
        self.geometry('{0}x{1}'.format(MAZE_W * UNIT, MAZE_H * UNIT))
        # self.geometry(f'{MAZE_W * UNIT} x {MAZE_H * UNIT}')
        self._build_maze()

    def _build_maze(self):
        self.canvas = tk.Canvas(self, bg=config.bgcolor,
                                height=MAZE_H * UNIT,
                                width=MAZE_W * UNIT)

        #create in grids
        for c in range(0, MAZE_W * UNIT, UNIT):
            x0, y0, x1, y1 = c, 0, c, MAZE_H * UNIT
            self.canvas.create_line(x0, y0, x1, y1)

        for r in range(0, MAZE_H * UNIT, UNIT):
            x0, y0, x1, y1 = 0, r, MAZE_W * UNIT, r
            self.canvas.create_line(x0, y0, x1, y1)

        origin = np.array([20, 20])

        # hell
        hell1_center = origin + np.array([UNIT * 2, UNIT])
        self.hell1 = self.canvas.create_rectangle(
            hell1_center[0]-15, hell1_center[1]-15,
            hell1_center[0]+15, hell1_center[1]+15,
            fill='black'
        )
        # hell2
        hell2_center = origin + np.array([UNIT, UNIT * 2])
        self.hell2 = self.canvas.create_rectangle(
            hell2_center[0]-15, hell2_center[1]-15,
            hell2_center[0]+15, hell2_center[1]+15,
            fill='black')

        # hell2
        hell3_center = origin + np.array([UNIT * 3, UNIT * 2])
        self.hell3 = self.canvas.create_rectangle(
            hell3_center[0]-15, hell3_center[1]-15,
            hell3_center[0]+15, hell3_center[1]+15,
            fill='black')

        # oval
        oval_center = origin + UNIT * 2
        self.oval = self.canvas.create_oval(
            oval_center[0] - 15, oval_center[1]-15,
            oval_center[0] + 15, oval_center[1] + 15,
            fill='yellow'
        )

        # red rect
        self.rect = self.canvas.create_rectangle(
            origin[0] - 15, origin[1]-15,
            origin[0] + 15, origin[1] + 15,
            fill='red'
        )
        self.canvas.pack()

    def reset(self):
        self.update()
        time.sleep(0.5)
        self.canvas.delete(self.rect)
        origin = np.array([20, 20])
        self.rect = self.canvas.create_rectangle(
            origin[0] - 15, origin[1]-15,
            origin[0] + 15, origin[1] + 15,
            fill='red'
        )
        return self.canvas.coords(self.rect)

    def step(self, action):
        s = self.canvas.coords(self.rect)
        base_action = np.array([0, 0])
        if action == 0:   # up
            if s[1] > UNIT:
                base_action[1] -= UNIT
        elif action == 1:   # down
            if s[1] < (MAZE_H - 1) * UNIT:
                base_action[1] += UNIT
        elif action == 2:   # right
            if s[0] < (MAZE_W - 1) * UNIT:
                base_action[0] += UNIT
        elif action == 3:   # left
            if s[0] > UNIT:
                base_action[0] -= UNIT

        self.canvas.move(self.rect, base_action[0], base_action[1])

        s_ = self.canvas.coords(self.rect)  # next state

        # reward function

        if s_ == self.canvas.coords(self.oval):
            reward = 1
            done = True
            s_ = 'terminal'
        elif s_ in [self.canvas.coords(self.hell1), self.canvas.coords(self.hell2), self.canvas.coords(self.hell3)]:
            reward = -0.0000001
            done = True
            s_ = 'terminal'
        else:
            reward = -0.000004
            done = False

        return s_, reward, done

    def render(self):
        time.sleep(0.1)
        self.update()


def update():
    for t in range(50):
        s = env.reset()
        while True:
            env.render()
            a = 1
            s, r, done = env.step(a)
            if done:
                break


if __name__ == '__main__':
    env = Maze()
    env.after(100, update)
    env.mainloop()
