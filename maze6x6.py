import numpy as np
import time
import tkinter as tk
from config import Config6x6


config = Config6x6()
width = config.width_in_each_side
UNIT = config.UNIT
MAZE_H = config.MAZE_H
MAZE_W = config.MAZE_W

#  print(f"width: {width}; UNIT:{UNIT}; MAZE_H: {MAZE_H}; MAZE_W: {MAZE_W}")

class Maze(tk.Tk, object):
    def __init__(self):
        super(Maze, self).__init__()
        self.action_space = config.action_space
        self.n_actions = len(self.action_space)
        self.title('shortest path in maze')
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

        # hell1
        hell1_center = origin + np.array([UNIT * 1, UNIT *0])
        self.hell1 = self.canvas.create_rectangle(hell1_center[0]-width, hell1_center[1]-width,hell1_center[0]+width, hell1_center[1]+width,fill='black')
        
        # hell2
        hell2_center = origin + np.array([UNIT *3 , UNIT * 1])
        self.hell2 = self.canvas.create_rectangle(hell2_center[0]-width, hell2_center[1]-width,hell2_center[0]+width, hell2_center[1]+width,fill='black')

        # hell3
        hell3_center = origin + np.array([UNIT * 4, UNIT * 1])
        self.hell3 = self.canvas.create_rectangle(hell3_center[0]-width, hell3_center[1]-width,hell3_center[0]+width, hell3_center[1]+width,fill='black')
        
        # hell4
        hell4_center = origin + np.array([UNIT * 1, UNIT * 2])
        self.hell4 = self.canvas.create_rectangle(hell4_center[0]-width, hell4_center[1]-width,hell4_center[0]+width, hell4_center[1]+width,fill='black')
        
        # hell5
        hell5_center = origin + np.array([UNIT * 1, UNIT * 3])
        self.hell5 = self.canvas.create_rectangle(hell5_center[0]-width, hell5_center[1]-width,hell5_center[0]+width, hell5_center[1]+width,fill='black')

        #hell6
        hell6_center = origin + np.array([UNIT * 2, UNIT * 3])
        self.hell6 = self.canvas.create_rectangle(hell6_center[0]-width, hell6_center[1]-width,hell6_center[0]+width, hell6_center[1]+width,fill='black')
        
        
        # hell7
        hell7_center = origin + np.array([UNIT * 5, UNIT * 3])
        self.hell7 = self.canvas.create_rectangle(hell7_center[0]-width, hell7_center[1]-width,hell7_center[0]+width, hell7_center[1]+width,fill='black')
        
        # hell8
        hell8_center = origin + np.array([UNIT * 5, UNIT * 4])
        self.hell8 = self.canvas.create_rectangle(hell8_center[0]-width, hell8_center[1]-width,hell8_center[0]+width, hell8_center[1]+width,fill='black')

        # hell9
        hell9_center = origin + np.array([UNIT * 0, UNIT * 5])
        self.hell9 = self.canvas.create_rectangle(hell9_center[0]-width, hell9_center[1]-width,hell9_center[0]+width, hell9_center[1]+width,fill='black')
        
        # hell10
        hell10_center = origin + np.array([UNIT * 1, UNIT *5])
        self.hell10 = self.canvas.create_rectangle(hell10_center[0]-width, hell10_center[1]-width,hell10_center[0]+width, hell10_center[1]+width,fill='black')
        
        # hell11
        hell11_center = origin + np.array([UNIT * 2, UNIT * 5])
        self.hell11 = self.canvas.create_rectangle(hell11_center[0]-width, hell11_center[1]-width,hell11_center[0]+width, hell11_center[1]+width,fill='black')

        #hell12
        hell12_center = origin + np.array([UNIT * 4, UNIT * 5])
        self.hell12 = self.canvas.create_rectangle(hell12_center[0]-width, hell12_center[1]-width,hell12_center[0]+width, hell12_center[1]+width,fill='black')
        

        # oval
        oval_center = origin + np.array([UNIT * 3, UNIT * 3])
        self.oval = self.canvas.create_oval(
            oval_center[0] - width, oval_center[1]-width,
            oval_center[0] + width, oval_center[1] + width,
            fill='yellow'
        )

        # red rect
        self.rect = self.canvas.create_rectangle(
            origin[0] - width, origin[1]-width,
            origin[0] + width, origin[1] + width,
            fill='red'
        )
        self.canvas.pack()
    
    def reset(self):
        self.update()
        time.sleep(0.5)
        self.canvas.delete(self.rect)
        origin = np.array([20, 20])
        self.rect = self.canvas.create_rectangle(
            origin[0] - width, origin[1]-width,
            origin[0] + width, origin[1] + width,
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

        #Hell coordinate list
        hell_list = [self.canvas.coords(self.hell1),
        self.canvas.coords(self.hell2),
        self.canvas.coords(self.hell3),
        self.canvas.coords(self.hell4),
        self.canvas.coords(self.hell5),
        self.canvas.coords(self.hell6),
        self.canvas.coords(self.hell7),
        self.canvas.coords(self.hell8),
        self.canvas.coords(self.hell9),
        self.canvas.coords(self.hell10),
        self.canvas.coords(self.hell11),
        self.canvas.coords(self.hell12),
        ]

        # reward function
        if s_ == self.canvas.coords(self.oval):
            reward = 1
            done = True
            s_ = 'terminal'
        elif s_ in hell_list:#[self.canvas.coords(self.hell1), self.canvas.coords(self.hell2), self.canvas.coords(self.hell3)]:
            reward = -1
            done = True
            s_ = 'terminal'
        else:
            reward = -0.000000000000000000000000001
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
