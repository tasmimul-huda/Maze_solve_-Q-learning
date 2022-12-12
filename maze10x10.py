import numpy as np
import time
import sys
import tkinter as tk
from config import Config10x10

config = Config10x10()
width = config.width_in_each_side
UNIT = config.UNIT
MAZE_H = config.MAZE_H
MAZE_W = config.MAZE_W

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
        hell1_center = origin + np.array([UNIT * 2, UNIT *0])
        self.hell1 = self.canvas.create_rectangle(hell1_center[0]-width, hell1_center[1]-width,hell1_center[0]+width, hell1_center[1]+width,fill='black')
        
        # hell2
        hell2_center = origin + np.array([UNIT *3 , UNIT * 0])
        self.hell2 = self.canvas.create_rectangle(hell2_center[0]-width, hell2_center[1]-width,hell2_center[0]+width, hell2_center[1]+width,fill='black')

        # hell3
        hell3_center = origin + np.array([UNIT * 8, UNIT * 1])
        self.hell3 = self.canvas.create_rectangle(hell3_center[0]-width, hell3_center[1]-width,hell3_center[0]+width, hell3_center[1]+width,fill='black')
        
        # hell4
        hell4_center = origin + np.array([UNIT * 3, UNIT * 2])
        self.hell4 = self.canvas.create_rectangle(hell4_center[0]-width, hell4_center[1]-width,hell4_center[0]+width, hell4_center[1]+width,fill='black')
        
        # hell5
        hell5_center = origin + np.array([UNIT * 4, UNIT * 2])
        self.hell5 = self.canvas.create_rectangle(hell5_center[0]-width, hell5_center[1]-width,hell5_center[0]+width, hell5_center[1]+width,fill='black')

        #hell6
        hell6_center = origin + np.array([UNIT * 5, UNIT * 2])
        self.hell6 = self.canvas.create_rectangle(hell6_center[0]-width, hell6_center[1]-width,hell6_center[0]+width, hell6_center[1]+width,fill='black')
        
        
        # hell7
        hell7_center = origin + np.array([UNIT * 1, UNIT * 3])
        self.hell7 = self.canvas.create_rectangle(hell7_center[0]-width, hell7_center[1]-width,hell7_center[0]+width, hell7_center[1]+width,fill='black')
        
        # hell8
        hell8_center = origin + np.array([UNIT * 1, UNIT * 4])
        self.hell8 = self.canvas.create_rectangle(hell8_center[0]-width, hell8_center[1]-width,hell8_center[0]+width, hell8_center[1]+width,fill='black')

        # hell9
        hell9_center = origin + np.array([UNIT * 5, UNIT * 4])
        self.hell9 = self.canvas.create_rectangle(hell9_center[0]-width, hell9_center[1]-width,hell9_center[0]+width, hell9_center[1]+width,fill='black')
        
        # hell10
        hell10_center = origin + np.array([UNIT * 3, UNIT *5])
        self.hell10 = self.canvas.create_rectangle(hell10_center[0]-width, hell10_center[1]-width,hell10_center[0]+width, hell10_center[1]+width,fill='black')
        
        # hell11
        hell11_center = origin + np.array([UNIT * 7, UNIT * 5])
        self.hell11 = self.canvas.create_rectangle(hell11_center[0]-width, hell11_center[1]-width,hell11_center[0]+width, hell11_center[1]+width,fill='black')

        #hell12
        hell12_center = origin + np.array([UNIT * 0, UNIT * 6])
        self.hell12 = self.canvas.create_rectangle(hell12_center[0]-width, hell12_center[1]-width,hell12_center[0]+width, hell12_center[1]+width,fill='black')
        
        #hell13
        hell13_center = origin + np.array([UNIT * 2, UNIT * 6])
        self.hell13 = self.canvas.create_rectangle(hell13_center[0]-width, hell13_center[1]-width,hell13_center[0]+width, hell13_center[1]+width,fill='black')
        
        #hell14
        hell14_center = origin + np.array([UNIT * 7, UNIT * 6])
        self.hell14 = self.canvas.create_rectangle(hell14_center[0]-width, hell14_center[1]-width,hell14_center[0]+width, hell14_center[1]+width,fill='black')

        #hell15
        hell15_center = origin + np.array([UNIT * 0, UNIT * 7])
        self.hell15 = self.canvas.create_rectangle(hell15_center[0]-width, 
                            hell15_center[1]-width,hell15_center[0]+width,
                            hell15_center[1]+width,fill='black')
        #hell16
        hell16_center = origin + np.array([UNIT * 2, UNIT * 7])
        self.hell16 = self.canvas.create_rectangle(hell16_center[0]-width, 
                            hell16_center[1]-width,hell16_center[0]+width,
                            hell16_center[1]+width,fill='black')

        #hell17
        hell17_center = origin + np.array([UNIT * 4, UNIT * 7])
        self.hell17 = self.canvas.create_rectangle(hell17_center[0]-width, 
                            hell17_center[1]-width,hell17_center[0]+width,
                            hell17_center[1]+width,fill='black')
        #hell18
        hell18_center = origin + np.array([UNIT * 1, UNIT * 9])
        self.hell18 = self.canvas.create_rectangle(hell18_center[0]-width, 
                            hell18_center[1]-width,hell18_center[0]+width,
                            hell18_center[1]+width,fill='black')
        #hell19
        hell19_center = origin + np.array([UNIT * 6, UNIT * 6])
        self.hell19 = self.canvas.create_rectangle(hell19_center[0]-width, 
                            hell19_center[1]-width,hell19_center[0]+width,
                            hell19_center[1]+width,fill='black')
        
        #hell20
        hell20_center = origin + np.array([UNIT * 9, UNIT * 3])
        self.hell20 = self.canvas.create_rectangle(hell20_center[0]-width, 
                            hell20_center[1]-width,hell20_center[0]+width,
                            hell20_center[1]+width,fill='black')

        #hell21
        hell21_center = origin + np.array([UNIT * 8, UNIT * 6])
        self.hell21 = self.canvas.create_rectangle(hell21_center[0]-width, 
                            hell21_center[1]-width,hell21_center[0]+width,
                            hell21_center[1]+width,fill='black')
        
        #hell22
        hell22_center = origin + np.array([UNIT * 5, UNIT * 1])
        self.hell22 = self.canvas.create_rectangle(hell22_center[0]-width, 
                            hell22_center[1]-width,hell22_center[0]+width,
                            hell22_center[1]+width,fill='black')
        
        #hell23
        hell23_center = origin + np.array([UNIT * 7, UNIT * 3])
        self.hell23 = self.canvas.create_rectangle(hell23_center[0]-width, 
                            hell23_center[1]-width,hell23_center[0]+width,
                            hell23_center[1]+width,fill='black')

        #hell24
        hell24_center = origin + np.array([UNIT * 7, UNIT * 4])
        self.hell24 = self.canvas.create_rectangle(hell24_center[0]-width, 
                            hell24_center[1]-width,hell24_center[0]+width,
                            hell24_center[1]+width,fill='black')

        #hell25
        hell25_center = origin + np.array([UNIT * 6, UNIT * 8])
        self.hell25 = self.canvas.create_rectangle(hell25_center[0]-width, 
                            hell25_center[1]-width,hell25_center[0]+width,
                            hell25_center[1]+width,fill='black')

        #hell26
        hell26_center = origin + np.array([UNIT * 5, UNIT * 9])
        self.hell26 = self.canvas.create_rectangle(hell26_center[0]-width, 
                            hell26_center[1]-width,hell26_center[0]+width,
                            hell26_center[1]+width,fill='black')
        





        # oval
        oval_center = origin + np.array([UNIT * 9, UNIT * 0])
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
                self.canvas.create_rectangle(s,fill='blue')
                base_action[1] -= UNIT
        elif action == 1:   # down
            if s[1] < (MAZE_H - 1) * UNIT:
                self.canvas.create_rectangle(s,fill='blue')
                base_action[1] += UNIT
        elif action == 2:   # right
            if s[0] < (MAZE_W - 1) * UNIT:
                self.canvas.create_rectangle(s,fill='blue')
                base_action[0] += UNIT
        elif action == 3:   # left
            if s[0] > UNIT:
                self.canvas.create_rectangle(s,fill='blue')
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
        self.canvas.coords(self.hell13),
        self.canvas.coords(self.hell14),
        self.canvas.coords(self.hell15),
        self.canvas.coords(self.hell16),
        self.canvas.coords(self.hell17),
        self.canvas.coords(self.hell18),
        self.canvas.coords(self.hell19),
        self.canvas.coords(self.hell20),
        self.canvas.coords(self.hell21),
        self.canvas.coords(self.hell22),
        self.canvas.coords(self.hell23),
        self.canvas.coords(self.hell24),
        self.canvas.coords(self.hell25),
        self.canvas.coords(self.hell26),
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
            reward = -0.00000000001
            done = False

        return s_, reward, done

    def render(self):
        time.sleep(0.1)
        self.update()


def update():
    for t in range(500):
        s = env.reset()
        while True:
            env.render()
            a = 1
            s, r, done = env.step(a)
            if done:
                break


if __name__ == '__main__':
    env = Maze()
    env.after(10, update)
    env.mainloop()
