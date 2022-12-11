class Config:
    UNIT = 40
    action_space = ['u', 'd', 'l', 'r']
    bgcolor = 'white'
    width_in_each_side = 20


class Config6x6(Config):
    MAZE_H = 6
    MAZE_W = 6
    
class Config10x10(Config):
    MAZE_H = 10
    MAZE_W = 10