from ursina import *
import time

app = Ursina()
Sky()
ground = Entity(model='plane',scale=(100,1,100),color = color.lime, texture = 'white_cube',texture_scale=(100,100), collider='box')

e = Entity(model='Rocket.obj', color=color.orange, position=(0,0,1), scale=1.5, rotation=(0,0,45), texture='brick')


lasttime = current_milli_time()
class Player(Entity):
    global lasttime

    def current_milli_time(self):
        return round(time.time() * 1000)

    def __init__(self, **kwargs):

        super().__init__()
        self.model='cube'
        self.color = color.red
        self.scale_y = 2
        

        for key, value in kwargs.items():
            setattr(self, key, value)

    def input(self, key):
        if key == 'space':
            self.animate_x(2, duration=1)

    def update(self):
        if self.current_milli_time()-lasttime >=100:#100 ms cooldown
            if held_keys['d']:
                print ('hi')
                lasttime = self.current_milli_time()
        #self.x += held_keys['d'] * time.dt * 10
        #self.x -= held_keys['a'] * time.dt * 10

player = Player(x=-1)

EditorCamera()  # add camera controls for orbiting and moving the camera

app.run()