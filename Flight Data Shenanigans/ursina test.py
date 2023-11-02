from ursina import *
import time

app = Ursina()
Sky()
ground = Entity(model='plane',scale=(100,1,100),color = color.lime, texture = 'white_cube',texture_scale=(100,100), collider='box')

e = Entity(model='cube', color=color.orange, position=(0,4,1), scale=1.5, rotation=(0,0,0), texture='brick')

def current_milli_time():
    return round(time.time() * 1000)



lasttime = current_milli_time()
frame = 0
def update():
    global lasttime
    if current_milli_time()-lasttime >=100:#100 ms cooldown
        if held_keys['d']:
            #print ('hi')
            e.world_rotation_x += 1
            e.world_rotation_y += 
            e.world_rotation_z +=
            if frame != len(t)-1:
                frame += 1
        #if held_keys['a']:




        lasttime = current_milli_time()
        #self.x -= held_keys['a'] * time.dt * 10



EditorCamera()  # add camera controls for orbiting and moving the camera

app.run()