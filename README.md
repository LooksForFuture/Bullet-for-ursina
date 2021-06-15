# Bullet-for-ursina
Bullet physics for ursina engine<br />
This library adds bullet physicsa to Ursina and is really easy to use.<br />
```python
from ursina import Ursina, Entity, Vec3, color, time
from physics3d import Debugger, BoxCollider, MeshCollider
from panda3d.bullet import BulletWorld

app = Ursina()
world = BulletWorld()
Debugger(world, wireframe=True)
world.setGravity(Vec3(0, -9.81, 0))

cube = Entity(model='cube', color=color.red)
MeshCollider(world, cube, mass=1)
ground = Entity(model='cube', position=(0, -3, 0), scale=(10, 0.1, 10), color=color.blue)
BoxCollider(world, ground)

def update():
    dt = time.dt
    world.doPhysics(dt)

app.run()
```
This library has some colliders and a debugger class to show you the colliders and you can configure it to show you what you want.<br />
<h2>Character controller</h2>
Also this library has a character controller which you can customize to make any game<br />
```python
from ursina import Ursina, Entity, Vec3, color, time, camera, held_keys
from ursina.physics3d import Debugger, BoxCollider
from ursina.physics3d.character_controller import CharacterController
from panda3d.bullet import BulletWorld

app = Ursina()
world = BulletWorld()
Debugger(world, wireframe=True)
world.setGravity(Vec3(0, -9.81, 0))

sphere = Entity()
player = CharacterController(world, sphere)

ground = Entity(model='cube', position=(0, -3, 0), scale=(10, 0.1, 10), color=color.blue)
BoxCollider(world, ground)

def update():
    speed = Vec3(0, 0, 0)
    if held_keys['w']:
        speed[2] = 1
    elif held_keys['s']:
        speed[2] = -1
    
    if held_keys['d']:
        speed[0] = 1
    
    elif held_keys['a']:
        speed[0] = -1
    
    if held_keys['space']:
        player.jump()
    
    player.move(speed, True)

    dt = time.dt
    world.doPhysics(dt)

app.run()
```
