# Bullet-for-ursina
Bullet physics for ursina engine<br />
This library adds bullet physicsa to Ursina and is really easy to use.<br />
<br />
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
This library has some colliders and a debugger class to show you the colliders and you can configure it to show you what you want.
