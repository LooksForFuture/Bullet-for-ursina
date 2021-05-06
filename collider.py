from statistics import mean
from ursina import Ursina, Entity, Vec3, Cylinder
from panda3d.bullet import (
    BulletRigidBodyNode, BulletGhostNode, BulletSphereShape, BulletBoxShape, BulletCylinderShape
    )

class Collider:
    def __init__(self, world:Ursina, entity:Entity, shape, name, rotation, mass, ghost):
        self.world = world
        if ghost:
            self.node = BulletGhostNode(name)
        else:
            self.node = BulletRigidBodyNode(name)
            self.node.setMass(mass)
        
        self.node.addShape(shape)
        self.np = world.render.attachNewNode(self.node)
        self.np.setPos(entity.x, entity.y, entity.z)

        if None in rotation:
            hpr = entity.getHpr()
            for x in range(len(hpr)):
                rotation[x] = hpr[x]
        self.np.setHpr(Vec3(*rotation))

        if ghost:
            world.attachGhost(self.node)
        else:
            world.attachRigidBody(self.node)

        if entity.parent:
            self.np.reparent_to(entity.parent)

        entity.reparent_to(self.np)
    
    def setCcdMotionThreshold(self, value):
        self.node.setCcdMotionThreshold(value)
    
    def setCcdSweptSphereRadius(self, value):
        self.node.setCcdSweptSphereRadius(value)
    
    def setCollideMask(self, mask):
        self.np.setCollideMask(mask)
    
    @property
    def angular_damping(self):
        return self.node.getAngularDamping()
    
    @angular_damping.setter
    def angular_damping(self, value):
        self.node.setAngularDamping(value)
    
    @property
    def angular_factor(self):
        return self.node.getAngularFactor()
    
    @angular_factor.setter
    def angular_factor(self, value):
        self.node.setAngularFactor(value)
    
    @property
    def angular_sleep_threshold(self):
        return self.node.getAngularSleepThreshold()
    
    @angular_sleep_threshold.setter
    def angular_sleep_threshold(self, value):
        self.node.setAngularSleepThreshold(value)
    
    @property
    def angular_velocity(self):
        return self.node.getAngularVelocity()
    
    @angular_velocity.setter
    def angularVelocity(self, value):
        self.node.setAngularVelocity(value)
    
    @property
    def gravity(self):
        return self.node.getGravity()
    
    @gravity.setter
    def gravity(self, value):
        self.node.setGravity(value)
    
    @property
    def inertia(self):
        return self.node.getInertia()
    
    @inertia.setter
    def inertia(self, value):
        self.node.setInertia(value)
    
    @property
    def linear_damping(self):
        return self.node.getLinearDamping()
    
    @linear_damping.setter
    def linear_damping(self, value):
        self.node.setLinearDamping(value)
    
    @property
    def linear_factor(self):
        return self.node.getLinearFactor()
    
    @linear_factor.setter
    def linear_factor(self, value):
        self.node.setLinearFactor(value)
    
    @property
    def linear_sleep_threshold(self):
        return self.node.getLinearSleepThreshold()
    
    @linear_sleep_threshold.setter
    def linear_sleep_threshold(self, value):
        self.node.setLinearSleepThreshold(value)
    
    @property
    def linear_velocity(self):
        return self.node.getLinearVelocity()
    
    @linear_velocity.setter
    def linear_velocity(self, value):
        self.node.setLinearVelocity(value)
    
    @property
    def mass(self):
        return self.node.getMass()
    
    @mass.setter
    def mass(self, value):
        return self.node.setMass(value)
    
    @property
    def inv_inertia_diag_local(self):
        return self.node.inv_inertia_diag_local
    
    @property
    def inv_inertia_tensor_world(self):
        return self.node.inv_inertia_tensor_world
    
    @property
    def inv_mass(self):
        return self.node.inv_mass
    
    @property
    def total_force(self):
        return self.node.total_force
    
    @property
    def total_torque(self):
        return self.node.total_torque
    
    def apply_central_force(self, force):
        self.node.applyCentralForce(force)
    
    def apply_central_impulse(self, impulse):
        self.node.applyCentralImpulse(impulse)
    
    def apply_force(self, force, pos):
        self.node.applyForce(force, pos)
    
    def apply_impulse(self, impulse, pos):
        self.node.applyImpulse(impulse, pos)
    
    def apply_torque(self, torque):
        self.node.applyTorque(torque)
    
    def apply_torque_impulse(self, torque):
        self.node.applyTorqueImpulse(torque)
    
    def clear_forces(self):
        self.node.clearForces()
    
    def pick_dirty_flag(self):
        return self.node.pickDirtyFlag()

class SphereCollider(Collider):
    def __init__(
        self, world:Ursina, entity:Entity,
        rotation=[None, None, None], scale=None,
        mass=0, ghost=False
        ) -> None:

        if scale == None:
            model = str(entity.model).split('/')[-1]
            if model == 'sphere':
                scale = mean(entity.scale)/2
            else:
                scale = 0.5

        super().__init__(world, entity, BulletSphereShape(scale), 'sphere', rotation, mass, ghost)

class BoxCollider(Collider):
    def __init__(
        self, world:Ursina, entity:Entity,
        rotation=[None, None, None], scale=[None, None, None],
        mass=0, ghost=False
        ) -> None:
        
        if None in scale:
            for x in range(len(entity.scale)):
                scale[x] = entity.scale[x]/2

        super().__init__(world, entity, BulletBoxShape(Vec3(*scale)), 'box', rotation, mass, ghost)

class CylinderCollider(Collider):
    def __init__(
        self, world:Ursina, entity:Entity,
        rotation=[None, None, None], radius=None, height=None,
        mass=0, ghost=False
        ):

        if radius==None or height==None:
            if type(entity.model) == Cylinder:
                radius=entity.model.radius
                height=entity.model.height
            else:
                radius=.5
                height=1
        super().__init__(world, entity, BulletCylinderShape(radius, height, 1), 'cylinder', rotation, mass, ghost)