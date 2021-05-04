from statistics import mean
from ursina import Ursina, Entity, Vec3, Cylinder
from panda3d.bullet import (
    BulletRigidBodyNode, BulletGhostNode, BulletSphereShape, BulletBoxShape
    )

class Collider:
    def __init__(self, world:Ursina, entity:Entity, shape, name, mass, ghost):
        self.world = world
        if ghost:
            self.node = BulletGhostNode(name)
        else:
            self.node = BulletRigidBodyNode(name)
            self.node.setMass(mass)
        
        self.node.addShape(shape)
        self.np = world.render.attachNewNode(self.node)
        self.np.setPos(entity.x, entity.y, entity.z)
        self.np.setHpr(entity.getHpr())

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
    def angularDamping(self):
        return self.node.getAngularDamping()
    
    @angularDamping.setter
    def angularDamping(self, value):
        self.node.setAngularDamping(value)
    
    @property
    def angularFactor(self):
        return self.node.getAngularFactor()
    
    @angularFactor.setter
    def angularFactor(self, value):
        self.node.setAngularFactor(value)
    
    @property
    def angularSleepThreshold(self):
        return self.node.getAngularSleepThreshold()
    
    @angularSleepThreshold.setter
    def angularSleepThreshold(self, value):
        self.node.setAngularSleepThreshold(value)
    
    @property
    def angularVelocity(self):
        return self.node.getAngularVelocity()
    
    @angularVelocity.setter
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
    def linearDamping(self):
        return self.node.getLinearDamping()
    
    @linearDamping.setter
    def linearDamping(self, value):
        self.node.setLinearDamping(value)
    
    @property
    def linearFactor(self):
        return self.node.getLinearFactor()
    
    @linearFactor.setter
    def linearFactor(self, value):
        self.node.setLinearFactor(value)
    
    @property
    def linearSleepThreshold(self):
        return self.node.getLinearSleepThreshold()
    
    @linearSleepThreshold.setter
    def linearSleepThreshold(self, value):
        self.node.setLinearSleepThreshold(value)
    
    @property
    def linearVelocity(self):
        return self.node.getLinearVelocity()
    
    @linearVelocity.setter
    def linearVelocity(self, value):
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
    
    def applyCentralForce(self, force):
        self.node.applyCentralForce(force)
    
    def applyCentralImpulse(self, impulse):
        self.node.applyCentralImpulse(impulse)
    
    def applyForce(self, force, pos):
        self.node.applyForce(force, pos)
    
    def applyImpulse(self, impulse, pos):
        self.node.applyImpulse(impulse, pos)
    
    def applyTorque(self, torque):
        self.node.applyTorque(torque)
    
    def applyTorqueImpulse(self, torque):
        self.node.applyTorqueImpulse(torque)
    
    def clearForces(self):
        self.node.clearForces()
    
    def pickDirtyFlag(self):
        return self.node.pickDirtyFlag()

class SphereCollider(Collider):
    def __init__(self, world:Ursina, entity:Entity, scale=0.5, mass=0, ghost=False) -> None:
        model = str(entity.model).split('/')[-1]
        if model == 'sphere':
            scale = mean(entity.scale)/2

        super().__init__(world, entity, BulletSphereShape(scale), 'sphere', mass, ghost)

class BoxCollider(Collider):
    def __init__(self, world:Ursina, entity:Entity, scale=(0.5, 0.5, 0.5), mass=0, ghost=False) -> None:
        model = str(entity.model).split('/')[-1]
        if model == 'cube':
            scale = (x/2 for x in entity.scale)

        super().__init__(world, entity, BulletBoxShape(Vec3(*scale)), 'box', mass, ghost)