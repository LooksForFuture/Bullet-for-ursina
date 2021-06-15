from ursina import application
from ursina import Entity, Vec3
from panda3d.bullet import (
    BulletWorld, BulletRigidBodyNode, BulletGhostNode, BulletSphereShape, BulletBoxShape,
    BulletCylinderShape, BulletCapsuleShape, BulletConeShape, BulletConvexHullShape,
    BulletTriangleMeshShape, BulletTriangleMesh, BulletDebugNode
    )

class Collider:
    def __init__(self, world:BulletWorld, entity:Entity, shape, name, rotation, mass, ghost):
        self.shape = shape
        if ghost:
            self.node = BulletGhostNode(name)
        else:
            self.node = BulletRigidBodyNode(name)
            self.node.setMass(mass)

        self.node.addShape(shape)
        self.np = application.base.render.attachNewNode(self.node)

        if ghost:
            world.attachGhost(self.node)
        else:
            world.attachRigidBody(self.node)

        if entity.parent:
            self.np.reparent_to(entity.parent)
        
        if None in rotation:
            hpr = entity.getHpr()
            for x in range(len(hpr)):
                rotation[x] = hpr[x]
        self.np.setHpr(Vec3(*rotation))
        
        self.np.setPos(entity.x, entity.y, entity.z)
        entity.reparent_to(self.np)
    
    @property
    def position(self):
        return self.np.getPos()
    
    @position.setter
    def position(self, val:list):
        if type(val) == Vec3:
            self.np.setPos(val.x, val.y, val.z)
        
        else:
            self.np.setPos(val[0], val[1], val[2])
    
    @property
    def x(self):
        return self.np.getPos()[0]
    
    @x.setter
    def x(self, val:float):
        pos = self.position
        self.np.setPos(val, pos[1], pos[2])
    
    @property
    def y(self):
        return self.np.getPos()[1]
    
    @y.setter
    def y(self, val:float):
        pos = self.position
        self.np.setPos(pos[0], val, pos[2])
    
    @property
    def z(self):
        return self.np.getPos()[2]
    
    @z.setter
    def z(self, val:float):
        pos = self.position
        self.np.setPos(pos[0], pos[1], val)
    
    @property
    def rotation(self):
        return self.np.getHpr()
    
    @rotation.setter
    def rotation(self, val:list):
        if type(val) == Vec3:
            self.np.setHpr(val.x, val.y, val.z)
        
        else:
            self.np.setHpr(val[0], val[1], val[2])
    
    @property
    def rotation_x(self):
        return self.np.getHpr()[0]
    
    @rotation_x.setter
    def rotation_x(self, val:float):
        rot = self.rotation
        self.np.setHpr(val, rot[1], rot[2])
    
    @property
    def rotation_y(self):
        return self.np.getHpr()[1]
    
    @rotation_y.setter
    def rotation_y(self, val:float):
        rot = self.rotation
        self.np.setHpr(rot[0], val, rot[2])
    
    @property
    def rotation_z(self):
        return self.np.getHpr()[2]
    
    @rotation_z.setter
    def rotation_z(self, val:float):
        rot = self.rotation
        self.np.setHpr(rot[0], rot[1], val)
    
    @property
    def scale(self):
        return self.np.getScale()
    
    @scale.setter
    def scale(self, val:list):
        if type(val) == Vec3:
            self.np.setScale(val.x, val.y, val.z)
        
        else:
            self.np.setScale(val[0], val[1], val[2])
    
    @property
    def scale_x(self):
        return self.np.getScale()[0]
    
    @scale_x.setter
    def scale_x(self, val:float):
        scale = self.scale
        self.np.setScale(val, scale[1], scale[2])
    
    @property
    def scale_y(self):
        return self.np.getScale()[1]
    
    @scale_y.setter
    def scale_y(self, val:float):
        scale = self.scale
        self.np.setScale(scale[0], val, scale[2])
    
    @property
    def scale_z(self):
        return self.np.getScale()[2]
    
    @scale_z.setter
    def scale_z(self, val:float):
        scale = self.scale
        self.np.setScale(scale[0], scale[1], val)
    
    @property
    def active(self):
        return self.node.active
    
    @active.setter
    def active(self, value):
        self.node.active = value
    
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
    def angular_velocity(self, value):
        self.node.setAngularVelocity(value)
    
    @property
    def anisotropic_friction(self):
        return self.node.anisotropic_friction
    
    @anisotropic_friction.setter
    def anisotropic_friction(self, value):
        self.node.setAnisotropicFriction(value)
    
    @property
    def ccd_motion_threshold(self):
        return self.node.ccd_motion_threshold
    
    @ccd_motion_threshold.setter
    def ccd_motion_threshold(self, value):
        self.node.setCcdMotionThreshold(value)
    
    @property
    def ccd_swept_sphere_radius(self):
        return self.node.ccd_swept_sphere_radius
    
    @ccd_swept_sphere_radius.setter
    def ccd_swept_sphere_radius(self, value):
        self.node.setCcdSweptSphereRadius(value)

    @property
    def collision_notification(self):
        return self.node.collision_notification
    
    @collision_notification.setter
    def collision_notification(self, value):
        self.node.notifyCollisions(value)
    
    @property
    def collision_response(self):
        return self.node.collision_response
    
    @collision_response.setter
    def collision_response(self, value):
        self.node.setCollisionResponse(value)
    
    @property
    def contact_processing_threshold(self):
        return self.node.contact_processing_threshold
    
    @contact_processing_threshold.setter
    def contact_processing_threshold(self, value):
        self.node.contact_processing_threshold = value
    
    @property
    def contact_response(self):
        return self.node.contact_response
    
    @property
    def deactivation_enabled(self):
        return self.node.deactivation_enabled
    
    @deactivation_enabled.setter
    def deactivation_enabled(self, value):
        self.node.deactivation_enabled = value
    
    @property
    def deactivation_time(self):
        return self.node.deactivation_time
    
    @deactivation_time.setter
    def deactivation_time(self, value):
        self.node.setDeactivationTime(value)
    
    @property
    def debug_enabled(self):
        return self.node.debug_enabled
    
    @debug_enabled.setter
    def debug_enabled(self, value):
        self.node.debug_enabled = value
    
    @property
    def friction(self):
        return self.node.friction
    
    @friction.setter
    def friction(self, value):
        self.node.setFriction(value)

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
    def inv_inertia_diag_local(self):
        return self.node.inv_inertia_diag_local
    
    @property
    def inv_inertia_tensor_world(self):
        return self.node.inv_inertia_tensor_world
    
    @property
    def inv_mass(self):
        return self.node.inv_mass
    
    @property
    def kinematic(self):
        return self.node.kinematic
    
    @kinematic.setter
    def kinematic(self, value):
        self.node.setKinematic(value)
    
    @property
    def overlapping_nodes(self):
        return self.node.overlapping_nodes
    
    @property
    def restitution(self):
        return self.node.restitution
    
    @restitution.setter
    def restitution(self, value):
        self.node.restitution = value
    
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
    def num_shapes(self):
        return self.node.getNumShapes()
    
    @property
    def shape_bounds(self):
        return self.node.shape_bounds
    
    @property
    def shape_mat(self):
        return self.node.shape_mat
    
    @property
    def shape_pos(self):
        return self.node.shape_pos
    
    @property
    def shape_transform(self):
        return self.node.shape_transform
    
    @property
    def shapes(self):
        return self.node.shapes
    
    @property
    def static(self):
        return self.node.static
    
    @static.setter
    def static(self, value):
        self.node.static = value
    
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
    
    def check_collision_with(self, entity:Entity):
        return self.node.checkCollisionWith(entity)
    
    def clear_forces(self):
        self.node.clearForces()
    
    def force_active(self, active: bool):
        self.node.forceActive(active)
    
    def pick_dirty_flag(self):
        return self.node.pickDirtyFlag()
    
    def set_active(self, active: bool, force: bool):
        self.node.setActive(active, force)
    
    def set_collide_mask(self, mask):
        self.np.setCollideMask(mask)
    
    def set_into_collide_mask(self, mask):
        self.node.setIntoCollideMask(mask)

class SphereCollider(Collider):
    def __init__(
        self, world:BulletWorld, entity:Entity,
        rotation=[None, None, None], scale=None,
        mass=0, ghost=False
        ) -> None:

        if scale == None:
            model = str(entity.model).split('/')[-1]
            if model == 'sphere':
                scale = entity.scale_x/2
            else:
                scale = 0.5

        super().__init__(world, entity, BulletSphereShape(scale), 'sphere', rotation, mass, ghost)

class BoxCollider(Collider):
    def __init__(
        self, world:BulletWorld, entity:Entity,
        rotation=[None, None, None], scale=[None, None, None],
        mass=0, ghost=False
        ) -> None:
        
        if None in scale:
            for x in range(len(entity.scale)):
                scale[x] = entity.scale[x]/2

        super().__init__(world, entity, BulletBoxShape(Vec3(*scale)), 'box', rotation, mass, ghost)

class CylinderCollider(Collider):
    def __init__(
        self, world:BulletWorld, entity:Entity,
        rotation=[None, None, None], radius=.5, height=1,
        mass=0, ghost=False
        ):
        super().__init__(world, entity, BulletCylinderShape(radius, height, 1), 'cylinder', rotation, mass, ghost)

class CapsuleCollider(Collider):
    def __init__(
        self, world:BulletWorld, entity:Entity,
        rotation=[None, None, None], radius=0.5, height=1,
        mass=0, ghost=False
        ):
        super().__init__(world, entity, BulletCapsuleShape(radius, height, 1), 'capsule', rotation, mass, ghost)

class ConeCollider(Collider):
    def __init__(
        self, world:BulletWorld, entity:Entity,
        rotation=[None, None, None], radius=0.5, height=1,
        mass=0, ghost=False
        ):
        super().__init__(world, entity, BulletConeShape(radius, height, 1), 'cone', rotation, mass, ghost)

class ConvexHullCollider(Collider):
    def __init__(
        self, world:BulletWorld, entity:Entity,
        rotation=[None, None, None], mass=0, ghost=False
        ):

        geomNodes = entity.model.findAllMatches('**/+GeomNode')
        geomNode = geomNodes.getPath(0).node()
        geom = geomNode.getGeom(0)
        shape = BulletConvexHullShape()
        shape.addGeom(geom)

        super().__init__(world, entity, shape, 'convex_hull', rotation, mass, ghost)

class MeshCollider(Collider):
    def __init__(
        self, world:BulletWorld, entity:Entity,
        rotation=[None, None, None], dynamic=False,
        mass=0, ghost=False
        ):

        geomNodes = entity.model.findAllMatches('**/+GeomNode')
        geomNode = geomNodes.getPath(0).node()
        geom = geomNode.getGeom(0)
        mesh = BulletTriangleMesh()
        mesh.addGeom(geom)

        shape = BulletTriangleMeshShape(mesh, dynamic=dynamic)

        super().__init__(world, entity, shape, 'mesh', rotation, mass, ghost)

class Debugger:
    def __init__(self, world:BulletWorld, **opts) -> None:
        self.node = BulletDebugNode('Debug')
        self.node.bounding_boxes=False
        self.node.constraints=False
        self.node.normals=False
        self.node.wireframe=False

        self.np = application.base.render.attachNewNode(self.node)
        self.np.show()
        world.setDebugNode(self.node)

        for x in opts:
            setattr(self.node, x, opts[x])
    
    @property
    def bounding_boxes(self):
        return self.node.bounding_boxes
    
    @bounding_boxes.setter
    def bounding_boxes(self, val:bool):
        self.node.bounding_boxes = val
    
    @property
    def constraints(self):
        return self.node.constraints
    
    @constraints.setter
    def constraints(self, val:bool):
        self.node.constraints = val
    
    @property
    def normals(self):
        return self.node.normals
    
    @normals.setter
    def normals(self, val:bool):
        self.node.normals = val
    
    @property
    def wireframe(self):
        return self.node.wireframe
    
    @wireframe.setter
    def wireframe(self, val:bool):
        self.node.wireframe = val
