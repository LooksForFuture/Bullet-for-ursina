from ursina import application
from ursina import Entity, Vec3
from physics3d.core import GhostNode
from panda3d.bullet import (
    BulletWorld, BulletSphereShape, BulletPlaneShape, BulletBoxShape,
    BulletCylinderShape, BulletCapsuleShape, BulletConeShape, BulletConvexHullShape,
    BulletTriangleMeshShape, BulletTriangleMesh, BulletDebugNode
    )

class Ghost(GhostNode):
    def __init__(self, world:BulletWorld, entity:Entity, shape, name, rotation):
        super().__init__(name)

        self.addShape(shape)
        self.np = application.base.render.attachNewNode(self)

        world.attachGhost(self)

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

class GhostSphere(Ghost):
    def __init__(
        self, world:BulletWorld, entity:Entity,
        rotation=[None, None, None], scale=None
        ) -> None:

        if scale == None:
            model = str(entity.model).split('/')[-1]
            if model == 'sphere':
                scale = entity.scale_x/2
            else:
                scale = 0.5

        super().__init__(world, entity, BulletSphereShape(scale), 'sphere', rotation)

class GhostPlane(Ghost):
    def __init__(
        self, world: BulletWorld, entity: Entity,
        rotation=[None, None, None]
        ) -> None:

        super().__init__(world, entity, BulletPlaneShape(Vec3(0, 1, 0), 0), 'plane', rotation)

class GhostBox(Ghost):
    def __init__(
        self, world:BulletWorld, entity:Entity,
        rotation=[None, None, None], scale=[None, None, None]
        ) -> None:
        
        if None in scale:
            for x in range(len(entity.scale)):
                scale[x] = entity.scale[x]/2

        super().__init__(world, entity, BulletBoxShape(Vec3(*scale)), 'box', rotation)

class GhostCylinder(Ghost):
    def __init__(
        self, world:BulletWorld, entity:Entity, radius=0.5,
        height=1, rotation=[None, None, None]
        ):
        super().__init__(world, entity, BulletCylinderShape(radius, height, 1), 'cylinder', rotation)

class GhostCapsule(Ghost):
    def __init__(
        self, world:BulletWorld, entity:Entity, radius=0.5,
        height=1, rotation=[None, None, None]
        ):
        super().__init__(world, entity, BulletCapsuleShape(radius, height, 1), 'capsule', rotation)

class GhostCone(Ghost):
    def __init__(
        self, world:BulletWorld, entity:Entity, radius=0.5,
        mass=0, height=1, rotation=[None, None, None]
        ):
        super().__init__(world, entity, BulletConeShape(radius, height, 1), 'cone', rotation)

class GhostConvexHull(Ghost):
    def __init__(
        self, world:BulletWorld, entity:Entity,
        mass=0, rotation=[None, None, None]
        ):

        geomNodes = entity.model.findAllMatches('**/+GeomNode')
        geomNode = geomNodes.getPath(0).node()
        geom = geomNode.getGeom(0)
        shape = BulletConvexHullShape()
        shape.addGeom(geom)

        super().__init__(world, entity, shape, 'convex_hull', rotation)

class GhostMesh(Ghost):
    def __init__(
        self, world:BulletWorld, entity:Entity,
        dynamic=False, rotation=[None, None, None]
        ):

        geomNodes = entity.model.findAllMatches('**/+GeomNode')
        geomNode = geomNodes.getPath(0).node()
        geom = geomNode.getGeom(0)
        mesh = BulletTriangleMesh()
        mesh.addGeom(geom)

        shape = BulletTriangleMeshShape(mesh, dynamic=dynamic)

        super().__init__(world, entity, shape, 'mesh', rotation)
