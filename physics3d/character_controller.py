from ursina import Entity, Vec3, application
from panda3d.core import BitMask32
from panda3d.bullet import BulletWorld, BulletCapsuleShape, BulletCharacterControllerNode

class CharacterController:
    def __init__(self, world:BulletWorld, entity:Entity, radius=1, height=2, name='Player', **opts) -> None:
        self.__shape = BulletCapsuleShape(radius/2, height/2, 1)
        self.node = BulletCharacterControllerNode(self.__shape, radius/2, name)
        self.np = application.base.render.attachNewNode(self.node)

        if entity.parent:
            self.np.reparent_to(entity.parent)
        
        rotation = [None, None, None]
        if None in rotation:
            hpr = entity.getHpr()
            for x in range(len(hpr)):
                rotation[x] = hpr[x]
        self.np.setHpr(Vec3(*rotation))
        
        self.np.setPos(entity.x, entity.y, entity.z)
        entity.reparent_to(self.np)

        self.np.setCollideMask(BitMask32.allOn())
        world.attachCharacter(self.node)

        self.__fall_speed = None
        self.__jump_speed = None
        self.__max_jump_height = None

        for x in opts:
            setattr(self, x, opts[x])
    
    def jump(self):
        self.node.doJump()
    
    def move(self, vel:Vec3, is_local:bool):
        self.node.setLinearMovement(vel, is_local)
    
    def rotate(self, omega:float):
        self.node.setAngularMovement(omega)
    
    def set_max_slope(self, slope:float):
        self.node.setMaxSlope(slope)
    
    def set_use_ghost_sweep_test(self, val:bool):
        self.node.setUseGhostSweepTest(val)

    @property
    def can_jump(self):
        return self.node.canJump()
    
    @property
    def fall_speed(self):
        return self.__fall_speed
    
    @fall_speed.setter
    def fall_speed(self, speed:float):
        self.__fall_speed = speed
        self.node.setFallSpeed(speed)
    
    @property
    def gravity(self):
        return self.node.gravity
    
    @gravity.setter
    def gravity(self, grav:float):
        self.node.setGravity(grav)
    
    @property
    def jump_speed(self):
        return self.__jump_speed
    
    @jump_speed.setter
    def jump_speed(self, speed:float):
        self.__jump_speed = speed
        self.node.setJumpSpeed(speed)
    
    @property
    def max_jump_height(self):
        return self.__max_jump_height
    
    @max_jump_height.setter
    def max_jump_height(self, max_jump_height:float):
        self.__max_jump_height = max_jump_height
        self.node.setMaxJumpHeight(max_jump_height)
    
    @property
    def on_ground(self):
        return self.node.isOnGround()
    
    @property
    def shape(self):
        return self.__shape
