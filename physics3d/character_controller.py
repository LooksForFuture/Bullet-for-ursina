from ursina import Entity, Vec3, application
from panda3d.bullet import BulletWorld, BulletCapsuleShape, BulletCharacterControllerNode

class CharacterController(BulletCharacterControllerNode):
    def __init__(self, world:BulletWorld, entity:Entity, radius=1, height=2, name='Player', **opts) -> None:
        super().__init__(BulletCapsuleShape(radius/2, height/2, 1), radius/2, name)
        self.np = application.base.render.attachNewNode(self)

        if entity.parent:
            self.np.reparent_to(entity.parent)
        
        rotation = Vec3(0, 0, 0)
        if None in rotation:
            hpr = entity.getHpr()
            for x in range(len(hpr)):
                rotation[x] = hpr[x]
        self.np.setHpr(rotation)
        
        self.np.setPos(entity.x, entity.y, entity.z)
        entity.reparent_to(self.np)

        world.attachCharacter(self)

        self.__fall_speed = None
        self.__jump_speed = None
        self.__max_jump_height = None

        for x in opts:
            setattr(self, x, opts[x])
    
    def jump(self):
        self.doJump()
    
    def move(self, vel:Vec3, is_local:bool):
        self.setLinearMovement(vel, is_local)
    
    def rotate(self, omega:float):
        self.setAngularMovement(omega)
    
    @property
    def can_jump(self):
        return self.canJump()
    
    @property
    def fall_speed(self):
        return self.__fall_speed
    
    @fall_speed.setter
    def fall_speed(self, speed:float):
        self.__fall_speed = speed
        self.setFallSpeed(speed)
    
    @property
    def gravity(self):
        return self.gravity
    
    @gravity.setter
    def gravity(self, grav:float):
        self.setGravity(grav)
    
    @property
    def jump_speed(self):
        return self.__jump_speed
    
    @jump_speed.setter
    def jump_speed(self, speed:float):
        self.__jump_speed = speed
        self.setJumpSpeed(speed)
    
    @property
    def max_jump_height(self):
        return self.__max_jump_height
    
    @max_jump_height.setter
    def max_jump_height(self, max_jump_height:float):
        self.__max_jump_height = max_jump_height
        self.setMaxJumpHeight(max_jump_height)
    
    @property
    def on_ground(self):
        return self.isOnGround()
