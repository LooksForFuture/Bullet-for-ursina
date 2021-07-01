from ursina import Vec3
from panda3d.core import LMatrix3
from panda3d.bullet import BulletBodyNode, BulletRigidBodyNode, BulletGhostNode

class BodyNode(BulletBodyNode):
    @property
    def active(self) -> bool:
        return super().active
    
    @active.setter
    def active(self, val:bool):
        super().active = val
    
    @property
    def anisotropic_friction(self) -> Vec3:
        return super().anisotropic_friction
    
    @anisotropic_friction.setter
    def anisotropic_friction(self, value:Vec3):
        self.setAnisotropicFriction(value)
    
    @property
    def ccd_motion_threshold(self) -> float:
        return super().ccd_motion_threshold
    
    @ccd_motion_threshold.setter
    def ccd_motion_threshold(self, value:float):
        self.setCcdMotionThreshold(value)
    
    @property
    def ccd_swept_sphere_radius(self) -> float:
        return super().ccd_swept_sphere_radius
    
    @ccd_swept_sphere_radius.setter
    def ccd_swept_sphere_radius(self, value:float):
        self.setCcdSweptSphereRadius(value)
    
    @property
    def collision_notification(self) -> bool:
        return super().collision_notification
    
    @collision_notification.setter
    def collision_notification(self, value:bool):
        self.notifyCollisions(value)
    
    @property
    def collision_response(self) -> bool:
        return super().collision_response
    
    @collision_response.setter
    def collision_response(self, value:bool):
        self.setCollisionResponse(value)
    
    @property
    def contact_processing_threshold(self) -> float:
        return super().contact_processing_threshold
    
    @contact_processing_threshold.setter
    def contact_processing_threshold(self, value:float):
        super().contact_processing_threshold = value
    
    @property
    def contact_response(self) -> bool:
        return super().contact_response
    
    @property
    def deactivation_enabled(self) -> bool:
        return super().deactivation_enabled
    
    @deactivation_enabled.setter
    def deactivation_enabled(self, value:bool):
        super().deactivation_enabled = value
    
    @property
    def deactivation_time(self) -> float:
        return super().deactivation_time
    
    @deactivation_time.setter
    def deactivation_time(self, value:float):
        self.setDeactivationTime(value)
    
    @property
    def debug_enabled(self) -> bool:
        return super().debug_enabled
    
    @debug_enabled.setter
    def debug_enabled(self, value:bool):
        super().debug_enabled = value
    
    @property
    def friction(self) -> float:
        return super().friction
    
    @friction.setter
    def friction(self, value:float):
        self.setFriction(value)
    
    @property
    def num_shapes(self) -> int:
        return self.getNumShapes()
    
    @property
    def restitution(self) -> float:
        return super().restitution
    
    @restitution.setter
    def restitution(self, value:float):
        super().restitution = value
    
    @property
    def shape_bounds(self):
        return super().shape_bounds
    
    @property
    def shapes(self) -> list:
        return self.getShapes()
    
    @property
    def has_anisotropic_friction(self) -> bool:
        return self.hasAnisotropicFriction()
    
    @property
    def kinematic(self) -> bool:
        return super().kinematic
    
    @kinematic.setter
    def kinematic(self, value:bool):
        self.setKinematic(value)
    
    @property
    def static(self) -> bool:
        return super().static
    
    @static.setter
    def static(self, val:bool):
        super().static = val

class RigidBodyNode(BodyNode, BulletRigidBodyNode):
    @property
    def angular_damping(self) -> float:
        return super().angular_damping
    
    @angular_damping.setter
    def angular_damping(self, val:float):
        self.setAngularDamping(val)
    
    @property
    def angular_factor(self) -> Vec3:
        return super().angular_factor
    
    @angular_factor.setter
    def angular_factor(self, factor:Vec3):
        self.setAngularFactor(factor)
    
    @property
    def angular_sleep_threshold(self) -> float:
        return super().angular_sleep_threshold
    
    @angular_sleep_threshold.setter
    def angular_sleep_threshold(self, threshold:float):
        self.setAngularSleepThreshold(threshold)
    
    @property
    def angular_velocity(self) -> Vec3:
        return super().angular_velocity
    
    @angular_velocity.setter
    def angular_velocity(self, velocity:Vec3):
        self.setAngularVelocity(velocity)
    
    @property
    def gravity(self) -> Vec3:
        return self.getGravity()
    
    @gravity.setter
    def gravity(self, gravity:Vec3):
        self.setGravity(gravity)
    
    @property
    def inertia(self) -> Vec3:
        return self.getInertia()
    
    @inertia.setter
    def inertia(self, inertia:Vec3):
        self.setInertia(inertia)
    
    @property
    def inv_inertia_diag_local(self) -> Vec3:
        return super().getInvInertiaDiagLocal()
    
    @property
    def inv_inertia_tensor_world(self) -> LMatrix3:
        return super().getInvInertiaTensorWorld()
    
    @property
    def inv_mass(self) -> float:
        return self.getInvMass()
    
    @property
    def linear_damping(self) -> float:
        return super().getLinearDamping()
    
    @linear_damping.setter
    def linear_damping(self, value:float):
        self.setLinearDamping(value)
    
    @property
    def linear_factor(self) -> Vec3:
        return self.getLinearFactor()
    
    @linear_factor.setter
    def linear_factor(self, factor:Vec3):
        self.setLinearFactor(factor)
    
    @property
    def linear_sleep_threshold(self) -> float:
        return self.getLinearSleepThreshold()
    
    @linear_sleep_threshold.setter
    def linear_sleep_threshold(self, threshold:float):
        self.setLinearSleepThreshold(threshold)
    
    @property
    def linear_velocity(self) -> Vec3:
        return self.getLinearVelocity()
    
    @linear_velocity.setter
    def linear_velocity(self, velocity:Vec3):
        self.setLinearVelocity(velocity)
    
    @property
    def mass(self) -> float:
        return self.getMass()
    
    @mass.setter
    def mass(self, mass:float):
        self.setMass(mass)
    
    @property
    def total_force(self) -> Vec3:
        return self.getTotalForce()
    
    @property
    def total_torque(self) -> Vec3:
        return self.getTotalTorque()

class GhostNode(BodyNode, BulletGhostNode):
    @property
    def num_overlapping_nodes(self) -> int:
        return self.getNumOverlappingNodes()
