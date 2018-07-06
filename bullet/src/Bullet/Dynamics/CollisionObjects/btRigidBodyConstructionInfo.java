/*
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it freely,
 * subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.Dynamics.CollisionObjects;

import Bullet.Collision.Shape.btCollisionShape;
import Bullet.LinearMath.btMotionState;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import java.util.Objects;

/**
 * The btRigidBodyConstructionInfo structure provides information to create a
 * rigid body. Setting mass to zero creates a fixed (non-dynamic) rigid body.
 * For dynamic objects, you can use the collision shape to approximate the local
 * inertia tensor, otherwise use the zero vector (default argument) You can use
 * the motion state to synchronize the world transform between physics and
 * graphics objects. And if the motion state is provided, the rigid body will
 * initialize its initial world transform from the motion state,
 * m_startWorldTransform is only used when you don't provide a motion state.
 *
 * @author Gregery Barton
 */
public class btRigidBodyConstructionInfo implements Serializable {

 public float m_mass;
 ///When a motionState is provided, the rigid body will initialize its world transform from the motion state
 ///In this case, m_startWorldTransform is ignored.
 public btMotionState m_motionState;
 final public btTransform m_startWorldTransform = new btTransform();
 final public btCollisionShape m_collisionShape;
 final public btVector3 m_localInertia = new btVector3();
 public float m_linearDamping;
 public float m_angularDamping;
 ///best simulation results when friction is non-zero
 public float m_friction;
 ///the m_rollingFriction prevents rounded shapes, such as spheres, cylinders and capsules from rolling forever.
 ///See Bullet/Demos/RollingFrictionDemo for usage
 public float m_rollingFriction;
 public float m_spinningFriction;//torsional friction around contact normal
 ///best simulation results using zero restitution.
 public float m_restitution;
 public float m_linearSleepingThreshold;
 public float m_angularSleepingThreshold;
 //Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
 //Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics system has improved, this should become obsolete
 public boolean m_additionalDamping;
 public float m_additionalDampingFactor;
 public float m_additionalLinearDampingThresholdSqr;
 public float m_additionalAngularDampingThresholdSqr;
 public float m_additionalAngularDampingFactor;
 public final btVector3 m_linearFactor = new btVector3(1, 1, 1);
 public final btVector3 m_angularFactor = new btVector3(1, 1, 1);

 public btRigidBodyConstructionInfo(float mass, btMotionState motionState,
  btCollisionShape collisionShape) {
  this(mass, motionState, collisionShape, new btVector3());
 }

 public btRigidBodyConstructionInfo(float mass, btMotionState motionState,
  btCollisionShape collisionShape, final btVector3 localInertia) {
  m_mass = mass;
  m_motionState = motionState;
  m_collisionShape = collisionShape;
  m_localInertia.set(localInertia);
  m_linearDamping = 0f;
  m_angularDamping = 0f;
  m_friction = 0.5f;
  m_rollingFriction = 0f;
  m_spinningFriction = 0f;
  m_restitution = 0f;
  m_linearSleepingThreshold = 0.8f;
  m_angularSleepingThreshold = 1.f;
  m_additionalDamping = false;
  m_additionalDampingFactor = 0.005f;
  m_additionalLinearDampingThresholdSqr = 0.01f;
  m_additionalAngularDampingThresholdSqr = 0.01f;
  m_additionalAngularDampingFactor = 0.01f;
  m_startWorldTransform.setIdentity();
 }

 @Override
 public int hashCode() {
  int hash = 3;
  hash = 71 * hash + Float.floatToIntBits(this.m_mass);
  hash = 71 * hash + Objects.hashCode(this.m_motionState);
  hash = 71 * hash + Objects.hashCode(this.m_startWorldTransform);
  hash = 71 * hash + Objects.hashCode(this.m_collisionShape);
  hash = 71 * hash + Objects.hashCode(this.m_localInertia);
  hash = 71 * hash + Float.floatToIntBits(this.m_linearDamping);
  hash = 71 * hash + Float.floatToIntBits(this.m_angularDamping);
  hash = 71 * hash + Float.floatToIntBits(this.m_friction);
  hash = 71 * hash + Float.floatToIntBits(this.m_rollingFriction);
  hash = 71 * hash + Float.floatToIntBits(this.m_spinningFriction);
  hash = 71 * hash + Float.floatToIntBits(this.m_restitution);
  hash = 71 * hash + Float.floatToIntBits(this.m_linearSleepingThreshold);
  hash = 71 * hash + Float.floatToIntBits(this.m_angularSleepingThreshold);
  hash = 71 * hash + (this.m_additionalDamping ? 1 : 0);
  hash = 71 * hash + Float.floatToIntBits(this.m_additionalDampingFactor);
  hash = 71 * hash + Float.floatToIntBits(
   this.m_additionalLinearDampingThresholdSqr);
  hash = 71 * hash + Float.floatToIntBits(
   this.m_additionalAngularDampingThresholdSqr);
  hash = 71 * hash + Float.floatToIntBits(this.m_additionalAngularDampingFactor);
  return hash;
 }

 @Override
 public boolean equals(Object obj) {
  if (this == obj) {
   return true;
  }
  if (obj == null) {
   return false;
  }
  if (getClass() != obj.getClass()) {
   return false;
  }
  final btRigidBodyConstructionInfo other = (btRigidBodyConstructionInfo) obj;
  if (Float.floatToIntBits(this.m_mass) != Float.floatToIntBits(other.m_mass)) {
   return false;
  }
  if (Float.floatToIntBits(this.m_linearDamping) != Float.floatToIntBits(
   other.m_linearDamping)) {
   return false;
  }
  if (Float.floatToIntBits(this.m_angularDamping) != Float.floatToIntBits(
   other.m_angularDamping)) {
   return false;
  }
  if (Float.floatToIntBits(this.m_friction) != Float.floatToIntBits(
   other.m_friction)) {
   return false;
  }
  if (Float.floatToIntBits(this.m_rollingFriction) != Float.floatToIntBits(
   other.m_rollingFriction)) {
   return false;
  }
  if (Float.floatToIntBits(this.m_spinningFriction) != Float.floatToIntBits(
   other.m_spinningFriction)) {
   return false;
  }
  if (Float.floatToIntBits(this.m_restitution) != Float.floatToIntBits(
   other.m_restitution)) {
   return false;
  }
  if (Float.floatToIntBits(this.m_linearSleepingThreshold) != Float
   .floatToIntBits(other.m_linearSleepingThreshold)) {
   return false;
  }
  if (Float.floatToIntBits(this.m_angularSleepingThreshold) != Float
   .floatToIntBits(other.m_angularSleepingThreshold)) {
   return false;
  }
  if (this.m_additionalDamping != other.m_additionalDamping) {
   return false;
  }
  if (Float.floatToIntBits(this.m_additionalDampingFactor) != Float
   .floatToIntBits(other.m_additionalDampingFactor)) {
   return false;
  }
  if (Float.floatToIntBits(this.m_additionalLinearDampingThresholdSqr) != Float
   .floatToIntBits(other.m_additionalLinearDampingThresholdSqr)) {
   return false;
  }
  if (Float.floatToIntBits(this.m_additionalAngularDampingThresholdSqr) != Float
   .floatToIntBits(other.m_additionalAngularDampingThresholdSqr)) {
   return false;
  }
  if (Float.floatToIntBits(this.m_additionalAngularDampingFactor) != Float
   .floatToIntBits(other.m_additionalAngularDampingFactor)) {
   return false;
  }
  if (!Objects.equals(this.m_motionState, other.m_motionState)) {
   return false;
  }
  if (!Objects.equals(this.m_startWorldTransform, other.m_startWorldTransform)) {
   return false;
  }
  if (!Objects.equals(this.m_collisionShape, other.m_collisionShape)) {
   return false;
  }
  if (!Objects.equals(this.m_localInertia, other.m_localInertia)) {
   return false;
  }
  return true;
 }

}
