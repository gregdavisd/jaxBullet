/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.Dynamics;

import Bullet.Dynamics.Constraint.btTypedConstraint;
import static Bullet.Collision.CollisionFlags.CF_STATIC_OBJECT;
import static Bullet.Collision.CollisionObjectTypes.CO_RIGID_BODY;
import Bullet.Collision.Broadphase.btBroadphaseProxy;
import Bullet.Collision.btCollisionObject;
import Bullet.Collision.Shape.btCollisionShape;
import static Bullet.Dynamics.btRigidBodyFlags.BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY;
import static Bullet.Extras.btMinMax.btClamped;
import Bullet.LinearMath.btMatrix3x3;
import Bullet.LinearMath.btMotionState;
import Bullet.LinearMath.btQuaternion;
import static Bullet.LinearMath.btQuaternion.quatRotate;
import static Bullet.LinearMath.btScalar.SIMD_HALF_PI;
import static Bullet.LinearMath.btScalar.btPow;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btTransformUtil;
import Bullet.LinearMath.btVector3;
import static Bullet.common.btAlignedObjectArray.findLinearSearch;
import java.io.Serializable;
import java.util.ArrayList;
import static javax.vecmath.VecMath.DEBUG_BLOCKS;
import static javax.vecmath.VecMath.is_good_matrix;

/**
 * The btRigidBody is the main class for rigid body objects. It is derived from btCollisionObject,
 * so it keeps a pointer to a btCollisionShape. It is recommended for performance and memory use to
 * share btCollisionShape objects whenever possible. There are 3 types of rigid bodies: - A) Dynamic
 * rigid bodies, with positive mass. Motion is controlled by rigid body dynamics. - B) Fixed objects
 * with zero mass. They are not moving (basically collision objects) - C) Kinematic objects, which
 * are objects without mass, but the user can move them. There is on-way interaction, and Bullet
 * calculates a velocity based on the timestep and previous and current world transform. Bullet
 * automatically deactivates dynamic rigid bodies, when the velocity is below a threshold for a
 * given time. Deactivated (sleeping) rigid bodies don't take any processing time, except a minor
 * broadphase collision detection impact (to allow active objects to activate/wake up sleeping
 * objects)
 *
 * @author Gregery Barton
 */
public class btRigidBody extends btCollisionObject implements Serializable {
 //'temporarily' global variables

 public static float gDeactivationTime = (2.f);
 public static boolean gDisableDeactivation = false;
 static int uniqueId = 0;
 protected final btMatrix3x3 m_invInertiaTensorWorld = new btMatrix3x3();
 protected final btVector3 m_linearVelocity = new btVector3();
 protected final btVector3 m_angularVelocity = new btVector3();
 protected float m_inverseMass;
 protected final btVector3 m_linearFactor = new btVector3();
 protected final btVector3 m_gravity = new btVector3();
 protected final btVector3 m_gravity_acceleration = new btVector3();
 protected final btVector3 m_invInertiaLocal = new btVector3();
 protected final btVector3 m_totalForce = new btVector3();
 protected final btVector3 m_totalTorque = new btVector3();
 protected float m_linearDamping;
 protected float m_angularDamping;
 protected boolean m_additionalDamping;
 protected float m_additionalDampingFactor;
 protected float m_additionalLinearDampingThresholdSqr;
 protected float m_additionalAngularDampingThresholdSqr;
 protected float m_additionalAngularDampingFactor;
 protected float m_linearSleepingThreshold;
 protected float m_angularSleepingThreshold;
 //m_optionalMotionState allows to automatic synchronize the world transform for active objects
 protected btMotionState m_optionalMotionState;
 //keep track of typed constraints referencing this rigid body, to disable collision between linked bodies
 protected final ArrayList<btTypedConstraint> m_constraintRefs = new ArrayList<>(0);
 protected int m_rigidbodyFlags;
 //protected int m_debugBodyId;
 protected final btVector3 m_deltaLinearVelocity = new btVector3();
 protected final btVector3 m_deltaAngularVelocity = new btVector3();
 protected final btVector3 m_angularFactor = new btVector3();
 protected final btVector3 m_invMass = new btVector3();
 protected final btVector3 m_pushVelocity = new btVector3();
 protected final btVector3 m_turnVelocity = new btVector3();

 ///btRigidBody constructor using construction info
 public btRigidBody(btRigidBodyConstructionInfo constructionInfo) {
  setupRigidBody(constructionInfo);
  assert (is_good_matrix(m_worldTransform));
 }

 ///btRigidBody constructor for backwards compatibility. 
 ///To specify friction (etc) during rigid body construction, please use the other constructor (using btRigidBodyConstructionInfo)
 public btRigidBody(float mass, btMotionState motionState, btCollisionShape collisionShape) {
  this(mass, motionState, collisionShape, new btVector3());
  assert (is_good_matrix(m_worldTransform));
 }

 public btRigidBody(float mass, btMotionState motionState, btCollisionShape collisionShape,
  final btVector3 localInertia) {
  btRigidBodyConstructionInfo cinfo = new btRigidBodyConstructionInfo(mass, motionState,
   collisionShape, localInertia);
  setupRigidBody(cinfo);
  assert (is_good_matrix(m_worldTransform));
 }

 ///setupRigidBody is only used internally by the constructor
 final void setupRigidBody(btRigidBodyConstructionInfo constructionInfo) {
  m_internalType = CO_RIGID_BODY;
  m_linearVelocity.set((0.0f), (0.0f), (0.0f));
  m_angularVelocity.set((0.f), (0.f), (0.f));
  m_angularFactor.set(constructionInfo.m_angularFactor);
  m_linearFactor.set(constructionInfo.m_linearFactor);
  m_gravity.set((0.0f), (0.0f), (0.0f));
  m_gravity_acceleration.set((0.0f), (0.0f), (0.0f));
  m_totalForce.set((0.0f), (0.0f), (0.0f));
  m_totalTorque.set((0.0f), (0.0f), (0.0f));
  setDamping(constructionInfo.m_linearDamping, constructionInfo.m_angularDamping);
  m_linearSleepingThreshold = constructionInfo.m_linearSleepingThreshold;
  m_angularSleepingThreshold = constructionInfo.m_angularSleepingThreshold;
  m_optionalMotionState = constructionInfo.m_motionState;
  m_contactSolverType = 0;
  m_frictionSolverType = 0;
  m_additionalDamping = constructionInfo.m_additionalDamping;
  m_additionalDampingFactor = constructionInfo.m_additionalDampingFactor;
  m_additionalLinearDampingThresholdSqr = constructionInfo.m_additionalLinearDampingThresholdSqr;
  m_additionalAngularDampingThresholdSqr = constructionInfo.m_additionalAngularDampingThresholdSqr;
  m_additionalAngularDampingFactor = constructionInfo.m_additionalAngularDampingFactor;
  if (m_optionalMotionState != null) {
   m_optionalMotionState.getWorldTransform(m_worldTransform);
  } else {
   m_worldTransform.set(constructionInfo.m_startWorldTransform);
  }
  m_interpolationWorldTransform.set(m_worldTransform);
  m_interpolationLinearVelocity.setZero();
  m_interpolationAngularVelocity.setZero();
  //moved to btCollisionObject
  m_friction = constructionInfo.m_friction;
  m_rollingFriction = constructionInfo.m_rollingFriction;
  m_spinningFriction = constructionInfo.m_spinningFriction;
  m_restitution = constructionInfo.m_restitution;
  setCollisionShape(constructionInfo.m_collisionShape);
  //m_debugBodyId = uniqueId++;
  setMassProps(constructionInfo.m_mass, constructionInfo.m_localInertia);
  updateInertiaTensor();
  m_rigidbodyFlags = BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY;
  m_deltaLinearVelocity.setZero();
  m_deltaAngularVelocity.setZero();
  m_invMass.set(m_linearFactor).scale(m_inverseMass);
  m_pushVelocity.setZero();
  m_turnVelocity.setZero();
 }

 public void proceedToTransform(final btTransform newTrans) {
  setCenterOfMassTransform(newTrans);
 }

 ///to keep collision detection and dynamics separate we don't store a rigidbody pointer
 ///but a rigidbody is derived from btCollisionObject, so we can safely perform an upcast
 public static btRigidBody upcast(btCollisionObject colObj) {
  if ((colObj.getInternalType() & CO_RIGID_BODY) != 0) {
   return (btRigidBody) colObj;
  }
  return null;
 }

 /// continuous collision detection needs prediction
 public void predictIntegratedTransform(float timeStep, final btTransform predictedTransform) {
  if (DEBUG_BLOCKS) {
   assert (is_good_matrix(m_worldTransform));
  }
  btTransformUtil
   .integrateTransform(m_worldTransform, m_linearVelocity, m_angularVelocity, timeStep,
    predictedTransform);
  if (DEBUG_BLOCKS) {
   assert (is_good_matrix(predictedTransform));
   assert (is_good_matrix(m_worldTransform));
  }
 }

 public void saveKinematicState(float timeStep) {
  //todo: clamp to some (user definable) safe minimum timestep, to limit maximum angular/linear velocities
  if (timeStep != (0.f)) {
   //if we use motionstate to synchronize world transforms, get the new kinematic/animated world transform
   if (getMotionState() != null) {
    getMotionState().getWorldTransform(m_worldTransform);
    if (DEBUG_BLOCKS) {
     assert (is_good_matrix(m_worldTransform));
    }
   }
   btTransformUtil.calculateVelocity(m_interpolationWorldTransform, m_worldTransform, timeStep,
    m_linearVelocity, m_angularVelocity);
   m_interpolationLinearVelocity.set(m_linearVelocity);
   m_interpolationAngularVelocity.set(m_angularVelocity);
   m_interpolationWorldTransform.set(m_worldTransform);
  }
 }

 public void applyGravity() {
  if (isStaticOrKinematicObject()) {
   return;
  }
  applyCentralForce(m_gravity);
 }

 public void setGravity(final btVector3 acceleration) {
  if (m_inverseMass != (0.0f)) {
   m_gravity.set(acceleration).scale(1.0f / m_inverseMass);
  }
  m_gravity_acceleration.set(acceleration);
 }

 public btVector3 getGravity() {
  return new btVector3(m_gravity_acceleration);
 }

 public void setDamping(float lin_damping, float ang_damping) {
  m_linearDamping = btClamped(lin_damping, 0.0f, 1.0f);
  m_angularDamping = btClamped(ang_damping, 0f, 1.0f);
 }

 public float getLinearDamping() {
  return m_linearDamping;
 }

 public float getAngularDamping() {
  return m_angularDamping;
 }

 public float getLinearSleepingThreshold() {
  return m_linearSleepingThreshold;
 }

 public float getAngularSleepingThreshold() {
  return m_angularSleepingThreshold;
 }

 public void applyDamping(float timeStep) {
  //On new damping: see discussion/issue report here: http://code.google.com/p/bullet/issues/detail?id=74
  //todo: do some performance comparisons (but other parts of the engine are probably bottleneck anyway
  m_linearVelocity.scale(btPow((1f) - m_linearDamping, timeStep));
  m_angularVelocity.scale(btPow((1f) - m_angularDamping, timeStep));
  if (m_additionalDamping) {
   //Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
   //Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics system has improved, this should become obsolete
   if ((m_angularVelocity.lengthSquared() < m_additionalAngularDampingThresholdSqr) &&
    (m_linearVelocity.lengthSquared() < m_additionalLinearDampingThresholdSqr)) {
    m_angularVelocity.scale(m_additionalDampingFactor);
    m_linearVelocity.scale(m_additionalDampingFactor);
   }
   float speed = m_linearVelocity.length();
   if (speed < m_linearDamping) {
    float dampVel = (0.005f);
    if (speed > dampVel) {
     final btVector3 dir = new btVector3(m_linearVelocity).normalize();
     m_linearVelocity.sub(dir.scale(dampVel));
    } else {
     m_linearVelocity.setZero();
    }
   }
   float angSpeed = m_angularVelocity.length();
   if (angSpeed < m_angularDamping) {
    float angDampVel = (0.005f);
    if (angSpeed > angDampVel) {
     final btVector3 dir = new btVector3(m_angularVelocity).normalize();
     m_angularVelocity.sub(dir.scale(angDampVel));
    } else {
     m_angularVelocity.setZero();
    }
   }
  }
 }

 @Override
 public btCollisionShape getCollisionShape() {
  return m_collisionShape;
 }

 public void setMassProps(float mass, final btVector3 inertia) {
  if (mass == (0.f)) {
   m_collisionFlags |= CF_STATIC_OBJECT;
   m_inverseMass = (0.f);
  } else {
   m_collisionFlags &= (~CF_STATIC_OBJECT);
   m_inverseMass = 1.0f / mass;
  }
  //Fg = m * a
  m_gravity.set(m_gravity_acceleration).scale(mass);
  m_invInertiaLocal.set(inertia.x() != (0.0f) ? (1.0f) / inertia.x() : (0.0f),
   inertia.y() != (0.0f) ? (1.0f) / inertia.y() : (0.0f),
   inertia.z() != (0.0f) ? (1.0f) / inertia.z() : (0.0f));
  m_invMass.set(m_linearFactor).scale(m_inverseMass);
 }

 public btVector3 getLinearFactor() {
  return new btVector3(m_linearFactor);
 }

 public btVector3 getLinearFactorPtr() {
  return m_linearFactor;
 }

 public void setLinearFactor(final btVector3 linearFactor) {
  m_linearFactor.set(linearFactor);
  m_invMass.set(m_linearFactor).scale(m_inverseMass);
 }

 public float getInvMass() {
  return m_inverseMass;
 }

 public btMatrix3x3 getInvInertiaTensorWorld() {
  return new btMatrix3x3(m_invInertiaTensorWorld);
 }

 public btMatrix3x3 getInvInertiaTensorWorldPtr() {
  return m_invInertiaTensorWorld;
 }
 public static final float MAX_ANGVEL = SIMD_HALF_PI;

 public void integrateVelocities(float step) {
  if (isStaticOrKinematicObject()) {
   return;
  }
  m_linearVelocity.add(new btVector3(m_totalForce).scale(m_inverseMass * step));
  m_angularVelocity.add(m_invInertiaTensorWorld.transform(new btVector3(m_totalTorque)).scale(step));
  /// clamp angular velocity. collision calculations will fail on higher angular velocities	
  float angvel = m_angularVelocity.length();
  if (angvel * step > MAX_ANGVEL) {
   m_angularVelocity.scale((MAX_ANGVEL / step) / angvel);
  }
 }

 public void setCenterOfMassTransform(final btTransform xform) {
  if (DEBUG_BLOCKS) {
   assert (is_good_matrix(m_worldTransform));
   assert (is_good_matrix(xform));
  }
  if (isKinematicObject()) {
   m_interpolationWorldTransform.set(m_worldTransform);
   assert (is_good_matrix(m_interpolationWorldTransform));
  } else {
   m_interpolationWorldTransform.set(xform);
  }
  m_interpolationLinearVelocity.set(getLinearVelocity());
  m_interpolationAngularVelocity.set(getAngularVelocity());
  m_worldTransform.set(xform);
  if (DEBUG_BLOCKS) {
   assert (is_good_matrix(m_worldTransform));
  }
  updateInertiaTensor();
 }

 public void applyCentralForce(final btVector3 force) {
  m_totalForce.add(new btVector3(force).mul(m_linearFactor));
 }

 public btVector3 getTotalForce() {
  return new btVector3(m_totalForce);
 }

 public btVector3 getTotalTorque() {
  return new btVector3(m_totalTorque);
 }

 public btVector3 getInvInertiaDiagLocal() {
  return new btVector3(m_invInertiaLocal);
 }

 public void setInvInertiaDiagLocal(final btVector3 diagInvInertia) {
  m_invInertiaLocal.set(diagInvInertia);
 }

 public void setSleepingThresholds(float linear, float angular) {
  m_linearSleepingThreshold = linear;
  m_angularSleepingThreshold = angular;
 }

 public void applyTorque(final btVector3 torque) {
  m_totalTorque.add(new btVector3(torque).mul(m_angularFactor));
 }

 public void applyForce(final btVector3 force, final btVector3 rel_pos) {
  applyCentralForce(force);
  applyTorque(new btVector3(rel_pos).cross(new btVector3(force).mul(m_linearFactor)));
 }

 public void applyCentralImpulse(final btVector3 impulse) {
  m_linearVelocity.add(new btVector3(impulse).mul(m_linearFactor).scale(m_inverseMass));
 }

 public void applyTorqueImpulse(final btVector3 torque) {
  m_angularVelocity.add(m_invInertiaTensorWorld.transform(new btVector3(torque)
   .mul(m_angularFactor)));
 }

 public void applyImpulse(final btVector3 impulse, final btVector3 rel_pos) {
  if (m_inverseMass != (0.f)) {
   applyCentralImpulse(impulse);
   /* in c++ this test evaluates m_angularFactor via   (float*)operator which is always non-zero
    */
//			if (m_angularFactor)
//			{
   applyTorqueImpulse(new btVector3(rel_pos).cross(new btVector3(impulse).mul(m_linearFactor)));
//			}
  }
 }

 public void clearForces() {
  m_totalForce.setZero();
  m_totalTorque.setZero();
 }

 public void updateInertiaTensor() {
  if (DEBUG_BLOCKS) {
   assert (is_good_matrix(m_worldTransform));
  }
  m_invInertiaTensorWorld
   .set(m_worldTransform
    .getBasis()
    .mul(m_invInertiaLocal)
    .mul(m_worldTransform
     .getBasis()
     .transpose()));
  if (DEBUG_BLOCKS) {
   assert (is_good_matrix(m_worldTransform));
  }
 }

 public btVector3 getCenterOfMassPosition() {
  return m_worldTransform.getOrigin();
 }

 public btQuaternion getOrientation() {
  final btQuaternion orn = new btQuaternion();
  orn.set(m_worldTransform.getBasis());
  return orn;
 }

 public btTransform getCenterOfMassTransform() {
  if (DEBUG_BLOCKS) {
   assert (is_good_matrix(m_worldTransform));
  }
  return new btTransform(m_worldTransform);
 }

 public btVector3 getLinearVelocity() {
  return new btVector3(m_linearVelocity);
 }

 public btVector3 getAngularVelocity() {
  return new btVector3(m_angularVelocity);
 }

 public btTransform getCenterOfMassTransformPtr() {
  if (DEBUG_BLOCKS) {
   assert (is_good_matrix(m_worldTransform));
  }
  return m_worldTransform;
 }

 public btVector3 getLinearVelocityPtr() {
  return m_linearVelocity;
 }

 public btVector3 getAngularVelocityPtr() {
  return m_angularVelocity;
 }

 public void setLinearVelocity(final btVector3 lin_vel) {
  m_linearVelocity.set(lin_vel);
 }

 public void setAngularVelocity(final btVector3 ang_vel) {
  m_angularVelocity.set(ang_vel);
 }

 public btVector3 getVelocityInLocalPoint(final btVector3 rel_pos) {
  //we also calculate lin/ang velocity for kinematic objects
  return new btVector3(m_angularVelocity).cross(rel_pos).add(m_linearVelocity);
 }

 public void translate(final btVector3 v) {
  if (DEBUG_BLOCKS) {
   assert (is_good_matrix(m_worldTransform));
  }
  m_worldTransform.setOrigin(m_worldTransform.getOrigin().add(v));
  if (DEBUG_BLOCKS) {
   assert (is_good_matrix(m_worldTransform));
  }
 }

 public void getAabb(final btVector3 aabbMin, final btVector3 aabbMax) {
  if (DEBUG_BLOCKS) {
   assert (is_good_matrix(m_worldTransform));
  }
  getCollisionShape().getAabb(m_worldTransform, aabbMin, aabbMax);
  if (DEBUG_BLOCKS) {
   assert (is_good_matrix(m_worldTransform));
  }
 }

 public float computeImpulseDenominator(final btVector3 pos, final btVector3 normal) {
  final btVector3 r0 = new btVector3(pos).sub(getCenterOfMassPosition());
  final btVector3 c0 = new btVector3(r0).cross(normal);
  final btVector3 vec = (getInvInertiaTensorWorld().transposeTransform(c0)).cross(r0);
  return m_inverseMass + normal.dot(vec);
 }

 public float computeAngularImpulseDenominator(final btVector3 axis) {
  final btVector3 vec = getInvInertiaTensorWorld().transposeTransform(new btVector3(axis));
  return axis.dot(vec);
 }

 public void updateDeactivation(float timeStep) {
  if ((getActivationState() == ISLAND_SLEEPING) || (getActivationState() == DISABLE_DEACTIVATION)) {
   return;
  }
  if ((getLinearVelocity().lengthSquared() < m_linearSleepingThreshold * m_linearSleepingThreshold) &&
   (getAngularVelocity().lengthSquared() < m_angularSleepingThreshold * m_angularSleepingThreshold)) {
   m_deactivationTime += timeStep;
  } else {
   m_deactivationTime = (0.f);
   setActivationState(0);
  }
 }

 public boolean wantsSleeping() {
  if (getActivationState() == DISABLE_DEACTIVATION) {
   return false;
  }
  //disable deactivation
  if (gDisableDeactivation || (gDeactivationTime == (0.f))) {
   return false;
  }
  if ((getActivationState() == ISLAND_SLEEPING) || (getActivationState() == WANTS_DEACTIVATION)) {
   return true;
  }
  return m_deactivationTime > gDeactivationTime;
 }

 public btBroadphaseProxy getBroadphaseProxy() {
  return m_broadphaseHandle;
 }

 public void setNewBroadphaseProxy(btBroadphaseProxy broadphaseProxy) {
  m_broadphaseHandle = broadphaseProxy;
 }

 //btMotionState allows to automatic synchronize the world transform for active objects
 public btMotionState getMotionState() {
  return m_optionalMotionState;
 }

 public void setMotionState(btMotionState motionState) {
  m_optionalMotionState = motionState;
  if (m_optionalMotionState != null) {
   if (DEBUG_BLOCKS) {
    assert (is_good_matrix(m_worldTransform));
   }
   motionState.getWorldTransform(m_worldTransform);
  }
 }
 //for experimental overriding of friction/contact solver func
 int m_contactSolverType;
 int m_frictionSolverType;

 public void setAngularFactor(final btVector3 angFac) {
  m_angularFactor.set(angFac);
 }

 public void setAngularFactor(float angFac) {
  m_angularFactor.set(angFac, angFac, angFac);
 }

 public btVector3 getAngularFactor() {
  return new btVector3(m_angularFactor);
 }

 //is this rigidbody added to a btCollisionWorld/btDynamicsWorld/btBroadphase?
 public boolean isInWorld() {
  return (getBroadphaseProxy() != null);
 }

 public void addConstraintRef(btTypedConstraint c) {
  ///disable collision with the 'other' body
  int index = findLinearSearch(m_constraintRefs, (c));
  //don't add constraints that are already referenced
  //assert(index == m_constraintRefs.size());
  if (index == m_constraintRefs.size()) {
   m_constraintRefs.add(c);
   btCollisionObject colObjA = c.getRigidBodyA();
   btCollisionObject colObjB = c.getRigidBodyB();
   if (colObjA == this) {
    colObjA.setIgnoreCollisionCheck(colObjB, true);
   } else {
    colObjB.setIgnoreCollisionCheck(colObjA, true);
   }
  }
 }

 public void removeConstraintRef(btTypedConstraint c) {
  int index = findLinearSearch(m_constraintRefs, (c));
  //don't remove constraints that are not referenced
  if (index < m_constraintRefs.size()) {
   m_constraintRefs.remove(c);
   btCollisionObject colObjA = c.getRigidBodyA();
   btCollisionObject colObjB = c.getRigidBodyB();
   if (colObjA == this) {
    colObjA.setIgnoreCollisionCheck(colObjB, false);
   } else {
    colObjB.setIgnoreCollisionCheck(colObjA, false);
   }
  }
 }

 public btTypedConstraint getConstraintRef(int index) {
  return m_constraintRefs.get(index);
 }

 public int getNumConstraintRefs() {
  return m_constraintRefs.size();
 }

 public void setFlags(int flags) {
  m_rigidbodyFlags = flags;
 }

 public int getFlags() {
  return m_rigidbodyFlags;
 }

 ///perform implicit force computation in world space
 public btVector3 computeGyroscopicImpulseImplicit_World(float step) {
  // use full newton-euler equations.  common practice to drop the wxIw term. want it for better tumbling behavior.
  // calculate using implicit euler step so it's stable.
  final btVector3 inertiaLocal = getLocalInertia();
  final btVector3 w0 = getAngularVelocity();
  if (DEBUG_BLOCKS) {
   assert (is_good_matrix(m_worldTransform));
  }
  final btMatrix3x3 I =
   m_worldTransform.getBasis().mul(inertiaLocal).mul(
   m_worldTransform.getBasis().transpose());
  // use newtons method to find implicit solution for new angular velocity (w')
  // f(w') = -(T*step + Iw) + Iw' + w' + w'xIw'*step = 0 
  // df/dw' = I + 1xIw'*step + w'xI*step
  final btVector3 w1 = new btVector3(w0);
  // one step of newton's method
  {
   final btVector3 fw = evalEulerEqn(w1, w0, new btVector3(), step, I);
   final btMatrix3x3 dfw = evalEulerEqnDeriv(w1, w0, step, I);
   final btVector3 dw = dfw.solve33(fw);
   //  btMatrix3x3 dfw_inv = dfw.inverse();
   //dw = dfw_inv*fw;
   w1.sub(dw);
  }
  return (w1.sub(w0));
 }

 ///perform implicit force computation in body space (inertial frame)
 public btVector3 computeGyroscopicImpulseImplicit_Body(float step) {
  final btVector3 idl = getLocalInertia();
  final btVector3 omega1 = getAngularVelocity();
  final btQuaternion q = getWorldTransformPtr().getRotation();
  // Convert to body coordinates
  final btVector3 omegab = quatRotate(new btQuaternion(q).conjugate(), omega1);
  final btMatrix3x3 Ib = new btMatrix3x3(
   idl.x(), 0, 0,
   0, idl.y(), 0,
   0, 0, idl.z());
  final btVector3 ibo = Ib.transform(new btVector3(omegab));
  // Residual vector
  final btVector3 f = new btVector3(omegab).cross(ibo).scale(step);
  final btMatrix3x3 skew0 = new btMatrix3x3();
  omegab.getSkewSymmetricMatrix(skew0);
  final btVector3 om = Ib.transform(new btVector3(omegab));
  final btMatrix3x3 skew1 = new btMatrix3x3();
  om.getSkewSymmetricMatrix(skew1);
  // Jacobian
  final btMatrix3x3 J = new btMatrix3x3(skew0).mul(Ib).sub(skew1).mul(step).add(Ib);
//	btMatrix3x3 Jinv = J.inverse();
//	btVector3 omega_div = Jinv*f;
  final btVector3 omega_div = J.solve33(f);
  // Single Newton-Raphson update
  omegab.sub(omega_div);//Solve33(J, f);
  // Back to world coordinates
  final btVector3 omega2 = quatRotate(q, omegab);
  final btVector3 gf = omega2.sub(omega1);
  return gf;
 }

 ///explicit version is best avoided, it gains energy
 public btVector3 computeGyroscopicForceExplicit(float maxGyroscopicForce) {
  final btVector3 inertiaLocal = getLocalInertia();
  final btMatrix3x3 inertiaTensorWorld = getWorldTransformPtr().getBasis().mul(inertiaLocal).mul(
   getWorldTransformPtr().getBasis().transpose());
  final btVector3 tmp = inertiaTensorWorld.transform(getAngularVelocity());
  final btVector3 gf = getAngularVelocity().cross(tmp);
  float l2 = gf.lengthSquared();
  if (l2 > maxGyroscopicForce * maxGyroscopicForce) {
   gf.scale((1.f) / btSqrt(l2) * maxGyroscopicForce);
  }
  return gf;
 }

 public btVector3 getLocalInertia() {
  final btVector3 inertiaLocal = new btVector3();
  final btVector3 inertia = m_invInertiaLocal;
  inertiaLocal.set(inertia.x() != (0.0f) ? (1.0f) / inertia.x() : (0.0f),
   inertia.y() != (0.0f) ? (1.0f) / inertia.y() : (0.0f),
   inertia.z() != (0.0f) ? (1.0f) / inertia.z() : (0.0f));
  return inertiaLocal;
 }

 public static btVector3 evalEulerEqn(final btVector3 w1, final btVector3 w0, final btVector3 T,
  float dt,
  final btMatrix3x3 I) {
  //btVector3 w2 = I*w1 + w1.cross(I*w1)*dt - (T*dt + I*w0);
  final btVector3 a = I.transform(new btVector3(w1));
  final btVector3 b = new btVector3(w1).cross(a).scale(dt);
  final btVector3 c = new btVector3(T).scale(dt).add(I.transform(new btVector3(w0)));
  return a.add(b).sub(c);
 }

 public static btMatrix3x3 evalEulerEqnDeriv(final btVector3 w1, final btVector3 w0, float dt,
  final btMatrix3x3 I) {
  final btMatrix3x3 w1x = new btMatrix3x3();
  final btMatrix3x3 Iw1x = new btMatrix3x3();
  final btVector3 Iwi = (I.transform(new btVector3(w1)));
  w1.getSkewSymmetricMatrix(w1x);
  Iwi.getSkewSymmetricMatrix(Iw1x);
  final btMatrix3x3 dfw1 = w1x.mul(I).sub(Iw1x).mul(dt).add(I);
  return dfw1;
 }
};
