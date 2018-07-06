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
package Bullet.Dynamics.ConstraintSolver;

import Bullet.Dynamics.CollisionObjects.btRigidBody;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btTransformUtil;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btSolverBody implements Serializable {

 private static final long serialVersionUID = 1L;
 public final btTransform m_worldTransform = new btTransform();
 public final btVector3 m_deltaLinearVelocity = new btVector3();
 public final btVector3 m_deltaAngularVelocity = new btVector3();
 public final btVector3 m_angularFactor = new btVector3();
 public final btVector3 m_linearFactor = new btVector3();
 public final btVector3 m_invMass = new btVector3();
 public final btVector3 m_pushVelocity = new btVector3();
 public final btVector3 m_turnVelocity = new btVector3();
 public final btVector3 m_linearVelocity = new btVector3();
 public final btVector3 m_angularVelocity = new btVector3();
 public final btVector3 m_externalForceImpulse = new btVector3();
 public final btVector3 m_externalTorqueImpulse = new btVector3();
 public btRigidBody m_originalBody;

 public void setWorldTransform(final btTransform worldTransform) {
  m_worldTransform.set(worldTransform);
 }

 public btTransform getWorldTransform() {
  return new btTransform(m_worldTransform);
 }

 public void getVelocityInLocalPointNoDelta(final btVector3 rel_pos,
  final btVector3 velocity) {
//assert(m_externalForceImpulse.lengthSquared()>0);
  if (m_originalBody != null) {
   velocity
    .set(m_angularVelocity)
    .add(m_externalTorqueImpulse)
    .cross(rel_pos)
    .add(m_linearVelocity)
    .add(m_externalForceImpulse);
  } else {
   velocity.setZero();
  }
 }

 public void getVelocityInLocalPointObsolete(final btVector3 rel_pos,
  final btVector3 velocity) {
  if (m_originalBody != null) {
   velocity.set(m_angularVelocity)
    .add(m_deltaAngularVelocity)
    .cross(rel_pos)
    .add(m_linearVelocity)
    .add(m_deltaLinearVelocity);
  } else {
   velocity.setZero();
  }
 }

 public void getAngularVelocity(final btVector3 angVel) {
  if (m_originalBody != null) {
   angVel
    .set(m_angularVelocity)
    .add(m_deltaAngularVelocity);
  } else {
   angVel.setZero();
  }
 }

 //Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position
 public void applyImpulse(final btVector3 linearComponent,
  final btVector3 angularComponent,
  float impulseMagnitude) {
  if (m_originalBody != null) {
   m_deltaLinearVelocity
    .add(new btVector3(linearComponent)
     .scale(impulseMagnitude)
     .mul(m_linearFactor));
   m_deltaAngularVelocity
    .add(new btVector3(m_angularFactor)
     .scale(impulseMagnitude)
     .mul(angularComponent));
  }
 }

 void internalApplyPushImpulse(final btVector3 linearComponent,
  final btVector3 angularComponent,
  float impulseMagnitude) {
  if (m_originalBody != null) {
   m_pushVelocity
    .add(new btVector3(linearComponent)
     .scale(impulseMagnitude)
     .mul(m_linearFactor));
   m_turnVelocity
    .add(new btVector3(m_angularFactor)
     .scale(impulseMagnitude)
     .mul(angularComponent));
  }
 }

 public btVector3 getDeltaLinearVelocity() {
  return new btVector3(m_deltaLinearVelocity);
 }

 public btVector3 getDeltaAngularVelocity() {
  return new btVector3(m_deltaAngularVelocity);
 }

 public btVector3 getPushVelocity() {
  return new btVector3(m_pushVelocity);
 }

 public btVector3 getTurnVelocity() {
  return new btVector3(m_turnVelocity);
 }

 ////////////////////////////////////////////////
 ///some internal methods, don't use them
 btVector3 internalGetDeltaLinearVelocity() {
  return m_deltaLinearVelocity;
 }

 btVector3 internalGetDeltaAngularVelocity() {
  return m_deltaAngularVelocity;
 }

 btVector3 internalGetAngularFactor() {
  return m_angularFactor;
 }

 btVector3 internalGetInvMass() {
  return m_invMass;
 }

 void internalSetInvMass(final btVector3 invMass) {
  m_invMass.set(invMass);
 }

 btVector3 internalGetPushVelocity() {
  return m_pushVelocity;
 }

 btVector3 internalGetTurnVelocity() {
  return m_turnVelocity;
 }

 void internalGetVelocityInLocalPointObsolete(final btVector3 rel_pos,
  final btVector3 velocity) {
  velocity
   .set(m_angularVelocity)
   .add(m_deltaAngularVelocity)
   .cross(rel_pos)
   .add(m_linearVelocity)
   .add(m_deltaLinearVelocity);
 }

 void internalGetAngularVelocity(final btVector3 angVel) {
  angVel.set(m_angularVelocity).add(m_deltaAngularVelocity);
 }

 //Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position
 void internalApplyImpulse(final btVector3 linearComponent,
  final btVector3 angularComponent,
  float impulseMagnitude) {
  if (m_originalBody != null) {
   m_deltaLinearVelocity
    .add(new btVector3(linearComponent)
     .scale(impulseMagnitude)
     .mul(m_linearFactor));
   m_deltaAngularVelocity
    .add(new btVector3(m_angularFactor)
     .scale(impulseMagnitude)
     .mul(angularComponent));
  }
 }

 void writebackVelocity() {
  if (m_originalBody != null) {
   m_linearVelocity.add(m_deltaLinearVelocity);
   m_angularVelocity.add(m_deltaAngularVelocity);
  }
 }

 public void writebackVelocityAndTransform(float timeStep,
  float splitImpulseTurnErp) {
  if (m_originalBody != null) {
   m_linearVelocity.add(m_deltaLinearVelocity);
   m_angularVelocity.add(m_deltaAngularVelocity);
   //correct the position/orientation based on push/turn recovery
   if (m_pushVelocity.x != 0.f || m_pushVelocity.y != 0 || m_pushVelocity.z != 0
    || m_turnVelocity.x != 0.f || m_turnVelocity.y != 0 || m_turnVelocity.z != 0) {
    final btTransform newTransform = new btTransform();
    btTransformUtil.integrateTransform(m_worldTransform, m_pushVelocity,
     new btVector3(
      m_turnVelocity).scale(splitImpulseTurnErp), timeStep, newTransform);
    m_worldTransform.set(newTransform);
   }
  }
 }

};
