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
package Bullet.Dynamics.Constraint;

import Bullet.Collision.btCollisionObject;
import Bullet.Collision.btPersistentManifold;
import Bullet.Dynamics.CollisionObjects.btRigidBody;
import Bullet.Dynamics.btContactSolverInfo;
import Bullet.Dynamics.btJacobianEntry;
import static Bullet.LinearMath.btScalar.btFabs;
import Bullet.LinearMath.btVector3;

/**
 * btContactConstraint can be automatically created to solve contact constraints
 * using the unified btTypedConstraint interface
 *
 * @author Gregery Barton
 */
public class btContactConstraint extends btTypedConstraint {

 protected btPersistentManifold m_contactManifold;

 public btContactConstraint(btPersistentManifold contactManifold,
  btRigidBody rbA, btRigidBody rbB) {
  super(CONTACT_CONSTRAINT_TYPE, rbA, rbB);
  m_contactManifold = contactManifold;
 }

 public void setContactManifold(btPersistentManifold contactManifold) {
  m_contactManifold = contactManifold;
 }

 public btPersistentManifold getContactManifold() {
  return m_contactManifold;
 }

 @Override
 public void getInfo1(btConstraintInfo1 info) {
 }

 @Override
 public void getInfo2(btConstraintInfo2 info) {
 }

 ///obsolete methods
 @Override
 public void buildJacobian() {
 }

///very basic collision resolution without friction
 public static float resolveSingleCollision(btRigidBody body1,
  btCollisionObject colObj2,
  final btVector3 contactPositionWorld, final btVector3 contactNormalOnB,
  btContactSolverInfo solverInfo, float distance) {
  btRigidBody body2 = btRigidBody.upcast(colObj2);
  final btVector3 normal = contactNormalOnB;
  final btVector3 rel_pos1 = new btVector3(contactPositionWorld).sub(body1
   .getWorldTransformPtr()
   .getOrigin());
  final btVector3 rel_pos2 = new btVector3(contactPositionWorld).sub(colObj2
   .getWorldTransformPtr()
   .getOrigin());
  final btVector3 vel1 = body1.getVelocityInLocalPoint(rel_pos1);
  final btVector3 vel2 = body2 != null ? body2.getVelocityInLocalPoint(rel_pos2) : new btVector3();
  final btVector3 vel = new btVector3(vel1).sub(vel2);
  float rel_vel;
  rel_vel = normal.dot(vel);
  float combinedRestitution = 0.f;
  float restitution = combinedRestitution * -rel_vel;
  float positionalError = solverInfo.m_erp * -distance / solverInfo.m_timeStep;
  float velocityError = -(1.0f + restitution) * rel_vel;// * damping;
  float denom0 = body1.computeImpulseDenominator(contactPositionWorld, normal);
  float denom1 = body2 != null ? body2.computeImpulseDenominator(
   contactPositionWorld, normal) : 0.f;
  float relaxation = 1.f;
  float jacDiagABInv = relaxation / (denom0 + denom1);
  float penetrationImpulse = positionalError * jacDiagABInv;
  float velocityImpulse = velocityError * jacDiagABInv;
  float normalImpulse = penetrationImpulse + velocityImpulse;
  normalImpulse = 0.f > normalImpulse ? 0.f : normalImpulse;
  body1.applyImpulse(new btVector3(normal).scale(normalImpulse), rel_pos1);
  if (body2 != null) {
   body2.applyImpulse(new btVector3(normal).negate().scale(normalImpulse),
    rel_pos2);
  }
  return normalImpulse;
 }

///resolveSingleBilateral is an obsolete methods used for vehicle friction between two dynamic objects
 public static void resolveSingleBilateral(btRigidBody body1,
  final btVector3 pos1,
  btRigidBody body2, final btVector3 pos2,
  float distance, final btVector3 normal, float[] impulse, float timeStep) {
  float normalLenSqr = normal.lengthSquared();
  assert (btFabs(normalLenSqr) < (1.1f));
  if (normalLenSqr > (1.1f)) {
   impulse[0] = (0.f);
   return;
  }
  final btVector3 rel_pos1 = new btVector3(pos1).sub(body1
   .getCenterOfMassPosition());
  final btVector3 rel_pos2 = new btVector3(pos2).sub(body2
   .getCenterOfMassPosition());
  //this jacobian entry could be re-used for all iterations
  final btVector3 vel1 = body1.getVelocityInLocalPoint(rel_pos1);
  final btVector3 vel2 = body2.getVelocityInLocalPoint(rel_pos2);
  final btVector3 vel = new btVector3(vel1).sub(vel2);
  btJacobianEntry jac = new btJacobianEntry(body1.getCenterOfMassTransform()
   .getBasis().transpose(),
   body2.getCenterOfMassTransform().getBasis().transpose(),
   rel_pos1, rel_pos2, normal, body1.getInvInertiaDiagLocal(), body1
   .getInvMass(),
   body2.getInvInertiaDiagLocal(), body2.getInvMass());
  float jacDiagAB = jac.getDiagonal();
  float jacDiagABInv = (1.f) / jacDiagAB;
  float rel_vel;
  // result of getRelativeVelocity not used
//	  rel_vel = jac.getRelativeVelocity(
//		body1.getLinearVelocity(),
//		body1.getCenterOfMassTransform().transposeTransform3x3(body1.getAngularVelocity()),
//		body2.getLinearVelocity(),
//		body2.getCenterOfMassTransform().transposeTransform3x3(body2.getAngularVelocity())); 
  rel_vel = normal.dot(vel);
  //todo: move this into proper structure
  float contactDamping = (0.2f);
  float velocityImpulse = -contactDamping * rel_vel * jacDiagABInv;
  impulse[0] = velocityImpulse;
 }

 @Override
 public void setParam(int num, float value, int axis) {
  throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
 }

 @Override
 public float getParam(int num, int axis) {
  throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
 }

}
