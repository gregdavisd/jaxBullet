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

import Bullet.Collision.btCollisionObject;
import Bullet.Collision.Broadphase.btDispatcher;
import Bullet.Collision.btIDebugDraw;
import Bullet.Collision.btManifoldPoint;
import static Bullet.Collision.btManifoldPoint.BT_CONTACT_FLAG_CONTACT_STIFFNESS_DAMPING;
import static Bullet.Collision.btManifoldPoint.BT_CONTACT_FLAG_HAS_CONTACT_CFM;
import static Bullet.Collision.btManifoldPoint.BT_CONTACT_FLAG_HAS_CONTACT_ERP;
import static Bullet.Collision.btManifoldPoint.BT_CONTACT_FLAG_LATERAL_FRICTION_INITIALIZED;
import Bullet.Collision.btPersistentManifold;
import Bullet.Dynamics.Constraint.btConstraintInfo2;
import Bullet.Dynamics.Constraint.btSolverBody;
import Bullet.Dynamics.Constraint.btSolverConstraint;
import static Bullet.Dynamics.Constraint.btConstraintSolverType.BT_SEQUENTIAL_IMPULSE_SOLVER;
import static Bullet.Dynamics.btRigidBodyFlags.BT_ENABLE_GYROSCOPIC_FORCE_EXPLICIT;
import static Bullet.Dynamics.btRigidBodyFlags.BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY;
import static Bullet.Dynamics.btRigidBodyFlags.BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD;
import Bullet.Dynamics.Constraint.btSolverConstraint.CFMPointer;
import Bullet.Dynamics.Constraint.btSolverConstraint.LowerLimitPointer;
import Bullet.Dynamics.Constraint.btSolverConstraint.RHSPointer;
import Bullet.Dynamics.Constraint.btSolverConstraint.UpperLimitPointer;
import Bullet.Dynamics.Constraint.btTypedConstraint;
import static Bullet.Dynamics.Constraint.btSolverMode.SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION;
import static Bullet.Dynamics.Constraint.btSolverMode.SOLVER_ENABLE_FRICTION_DIRECTION_CACHING;
import static Bullet.Dynamics.Constraint.btSolverMode.SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS;
import static Bullet.Dynamics.Constraint.btSolverMode.SOLVER_SIMD;
import static Bullet.Dynamics.Constraint.btSolverMode.SOLVER_USE_2_FRICTION_DIRECTIONS;
import static Bullet.Dynamics.Constraint.btSolverMode.SOLVER_USE_WARMSTARTING;
import Bullet.Dynamics.Constraint.btTypedConstraint.btConstraintInfo1;
import Bullet.Dynamics.btContactSolverInfo;
import Bullet.Dynamics.btJointFeedback;
import Bullet.Dynamics.btRigidBody;
import Bullet.Dynamics.btSingleConstraintRowSolver;
import static Bullet.LinearMath.btQuickprof.BT_PROFILE;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.SIMD_INFINITY;
import static Bullet.LinearMath.btScalar.btFabs;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.btPlaneSpace1;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import javax.vecmath.FloatSmartPointer;
import static javax.vecmath.VecMath.DEBUG_BLOCKS;
import static Bullet.Dynamics.Constraint.btSolverMode.SOLVER_RANDOMIZE_ORDER;
import java.util.concurrent.ThreadLocalRandom;

/**
 *
 * @author Gregery Barton
 */
public class btSequentialImpulseConstraintSolver extends btConstraintSolver implements Serializable {

 static int gNumSplitImpulseRecoveries = 0;
 private static final long serialVersionUID = 1L;
 long m_btSeed2 = ThreadLocalRandom.current().nextLong();

 static float gResolveSingleConstraintRowGeneric_scalar_reference(btSolverBody body1,
  btSolverBody body2,
  btSolverConstraint c) {
  float deltaImpulse = c.m_rhs - (c.m_appliedImpulse) * c.m_cfm;
  float deltaVel1Dotn = c.m_contactNormal1.dot(body1.internalGetDeltaLinearVelocity()) +
    c.m_relpos1CrossNormal.dot(body1.
    internalGetDeltaAngularVelocity());
  float deltaVel2Dotn = c.m_contactNormal2.dot(body2.internalGetDeltaLinearVelocity()) +
    c.m_relpos2CrossNormal.dot(body2.
    internalGetDeltaAngularVelocity());
  //	const float delta_rel_vel	=	deltaVel1Dotn-deltaVel2Dotn;
  deltaImpulse -= deltaVel1Dotn * c.m_jacDiagABInv;
  deltaImpulse -= deltaVel2Dotn * c.m_jacDiagABInv;
  float sum = (c.m_appliedImpulse) + deltaImpulse;
  if (sum < c.m_lowerLimit) {
   deltaImpulse = c.m_lowerLimit - c.m_appliedImpulse;
   c.m_appliedImpulse = c.m_lowerLimit;
  } else if (sum > c.m_upperLimit) {
   deltaImpulse = c.m_upperLimit - c.m_appliedImpulse;
   c.m_appliedImpulse = c.m_upperLimit;
  } else {
   c.m_appliedImpulse = sum;
  }
  body1.internalApplyImpulse(new btVector3(c.m_contactNormal1).mul(body1.internalGetInvMass()),
   c.m_angularComponentA,
   deltaImpulse);
  body2.internalApplyImpulse(new btVector3(c.m_contactNormal2).mul(body2.internalGetInvMass()),
   c.m_angularComponentB,
   deltaImpulse);
  return deltaImpulse;
 }

 static float gResolveSingleConstraintRowLowerLimit_scalar_reference(btSolverBody body1,
  btSolverBody body2,
  btSolverConstraint c) {
  float deltaImpulse = c.m_rhs - (c.m_appliedImpulse) * c.m_cfm;
  float deltaVel1Dotn = c.m_contactNormal1.dot(body1.internalGetDeltaLinearVelocity()) +
    c.m_relpos1CrossNormal.dot(body1.
    internalGetDeltaAngularVelocity());
  float deltaVel2Dotn = c.m_contactNormal2.dot(body2.internalGetDeltaLinearVelocity()) +
    c.m_relpos2CrossNormal.dot(body2.
    internalGetDeltaAngularVelocity());
  deltaImpulse -= deltaVel1Dotn * c.m_jacDiagABInv;
  deltaImpulse -= deltaVel2Dotn * c.m_jacDiagABInv;
  float sum = (c.m_appliedImpulse) + deltaImpulse;
  if (sum < c.m_lowerLimit) {
   deltaImpulse = c.m_lowerLimit - c.m_appliedImpulse;
   c.m_appliedImpulse = c.m_lowerLimit;
  } else {
   c.m_appliedImpulse = sum;
  }
  body1.internalApplyImpulse(new btVector3(c.m_contactNormal1).mul(body1.internalGetInvMass()),
   c.m_angularComponentA,
   deltaImpulse);
  body2.internalApplyImpulse(new btVector3(c.m_contactNormal2).mul(body2.internalGetInvMass()),
   c.m_angularComponentB,
   deltaImpulse);
  return deltaImpulse;
 }

 static void applyAnisotropicFriction(btCollisionObject colObj, final btVector3 frictionDirection,
  int frictionMode) {
  if (colObj != null && colObj.hasAnisotropicFriction(frictionMode)) {
   // transform to local coordinates
   final btVector3 loc_lateral = colObj.getWorldTransformPtr().transposeTransform3x3(new btVector3(
    frictionDirection));
   final btVector3 friction_scaling = colObj.getAnisotropicFriction();
   //apply anisotropic friction
   loc_lateral.mul(friction_scaling);
   // ... and transform it back to global coordinates
   colObj.getWorldTransformPtr().transform3x3(frictionDirection.set(loc_lateral));
  }
 }
 final ArrayList<btSolverBody> solver = new ArrayList<>(0);
 final ArrayList<btSolverConstraint> contact = new ArrayList<>(0);
 final ArrayList<btSolverConstraint> non_contact = new ArrayList<>(0);
 final ArrayList<btSolverConstraint> contact_friction = new ArrayList<>(0);
 final ArrayList<btSolverConstraint> rolling_friction = new ArrayList<>(0);
 btTypedConstraint.btConstraintInfo1[] constraint_info1 = new btTypedConstraint.btConstraintInfo1[0];
 int m_maxOverrideNumSolverIterations;
 btSolverBody m_fixedBody;
 // When running solvers on multiple threads, a race condition exists for Kinematic objects that
 // participate in more than one solver.
 // The getOrInitSolverBody() function writes the companionId of each body (storing the index of the solver body
 // for the current solver). For normal dynamic bodies it isn't an issue because they can only be in one island
 // (and therefore one thread) at a time. But kinematic bodies can be in multiple islands at once.
 // To avoid this race condition, this solver does not write the companionId, instead it stores the solver body
 // index in this solver-local table, indexed by the uniqueId of the body.
 //btAlignedObjectArray<int>	m_kinematicBodyUniqueIdToSolverBodyTable;  // only used for multithreading
 btSingleConstraintRowSolver m_resolveSingleConstraintRowGeneric;
 btSingleConstraintRowSolver m_resolveSingleConstraintRowLowerLimit;
 float m_leastSquaresResidual;

 public btSequentialImpulseConstraintSolver() {
  m_resolveSingleConstraintRowGeneric = getScalarConstraintRowSolverGeneric();
  m_resolveSingleConstraintRowLowerLimit = getScalarConstraintRowSolverLowerLimit();
 }

 void setupFrictionConstraint(btSolverConstraint solverConstraint, final btVector3 normalAxis,
  btSolverBody solverBodyA,
  btSolverBody solverBodyB,
  btManifoldPoint cp, final btVector3 rel_pos1, final btVector3 rel_pos2,
  btCollisionObject colObj0, btCollisionObject colObj1, float relaxation) {
  setupFrictionConstraint(solverConstraint, normalAxis, solverBodyA, solverBodyB, cp, rel_pos1,
   rel_pos2, colObj0, colObj1,
   relaxation, 0, 0);
 }

 void setupFrictionConstraint(btSolverConstraint solverConstraint, final btVector3 normalAxis,
  btSolverBody solverBodyA,
  btSolverBody solverBodyB,
  btManifoldPoint cp, final btVector3 rel_pos1, final btVector3 rel_pos2,
  btCollisionObject colObj0, btCollisionObject colObj1, float relaxation,
  float desiredVelocity) {
  setupFrictionConstraint(solverConstraint, normalAxis, solverBodyA, solverBodyB, cp, rel_pos1,
   rel_pos2, colObj0, colObj1,
   relaxation, desiredVelocity, 0);
 }

 void setupFrictionConstraint(btSolverConstraint solverConstraint, final btVector3 normalAxis,
  btSolverBody solverBodyA,
  btSolverBody solverBodyB,
  btManifoldPoint cp, final btVector3 rel_pos1, final btVector3 rel_pos2,
  btCollisionObject colObj0, btCollisionObject colObj1, float relaxation,
  float desiredVelocity, float cfmSlip) {
  btRigidBody body0 = solverBodyA.m_originalBody;
  btRigidBody body1 = solverBodyB.m_originalBody;
  solverConstraint.m_solverBodyA = solverBodyA;
  solverConstraint.m_solverBodyB = solverBodyB;
  solverConstraint.m_friction = cp.m_combinedFriction;
  solverConstraint.m_originalContactPoint = null;
  solverConstraint.m_appliedImpulse = 0.f;
  solverConstraint.m_appliedPushImpulse = 0.f;
  if (body0 != null) {
   solverConstraint.m_contactNormal1.set(normalAxis);
   final btVector3 ftorqueAxis1 = new btVector3(rel_pos1).cross(solverConstraint.m_contactNormal1);
   solverConstraint.m_relpos1CrossNormal.set(ftorqueAxis1);
   solverConstraint.m_angularComponentA
    .set(
     body0.getInvInertiaTensorWorld()
     .transform(ftorqueAxis1)
     .mul(body0.getAngularFactor()));
  } else {
   solverConstraint.m_contactNormal1.setZero();
   solverConstraint.m_relpos1CrossNormal.setZero();
   solverConstraint.m_angularComponentA.setZero();
  }
  if (body1 != null) {
   solverConstraint.m_contactNormal2.set(normalAxis).negate();
   final btVector3 ftorqueAxis1 = new btVector3(rel_pos2).cross(solverConstraint.m_contactNormal2);
   solverConstraint.m_relpos2CrossNormal.set(ftorqueAxis1);
   solverConstraint.m_angularComponentB
    .set(body1.getInvInertiaTensorWorld()
     .transform(ftorqueAxis1)
     .mul(body1.getAngularFactor()));
  } else {
   solverConstraint.m_contactNormal2.setZero();
   solverConstraint.m_relpos2CrossNormal.setZero();
   solverConstraint.m_angularComponentB.setZero();
  }
  {
   final btVector3 vec = new btVector3();
   float denom0 = 0.f;
   float denom1 = 0.f;
   if (body0 != null) {
    vec.set(solverConstraint.m_angularComponentA).cross(rel_pos1);
    denom0 = body0.getInvMass() + normalAxis.dot(vec);
   }
   if (body1 != null) {
    vec.set(solverConstraint.m_angularComponentB).negate().cross(rel_pos2);
    denom1 = body1.getInvMass() + normalAxis.dot(vec);
   }
   float denom = relaxation / (denom0 + denom1);
   solverConstraint.m_jacDiagABInv = denom;
  }
  {
   float rel_vel;
   float vel1Dotn =
    solverConstraint.m_contactNormal1
    .dot(body0 != null ? new btVector3(solverBodyA.m_linearVelocity)
     .add(solverBodyA.m_externalForceImpulse) : new btVector3()) +
     solverConstraint.m_relpos1CrossNormal
    .dot(body0 != null ? solverBodyA.m_angularVelocity : new btVector3());
   float vel2Dotn = solverConstraint.m_contactNormal2
    .dot(body1 != null ? new btVector3(solverBodyB.m_linearVelocity)
     .add(solverBodyB.m_externalForceImpulse) : new btVector3()) +
     solverConstraint.m_relpos2CrossNormal
    .dot(body1 != null ? solverBodyB.m_angularVelocity : new btVector3());
   rel_vel = vel1Dotn + vel2Dotn;
//		float positionalError = 0.f;
   float velocityError = desiredVelocity - rel_vel;
   float velocityImpulse = velocityError * solverConstraint.m_jacDiagABInv;
   solverConstraint.m_rhs = velocityImpulse;
   solverConstraint.m_rhsPenetration = 0.f;
   solverConstraint.m_cfm = cfmSlip;
   solverConstraint.m_lowerLimit = -solverConstraint.m_friction;
   solverConstraint.m_upperLimit = solverConstraint.m_friction;
  }
 }

 void setupTorsionalFrictionConstraint(btSolverConstraint solverConstraint,
  final btVector3 normalAxis,
  btSolverBody solverBodyA,
  btSolverBody solverBodyB,
  btManifoldPoint cp, float combinedTorsionalFriction, final btVector3 rel_pos1,
  final btVector3 rel_pos2,
  btCollisionObject colObj0, btCollisionObject colObj1, float relaxation) {
  setupTorsionalFrictionConstraint(solverConstraint, normalAxis, solverBodyA, solverBodyB, cp,
   combinedTorsionalFriction,
   rel_pos1, rel_pos2, colObj0, colObj1, relaxation, 0f, 0f);
 }

 void setupTorsionalFrictionConstraint(btSolverConstraint solverConstraint,
  final btVector3 normalAxis,
  btSolverBody solverBodyA,
  btSolverBody solverBodyB,
  btManifoldPoint cp, float combinedTorsionalFriction, final btVector3 rel_pos1,
  final btVector3 rel_pos2,
  btCollisionObject colObj0, btCollisionObject colObj1, float relaxation, float desiredVelocity) {
  setupTorsionalFrictionConstraint(solverConstraint, normalAxis, solverBodyA, solverBodyB, cp,
   combinedTorsionalFriction,
   rel_pos1, rel_pos2, colObj0, colObj1, relaxation, desiredVelocity, 0f);
 }

 void setupTorsionalFrictionConstraint(btSolverConstraint solverConstraint,
  final btVector3 normalAxis1,
  btSolverBody solverBodyA,
  btSolverBody solverBodyB,
  btManifoldPoint cp, float combinedTorsionalFriction, final btVector3 rel_pos1,
  final btVector3 rel_pos2,
  btCollisionObject colObj0, btCollisionObject colObj1, float relaxation,
  float desiredVelocity, float cfmSlip) {
  solverConstraint.m_contactNormal1.set(normalAxis1);
  solverConstraint.m_contactNormal2.set(normalAxis1).negate();
  btRigidBody body0 = solverBodyA.m_originalBody;
  btRigidBody body1 = solverBodyB.m_originalBody;
  solverConstraint.m_solverBodyA = solverBodyA;
  solverConstraint.m_solverBodyB = solverBodyB;
  solverConstraint.m_friction = combinedTorsionalFriction;
  solverConstraint.m_originalContactPoint = 0;
  solverConstraint.m_appliedImpulse = 0.f;
  solverConstraint.m_appliedPushImpulse = 0.f;
  {
   final btVector3 ftorqueAxis1 = new btVector3(normalAxis1).negate();
   solverConstraint.m_relpos1CrossNormal.set(ftorqueAxis1);
   solverConstraint.m_angularComponentA.set(
    body0 != null ? body0.getInvInertiaTensorWorldPtr()
     .transform(ftorqueAxis1)
     .mul(body0.getAngularFactor()) : new btVector3());
  }
  {
   final btVector3 ftorqueAxis1 = new btVector3(normalAxis1);
   solverConstraint.m_relpos2CrossNormal.set(ftorqueAxis1);
   solverConstraint.m_angularComponentB.set(
    body1 != null ? body1.getInvInertiaTensorWorldPtr().transform(ftorqueAxis1).mul(body1
     .getAngularFactor()) :
      new btVector3());
  }
  {
   final btVector3 iMJaA =
    body0 != null ? body0.getInvInertiaTensorWorldPtr()
     .transform(new btVector3(solverConstraint.m_relpos1CrossNormal)) : new btVector3();
   final btVector3 iMJaB =
    body1 != null ? body1.getInvInertiaTensorWorldPtr().transform(new btVector3(
      solverConstraint.m_relpos2CrossNormal)) :
      new btVector3();
   float sum = 0;
   sum += iMJaA.dot(solverConstraint.m_relpos1CrossNormal);
   sum += iMJaB.dot(solverConstraint.m_relpos2CrossNormal);
   solverConstraint.m_jacDiagABInv = (1.f) / sum;
  }
  {
   float rel_vel;
   float vel1Dotn =
    solverConstraint.m_contactNormal1
    .dot(body0 != null ? new btVector3(solverBodyA.m_linearVelocity)
     .add(solverBodyA.m_externalForceImpulse) : new btVector3()) +
     solverConstraint.m_relpos1CrossNormal
    .dot(body0 != null ? solverBodyA.m_angularVelocity : new btVector3());
   float vel2Dotn =
    solverConstraint.m_contactNormal2
    .dot(body1 != null ? new btVector3(solverBodyB.m_linearVelocity)
     .add(solverBodyB.m_externalForceImpulse) : new btVector3()) +
     solverConstraint.m_relpos2CrossNormal
    .dot(body1 != null ? solverBodyB.m_angularVelocity : new btVector3());
   rel_vel = vel1Dotn + vel2Dotn;
//		float positionalError = 0.f;
   float velocityError = desiredVelocity - rel_vel;
   float velocityImpulse = velocityError * (solverConstraint.m_jacDiagABInv);
   solverConstraint.m_rhs = velocityImpulse;
   solverConstraint.m_cfm = cfmSlip;
   solverConstraint.m_lowerLimit = -solverConstraint.m_friction;
   solverConstraint.m_upperLimit = solverConstraint.m_friction;
  }
 }

 btSolverConstraint addFrictionConstraint(final btVector3 normalAxis, btSolverBody solverBodyA,
  btSolverBody solverBodyB,
  btSolverConstraint solverFriction, btManifoldPoint cp, final btVector3 rel_pos1,
  final btVector3 rel_pos2,
  btCollisionObject colObj0,
  btCollisionObject colObj1, float relaxation) {
  return addFrictionConstraint(normalAxis, solverBodyA, solverBodyB, solverFriction, cp, rel_pos1,
   rel_pos2, colObj0,
   colObj1,
   relaxation, 0f, 0f);
 }

 btSolverConstraint addFrictionConstraint(final btVector3 normalAxis, btSolverBody solverBodyA,
  btSolverBody solverBodyB,
  btSolverConstraint solverFriction, btManifoldPoint cp, final btVector3 rel_pos1,
  final btVector3 rel_pos2,
  btCollisionObject colObj0,
  btCollisionObject colObj1, float relaxation, float desiredVelocity) {
  return addFrictionConstraint(normalAxis, solverBodyA, solverBodyB, solverFriction, cp, rel_pos1,
   rel_pos2, colObj0,
   colObj1,
   relaxation, desiredVelocity, 0);
 }

 btSolverConstraint addFrictionConstraint(final btVector3 normalAxis, btSolverBody solverBodyA,
  btSolverBody solverBodyB,
  btSolverConstraint solverFriction, btManifoldPoint cp, final btVector3 rel_pos1,
  final btVector3 rel_pos2,
  btCollisionObject colObj0,
  btCollisionObject colObj1, float relaxation, float desiredVelocity, float cfmSlip) {
  btSolverConstraint solverConstraint = new btSolverConstraint();
  solverConstraint.m_solverFriction = solverFriction;
  contact_friction.add(solverConstraint);
  setupFrictionConstraint(solverConstraint, normalAxis, solverBodyA, solverBodyB, cp, rel_pos1,
   rel_pos2,
   colObj0, colObj1, relaxation, desiredVelocity, cfmSlip);
  return solverConstraint;
 }

 btSolverConstraint addTorsionalFrictionConstraint(final btVector3 normalAxis,
  btSolverBody solverBodyA,
  btSolverBody solverBodyB,
  btSolverConstraint solverFriction, btManifoldPoint cp, float torsionalFriction,
  final btVector3 rel_pos1, final btVector3 rel_pos2,
  btCollisionObject colObj0, btCollisionObject colObj1, float relaxation) {
  return addTorsionalFrictionConstraint(normalAxis, solverBodyA, solverBodyB, solverFriction, cp,
   torsionalFriction,
   rel_pos1,
   rel_pos2, colObj0, colObj1, relaxation, 0f, 0f);
 }

 btSolverConstraint addTorsionalFrictionConstraint(final btVector3 normalAxis,
  btSolverBody solverBodyA,
  btSolverBody solverBodyB,
  btSolverConstraint solverFriction, btManifoldPoint cp, float torsionalFriction,
  final btVector3 rel_pos1, final btVector3 rel_pos2,
  btCollisionObject colObj0, btCollisionObject colObj1, float relaxation, float desiredVelocity) {
  return addTorsionalFrictionConstraint(normalAxis, solverBodyA, solverBodyB, solverFriction, cp,
   torsionalFriction,
   rel_pos1,
   rel_pos2, colObj0, colObj1, relaxation, desiredVelocity, 0f);
 }

 btSolverConstraint addTorsionalFrictionConstraint(final btVector3 normalAxis,
  btSolverBody solverBodyA,
  btSolverBody solverBodyB,
  btSolverConstraint solverFriction, btManifoldPoint cp, float torsionalFriction,
  final btVector3 rel_pos1, final btVector3 rel_pos2,
  btCollisionObject colObj0, btCollisionObject colObj1, float relaxation, float desiredVelocity,
  float cfmSlip) {
  btSolverConstraint solverConstraint = new btSolverConstraint();
  solverConstraint.m_solverFriction = solverFriction;
  rolling_friction.add(solverConstraint);
  setupTorsionalFrictionConstraint(solverConstraint, normalAxis, solverBodyA, solverBodyB, cp,
   torsionalFriction, rel_pos1,
   rel_pos2,
   colObj0, colObj1, relaxation, desiredVelocity, cfmSlip);
  return solverConstraint;
 }

 void attach_friction_to_contact_constraint(btSolverConstraint contact, btSolverConstraint friction) {
  if (contact.m_solverFriction == null) {
   contact.m_solverFriction = friction;
  } else if (contact.m_solverFriction2 == null) {
   contact.m_solverFriction2 = friction;
  } else {
   assert (false);
  }
 }

 void setupContactConstraint(btSolverConstraint solverConstraint, btSolverBody solverBodyA,
  btSolverBody solverBodyB,
  btManifoldPoint cp,
  btContactSolverInfo infoGlobal, float[] relaxation, final btVector3 rel_pos1,
  final btVector3 rel_pos2) {
  btSolverBody bodyA = solverBodyA;
  btSolverBody bodyB = solverBodyB;
  btRigidBody rb0 = bodyA.m_originalBody;
  btRigidBody rb1 = bodyB.m_originalBody;
  relaxation[0] = infoGlobal.m_sor;
  float invTimeStep = (1f) / infoGlobal.m_timeStep;
  //cfm = 1 /       ( dt * kp + kd )
  //erp = dt * kp / ( dt * kp + kd )
  float cfm = infoGlobal.m_globalCfm;
  float erp = infoGlobal.m_erp2;
  if ((cp.m_contactPointFlags & BT_CONTACT_FLAG_HAS_CONTACT_CFM) != 0 || (cp.m_contactPointFlags &
    BT_CONTACT_FLAG_HAS_CONTACT_ERP) != 0) {
   if ((cp.m_contactPointFlags & BT_CONTACT_FLAG_HAS_CONTACT_CFM) != 0) {
    cfm = cp.m_contactCFM;
   }
   if ((cp.m_contactPointFlags & BT_CONTACT_FLAG_HAS_CONTACT_ERP) != 0) {
    erp = cp.m_contactERP;
   }
  } else if ((cp.m_contactPointFlags & BT_CONTACT_FLAG_CONTACT_STIFFNESS_DAMPING) != 0) {
   float denom = (infoGlobal.m_timeStep * cp.m_combinedContactStiffness1 +
     cp.m_combinedContactDamping1);
   if (denom < SIMD_EPSILON) {
    denom = SIMD_EPSILON;
   }
   cfm = (1f) / denom;
   erp = (infoGlobal.m_timeStep * cp.m_combinedContactStiffness1) / denom;
  }
  cfm *= invTimeStep;
  final btVector3 torqueAxis0 = new btVector3(rel_pos1)
   .cross(cp.m_normalWorldOnB);
  solverConstraint.m_angularComponentA
   .set(rb0 != null ? rb0.getInvInertiaTensorWorld()
    .transform(new btVector3(torqueAxis0))
    .mul(rb0.getAngularFactor()) :
     new btVector3());
  final btVector3 torqueAxis1 = new btVector3(rel_pos2)
   .cross(cp.m_normalWorldOnB);
  solverConstraint.m_angularComponentB
   .set(rb1 != null ? rb1.getInvInertiaTensorWorld()
    .transform(new btVector3(torqueAxis1).negate())
    .mul(rb1.getAngularFactor()) :
     new btVector3());
  {
   final btVector3 vec = new btVector3();
   float denom0 = 0.f;
   float denom1 = 0.f;
   if (rb0 != null) {
    vec.set(solverConstraint.m_angularComponentA).cross(rel_pos1);
    denom0 = rb0.getInvMass() + cp.m_normalWorldOnB.dot(vec);
   }
   if (rb1 != null) {
    vec.set(solverConstraint.m_angularComponentB).negate().cross(rel_pos2);
    denom1 = rb1.getInvMass() + cp.m_normalWorldOnB.dot(vec);
   }
   float denom = relaxation[0] / (denom0 + denom1 + cfm);
   solverConstraint.m_jacDiagABInv = denom;
  }
  if (rb0 != null) {
   solverConstraint.m_contactNormal1.set(cp.m_normalWorldOnB);
   solverConstraint.m_relpos1CrossNormal.set(torqueAxis0);
  } else {
   solverConstraint.m_contactNormal1.setZero();
   solverConstraint.m_relpos1CrossNormal.setZero();
  }
  if (rb1 != null) {
   solverConstraint.m_contactNormal2.set(cp.m_normalWorldOnB).negate();
   solverConstraint.m_relpos2CrossNormal.set(torqueAxis1).negate();
  } else {
   solverConstraint.m_contactNormal2.setZero();
   solverConstraint.m_relpos2CrossNormal.setZero();
  }
  float restitution;
  float penetration = cp.getDistance() + infoGlobal.m_linearSlop;
  {
   final btVector3 vel1;
   final btVector3 vel2;
   vel1 = rb0 != null ? rb0.getVelocityInLocalPoint(rel_pos1) : new btVector3();
   vel2 = rb1 != null ? rb1.getVelocityInLocalPoint(rel_pos2) : new btVector3();
   //			btVector3 vel2 = rb1 ? rb1.getVelocityInLocalPoint(rel_pos2) : btVector3(0,0,0);
   final btVector3 vel = new btVector3(vel1).sub(vel2);
   float rel_vel = cp.m_normalWorldOnB.dot(vel);
   solverConstraint.m_friction = cp.m_combinedFriction;
   restitution = restitutionCurve(rel_vel, cp.m_combinedRestitution);
   if (restitution <= (0.f)) {
    restitution = 0.f;
   }
  }
  ///warm starting (or zero if disabled)
  if ((infoGlobal.m_solverMode & SOLVER_USE_WARMSTARTING) != 0) {
   solverConstraint.m_appliedImpulse = cp.m_appliedImpulse * infoGlobal.m_warmstartingFactor;
   if (rb0 != null) {
    bodyA.internalApplyImpulse(
     new btVector3(solverConstraint.m_contactNormal1)
     .mul(bodyA.internalGetInvMass())
     .mul(rb0.getLinearFactorPtr()),
     solverConstraint.m_angularComponentA,
     solverConstraint.m_appliedImpulse);
   }
   if (rb1 != null) {
    bodyB.internalApplyImpulse(new btVector3(solverConstraint.m_contactNormal2)
     .negate()
     .mul(bodyB.internalGetInvMass())
     .mul(rb1.getLinearFactorPtr()),
     new btVector3(solverConstraint.m_angularComponentB).negate(),
     -solverConstraint.m_appliedImpulse);
   }
  } else {
   solverConstraint.m_appliedImpulse = 0.f;
  }
  solverConstraint.m_appliedPushImpulse = 0.f;
  {
   final btVector3 externalForceImpulseA = bodyA.m_originalBody != null ?
     bodyA.m_externalForceImpulse :
     new btVector3();
   final btVector3 externalTorqueImpulseA = bodyA.m_originalBody != null ?
     bodyA.m_externalTorqueImpulse :
     new btVector3();
   final btVector3 externalForceImpulseB = bodyB.m_originalBody != null ?
     bodyB.m_externalForceImpulse :
     new btVector3();
   final btVector3 externalTorqueImpulseB = bodyB.m_originalBody != null ?
     bodyB.m_externalTorqueImpulse :
     new btVector3();
   float vel1Dotn =
    solverConstraint.m_contactNormal1
    .dot(new btVector3(bodyA.m_linearVelocity)
     .add(externalForceImpulseA)) + solverConstraint.m_relpos1CrossNormal
    .dot(new btVector3(bodyA.m_angularVelocity)
     .add(externalTorqueImpulseA));
   float vel2Dotn =
    solverConstraint.m_contactNormal2
    .dot(
     new btVector3(bodyB.m_linearVelocity)
     .add(externalForceImpulseB)) + solverConstraint.m_relpos2CrossNormal
    .dot(new btVector3(bodyB.m_angularVelocity)
     .add(externalTorqueImpulseB));
   float rel_vel = vel1Dotn + vel2Dotn;
   float positionalError;
   float velocityError = restitution - rel_vel;// * damping;
   if (penetration > 0) {
    positionalError = 0;
    velocityError -= penetration * invTimeStep;
   } else {
    positionalError = -penetration * erp * invTimeStep;
   }
   float penetrationImpulse = positionalError * solverConstraint.m_jacDiagABInv;
   float velocityImpulse = velocityError * solverConstraint.m_jacDiagABInv;
   if (!infoGlobal.m_splitImpulse || (penetration > infoGlobal.m_splitImpulsePenetrationThreshold)) {
    //combine position and velocity into rhs
    solverConstraint.m_rhs = penetrationImpulse + velocityImpulse;//-solverConstraint.m_contactNormal1.dot(bodyA.m_externalForce*bodyA.m_invMass-bodyB.m_externalForce/bodyB.m_invMass)*solverConstraint.m_jacDiagABInv;
    solverConstraint.m_rhsPenetration = 0.f;
   } else {
    //split position and velocity into rhs and m_rhsPenetration
    solverConstraint.m_rhs = velocityImpulse;
    solverConstraint.m_rhsPenetration = penetrationImpulse;
   }
   solverConstraint.m_cfm = cfm * solverConstraint.m_jacDiagABInv;
   solverConstraint.m_lowerLimit = 0;
   solverConstraint.m_upperLimit = 1e10f;
  }
 }

 void setFrictionConstraintImpulse(btSolverConstraint solverConstraint, btSolverBody solverBodyA,
  btSolverBody solverBodyB,
  btManifoldPoint cp, btContactSolverInfo infoGlobal) {
  btSolverBody bodyA = solverBodyA;
  btSolverBody bodyB = solverBodyB;
  btRigidBody rb0 = bodyA.m_originalBody;
  btRigidBody rb1 = bodyB.m_originalBody;
  {
   btSolverConstraint frictionConstraint1 = solverConstraint.m_solverFriction;
   if ((infoGlobal.m_solverMode & SOLVER_USE_WARMSTARTING) != 0) {
    frictionConstraint1.m_appliedImpulse = cp.m_appliedImpulseLateral1 *
      infoGlobal.m_warmstartingFactor;
    if (rb0 != null) {
     bodyA.internalApplyImpulse(
      new btVector3(frictionConstraint1.m_contactNormal1)
      .scale(rb0.getInvMass())
      .mul(rb0.getLinearFactorPtr()),
      frictionConstraint1.m_angularComponentA,
      frictionConstraint1.m_appliedImpulse);
    }
    if (rb1 != null) {
     bodyB.internalApplyImpulse(
      new btVector3(frictionConstraint1.m_contactNormal2)
      .negate()
      .scale(rb1.getInvMass())
      .mul(rb1.getLinearFactorPtr()),
      new btVector3(frictionConstraint1.m_angularComponentB).negate(),
      -frictionConstraint1.m_appliedImpulse);
    }
   } else {
    frictionConstraint1.m_appliedImpulse = 0.f;
   }
  }
  if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS) != 0) {
   btSolverConstraint frictionConstraint2 = solverConstraint.m_solverFriction2;
   if ((infoGlobal.m_solverMode & SOLVER_USE_WARMSTARTING) != 0) {
    frictionConstraint2.m_appliedImpulse =
      cp.m_appliedImpulseLateral2 * infoGlobal.m_warmstartingFactor;
    if (rb0 != null) {
     bodyA.internalApplyImpulse(
      new btVector3(frictionConstraint2.m_contactNormal1)
      .scale(rb0.getInvMass()),
      frictionConstraint2.m_angularComponentA,
      frictionConstraint2.m_appliedImpulse);
    }
    if (rb1 != null) {
     bodyB.internalApplyImpulse(
      new btVector3(frictionConstraint2.m_contactNormal2).negate()
      .scale(rb1.getInvMass()),
      new btVector3(frictionConstraint2.m_angularComponentB).negate(),
      -frictionConstraint2.m_appliedImpulse);
    }
   } else {
    frictionConstraint2.m_appliedImpulse = 0.f;
   }
  }
 }

 float restitutionCurve(float rel_vel, float restitution) {
  float rest = restitution * -rel_vel;
  return rest;
 }

 void convertContacts(List<btPersistentManifold> manifoldPtr, int numManifolds,
  btContactSolverInfo infoGlobal) {
  int i;
  for (btPersistentManifold manifold : manifoldPtr) {
   convertContact(manifold, infoGlobal);
  }
 }

 void convertContact(btPersistentManifold manifold, btContactSolverInfo infoGlobal) {
  btCollisionObject colObj0, colObj1;
  colObj0 = (btCollisionObject) manifold.getBody0();
  colObj1 = (btCollisionObject) manifold.getBody1();
  btSolverBody solverBodyA = getOrInitSolverBody(colObj0, infoGlobal.m_timeStep);
  btSolverBody solverBodyB = getOrInitSolverBody(colObj1, infoGlobal.m_timeStep);
  ///avoid collision response between two static objects
  if (solverBodyA == null || (solverBodyA.m_invMass.fuzzyZero() && (solverBodyB == null ||
    solverBodyB.m_invMass
   .fuzzyZero()))) {
   return;
  }
  int rollingFriction = 1;
  for (int j = 0; j < manifold.getNumContacts(); j++) {
   btManifoldPoint cp = manifold.getContactPoint(j);
   if (cp.getDistance() <= manifold.getContactProcessingThreshold()) {
    final btVector3 rel_pos1 = new btVector3();
    final btVector3 rel_pos2 = new btVector3();
    float[] relaxation = new float[1];
    btSolverConstraint solverConstraint = new btSolverConstraint();
    solverConstraint.m_solverBodyA = solverBodyA;
    solverConstraint.m_solverBodyB = solverBodyB;
    solverConstraint.m_originalContactPoint = cp;
    contact.add(solverConstraint);
    final btVector3 pos1 = cp.getPositionWorldOnA();
    final btVector3 pos2 = cp.getPositionWorldOnB();
    rel_pos1.set(pos1).sub(colObj0.getWorldTransformPtr().getOrigin());
    rel_pos2.set(pos2).sub(colObj1.getWorldTransformPtr().getOrigin());
    final btVector3 vel1 = new btVector3();
    final btVector3 vel2 = new btVector3();
    solverBodyA.getVelocityInLocalPointNoDelta(rel_pos1, vel1);
    solverBodyB.getVelocityInLocalPointNoDelta(rel_pos2, vel2);
    final btVector3 vel = new btVector3(vel1).sub(vel2);
    float rel_vel = cp.m_normalWorldOnB.dot(vel);
    setupContactConstraint(solverConstraint, solverBodyA, solverBodyB, cp, infoGlobal, relaxation,
     rel_pos1,
     rel_pos2);
    /////setup the friction constraints
    if ((cp.m_combinedRollingFriction > 0.f) && (rollingFriction > 0)) {
     {
      addTorsionalFrictionConstraint(
       cp.m_normalWorldOnB, solverBodyA, solverBodyB,
       solverConstraint, cp,
       cp.m_combinedSpinningFriction, rel_pos1, rel_pos2, colObj0, colObj1, relaxation[0]);
      final btVector3 axis0 = new btVector3();
      final btVector3 axis1 = new btVector3();
      btPlaneSpace1(cp.m_normalWorldOnB, axis0, axis1);
      axis0.normalize();
      axis1.normalize();
      applyAnisotropicFriction(colObj0, axis0, btCollisionObject.CF_ANISOTROPIC_ROLLING_FRICTION);
      applyAnisotropicFriction(colObj1, axis0, btCollisionObject.CF_ANISOTROPIC_ROLLING_FRICTION);
      applyAnisotropicFriction(colObj0, axis1, btCollisionObject.CF_ANISOTROPIC_ROLLING_FRICTION);
      applyAnisotropicFriction(colObj1, axis1, btCollisionObject.CF_ANISOTROPIC_ROLLING_FRICTION);
      if (axis0.length() > 0.001f) {
       addTorsionalFrictionConstraint(axis0,
        solverBodyA, solverBodyB,
        solverConstraint,
        cp,
        cp.m_combinedRollingFriction, rel_pos1, rel_pos2, colObj0, colObj1, relaxation[0]);
      }
      if (axis1.length() > 0.001f) {
       addTorsionalFrictionConstraint(axis1,
        solverBodyA, solverBodyB,
        solverConstraint,
        cp,
        cp.m_combinedRollingFriction, rel_pos1, rel_pos2, colObj0, colObj1, relaxation[0]);
      }
     }
    }
    ///Bullet has several options to set the friction directions
    ///By default, each contact has only a single friction direction that is recomputed automatically very frame
    ///based on the relative linear velocity.
    ///If the relative velocity it zero, it will automatically compute a friction direction.
    ///You can also enable two friction directions, using the SOLVER_USE_2_FRICTION_DIRECTIONS.
    ///In that case, the second friction direction will be orthogonal to both contact normal and first friction direction.
    ///
    ///If you choose SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION, then the friction will be independent from the relative projected velocity.
    ///
    ///The user can manually override the friction directions for certain contacts using a contact callback,
    ///and set the cp.m_lateralFrictionInitialized to true
    ///In that case, you can set the target relative motion in each friction direction (cp.m_contactMotion1 and cp.m_contactMotion2)
    ///this will give a conveyor belt effect
    ///
    if ((infoGlobal.m_solverMode & SOLVER_ENABLE_FRICTION_DIRECTION_CACHING) == 0 ||
      (cp.m_contactPointFlags & BT_CONTACT_FLAG_LATERAL_FRICTION_INITIALIZED) == 0) {
     cp.m_lateralFrictionDir1.scaleAdd(-rel_vel, cp.m_normalWorldOnB, vel);
     float lat_rel_vel = cp.m_lateralFrictionDir1.lengthSquared();
     if ((infoGlobal.m_solverMode & SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION) == 0 &&
       lat_rel_vel > SIMD_EPSILON) {
      cp.m_lateralFrictionDir1.scale(1.f / btSqrt(lat_rel_vel));
      applyAnisotropicFriction(colObj0, cp.m_lateralFrictionDir1,
       btCollisionObject.CF_ANISOTROPIC_FRICTION);
      applyAnisotropicFriction(colObj1, cp.m_lateralFrictionDir1,
       btCollisionObject.CF_ANISOTROPIC_FRICTION);
      attach_friction_to_contact_constraint(solverConstraint, addFrictionConstraint(
       cp.m_lateralFrictionDir1, solverBodyA, solverBodyB,
       solverConstraint, cp, rel_pos1,
       rel_pos2, colObj0, colObj1, relaxation[0]));
      if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS) != 0) {
       cp.m_lateralFrictionDir2.set(cp.m_lateralFrictionDir1).cross(cp.m_normalWorldOnB);
       cp.m_lateralFrictionDir2.normalize();//??
       applyAnisotropicFriction(colObj0, cp.m_lateralFrictionDir2,
        btCollisionObject.CF_ANISOTROPIC_FRICTION);
       applyAnisotropicFriction(colObj1, cp.m_lateralFrictionDir2,
        btCollisionObject.CF_ANISOTROPIC_FRICTION);
       attach_friction_to_contact_constraint(solverConstraint, addFrictionConstraint(
        cp.m_lateralFrictionDir2, solverBodyA, solverBodyB,
        solverConstraint, cp,
        rel_pos1, rel_pos2, colObj0, colObj1, relaxation[0]));
      }
     } else {
      btPlaneSpace1(cp.m_normalWorldOnB, cp.m_lateralFrictionDir1, cp.m_lateralFrictionDir2);
      applyAnisotropicFriction(colObj0, cp.m_lateralFrictionDir1,
       btCollisionObject.CF_ANISOTROPIC_FRICTION);
      applyAnisotropicFriction(colObj1, cp.m_lateralFrictionDir1,
       btCollisionObject.CF_ANISOTROPIC_FRICTION);
      attach_friction_to_contact_constraint(solverConstraint, addFrictionConstraint(
       cp.m_lateralFrictionDir1, solverBodyA, solverBodyB,
       solverConstraint, cp, rel_pos1,
       rel_pos2, colObj0, colObj1, relaxation[0]));
      if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS) != 0) {
       assert (solverConstraint.m_solverFriction2 == null);
       applyAnisotropicFriction(colObj0, cp.m_lateralFrictionDir2,
        btCollisionObject.CF_ANISOTROPIC_FRICTION);
       applyAnisotropicFriction(colObj1, cp.m_lateralFrictionDir2,
        btCollisionObject.CF_ANISOTROPIC_FRICTION);
       attach_friction_to_contact_constraint(solverConstraint, addFrictionConstraint(
        cp.m_lateralFrictionDir2, solverBodyA, solverBodyB,
        solverConstraint, cp,
        rel_pos1, rel_pos2, colObj0, colObj1, relaxation[0]));
      }
      if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS) != 0 &&
        (infoGlobal.m_solverMode & SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION) != 0) {
       cp.m_contactPointFlags |= BT_CONTACT_FLAG_LATERAL_FRICTION_INITIALIZED;
      }
     }
    } else {
     attach_friction_to_contact_constraint(solverConstraint, addFrictionConstraint(
      cp.m_lateralFrictionDir1, solverBodyA, solverBodyB,
      solverConstraint,
      cp, rel_pos1,
      rel_pos2, colObj0, colObj1, relaxation[0], cp.m_contactMotion1, cp.m_frictionCFM));
     if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS) != 0) {
      attach_friction_to_contact_constraint(solverConstraint, addFrictionConstraint(
       cp.m_lateralFrictionDir2, solverBodyA, solverBodyB,
       solverConstraint, cp, rel_pos1,
       rel_pos2, colObj0, colObj1, relaxation[0], cp.m_contactMotion2, cp.m_frictionCFM));
     }
    }
    setFrictionConstraintImpulse(solverConstraint, solverBodyA, solverBodyB, cp, infoGlobal);
   }
  }
 }

 float resolveSplitPenetrationSIMD(
  btSolverBody bodyA, btSolverBody bodyB,
  btSolverConstraint contactConstraint) {
  return resolveSplitPenetrationImpulseCacheFriendly(bodyA, bodyB, contactConstraint);
 }

 float resolveSplitPenetrationImpulseCacheFriendly(
  btSolverBody bodyA, btSolverBody bodyB,
  btSolverConstraint contactConstraint) {
  float deltaImpulse = 0.f;
  if (contactConstraint.m_rhsPenetration != 0.0f) {
   gNumSplitImpulseRecoveries++;
   deltaImpulse = contactConstraint.m_rhsPenetration - (contactConstraint.m_appliedPushImpulse) *
     contactConstraint.m_cfm;
   float deltaVel1Dotn = contactConstraint.m_contactNormal1.dot(bodyA.internalGetPushVelocity()) +
     contactConstraint.m_relpos1CrossNormal.dot(bodyA.internalGetTurnVelocity());
   float deltaVel2Dotn = contactConstraint.m_contactNormal2.dot(bodyB.internalGetPushVelocity()) +
     contactConstraint.m_relpos2CrossNormal.dot(bodyB.internalGetTurnVelocity());
   deltaImpulse -= deltaVel1Dotn * contactConstraint.m_jacDiagABInv;
   deltaImpulse -= deltaVel2Dotn * contactConstraint.m_jacDiagABInv;
   float sum = (contactConstraint.m_appliedPushImpulse) + deltaImpulse;
   if (sum < contactConstraint.m_lowerLimit) {
    deltaImpulse = contactConstraint.m_lowerLimit - contactConstraint.m_appliedPushImpulse;
    contactConstraint.m_appliedPushImpulse = contactConstraint.m_lowerLimit;
   } else {
    contactConstraint.m_appliedPushImpulse = sum;
   }
   bodyA.internalApplyPushImpulse(new btVector3(contactConstraint.m_contactNormal1).mul(bodyA
    .internalGetInvMass()),
    contactConstraint.m_angularComponentA, deltaImpulse);
   bodyB.internalApplyPushImpulse(new btVector3(contactConstraint.m_contactNormal2).mul(bodyB
    .internalGetInvMass()),
    contactConstraint.m_angularComponentB, deltaImpulse);
  }
  return deltaImpulse;
 }

 //internal method
 btSolverBody getOrInitSolverBody(btCollisionObject body, float timeStep) {
  btSolverBody solverBodyA;
  if (body.getCompanion() != null) {
   //body has already been converted
   solverBodyA = (btSolverBody) body.getCompanion();
  } else {
   btRigidBody rb = btRigidBody.upcast(body);
   //convert both active and kinematic objects (for their velocity)
   if (rb != null && (rb.getInvMass() != 0.0f || rb.isKinematicObject())) {
    solverBodyA = new btSolverBody();
    initSolverBody(solverBodyA, body, timeStep);
    body.setCompanion(solverBodyA);
   } else {
    if (m_fixedBody == null) {
     m_fixedBody = new btSolverBody();
     initSolverBody(m_fixedBody, null, timeStep);
    }
    return m_fixedBody;
//			return 0;//assume first one is a fixed solver body
   }
  }
  return solverBodyA;
 }

 void initSolverBody(btSolverBody solverBody, btCollisionObject collisionObject, float timeStep) {
  btRigidBody rb = collisionObject != null ? btRigidBody.upcast(collisionObject) : null;
  solverBody.internalGetDeltaLinearVelocity().setZero();
  solverBody.internalGetDeltaAngularVelocity().setZero();
  solverBody.internalGetPushVelocity().setZero();
  solverBody.internalGetTurnVelocity().setZero();
  if (rb != null) {
   solverBody.m_worldTransform.set(rb.getWorldTransformPtr());
   solverBody.internalSetInvMass(new btVector3(rb.getInvMass(), rb.getInvMass(), rb.getInvMass())
    .mul(rb.getLinearFactorPtr()));
   solverBody.m_originalBody = rb;
   solverBody.m_angularFactor.set(rb.getAngularFactor());
   solverBody.m_linearFactor.set(rb.getLinearFactorPtr());
   solverBody.m_linearVelocity.set(rb.getLinearVelocity());
   solverBody.m_angularVelocity.set(rb.getAngularVelocity());
   solverBody.m_externalForceImpulse.set(rb.getTotalForce().scale(rb.getInvMass() * timeStep));
   solverBody.m_externalTorqueImpulse.set(rb.getInvInertiaTensorWorld().transposeTransform(
    rb.getTotalTorque()).scale(timeStep));
  } else {
   solverBody.m_worldTransform.setIdentity();
   solverBody.internalSetInvMass(new btVector3());
   solverBody.m_originalBody = null;
   solverBody.m_angularFactor.set(1, 1, 1);
   solverBody.m_linearFactor.set(1, 1, 1);
   solverBody.m_linearVelocity.setZero();
   solverBody.m_angularVelocity.setZero();
   solverBody.m_externalForceImpulse.setZero();
   solverBody.m_externalTorqueImpulse.setZero();
  }
  solver.add(solverBody);
 }

 float resolveSingleConstraintRowGeneric(btSolverBody bodyA, btSolverBody bodyB,
  btSolverConstraint contactConstraint) {
  return gResolveSingleConstraintRowGeneric_scalar_reference(bodyA, bodyB, contactConstraint);
 }
//#define VERBOSE_RESIDUAL_PRINTF 1
///This is the scalar reference implementation of solving a single constraint row, the innerloop of the Projected Gauss Seidel/Sequential Impulse constraint solver
///Below are optional SSE2 and SSE4/FMA3 versions. We assume most hardware has SSE2. For SSE4/FMA3 we perform a CPU feature check.

 float resolveSingleConstraintRowGenericSIMD(btSolverBody bodyA, btSolverBody bodyB,
  btSolverConstraint contactConstraint) {
  return resolveSingleConstraintRowGeneric(bodyA, bodyB, contactConstraint);
 }

 float resolveSingleConstraintRowLowerLimit(btSolverBody bodyA, btSolverBody bodyB,
  btSolverConstraint contactConstraint) {
  return gResolveSingleConstraintRowLowerLimit_scalar_reference(bodyA, bodyB, contactConstraint);
 }

 float resolveSingleConstraintRowLowerLimitSIMD(btSolverBody bodyA, btSolverBody bodyB,
  btSolverConstraint contactConstraint) {
  return resolveSingleConstraintRowLowerLimit(bodyA, bodyB, contactConstraint);
 }

 void solveGroupCacheFriendlySplitImpulseIterations(List<btCollisionObject> bodies,
  int numBodies,
  List<btPersistentManifold> manifoldPtr, int numManifolds,
  List<btTypedConstraint> constraints, int numConstraints,
  btContactSolverInfo infoGlobal, btIDebugDraw debugDrawer) {
  int iteration;
  if (infoGlobal.m_splitImpulse) {
   if ((infoGlobal.m_solverMode & SOLVER_SIMD) != 0) {
    for (iteration = 0; iteration < infoGlobal.m_numIterations; iteration++) {
     float leastSquaresResidual = 0.f;
     {
      for (btSolverConstraint solveManifold : contact) {
       float residual = resolveSplitPenetrationSIMD(
        solveManifold.m_solverBodyA,
        solveManifold.m_solverBodyB, solveManifold);
       leastSquaresResidual += residual * residual;
      }
     }
     if (leastSquaresResidual <= infoGlobal.m_leastSquaresResidualThreshold || iteration >=
       (infoGlobal.m_numIterations - 1)) {
      break;
     }
    }
   } else {
    for (iteration = 0; iteration < infoGlobal.m_numIterations; iteration++) {
     float leastSquaresResidual = 0.f;
     {
      for (btSolverConstraint solveManifold : contact) {
       float residual = resolveSplitPenetrationImpulseCacheFriendly(solveManifold.m_solverBodyA,
        solveManifold.m_solverBodyB,
        solveManifold);
       leastSquaresResidual += residual * residual;
      }
      if (leastSquaresResidual <= infoGlobal.m_leastSquaresResidualThreshold || iteration >=
        (infoGlobal.m_numIterations - 1)) {
       break;
      }
     }
    }
   }
  }
 }

 float solveGroupCacheFriendlyFinish(List<btCollisionObject> bodies, int numBodies,
  btContactSolverInfo infoGlobal) {
  int i, j;
  if ((infoGlobal.m_solverMode & SOLVER_USE_WARMSTARTING) != 0) {
   for (btSolverConstraint solveManifold : contact) {
    btManifoldPoint pt = (btManifoldPoint) solveManifold.m_originalContactPoint;
    assert (pt != null);
    pt.m_appliedImpulse = solveManifold.m_appliedImpulse;
    pt.m_appliedImpulseLateral1 = solveManifold.m_solverFriction.m_appliedImpulse;
    if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS) != 0) {
     pt.m_appliedImpulseLateral2 = solveManifold.m_solverFriction2.m_appliedImpulse;
    }
    //do a callback here?
   }
  }
  for (btSolverConstraint solverConstr : non_contact) {
   btTypedConstraint constr = (btTypedConstraint) solverConstr.m_originalContactPoint;
   btJointFeedback fb = constr.getJointFeedback();
   if (fb != null) {
    fb.m_appliedForceBodyA
     .add(new btVector3(solverConstr.m_contactNormal1)
      .scale(solverConstr.m_appliedImpulse)
      .mul(constr.getRigidBodyA().getLinearFactorPtr())
      .scale(1.0f / infoGlobal.m_timeStep));
    fb.m_appliedForceBodyB
     .add(new btVector3(solverConstr.m_contactNormal2)
      .scale(solverConstr.m_appliedImpulse)
      .mul(constr.getRigidBodyB().getLinearFactorPtr())
      .scale(1.0f / infoGlobal.m_timeStep));
    fb.m_appliedTorqueBodyA
     .add(new btVector3(solverConstr.m_relpos1CrossNormal)
      .mul(constr.getRigidBodyA().getAngularFactor())
      .scale(solverConstr.m_appliedImpulse)
      .scale(1.0f / infoGlobal.m_timeStep));
    fb.m_appliedTorqueBodyB
     .add(new btVector3(solverConstr.m_relpos2CrossNormal)
      .mul(constr.getRigidBodyB().getAngularFactor())
      .scale(solverConstr.m_appliedImpulse)
      .scale(1.0f / infoGlobal.m_timeStep));
    /*
     * RGM ????
     */
   }
   constr.internalSetAppliedImpulse(solverConstr.m_appliedImpulse);
   if (btFabs(solverConstr.m_appliedImpulse) >= constr.getBreakingImpulseThreshold()) {
    constr.setEnabled(false);
   }
  }
  for (btSolverBody solverBody : solver) {
   btRigidBody body = solverBody.m_originalBody;
   if (body != null) {
    if (infoGlobal.m_splitImpulse) {
     solverBody.writebackVelocityAndTransform(infoGlobal.m_timeStep,
      infoGlobal.m_splitImpulseTurnErp);
    } else {
     solverBody.writebackVelocity();
    }
    body.setLinearVelocity(
     new btVector3(solverBody.m_linearVelocity)
     .add(solverBody.m_externalForceImpulse));
    body.setAngularVelocity(
     new btVector3(solverBody.m_angularVelocity)
     .add(solverBody.m_externalTorqueImpulse));
    if (infoGlobal.m_splitImpulse) {
     body.setWorldTransform(solverBody.m_worldTransform);
    }
    body.setCompanion(null);
   }
  }
  contact.clear();
  contact_friction.clear();
  rolling_friction.clear();
  solver.clear();
  non_contact.clear();
  return 0.f;
 }

 float solveSingleIteration(int iteration, List<btCollisionObject> bodies, int numBodies,
  List<btPersistentManifold> manifoldPtr, int numManifolds,
  List<btTypedConstraint> constraints, int numConstraints,
  btContactSolverInfo infoGlobal, btIDebugDraw debugDrawer) {
  float leastSquaresResidual = 0.f;
  if ((infoGlobal.m_solverMode & SOLVER_RANDOMIZE_ORDER) != 0) {
   //contact/friction constraints are not solved more than
   shuffle_list(non_contact);
   if (iteration < infoGlobal.m_numIterations) {
    {
     shuffle_list(contact);
     shuffle_list(contact_friction);
    }
   }
  }
  if ((infoGlobal.m_solverMode & SOLVER_SIMD) != 0) {
   ///solve all joint constraints, using SIMD, if available
   for (btSolverConstraint constraint : non_contact) {
    if (iteration < constraint.m_overrideNumSolverIterations) {
     float residual = resolveSingleConstraintRowGenericSIMD(constraint.m_solverBodyA,
      constraint.m_solverBodyB,
      constraint);
     leastSquaresResidual += residual * residual;
    }
   }
   if (iteration < infoGlobal.m_numIterations) {
//    for (btTypedConstraint constraint : constraints.subList(0, numConstraints)) {
//     if (constraint.isEnabled()) {
//      btSolverBody bodyA = getOrInitSolverBody(constraint.getRigidBodyA(), infoGlobal.m_timeStep);
//      btSolverBody bodyB = getOrInitSolverBody(constraint.getRigidBodyB(), infoGlobal.m_timeStep);
//      constraint.solveConstraintObsolete(bodyA, bodyB, infoGlobal.m_timeStep);
//     }
//    }
    ///solve all contact constraints using SIMD, if available
    if ((infoGlobal.m_solverMode & SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS) != 0) {
     for (btSolverConstraint solveContact : contact) {
      float totalImpulse;
      {
       float residual = resolveSingleConstraintRowLowerLimitSIMD(solveContact.m_solverBodyA,
        solveContact.m_solverBodyB,
        solveContact);
       leastSquaresResidual += residual * residual;
       totalImpulse = solveContact.m_appliedImpulse;
      }
      boolean applyFriction = true;
      if (applyFriction) {
       {
        btSolverConstraint solveFriction = solveContact.m_solverFriction;
        if (totalImpulse > (0f)) {
         solveFriction.m_lowerLimit = -(solveFriction.m_friction * totalImpulse);
         solveFriction.m_upperLimit = solveFriction.m_friction * totalImpulse;
         float residual = resolveSingleConstraintRowGenericSIMD(solveFriction.m_solverBodyA,
          solveFriction.m_solverBodyB,
          solveFriction);
         leastSquaresResidual += residual * residual;
        }
       }
       if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS) != 0) {
        btSolverConstraint solverFriction2 = solveContact.m_solverFriction2;
        if (totalImpulse > (0f)) {
         solverFriction2.m_lowerLimit = -(solverFriction2.m_friction * totalImpulse);
         solverFriction2.m_upperLimit = solverFriction2.m_friction * totalImpulse;
         float residual = resolveSingleConstraintRowGenericSIMD(solverFriction2.m_solverBodyA,
          solverFriction2.m_solverBodyB,
          solverFriction2);
         leastSquaresResidual += residual * residual;
        }
       }
      }
     }
    } else//SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS
    {
     //solve the friction constraints after all contact constraints, don't interleave them
     int j;
     for (btSolverConstraint solveContact : contact) {
      float residual = resolveSingleConstraintRowLowerLimitSIMD(solveContact.m_solverBodyA,
       solveContact.m_solverBodyB,
       solveContact);
      leastSquaresResidual += residual * residual;
     }
     ///solve all friction constraints, using SIMD, if available
     for (btSolverConstraint solveFriction : contact_friction) {
      float totalImpulse = solveFriction.m_solverFriction.m_appliedImpulse;
      if (totalImpulse > (0f)) {
       solveFriction.m_lowerLimit = -(solveFriction.m_friction * totalImpulse);
       solveFriction.m_upperLimit = solveFriction.m_friction * totalImpulse;
       float residual = resolveSingleConstraintRowGenericSIMD(solveFriction.m_solverBodyA,
        solveFriction.m_solverBodyB,
        solveFriction);
       leastSquaresResidual += residual * residual;
      }
     }
     for (btSolverConstraint rollingFrictionConstraint : rolling_friction) {
      float totalImpulse = rollingFrictionConstraint.m_solverFriction.m_appliedImpulse;
      if (totalImpulse > (0f)) {
       float rollingFrictionMagnitude = rollingFrictionConstraint.m_friction * totalImpulse;
       if (rollingFrictionMagnitude > rollingFrictionConstraint.m_friction) {
        rollingFrictionMagnitude = rollingFrictionConstraint.m_friction;
       }
       rollingFrictionConstraint.m_lowerLimit = -rollingFrictionMagnitude;
       rollingFrictionConstraint.m_upperLimit = rollingFrictionMagnitude;
       float residual = resolveSingleConstraintRowGenericSIMD(
        rollingFrictionConstraint.m_solverBodyA,
        rollingFrictionConstraint.m_solverBodyB, rollingFrictionConstraint);
       leastSquaresResidual += residual * residual;
      }
     }
    }
   }
  } else {
   //non-SIMD version
   ///solve all joint constraints
   for (btSolverConstraint constraint : non_contact) {
    if (iteration < constraint.m_overrideNumSolverIterations) {
     float residual = resolveSingleConstraintRowGeneric(constraint.m_solverBodyA,
      constraint.m_solverBodyB,
      constraint);
     leastSquaresResidual += residual * residual;
    }
   }
   if (iteration < infoGlobal.m_numIterations) {
//    for (btTypedConstraint constraint : constraints.subList(0, numConstraints)) {
//     if (constraint.isEnabled()) {
//      btSolverBody bodyA = getOrInitSolverBody(constraint.getRigidBodyA(), infoGlobal.m_timeStep);
//      btSolverBody bodyB = getOrInitSolverBody(constraint.getRigidBodyB(), infoGlobal.m_timeStep);
//      constraint.solveConstraintObsolete(bodyA, bodyB, infoGlobal.m_timeStep);
//     }
//    }
    ///solve all contact constraints
    for (btSolverConstraint solveManifold : contact) {
     float residual = resolveSingleConstraintRowLowerLimit(solveManifold.m_solverBodyA,
      solveManifold.m_solverBodyB,
      solveManifold);
     leastSquaresResidual += residual * residual;
    }
    ///solve all friction constraints
    for (btSolverConstraint solveManifold : contact_friction) {
     float totalImpulse = solveManifold.m_solverFriction.m_appliedImpulse;
     if (totalImpulse > (0f)) {
      solveManifold.m_lowerLimit = -(solveManifold.m_friction * totalImpulse);
      solveManifold.m_upperLimit = solveManifold.m_friction * totalImpulse;
      float residual =
       resolveSingleConstraintRowGeneric(solveManifold.m_solverBodyA, solveManifold.m_solverBodyB,
        solveManifold);
      leastSquaresResidual += residual * residual;
     }
    }
    for (btSolverConstraint rollingFrictionConstraint : rolling_friction) {
     float totalImpulse = rollingFrictionConstraint.m_solverFriction.m_appliedImpulse;
     if (totalImpulse > (0f)) {
      float rollingFrictionMagnitude = rollingFrictionConstraint.m_friction * totalImpulse;
      if (rollingFrictionMagnitude > rollingFrictionConstraint.m_friction) {
       rollingFrictionMagnitude = rollingFrictionConstraint.m_friction;
      }
      rollingFrictionConstraint.m_lowerLimit = -rollingFrictionMagnitude;
      rollingFrictionConstraint.m_upperLimit = rollingFrictionMagnitude;
      float residual = resolveSingleConstraintRowGeneric(rollingFrictionConstraint.m_solverBodyA,
       rollingFrictionConstraint.m_solverBodyB, rollingFrictionConstraint);
      leastSquaresResidual += residual * residual;
     }
    }
   }
  }
  return leastSquaresResidual;
 }

/// btSequentialImpulseConstraintSolver Sequentially applies impulses
 float solveGroupCacheFriendlySetup(List<btCollisionObject> bodies, int numBodies,
  List<btPersistentManifold> manifoldPtr, int numManifolds,
  List<btTypedConstraint> constraints, int numConstraints,
  btContactSolverInfo infoGlobal, btIDebugDraw debugDrawer) {
  m_fixedBody = null;
  BT_PROFILE("solveGroupCacheFriendlySetup");
  m_maxOverrideNumSolverIterations = 0;
  if (DEBUG_BLOCKS) {
   //make sure that dynamic bodies exist for all (enabled) constraints
   for (int i = 0; i < numConstraints; i++) {
    btTypedConstraint constraint = constraints.get(i);
    if (constraint.isEnabled()) {
     if (!constraint.getRigidBodyA().isStaticOrKinematicObject()) {
      boolean found = false;
      for (int b = 0; b < numBodies; b++) {
       if (constraint.getRigidBodyA() == bodies.get(b)) {
        found = true;
        break;
       }
      }
      assert (found);
     }
     if (!constraint.getRigidBodyB().isStaticOrKinematicObject()) {
      boolean found = false;
      for (int b = 0; b < numBodies; b++) {
       if (constraint.getRigidBodyB() == bodies.get(b)) {
        found = true;
        break;
       }
      }
      assert (found);
     }
    }
   }
   //make sure that dynamic bodies exist for all contact manifolds
   for (int i = 0; i < numManifolds; i++) {
    if (!manifoldPtr.get(i).getBody0().isStaticOrKinematicObject()) {
     boolean found = false;
     for (int b = 0; b < numBodies; b++) {
      if (manifoldPtr.get(i).getBody0() == bodies.get(b)) {
       found = true;
       break;
      }
     }
     assert (found);
    }
    if (!manifoldPtr.get(i).getBody1().isStaticOrKinematicObject()) {
     boolean found = false;
     for (int b = 0; b < numBodies; b++) {
      if (manifoldPtr.get(i).getBody1() == bodies.get(b)) {
       found = true;
       break;
      }
     }
     assert (found);
    }
   }
  }
  for (btCollisionObject body : bodies.subList(0, numBodies)) {
   body.setCompanion(null);
  }
  solver.clear();
  //convert all bodies
  for (btCollisionObject object : bodies.subList(0, numBodies)) {
   btSolverBody solverBody = getOrInitSolverBody(object, infoGlobal.m_timeStep);
   btRigidBody body = btRigidBody.upcast(object);
   if (body != null && body.getInvMass() != 0.0f) {
    final btVector3 gyroForce = new btVector3();
    if ((body.getFlags() & BT_ENABLE_GYROSCOPIC_FORCE_EXPLICIT) != 0) {
     gyroForce.set(body.computeGyroscopicForceExplicit(infoGlobal.m_maxGyroscopicForce));
     solverBody.m_externalTorqueImpulse
      .sub(body.getInvInertiaTensorWorld().transposeTransform(new btVector3(gyroForce))
       .scale(infoGlobal.m_timeStep));
    }
    if ((body.getFlags() & BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD) != 0) {
     gyroForce.set(body.computeGyroscopicImpulseImplicit_World(infoGlobal.m_timeStep));
     solverBody.m_externalTorqueImpulse.add(gyroForce);
    }
    if ((body.getFlags() & BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY) != 0) {
     gyroForce.set(body.computeGyroscopicImpulseImplicit_Body(infoGlobal.m_timeStep));
     solverBody.m_externalTorqueImpulse.add(gyroForce);
    }
   }
  }
  if (true) {
   for (btTypedConstraint constraint : constraints.subList(0, numConstraints)) {
    constraint.buildJacobian();
    constraint.internalSetAppliedImpulse(0.0f);
   }
  }
  {
   int constraint_num = 0;
   //calculate the total number of contraint rows
   resize_info1(numConstraints);
   for (btTypedConstraint constraint : constraints.subList(0, numConstraints)) {
    btTypedConstraint.btConstraintInfo1 info1 = constraint_info1[constraint_num];
    ++constraint_num;
    btJointFeedback fb = constraint.getJointFeedback();
    if (fb != null) {
     fb.m_appliedForceBodyA.setZero();
     fb.m_appliedTorqueBodyA.setZero();
     fb.m_appliedForceBodyB.setZero();
     fb.m_appliedTorqueBodyB.setZero();
    }
    if (constraint.isEnabled()) {
     if (DEBUG_BLOCKS) {
      info1.m_numConstraintRows = -999_999_999;
      info1.nub = -999_999_998;
     }
     constraint.getInfo1(info1);
     if (DEBUG_BLOCKS) {
      assert (info1.m_numConstraintRows >= 0);
      assert (info1.nub >= 0);
     }
    } else {
     info1.m_numConstraintRows = 0;
     info1.nub = 0;
    }
   }
   ///setup the btSolverConstraints
   assert (non_contact.isEmpty());
   ArrayList<btSolverConstraint> constraint_rows = new ArrayList<>(0);
   assert (constraint_info1.length == numConstraints);
   for (int i = 0; i < constraint_info1.length; i++) {
    btTypedConstraint.btConstraintInfo1 info1 = constraint_info1[i];
    if (info1.m_numConstraintRows != 0) {
     constraint_rows.clear();
     for (int row = 0; row < info1.m_numConstraintRows; ++row) {
      final btSolverConstraint new_row = new btSolverConstraint();
      non_contact.add(new_row);
      constraint_rows.add(new_row);
     }
     btTypedConstraint constraint = constraints.get(i);
     btRigidBody rbA = constraint.getRigidBodyA();
     btRigidBody rbB = constraint.getRigidBodyB();
     btSolverBody bodyAPtr = getOrInitSolverBody(rbA, infoGlobal.m_timeStep);
     btSolverBody bodyBPtr = getOrInitSolverBody(rbB, infoGlobal.m_timeStep);
     int overrideNumSolverIterations = constraint.getOverrideNumSolverIterations() > 0 ? constraint.
      getOverrideNumSolverIterations() : infoGlobal.m_numIterations;
     if (overrideNumSolverIterations > m_maxOverrideNumSolverIterations) {
      m_maxOverrideNumSolverIterations = overrideNumSolverIterations;
     }
     int j;
     for (btSolverConstraint currentConstraintRow : constraint_rows) {
      currentConstraintRow.m_lowerLimit = -SIMD_INFINITY;
      currentConstraintRow.m_upperLimit = SIMD_INFINITY;
      currentConstraintRow.m_appliedImpulse = 0.f;
      currentConstraintRow.m_appliedPushImpulse = 0.f;
      currentConstraintRow.m_solverBodyA = bodyAPtr;
      currentConstraintRow.m_solverBodyB = bodyBPtr;
      currentConstraintRow.m_overrideNumSolverIterations = overrideNumSolverIterations;
     }
     bodyAPtr.internalGetDeltaLinearVelocity().setZero();
     bodyAPtr.internalGetDeltaAngularVelocity().setZero();
     bodyAPtr.internalGetPushVelocity().setZero();
     bodyAPtr.internalGetTurnVelocity().setZero();
     bodyBPtr.internalGetDeltaLinearVelocity().setZero();
     bodyBPtr.internalGetDeltaAngularVelocity().setZero();
     bodyBPtr.internalGetPushVelocity().setZero();
     bodyBPtr.internalGetTurnVelocity().setZero();
     btConstraintInfo2 info2 = new btConstraintInfo2();
     info2.fps = 1.f / infoGlobal.m_timeStep;
     info2.erp = infoGlobal.m_erp;
     info2.m_J1linearAxis = contactNormal1_array(constraint_rows);
     info2.m_J1angularAxis = relpos1CrossNormal_array(constraint_rows);
     info2.m_J2linearAxis = contactNormal2_array(constraint_rows);
     info2.m_J2angularAxis = relpos2CrossNormal_array(constraint_rows);
     //info2.rowskip = 1;//check this
     info2.m_constraintError = rhs_array(constraint_rows);
     constraint_rows.get(0).m_cfm = infoGlobal.m_globalCfm;
     info2.m_damping = infoGlobal.m_damping;
     info2.cfm = cfm_array(constraint_rows);
     info2.m_lowerLimit = lowerLimit_array(constraint_rows);
     info2.m_upperLimit = upperLimit_array(constraint_rows);
     info2.m_numIterations = infoGlobal.m_numIterations;
     constraint.getInfo2(info2);
     ///finalize the constraint setup
     for (btSolverConstraint solverConstraint : constraint_rows) {
      if (solverConstraint.m_upperLimit >= constraint.getBreakingImpulseThreshold()) {
       solverConstraint.m_upperLimit = constraint.getBreakingImpulseThreshold();
      }
      if (solverConstraint.m_lowerLimit <= -constraint.getBreakingImpulseThreshold()) {
       solverConstraint.m_lowerLimit = -constraint.getBreakingImpulseThreshold();
      }
      solverConstraint.m_originalContactPoint = constraint;
      {
       final btVector3 ftorqueAxis1 = new btVector3(solverConstraint.m_relpos1CrossNormal);
       solverConstraint.m_angularComponentA
        .set(constraint.getRigidBodyA().getInvInertiaTensorWorldPtr()
         .transform(ftorqueAxis1)
         .mul(constraint.getRigidBodyA().getAngularFactor()));
      }
      {
       final btVector3 ftorqueAxis2 = new btVector3(solverConstraint.m_relpos2CrossNormal);
       solverConstraint.m_angularComponentB
        .set(constraint.getRigidBodyB().getInvInertiaTensorWorldPtr()
         .transform(ftorqueAxis2)
         .mul(constraint.getRigidBodyB().getAngularFactor()));
      }
      {
       final btVector3 iMJlA = new btVector3(solverConstraint.m_contactNormal1).scale(rbA
        .getInvMass());
       final btVector3 iMJaA = rbA.getInvInertiaTensorWorldPtr().transform(new btVector3(
        solverConstraint.m_relpos1CrossNormal));
       final btVector3 iMJlB = new btVector3(solverConstraint.m_contactNormal2).scale(rbB
        .getInvMass());//sign of normal?
       final btVector3 iMJaB = rbB.getInvInertiaTensorWorldPtr().transform(new btVector3(
        solverConstraint.m_relpos2CrossNormal));
       float sum = iMJlA.dot(solverConstraint.m_contactNormal1);
       sum += iMJaA.dot(solverConstraint.m_relpos1CrossNormal);
       sum += iMJlB.dot(solverConstraint.m_contactNormal2);
       sum += iMJaB.dot(solverConstraint.m_relpos2CrossNormal);
       float fsum = btFabs(sum);
       assert (fsum > SIMD_EPSILON);
       solverConstraint.m_jacDiagABInv = fsum > SIMD_EPSILON ? (1.f) / sum : 0.f;
      }
      {
       float rel_vel;
       final btVector3 externalForceImpulseA =
        bodyAPtr.m_originalBody != null ? bodyAPtr.m_externalForceImpulse : new btVector3();
       final btVector3 externalTorqueImpulseA = bodyAPtr.m_originalBody != null ?
         bodyAPtr.m_externalTorqueImpulse : new btVector3();
       final btVector3 externalForceImpulseB =
        bodyBPtr.m_originalBody != null ? bodyBPtr.m_externalForceImpulse : new btVector3();
       final btVector3 externalTorqueImpulseB = bodyBPtr.m_originalBody != null ?
         bodyBPtr.m_externalTorqueImpulse : new btVector3();
       float vel1Dotn = solverConstraint.m_contactNormal1.dot(rbA.getLinearVelocity().add(
        externalForceImpulseA)) + solverConstraint.m_relpos1CrossNormal.dot(rbA.getAngularVelocity()
         .add(
          externalTorqueImpulseA));
       float vel2Dotn = solverConstraint.m_contactNormal2.dot(rbB.getLinearVelocity().add(
        externalForceImpulseB)) + solverConstraint.m_relpos2CrossNormal.dot(rbB.getAngularVelocity()
         .add(
          externalTorqueImpulseB));
       rel_vel = vel1Dotn + vel2Dotn;
       float restitution = 0.f;
       float positionalError = solverConstraint.m_rhs;//already filled in by getConstraintInfo2
       float velocityError = restitution - rel_vel * info2.m_damping;
       float penetrationImpulse = positionalError * solverConstraint.m_jacDiagABInv;
       float velocityImpulse = velocityError * solverConstraint.m_jacDiagABInv;
       solverConstraint.m_rhs = penetrationImpulse + velocityImpulse;
       solverConstraint.m_appliedImpulse = 0.f;
      }
     }
    }
   }
   convertContacts(manifoldPtr, numManifolds, infoGlobal);
  }
  return 0.f;
 }

 float solveGroupCacheFriendlyIterations(List<btCollisionObject> bodies, int numBodies,
  List<btPersistentManifold> manifoldPtr, int numManifolds,
  List<btTypedConstraint> constraints, int numConstraints,
  btContactSolverInfo infoGlobal, btIDebugDraw debugDrawer) {
  BT_PROFILE("solveGroupCacheFriendlyIterations");
  {
   ///this is a special step to resolve penetrations (just for contacts)
   solveGroupCacheFriendlySplitImpulseIterations(bodies, numBodies, manifoldPtr, numManifolds,
    constraints,
    numConstraints,
    infoGlobal, debugDrawer);
   int maxIterations =
    
    m_maxOverrideNumSolverIterations > infoGlobal.m_numIterations ? m_maxOverrideNumSolverIterations :
      infoGlobal.m_numIterations;
   for (int iteration = 0; iteration < maxIterations; iteration++) {
    m_leastSquaresResidual = solveSingleIteration(iteration, bodies, numBodies, manifoldPtr,
     numManifolds,
     constraints,
     numConstraints, infoGlobal, debugDrawer);
    if (m_leastSquaresResidual <= infoGlobal.m_leastSquaresResidualThreshold || (iteration >=
      (maxIterations - 1))) {
     break;
    }
   }
  }
  return 0.f;
 }

 public void destroy() {
 }

 @Override
 public float solveGroup(List<btCollisionObject> bodies, int numBodies,
  List<btPersistentManifold> manifold,
  int numManifolds, List<btTypedConstraint> constraints, int numConstraints,
  btContactSolverInfo info,
  btIDebugDraw debugDrawer, btDispatcher dispatcher) {
//  LockStep x =  new LockStep();
//  LockStep.solveGroup(bodies,numBodies,manifold,numManifolds,constraints,numConstraints,info,debugDrawer,dispatcher);
  BT_PROFILE("solveGroup");
  //you need to provide at least some bodies
  solveGroupCacheFriendlySetup(bodies, numBodies, manifold, numManifolds, constraints,
   numConstraints, info, debugDrawer);
  solveGroupCacheFriendlyIterations(bodies, numBodies, manifold, numManifolds, constraints,
   numConstraints, info,
   debugDrawer);
  solveGroupCacheFriendlyFinish(bodies, numBodies, info);
  return 0.f;
 }

 ///clear internal cached data and reset random seed
 @Override
 public void reset() {
  m_btSeed2 = 0;
 }

 public long btRand2() {
  m_btSeed2 = (1664525L * m_btSeed2 + 1013904223L) & 0xefffffff;
  return m_btSeed2;
 }

 public int btRandInt2(int n) {
  // seems good; xor-fold and modulus
  long un = (n);
  long r = btRand2();
  // note: probably more aggressive than it needs to be -- might be
  //       able to get away without one or two of the innermost branches.
  if (un <= 0x00010000L) {
   r ^= (r >>> 16);
   if (un <= 0x00000100L) {
    r ^= (r >>> 8);
    if (un <= 0x00000010L) {
     r ^= (r >>> 4);
     if (un <= 0x00000004L) {
      r ^= (r >>> 2);
      if (un <= 0x00000002L) {
       r ^= (r >>> 1);
      }
     }
    }
   }
  }
  return Math.abs((int) (r % un));
 }

 public void setRandSeed(long seed) {
  m_btSeed2 = seed;
 }

 public long getRandSeed() {
  return m_btSeed2;
 }

 @Override
 public int getSolverType() {
  return BT_SEQUENTIAL_IMPULSE_SOLVER;
 }

 public btSingleConstraintRowSolver getActiveConstraintRowSolverGeneric() {
  return m_resolveSingleConstraintRowGeneric;
 }

 public void setConstraintRowSolverGeneric(btSingleConstraintRowSolver rowSolver) {
  m_resolveSingleConstraintRowGeneric = rowSolver;
 }

 public btSingleConstraintRowSolver getActiveConstraintRowSolverLowerLimit() {
  return m_resolveSingleConstraintRowLowerLimit;
 }

 public void setConstraintRowSolverLowerLimit(btSingleConstraintRowSolver rowSolver) {
  m_resolveSingleConstraintRowLowerLimit = rowSolver;
 }

 ///Various implementations of solving a single constraint row using a generic equality constraint, using scalar reference, SSE2 or SSE4
 public final btSingleConstraintRowSolver getScalarConstraintRowSolverGeneric() {
  return new btSingleConstraintRowSolverImpl4();
 }

 public final btSingleConstraintRowSolver getSSE2ConstraintRowSolverGeneric() {
  return new btSingleConstraintRowSolverImpl3();
 }

 public final btSingleConstraintRowSolver getSSE4_1ConstraintRowSolverGeneric() {
  return new btSingleConstraintRowSolverImpl2();
 }

 ///Various implementations of solving a single constraint row using an inequality (lower limit) constraint, using scalar reference, SSE2 or SSE4
 final btSingleConstraintRowSolver getScalarConstraintRowSolverLowerLimit() {
  return new btSingleConstraintRowSolverImpl();
 }

 public final btSingleConstraintRowSolver getSSE2ConstraintRowSolverLowerLimit() {
  return new btSingleConstraintRowSolverImpl1();
 }

 public final btSingleConstraintRowSolver getSSE4_1ConstraintRowSolverLowerLimit() {
  return new btSingleConstraintRowSolverImpl5();
 }

 private void resize_info1(int numConstraints) {
  if (constraint_info1.length != numConstraints) {
   constraint_info1 = Arrays.copyOf(constraint_info1, numConstraints);
   for (int i = 0; i < constraint_info1.length; ++i) {
    if (constraint_info1[i] == null) {
     constraint_info1[i] = new btConstraintInfo1();
    }
   }
  }
 }

 private btVector3[] contactNormal1_array(ArrayList<btSolverConstraint> constraint_rows) {
  btVector3[] result = new btVector3[constraint_rows.size()];
  for (int i = 0; i < constraint_rows.size(); ++i) {
   btSolverConstraint constraint_row = constraint_rows.get(i);
   result[i] = constraint_row.m_contactNormal1;
  }
  return result;
 }

 private btVector3[] relpos1CrossNormal_array(ArrayList<btSolverConstraint> constraint_rows) {
  btVector3[] result = new btVector3[constraint_rows.size()];
  for (int i = 0; i < constraint_rows.size(); ++i) {
   btSolverConstraint constraint_row = constraint_rows.get(i);
   result[i] = constraint_row.m_relpos1CrossNormal;
  }
  return result;
 }

 private btVector3[] contactNormal2_array(ArrayList<btSolverConstraint> constraint_rows) {
  btVector3[] result = new btVector3[constraint_rows.size()];
  for (int i = 0; i < constraint_rows.size(); ++i) {
   btSolverConstraint constraint_row = constraint_rows.get(i);
   result[i] = constraint_row.m_contactNormal2;
  }
  return result;
 }

 private btVector3[] relpos2CrossNormal_array(ArrayList<btSolverConstraint> constraint_rows) {
  btVector3[] result = new btVector3[constraint_rows.size()];
  for (int i = 0; i < constraint_rows.size(); ++i) {
   btSolverConstraint constraint_row = constraint_rows.get(i);
   result[i] = constraint_row.m_relpos2CrossNormal;
  }
  return result;
 }

 private FloatSmartPointer[] rhs_array(ArrayList<btSolverConstraint> constraint_rows) {
  FloatSmartPointer[] result = new FloatSmartPointer[constraint_rows.size()];
  for (int i = 0; i < constraint_rows.size(); ++i) {
   btSolverConstraint constraint_row = constraint_rows.get(i);
   result[i] = new RHSPointer(constraint_row);
  }
  return result;
 }

 private FloatSmartPointer[] cfm_array(ArrayList<btSolverConstraint> constraint_rows) {
  FloatSmartPointer[] result = new FloatSmartPointer[constraint_rows.size()];
  for (int i = 0; i < constraint_rows.size(); ++i) {
   btSolverConstraint constraint_row = constraint_rows.get(i);
   result[i] = new CFMPointer(constraint_row);
  }
  return result;
 }

 private FloatSmartPointer[] lowerLimit_array(ArrayList<btSolverConstraint> constraint_rows) {
  FloatSmartPointer[] result = new FloatSmartPointer[constraint_rows.size()];
  for (int i = 0; i < constraint_rows.size(); ++i) {
   btSolverConstraint constraint_row = constraint_rows.get(i);
   result[i] = new LowerLimitPointer(constraint_row);
  }
  return result;
 }

 private FloatSmartPointer[] upperLimit_array(ArrayList<btSolverConstraint> constraint_rows) {
  FloatSmartPointer[] result = new FloatSmartPointer[constraint_rows.size()];
  for (int i = 0; i < constraint_rows.size(); ++i) {
   btSolverConstraint constraint_row = constraint_rows.get(i);
   result[i] = new UpperLimitPointer(constraint_row);
  }
  return result;
 }

 private static class btSingleConstraintRowSolverImpl implements btSingleConstraintRowSolver,
  Serializable {

  private static final long serialVersionUID = 1L;

  public btSingleConstraintRowSolverImpl() {
  }

  @Override
  public float solve(btSolverBody body0, btSolverBody body1, btSolverConstraint constraint) {
   return gResolveSingleConstraintRowLowerLimit_scalar_reference(body1, body1, constraint);
  }
 }

 private static class btSingleConstraintRowSolverImpl1 implements btSingleConstraintRowSolver,
  Serializable {

  public btSingleConstraintRowSolverImpl1() {
  }

  @Override
  public float solve(btSolverBody body0, btSolverBody body1, btSolverConstraint constraint) {
   return gResolveSingleConstraintRowLowerLimit_scalar_reference(body1, body1, constraint);
  }
 }

 private static class btSingleConstraintRowSolverImpl2 implements btSingleConstraintRowSolver,
  Serializable {

  public btSingleConstraintRowSolverImpl2() {
  }

  @Override
  public float solve(btSolverBody body0, btSolverBody body1, btSolverConstraint constraint) {
   return gResolveSingleConstraintRowGeneric_scalar_reference(body1, body1, constraint);
  }
 }

 private static class btSingleConstraintRowSolverImpl3 implements btSingleConstraintRowSolver,
  Serializable {

  private static final long serialVersionUID = 1L;

  public btSingleConstraintRowSolverImpl3() {
  }

  @Override
  public float solve(btSolverBody body0, btSolverBody body1, btSolverConstraint constraint) {
   return gResolveSingleConstraintRowGeneric_scalar_reference(body1, body1, constraint);
  }
 }

 private static class btSingleConstraintRowSolverImpl4 implements btSingleConstraintRowSolver,
  Serializable {

  private static final long serialVersionUID = 1L;

  public btSingleConstraintRowSolverImpl4() {
  }

  @Override
  public float solve(btSolverBody body0, btSolverBody body1, btSolverConstraint constraint) {
   return gResolveSingleConstraintRowGeneric_scalar_reference(body1, body1, constraint);
  }
 }

 private static class btSingleConstraintRowSolverImpl5 implements btSingleConstraintRowSolver,
  Serializable {

  private static final long serialVersionUID = 1L;

  public btSingleConstraintRowSolverImpl5() {
  }

  @Override
  public float solve(btSolverBody body0, btSolverBody body1, btSolverConstraint constraint) {
   return gResolveSingleConstraintRowLowerLimit_scalar_reference(body1, body1, constraint);
  }
 }

 void shuffle_list(List list) {
  for (int j = 0; j < list.size(); ++j) {
   int swapi = btRandInt2(j + 1);
   Collections.swap(list, j, swapi);
  }
 }
}
