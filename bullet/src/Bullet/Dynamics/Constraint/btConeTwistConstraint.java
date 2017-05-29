/*
Bullet Continuous Collision Detection and Physics Library
btConeTwistConstraint is Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marcus Hennix
 */
package Bullet.Dynamics.Constraint;

import Bullet.Dynamics.btJacobianEntry;
import Bullet.Dynamics.btRigidBody;
import static Bullet.Extras.btMinMax.btMax;
import Bullet.LinearMath.btMatrix3x3;
import Bullet.LinearMath.btQuaternion;
import static Bullet.LinearMath.btQuaternion.quatRotate;
import static Bullet.LinearMath.btQuaternion.shortestArcQuat;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.SIMD_INFINITY;
import static Bullet.LinearMath.btScalar.SIMD_PI;
import static Bullet.LinearMath.btScalar.btAtan2;
import static Bullet.LinearMath.btScalar.btAtan2Fast;
import static Bullet.LinearMath.btScalar.btCos;
import static Bullet.LinearMath.btScalar.btFabs;
import static Bullet.LinearMath.btScalar.btFuzzyZero;
import static Bullet.LinearMath.btScalar.btSin;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btTransformUtil;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.btPlaneSpace1;
import java.io.Serializable;
import javax.vecmath.AxisAngle4f;

/**
 *
 *
 * Overview:
 *
 * btConeTwistConstraint can be used to simulate ragdoll joints (upper arm, leg etc). It is a fixed
 * translation, 3 degree-of-freedom (DOF) rotational "joint". It divides the 3 rotational DOFs into
 * swing (movement within a cone) and twist. Swing is divided into swing1 and swing2 which can have
 * different limits, giving an elliptical shape. (Note: the cone's base isn't flat, so this ellipse
 * is "embedded" on the surface of a sphere.)
 *
 * In the contraint's frame of reference: twist is along the x-axis, and swing 1 and 2 are along the
 * z and y axes respectively.
 *
 * btConeTwistConstraint can be used to simulate ragdoll joints (upper arm, leg etc)
 *
 * @author Gregery Barton
 */
public class btConeTwistConstraint extends btTypedConstraint implements Serializable {
 //enum btConeTwistFlags

 static final boolean CONETWIST_USE_OBSOLETE_SOLVER = false;
 public static final int BT_CONETWIST_FLAGS_LIN_CFM = 1;
 public static final int BT_CONETWIST_FLAGS_LIN_ERP = 2;
 public static final int BT_CONETWIST_FLAGS_ANG_CFM = 4;
 final btJacobianEntry[] m_jac = new btJacobianEntry[3]; //3 orthogonal linear constraints
 final btTransform m_rbAFrame = new btTransform();
 final btTransform m_rbBFrame = new btTransform();
 float m_limitSoftness;
 float m_biasFactor;
 float m_relaxationFactor;
 float m_damping;
 float m_swingSpan1;
 float m_swingSpan2;
 float m_twistSpan;
 float m_fixThresh;
 final btVector3 m_swingAxis = new btVector3();
 final btVector3 m_twistAxis = new btVector3();
 float m_kSwing;
 float m_kTwist;
 float m_twistLimitSign;
 float m_swingCorrection;
 float m_twistCorrection;
 float m_twistAngle;
 float m_accSwingLimitImpulse;
 float m_accTwistLimitImpulse;
 boolean m_angularOnly;
 boolean m_solveTwistLimit;
 boolean m_solveSwingLimit;
 boolean m_useSolveConstraintObsolete;
 // not yet used...
 float m_swingLimitRatio;
 float m_twistLimitRatio;
 final btVector3 m_twistAxisA = new btVector3();
 // motor
 boolean m_bMotorEnabled;
 boolean m_bNormalizedMotorStrength;
 final btQuaternion m_qTarget = new btQuaternion();
 float m_maxMotorImpulse;
 final btVector3 m_accMotorImpulse = new btVector3();
 // parameters
 int m_flags;
 float m_linCFM;
 float m_linERP;
 float m_angCFM;
 static final float CONETWIST_DEF_FIX_THRESH = (0.05f);

 private void init() {
  m_angularOnly = false;
  m_solveTwistLimit = false;
  m_solveSwingLimit = false;
  m_bMotorEnabled = false;
  m_maxMotorImpulse = (-1f);
  setLimit((BT_LARGE_FLOAT), (BT_LARGE_FLOAT), (BT_LARGE_FLOAT));
  m_damping = (0.01f);
  m_fixThresh = CONETWIST_DEF_FIX_THRESH;
  m_flags = 0;
  m_linCFM = (0.f);
  m_linERP = (0.7f);
  m_angCFM = (0.f);
 }

 void computeConeLimitInfo(final btQuaternion qCone, // in
  float[] swingAngle, final btVector3 vSwingAxis, float[] swingLimit) // all outs
 {
  swingAngle[0] = qCone.getAngle();
  if (swingAngle[0] > SIMD_EPSILON) {
   vSwingAxis.set(qCone.x(), qCone.y(), qCone.z());
   vSwingAxis.normalize();
   // Compute limit for given swing. tricky:
   // Given a swing axis, we're looking for the intersection with the bounding cone ellipse.
   // (Since we're dealing with angles, this ellipse is embedded on the surface of a sphere.)
   // For starters, compute the direction from center to surface of ellipse.
   // This is just the perpendicular (ie. rotate 2D vector by PI/2) of the swing axis.
   // (vSwingAxis is the cone rotation (in z,y); change vars and rotate to (x,y) coords.)
   float xEllipse = vSwingAxis.y();
   float yEllipse = -vSwingAxis.z();
   // Now, we use the slope of the vector (using x/yEllipse) and find the length
   // of the line that intersects the ellipse:
   //  x^2   y^2
   //  --- + --- = 1, where a and b are semi-major axes 2 and 1 respectively (ie. the limits)
   //  a^2   b^2
   // Do the math and it should be clear.
   swingLimit[0] = m_swingSpan1; // if xEllipse == 0, we have a pure vSwingAxis.z rotation: just use swingspan1
   if (btFabs(xEllipse) > SIMD_EPSILON) {
    float surfaceSlope2 = (yEllipse * yEllipse) / (xEllipse * xEllipse);
    float norm = 1f / (m_swingSpan2 * m_swingSpan2);
    norm += surfaceSlope2 / (m_swingSpan1 * m_swingSpan1);
    float swingLimit2 = (1f + surfaceSlope2) / norm;
    swingLimit[0] = btSqrt(swingLimit2);
   }
  } else if (swingAngle[0] < 0) {
   // this should never happen!
  }
 }

 void computeTwistLimitInfo(final btQuaternion qTwist, // in
  float[] twistAngle, final btVector3 vTwistAxis) // all outs
 {
  final btQuaternion qMinTwist = new btQuaternion(qTwist);
  twistAngle[0] = qTwist.getAngle();
  if (twistAngle[0] > SIMD_PI) // long way around. flip quat and recalculate.
  {
   qMinTwist.set(qTwist).negate();
   twistAngle[0] = qMinTwist.getAngle();
  }
  if (twistAngle[0] < 0) {
   // this should never happen
  }
  vTwistAxis.set(qMinTwist.x(), qMinTwist.y(), qMinTwist.z());
  if (twistAngle[0] > SIMD_EPSILON) {
   vTwistAxis.normalize();
  }
 }

 void adjustSwingAxisToUseEllipseNormal(final btVector3 vSwingAxis) {
  // the swing axis is computed as the "twist-free" cone rotation,
  // but the cone limit is not circular, but elliptical (if swingspan1 != swingspan2).
  // so, if we're outside the limits, the closest way back inside the cone isn't 
  // along the vector back to the center. better (and more stable) to use the ellipse normal.
  // convert swing axis to direction from center to surface of ellipse
  // (ie. rotate 2D vector by PI/2)
  float y = -vSwingAxis.z();
  float z = vSwingAxis.y();
  // do the math...
  if (btFabs(z) > SIMD_EPSILON) // avoid division by 0. and we don't need an update if z == 0.
  {
   // compute gradient/normal of ellipse surface at current "point"
   float grad = y / z;
   grad *= m_swingSpan2 / m_swingSpan1;
   // adjust y/z to represent normal at point (instead of vector to point)
   if (y > 0) {
    y = btFabs(grad * z);
   } else {
    y = -btFabs(grad * z);
   }
   // convert ellipse direction back to swing axis
   vSwingAxis.setZ(-y);
   vSwingAxis.setY(z);
   vSwingAxis.normalize();
  }
 }

 public btConeTwistConstraint(btRigidBody rbA, btRigidBody rbB, final btTransform rbAFrame,
  final btTransform rbBFrame) {
  super(CONETWIST_CONSTRAINT_TYPE, rbA, rbB);
  m_rbAFrame.set(rbAFrame);
  m_rbBFrame.set(rbBFrame);
  m_angularOnly = false;
  m_useSolveConstraintObsolete = CONETWIST_USE_OBSOLETE_SOLVER;
  init();
 }

 public btConeTwistConstraint(btRigidBody rbA, final btTransform rbAFrame) {
  super(CONETWIST_CONSTRAINT_TYPE, rbA);
  m_rbAFrame.set(rbAFrame);
  m_angularOnly = false;
  m_useSolveConstraintObsolete = CONETWIST_USE_OBSOLETE_SOLVER;
  m_rbBFrame.set(m_rbAFrame);
  m_rbBFrame.setOrigin(new btVector3());
  init();
 }

 @Override
 public void buildJacobian() {
  if (m_useSolveConstraintObsolete) {
   m_appliedImpulse = 0.0f;
   m_accTwistLimitImpulse = (0.f);
   m_accSwingLimitImpulse = (0.f);
   m_accMotorImpulse.setZero();
   if (!m_angularOnly) {
    final btVector3 pivotAInW = m_rbA.getCenterOfMassTransform().transform(m_rbAFrame.getOrigin());
    final btVector3 pivotBInW = m_rbB.getCenterOfMassTransform().transform(m_rbBFrame.getOrigin());
    final btVector3 relPos = new btVector3(pivotBInW).sub(pivotAInW);
    btVector3[] normal = new btVector3[3];
    btVector3.init(normal);
    if (relPos.lengthSquared() > SIMD_EPSILON) {
     normal[0].set(relPos).normalize();
    } else {
     normal[0].set((1.0f), 0f, 0f);
    }
    btPlaneSpace1(normal[0], normal[1], normal[2]);
    for (int i = 0; i < 3; i++) {
     m_jac[i] =
      new btJacobianEntry(
       m_rbA.getCenterOfMassTransform().getBasis().transpose(),
       m_rbB.getCenterOfMassTransform().getBasis().transpose(),
       new btVector3(pivotAInW).sub(m_rbA.getCenterOfMassPosition()),
       new btVector3(pivotBInW).sub(m_rbB.getCenterOfMassPosition()),
       normal[i],
       m_rbA.getInvInertiaDiagLocal(),
       m_rbA.getInvMass(),
       m_rbB.getInvInertiaDiagLocal(),
       m_rbB.getInvMass());
    }
   }
   calcAngleInfo2(m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform(), m_rbA
    .getInvInertiaTensorWorld(), m_rbB.getInvInertiaTensorWorld());
  }
 }

 @Override
 public void getInfo1(btConstraintInfo1 info) {
  if (m_useSolveConstraintObsolete) {
   info.m_numConstraintRows = 0;
   info.nub = 0;
  } else {
   info.m_numConstraintRows = 3;
   info.nub = 3;
   calcAngleInfo2(m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform(), m_rbA
    .getInvInertiaTensorWorld(), m_rbB.getInvInertiaTensorWorld());
   if (m_solveSwingLimit) {
    info.m_numConstraintRows++;
    info.nub--;
    if ((m_swingSpan1 < m_fixThresh) && (m_swingSpan2 < m_fixThresh)) {
     info.m_numConstraintRows++;
     info.nub--;
    }
   }
   if (m_solveTwistLimit) {
    info.m_numConstraintRows++;
    info.nub--;
   }
  }
 }

 void getInfo1NonVirtual(btConstraintInfo1 info) {
  //always reserve 6 rows: object transform is not available on SPU
  info.m_numConstraintRows = 6;
  info.nub = 0;
 }

 @Override
 public void getInfo2(btConstraintInfo2 info) {
  getInfo2NonVirtual(info, m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform(), m_rbA
   .getInvInertiaTensorWorld(), m_rbB.getInvInertiaTensorWorld());
 }

 final void getInfo2NonVirtual(btConstraintInfo2 info, final btTransform transA,
  final btTransform transB,
  final btMatrix3x3 invInertiaWorldA, final btMatrix3x3 invInertiaWorldB) {
  calcAngleInfo2(transA, transB, invInertiaWorldA, invInertiaWorldB);
  assert (!m_useSolveConstraintObsolete);
  // set jacobian
  info.m_J1linearAxis[0].x = 1;
  info.m_J1linearAxis[info.rowskip].y = 1;
  info.m_J1linearAxis[2 * info.rowskip].z = 1;
  final btVector3 a1 = transA.transform3x3(m_rbAFrame.getOrigin());
  {
   final btVector3 angular0 = (info.m_J1angularAxis[0]);
   final btVector3 angular1 = (info.m_J1angularAxis[info.rowskip]);
   final btVector3 angular2 = (info.m_J1angularAxis[2 * info.rowskip]);
   final btVector3 a1neg = new btVector3(a1).negate();
   a1neg.getSkewSymmetricMatrix(angular0, angular1, angular2);
  }
  info.m_J2linearAxis[0].x = -1;
  info.m_J2linearAxis[info.rowskip].y = -1;
  info.m_J2linearAxis[2 * info.rowskip].z = -1;
  final btVector3 a2 = transB.transform3x3(m_rbBFrame.getOrigin());
  {
   final btVector3 angular0 = (info.m_J2angularAxis[0]);
   final btVector3 angular1 = (info.m_J2angularAxis[info.rowskip]);
   final btVector3 angular2 = (info.m_J2angularAxis[2 * info.rowskip]);
   a2.getSkewSymmetricMatrix(angular0, angular1, angular2);
  }
  // set right hand side
  float linERP = ((m_flags & BT_CONETWIST_FLAGS_LIN_ERP) != 0) ? m_linERP : info.erp;
  float k = info.fps * linERP;
  int j;
  for (j = 0; j < 3; j++) {
   info.m_constraintError[j * info.rowskip].set(k * (a2.getElement(j) + transB.getOrigin()
    .getElement(j) - a1.getElement(j) - transA.getOrigin().getElement(j)));
   info.m_lowerLimit[j * info.rowskip].set(-SIMD_INFINITY);
   info.m_upperLimit[j * info.rowskip].set(SIMD_INFINITY);
   if ((m_flags & BT_CONETWIST_FLAGS_LIN_CFM) != 0) {
    info.cfm[j * info.rowskip].set(m_linCFM);
   }
  }
  int row = 3;
  int srow = row * info.rowskip;
  final btVector3 ax1 = new btVector3();
  // angular limits
  if (m_solveSwingLimit) {
   btVector3[] J1 = info.m_J1angularAxis;
   btVector3[] J2 = info.m_J2angularAxis;
   if ((m_swingSpan1 < m_fixThresh) && (m_swingSpan2 < m_fixThresh)) {
    final btTransform trA = new btTransform(transA).mul(m_rbAFrame);
    final btVector3 p = trA.getBasisColumn(1);
    final btVector3 q = trA.getBasisColumn(2);
    int srow1 = srow + info.rowskip;
    J1[srow].set(p);
    J1[srow1].set(q);
    J2[srow].set(p).negate();
    J2[srow1].set(q).negate();
    float fact = info.fps * m_relaxationFactor;
    info.m_constraintError[srow].set(fact * m_swingAxis.dot(p));
    info.m_constraintError[srow1].set(fact * m_swingAxis.dot(q));
    info.m_lowerLimit[srow].set(-SIMD_INFINITY);
    info.m_upperLimit[srow].set(SIMD_INFINITY);
    info.m_lowerLimit[srow1].set(-SIMD_INFINITY);
    info.m_upperLimit[srow1].set(SIMD_INFINITY);
    srow = srow1 + info.rowskip;
   } else {
    ax1.set(m_swingAxis).scale(m_relaxationFactor * m_relaxationFactor);
    J1[srow].set(ax1);
    J2[srow].set(ax1).negate();
    k = info.fps * m_biasFactor;
    info.m_constraintError[srow].set(k * m_swingCorrection);
    if ((m_flags & BT_CONETWIST_FLAGS_ANG_CFM) != 0) {
     info.cfm[srow].set(m_angCFM);
    }
    // m_swingCorrection is always positive or 0
    info.m_lowerLimit[srow].set(0);
    info.m_upperLimit[srow].set(
     (m_bMotorEnabled && m_maxMotorImpulse >= 0.0f) ? m_maxMotorImpulse : SIMD_INFINITY);
    srow += info.rowskip;
   }
  }
  if (m_solveTwistLimit) {
   ax1.set(m_twistAxis).scale(m_relaxationFactor * m_relaxationFactor);
   btVector3[] J1 = info.m_J1angularAxis;
   btVector3[] J2 = info.m_J2angularAxis;
   J1[srow].set(ax1);
   J2[srow].set(ax1).negate();
   k = info.fps * m_biasFactor;
   info.m_constraintError[srow].set(k * m_twistCorrection);
   if ((m_flags & BT_CONETWIST_FLAGS_ANG_CFM) != 0) {
    info.cfm[srow].set(m_angCFM);
   }
   if (m_twistSpan > 0.0f) {
    if (m_twistCorrection > 0.0f) {
     info.m_lowerLimit[srow].set(0);
     info.m_upperLimit[srow].set(SIMD_INFINITY);
    } else {
     info.m_lowerLimit[srow].set(-SIMD_INFINITY);
     info.m_upperLimit[srow].set(0);
    }
   } else {
    info.m_lowerLimit[srow].set(-SIMD_INFINITY);
    info.m_upperLimit[srow].set(SIMD_INFINITY);
   }
   srow += info.rowskip;
  }
 }
 static boolean bDoTorque = true;

 @Override
 public void solveConstraintObsolete(btSolverBody bodyA, btSolverBody bodyB, float timeStep) {
  if (m_useSolveConstraintObsolete) {
   final btVector3 pivotAInW = m_rbA.getCenterOfMassTransform().transform(m_rbAFrame.getOrigin());
   final btVector3 pivotBInW = m_rbB.getCenterOfMassTransform().transform(m_rbBFrame.getOrigin());
   float tau = (0.3f);
   //linear part
   if (!m_angularOnly) {
    final btVector3 rel_pos1 = new btVector3(pivotAInW).sub(m_rbA.getCenterOfMassPosition());
    final btVector3 rel_pos2 = new btVector3(pivotBInW).sub(m_rbB.getCenterOfMassPosition());
    final btVector3 vel1 = new btVector3();
    bodyA.internalGetVelocityInLocalPointObsolete(rel_pos1, vel1);
    final btVector3 vel2 = new btVector3();
    bodyB.internalGetVelocityInLocalPointObsolete(rel_pos2, vel2);
    final btVector3 vel = new btVector3(vel1).sub(vel2);
    for (int i = 0; i < 3; i++) {
     final btVector3 normal = m_jac[i].m_linearJointAxis;
     float jacDiagABInv = (1.f) / m_jac[i].getDiagonal();
     float rel_vel;
     rel_vel = normal.dot(vel);
     //positional error (zeroth order error)
     float depth = -(new btVector3(pivotAInW).sub(pivotBInW)).dot(normal); //this is the error projected on the normal
     float impulse = depth * tau / timeStep * jacDiagABInv - rel_vel * jacDiagABInv;
     m_appliedImpulse += impulse;
     final btVector3 ftorqueAxis1 = new btVector3(rel_pos1).cross(normal);
     final btVector3 ftorqueAxis2 = new btVector3(rel_pos2).cross(normal);
     bodyA.internalApplyImpulse(new btVector3(normal).scale(m_rbA.getInvMass()), m_rbA
      .getInvInertiaTensorWorld().transform(new btVector3(ftorqueAxis1)), impulse);
     bodyB.internalApplyImpulse(new btVector3(normal).scale(m_rbB.getInvMass()), m_rbB
      .getInvInertiaTensorWorld().transform(new btVector3(ftorqueAxis2)), -impulse);
    }
   }
   // apply motor
   if (m_bMotorEnabled) {
    // compute current and predicted transforms
    final btTransform trACur = m_rbA.getCenterOfMassTransform();
    final btTransform trBCur = m_rbB.getCenterOfMassTransform();
    final btVector3 omegaA = new btVector3();
    bodyA.internalGetAngularVelocity(omegaA);
    final btVector3 omegaB = new btVector3();
    bodyB.internalGetAngularVelocity(omegaB);
    final btTransform trAPred = new btTransform();
    trAPred.setIdentity();
    final btVector3 zerovec = new btVector3();
    btTransformUtil.integrateTransform(
     trACur, zerovec, omegaA, timeStep, trAPred);
    final btTransform trBPred = new btTransform();
    trBPred.setIdentity();
    btTransformUtil.integrateTransform(
     trBCur, zerovec, omegaB, timeStep, trBPred);
    // compute desired transforms in world
    final btTransform trPose = new btTransform().set(m_qTarget);
    final btTransform trABDes = new btTransform(m_rbBFrame).mul(trPose).mul(new btTransform(
     m_rbAFrame).invert());
    final btTransform trADes = new btTransform(trBPred).mul(trABDes);
    final btTransform trBDes = new btTransform(trAPred).mul(new btTransform(trABDes).invert());
    // compute desired omegas in world
    final btVector3 omegaADes = new btVector3();
    final btVector3 omegaBDes = new btVector3();
    btTransformUtil.calculateVelocity(trACur, trADes, timeStep, zerovec, omegaADes);
    btTransformUtil.calculateVelocity(trBCur, trBDes, timeStep, zerovec, omegaBDes);
    // compute delta omegas
    final btVector3 dOmegaA = new btVector3(omegaADes).sub(omegaA);
    final btVector3 dOmegaB = new btVector3(omegaBDes).sub(omegaB);
    // compute weighted avg axis of dOmega (weighting based on inertias)
    final btVector3 axisA = new btVector3();
    final btVector3 axisB = new btVector3();
    float kAxisAInv = 0, kAxisBInv = 0;
    if (dOmegaA.lengthSquared() > SIMD_EPSILON) {
     axisA.set(dOmegaA).normalize();
     kAxisAInv = getRigidBodyA().computeAngularImpulseDenominator(axisA);
    }
    if (dOmegaB.lengthSquared() > SIMD_EPSILON) {
     axisB.set(dOmegaB).normalize();
     kAxisBInv = getRigidBodyB().computeAngularImpulseDenominator(axisB);
    }
    final btVector3 avgAxis = new btVector3(axisA).scale(kAxisAInv).mul(new btVector3(axisB).scale(
     kAxisBInv));
    if (bDoTorque && avgAxis.lengthSquared() > SIMD_EPSILON) {
     avgAxis.normalize();
     kAxisAInv = getRigidBodyA().computeAngularImpulseDenominator(avgAxis);
     kAxisBInv = getRigidBodyB().computeAngularImpulseDenominator(avgAxis);
     float kInvCombined = kAxisAInv + kAxisBInv;
     final btVector3 impulse = (new btVector3(dOmegaA).scale(kAxisAInv).sub(new btVector3(dOmegaB)
      .scale(kAxisBInv))).scale(1.0f / (kInvCombined * kInvCombined));
     if (m_maxMotorImpulse >= 0) {
      float fMaxImpulse = m_maxMotorImpulse;
      if (m_bNormalizedMotorStrength) {
       fMaxImpulse /= kAxisAInv;
      }
      final btVector3 newUnclampedAccImpulse = new btVector3(m_accMotorImpulse).add(impulse);
      float newUnclampedMag = newUnclampedAccImpulse.length();
      if (newUnclampedMag > fMaxImpulse) {
       newUnclampedAccImpulse.normalize();
       newUnclampedAccImpulse.scale(fMaxImpulse);
       impulse.set(newUnclampedAccImpulse).sub(m_accMotorImpulse);
      }
      m_accMotorImpulse.add(impulse);
     }
     float impulseMag = impulse.length();
     final btVector3 impulseAxis = new btVector3(impulse).scale(1.0f / impulseMag);
     bodyA.internalApplyImpulse(new btVector3(), m_rbA.getInvInertiaTensorWorld().transform(
      new btVector3(impulseAxis)), impulseMag);
     bodyB.internalApplyImpulse(new btVector3(), m_rbB.getInvInertiaTensorWorld().transform(
      new btVector3(impulseAxis)), -impulseMag);
    }
   } else if (m_damping > SIMD_EPSILON) // no motor: do a little damping
   {
    final btVector3 angVelA = new btVector3();
    bodyA.internalGetAngularVelocity(angVelA);
    final btVector3 angVelB = new btVector3();
    bodyB.internalGetAngularVelocity(angVelB);
    final btVector3 relVel = new btVector3(angVelB).sub(angVelA);
    if (relVel.lengthSquared() > SIMD_EPSILON) {
     final btVector3 relVelAxis = new btVector3(relVel).normalize();
     float m_kDamping = (1.f) / (getRigidBodyA().computeAngularImpulseDenominator(relVelAxis) +
      getRigidBodyB().computeAngularImpulseDenominator(relVelAxis));
     final btVector3 impulse = new btVector3(relVel).scale(m_damping * m_kDamping);
     float impulseMag = impulse.length();
     final btVector3 impulseAxis = new btVector3(impulse).scale(1.0f / impulseMag);
     bodyA.internalApplyImpulse(new btVector3(), m_rbA.getInvInertiaTensorWorld().transform(
      new btVector3(impulseAxis)), impulseMag);
     bodyB.internalApplyImpulse(new btVector3(), m_rbB.getInvInertiaTensorWorld().transform(
      new btVector3(impulseAxis)), -impulseMag);
    }
   }
   // joint limits
   {
    ///solve angular part
    final btVector3 angVelA = new btVector3();
    bodyA.internalGetAngularVelocity(angVelA);
    final btVector3 angVelB = new btVector3();
    bodyB.internalGetAngularVelocity(angVelB);
    // solve swing limit
    if (m_solveSwingLimit) {
     float amplitude = m_swingLimitRatio * m_swingCorrection * m_biasFactor / timeStep;
     float relSwingVel = (new btVector3(angVelB).sub(angVelA)).dot(m_swingAxis);
     if (relSwingVel > 0) {
      amplitude += m_swingLimitRatio * relSwingVel * m_relaxationFactor;
     }
     float impulseMag = amplitude * m_kSwing;
     // Clamp the accumulated impulse
     float temp = m_accSwingLimitImpulse;
     m_accSwingLimitImpulse = btMax(m_accSwingLimitImpulse + impulseMag, (0.0f));
     impulseMag = m_accSwingLimitImpulse - temp;
     final btVector3 impulse = new btVector3(m_swingAxis).scale(impulseMag);
     // don't let cone response affect twist
     // (this can happen since body A's twist doesn't match body B's AND we use an elliptical cone limit)
     {
      final btVector3 impulseTwistCouple = new btVector3(m_twistAxisA).scale(impulse.dot(
       m_twistAxisA));
      final btVector3 impulseNoTwistCouple = new btVector3(impulse).sub(impulseTwistCouple);
      impulse.set(impulseNoTwistCouple);
     }
     impulseMag = impulse.length();
     final btVector3 noTwistSwingAxis = new btVector3(impulse).scale(1.0f / impulseMag);
     bodyA.internalApplyImpulse(new btVector3(), m_rbA.getInvInertiaTensorWorld().transform(
      new btVector3(noTwistSwingAxis)), impulseMag);
     bodyB.internalApplyImpulse(new btVector3(), m_rbB.getInvInertiaTensorWorld().transform(
      new btVector3(noTwistSwingAxis)), -impulseMag);
    }
    // solve twist limit
    if (m_solveTwistLimit) {
     float amplitude = m_twistLimitRatio * m_twistCorrection * m_biasFactor / timeStep;
     float relTwistVel = (new btVector3(angVelB).sub(angVelA)).dot(m_twistAxis);
     if (relTwistVel > 0) // only damp when moving towards limit (m_twistAxis flipping is important)
     {
      amplitude += m_twistLimitRatio * relTwistVel * m_relaxationFactor;
     }
     float impulseMag = amplitude * m_kTwist;
     // Clamp the accumulated impulse
     float temp = m_accTwistLimitImpulse;
     m_accTwistLimitImpulse = btMax(m_accTwistLimitImpulse + impulseMag, (0.0f));
     impulseMag = m_accTwistLimitImpulse - temp;
     //		btVector3 impulse = m_twistAxis * impulseMag;
     bodyA.internalApplyImpulse(new btVector3(), m_rbA.getInvInertiaTensorWorld().transform(
      new btVector3(m_twistAxis)), impulseMag);
     bodyB.internalApplyImpulse(new btVector3(), m_rbB.getInvInertiaTensorWorld().transform(
      new btVector3(m_twistAxis)), -impulseMag);
    }
   }
  }
 }

 void updateRHS(float timeStep) {
 }

 @Override
 public btRigidBody getRigidBodyA() {
  return m_rbA;
 }

 @Override
 public btRigidBody getRigidBodyB() {
  return m_rbB;
 }

 public void setAngularOnly(boolean angularOnly) {
  m_angularOnly = angularOnly;
 }

 boolean getAngularOnly() {
  return m_angularOnly;
 }

 public void setLimit(int limitIndex, float limitValue) {
  switch (limitIndex) {
   case 3: {
    m_twistSpan = limitValue;
    break;
   }
   case 4: {
    m_swingSpan2 = limitValue;
    break;
   }
   case 5: {
    m_swingSpan1 = limitValue;
    break;
   }
   default: {
   }
  }
 }

 float getLimit(int limitIndex) {
  switch (limitIndex) {
   case 3: {
    return m_twistSpan;
   }
   case 4: {
    return m_swingSpan2;
   }
   case 5: {
    return m_swingSpan1;
   }
   default: {
    assert (false);// "Invalid limitIndex specified for btConeTwistConstraint");
    return 0.0f;
   }
  }
 }

 // setLimit(), a few notes:
 // _softness:
 //		0.1, recommend ~0.8.1.
 //		describes % of limits where movement is free.
 //		beyond this softness %, the limit is gradually enforced until the "hard" (1.0) limit is reached.
 // _biasFactor:
 //		0.1?, recommend 0.3 +/-0.3 or so.
 //		strength with which constraint resists zeroth order (angular, not angular velocity) limit violation.
 // __relaxationFactor:
 //		0.1, recommend to stay near 1.
 //		the lower the value, the less the constraint will fight velocities which violate the angular limits.
 public void setLimit(float _swingSpan1, float _swingSpan2, float _twistSpan) {
  setLimit(_swingSpan1, _swingSpan2, _twistSpan, 1.0f);
 }

 public void setLimit(float _swingSpan1, float _swingSpan2, float _twistSpan, float _softness) {
  setLimit(_swingSpan1, _swingSpan2, _twistSpan, _softness, 0.3f);
 }

 public void setLimit(float _swingSpan1, float _swingSpan2, float _twistSpan, float _softness,
  float _biasFactor) {
  setLimit(_swingSpan1, _swingSpan2, _twistSpan, _softness, _biasFactor, 1.0f);
 }

 public void setLimit(float _swingSpan1, float _swingSpan2, float _twistSpan, float _softness,
  float _biasFactor, float _relaxationFactor) {
  m_swingSpan1 = _swingSpan1;
  m_swingSpan2 = _swingSpan2;
  m_twistSpan = _twistSpan;
  m_limitSoftness = _softness;
  m_biasFactor = _biasFactor;
  m_relaxationFactor = _relaxationFactor;
 }

 public btTransform getAFrame() {
  return new btTransform(m_rbAFrame);
 }

 public btTransform getBFrame() {
  return new btTransform(m_rbBFrame);
 }

 ;

 int getSolveTwistLimit() {
  return m_solveTwistLimit ? 1 : 0;
 }

 int getSolveSwingLimit() {
  return m_solveSwingLimit ? 1 : 0;
 }

 public float getTwistLimitSign() {
  return m_twistLimitSign;
 }

 void calcAngleInfo() {
  m_swingCorrection = (0.f);
  m_twistLimitSign = (0.f);
  m_solveTwistLimit = false;
  m_solveSwingLimit = false;
  final btVector3 b1Axis1 = new btVector3();
  final btVector3 b1Axis2 = new btVector3();
  final btVector3 b1Axis3 = new btVector3();
  final btVector3 b2Axis1 = new btVector3();
  final btVector3 b2Axis2 = new btVector3();
  b1Axis1.set(getRigidBodyA().getCenterOfMassTransform().transform3x3(m_rbAFrame.getBasisColumn(0)));
  b2Axis1.set(getRigidBodyB().getCenterOfMassTransform().transform3x3(m_rbBFrame.getBasisColumn(0)));
  float swing1 = (0.f), swing2 = (0.f);
  float swx = (0.f), swy = (0.f);
  float thresh = (10.f);
  float fact;
  // Get Frame into world space
  if (m_swingSpan1 >= (0.05f)) {
   b1Axis2
    .set(getRigidBodyA().getCenterOfMassTransform().transform3x3(m_rbAFrame.getBasisColumn(1)));
   swx = b2Axis1.dot(b1Axis1);
   swy = b2Axis1.dot(b1Axis2);
   swing1 = btAtan2Fast(swy, swx);
   fact = (swy * swy + swx * swx) * thresh * thresh;
   fact /= (fact + (1.0f));
   swing1 *= fact;
  }
  if (m_swingSpan2 >= (0.05f)) {
   b1Axis3
    .set(getRigidBodyA().getCenterOfMassTransform().transform3x3(m_rbAFrame.getBasisColumn(2)));
   swx = b2Axis1.dot(b1Axis1);
   swy = b2Axis1.dot(b1Axis3);
   swing2 = btAtan2Fast(swy, swx);
   fact = (swy * swy + swx * swx) * thresh * thresh;
   fact /= (fact + (1.0f));
   swing2 *= fact;
  }
  float RMaxAngle1Sq = 1.0f / (m_swingSpan1 * m_swingSpan1);
  float RMaxAngle2Sq = 1.0f / (m_swingSpan2 * m_swingSpan2);
  float EllipseAngle = btFabs(swing1 * swing1) * RMaxAngle1Sq + btFabs(swing2 * swing2) *
   RMaxAngle2Sq;
  if (EllipseAngle > 1.0f) {
   m_swingCorrection = EllipseAngle - 1.0f;
   m_solveSwingLimit = true;
   // Calculate necessary axis & factors
   m_swingAxis
    .set(b2Axis1)
    .cross(new btVector3(b1Axis2)
     .scale(b2Axis1.dot(b1Axis2))
     .add(new btVector3(b1Axis3)
      .scale(b2Axis1.dot(b1Axis3))));
   m_swingAxis.normalize();
   float swingAxisSign = (b2Axis1.dot(b1Axis1) >= 0.0f) ? 1.0f : -1.0f;
   m_swingAxis.scale(swingAxisSign);
  }
  // Twist limits
  if (m_twistSpan >= (0.f)) {
   b2Axis2
    .set(getRigidBodyB().getCenterOfMassTransform().transform3x3(m_rbBFrame.getBasisColumn(1)));
   final btQuaternion rotationArc = shortestArcQuat(b2Axis1, b1Axis1);
   final btVector3 TwistRef = quatRotate(rotationArc, b2Axis2);
   float twist = btAtan2Fast(TwistRef.dot(b1Axis3), TwistRef.dot(b1Axis2));
   m_twistAngle = twist;
   float lockedFreeFactor = (m_twistSpan > (0.05f)) ? (1.0f) : (0.f);
   if (twist <= -m_twistSpan * lockedFreeFactor) {
    m_twistCorrection = -(twist + m_twistSpan);
    m_solveTwistLimit = true;
    m_twistAxis.set(b2Axis1).add(b1Axis1).scale(0.5f);
    m_twistAxis.normalize();
    m_twistAxis.negate();
   } else if (twist > m_twistSpan * lockedFreeFactor) {
    m_twistCorrection = (twist - m_twistSpan);
    m_solveTwistLimit = true;
    m_twistAxis.set(b2Axis1).add(b1Axis1).scale(0.5f);
    m_twistAxis.normalize();
   }
  }
 }
 static final btVector3 vTwist = new btVector3(1, 0, 0); // twist axis in constraint's space

 void calcAngleInfo2(final btTransform transA, final btTransform transB,
  final btMatrix3x3 invInertiaWorldA, final btMatrix3x3 invInertiaWorldB) {
  m_swingCorrection = (0.f);
  m_twistLimitSign = (0.f);
  m_solveTwistLimit = false;
  m_solveSwingLimit = false;
  // compute rotation of A wrt B (in constraint space)
  if (m_bMotorEnabled && (!m_useSolveConstraintObsolete)) {	// it is assumed that setMotorTarget() was alredy called 
   // and motor target m_qTarget is within constraint limits
   // TODO : split rotation to pure swing and pure twist
   // compute desired transforms in world
   final btTransform trPose = new btTransform().set(m_qTarget);
   final btTransform trA = new btTransform(transA).mul(m_rbAFrame);
   final btTransform trB = new btTransform(transB).mul(m_rbBFrame);
   final btTransform trDeltaAB = new btTransform(trB).mul(trPose).mul(new btTransform(trA).invert());
   final btQuaternion qDeltaAB = new btQuaternion().set(trDeltaAB);
   final btVector3 swingAxis = new btVector3(qDeltaAB.x(), qDeltaAB.y(), qDeltaAB.z());
   float swingAxisLen2 = swingAxis.lengthSquared();
   if (btFuzzyZero(swingAxisLen2)) {
    return;
   }
   m_swingAxis.set(swingAxis);
   m_swingAxis.normalize();
   m_swingCorrection = qDeltaAB.getAngle();
   if (!btFuzzyZero(m_swingCorrection)) {
    m_solveSwingLimit = true;
   }
   return;
  }
  {
   // compute rotation of A wrt B (in constraint space)
   final btQuaternion qA = new btQuaternion().set(transA).mul(new btQuaternion().set(m_rbAFrame));
   final btQuaternion qB = new btQuaternion().set(transB).mul(new btQuaternion().set(m_rbBFrame));
   final btQuaternion qAB = new btQuaternion(qB).conjugate().mul(qA);
   // split rotation into cone and twist
   // (all this is done from B's perspective. Maybe I should be averaging axes...)
   final btVector3 vConeNoTwist = quatRotate(qAB, vTwist);
   vConeNoTwist.normalize();
   final btQuaternion qABCone = shortestArcQuat(vTwist, vConeNoTwist);
   qABCone.normalize();
   final btQuaternion qABTwist = new btQuaternion(qABCone).conjugate().mul(qAB);
   qABTwist.normalize();
   if (m_swingSpan1 >= m_fixThresh && m_swingSpan2 >= m_fixThresh) {
    float[] swingAngle = new float[1], swingLimit = new float[1];
    final btVector3 swingAxis = new btVector3();
    computeConeLimitInfo(qABCone, swingAngle, swingAxis, swingLimit);
    if (swingAngle[0] > swingLimit[0] * m_limitSoftness) {
     m_solveSwingLimit = true;
     // compute limit ratio: 0.1, where
     // 0 == beginning of soft limit
     // 1 == hard/real limit
     m_swingLimitRatio = 1.f;
     if (swingAngle[0] < swingLimit[0] && m_limitSoftness < 1.f - SIMD_EPSILON) {
      m_swingLimitRatio = (swingAngle[0] - swingLimit[0] * m_limitSoftness) / (swingLimit[0] -
       swingLimit[0] * m_limitSoftness);
     }
     // swing correction tries to get back to soft limit
     m_swingCorrection = swingAngle[0] - (swingLimit[0] * m_limitSoftness);
     // adjustment of swing axis (based on ellipse normal)
     adjustSwingAxisToUseEllipseNormal(swingAxis);
     // Calculate necessary axis & factors		
     m_swingAxis.set(quatRotate(qB, new btVector3(swingAxis).negate()));
     m_twistAxisA.setZero();
     m_kSwing = (1.f) / (computeAngularImpulseDenominator(m_swingAxis, invInertiaWorldA) +
      computeAngularImpulseDenominator(m_swingAxis, invInertiaWorldB));
    }
   } else {
    // you haven't set any limits;
    // or you're trying to set at least one of the swing limits too small. (if so, do you really want a conetwist constraint?)
    // anyway, we have either hinge or fixed joint
    final btVector3 ivA = transA.transform3x3(m_rbAFrame.getBasisColumn(0));
    final btVector3 jvA = transA.transform3x3(m_rbAFrame.getBasisColumn(1));
    final btVector3 kvA = transA.transform3x3(m_rbAFrame.getBasisColumn(2));
    final btVector3 ivB = transB.transform3x3(m_rbBFrame.getBasisColumn(0));
    final btVector3 target = new btVector3();
    float x = ivB.dot(ivA);
    float y = ivB.dot(jvA);
    float z = ivB.dot(kvA);
    if ((m_swingSpan1 < m_fixThresh) && (m_swingSpan2 < m_fixThresh)) { // fixed. We'll need to add one more row to constraint
     if ((!btFuzzyZero(y)) || (!(btFuzzyZero(z)))) {
      m_solveSwingLimit = true;
      m_swingAxis.set(ivB).negate().cross(ivA);
     }
    } else {
     if (m_swingSpan1 < m_fixThresh) {
      // hinge around Y axis
//					if(!(btFuzzyZero(y)))
      if ((!(btFuzzyZero(x))) || (!(btFuzzyZero(z)))) {
       m_solveSwingLimit = true;
       if (m_swingSpan2 >= m_fixThresh) {
        y = (0.f);
        float span2 = btAtan2(z, x);
        if (span2 > m_swingSpan2) {
         x = btCos(m_swingSpan2);
         z = btSin(m_swingSpan2);
        } else if (span2 < -m_swingSpan2) {
         x = btCos(m_swingSpan2);
         z = -btSin(m_swingSpan2);
        }
       }
      }
     } else if ((!(btFuzzyZero(x))) || (!(btFuzzyZero(y)))) {
// hinge around Z axis
      //					if(!btFuzzyZero(z))
      m_solveSwingLimit = true;
      if (m_swingSpan1 >= m_fixThresh) {
       z = (0.f);
       float span1 = btAtan2(y, x);
       if (span1 > m_swingSpan1) {
        x = btCos(m_swingSpan1);
        y = btSin(m_swingSpan1);
       } else if (span1 < -m_swingSpan1) {
        x = btCos(m_swingSpan1);
        y = -btSin(m_swingSpan1);
       }
      }
     }
     target.x = x * ivA.x + y * jvA.x + z * kvA.x;
     target.y = x * ivA.y + y * jvA.y + z * kvA.y;
     target.z = x * ivA.z + y * jvA.z + z * kvA.z;
     target.normalize();
     m_swingAxis.set(ivB).negate().cross(target);
     m_swingCorrection = m_swingAxis.length();
     if (!btFuzzyZero(m_swingCorrection)) {
      m_swingAxis.normalize();
     }
    }
   }
   if (m_twistSpan >= (0.f)) {
    final btVector3 twistAxis = new btVector3();
    {
     float[] twistAngle_out = new float[1];
     computeTwistLimitInfo(qABTwist, twistAngle_out, twistAxis);
     m_twistAngle = twistAngle_out[0];
    }
    if (m_twistAngle > m_twistSpan * m_limitSoftness) {
     m_solveTwistLimit = true;
     m_twistLimitRatio = 1.f;
     if (m_twistAngle < m_twistSpan && m_limitSoftness < 1.f - SIMD_EPSILON) {
      m_twistLimitRatio = (m_twistAngle - m_twistSpan * m_limitSoftness) / (m_twistSpan -
       m_twistSpan * m_limitSoftness);
     }
     // twist correction tries to get back to soft limit
     m_twistCorrection = m_twistAngle - (m_twistSpan * m_limitSoftness);
     m_twistAxis.set(quatRotate(qB, new btVector3(twistAxis).negate()));
     m_kTwist = (1.f) / (computeAngularImpulseDenominator(m_twistAxis, invInertiaWorldA) +
      computeAngularImpulseDenominator(m_twistAxis, invInertiaWorldB));
    }
    if (m_solveSwingLimit) {
     m_twistAxisA.set(quatRotate(qA, new btVector3(twistAxis).negate()));
    }
   } else {
    m_twistAngle = (0.f);
   }
  }
 }

 float getSwingSpan1() {
  return m_swingSpan1;
 }

 public float getSwingSpan2() {
  return m_swingSpan2;
 }

 public float getTwistSpan() {
  return m_twistSpan;
 }

 public float getLimitSoftness() {
  return m_limitSoftness;
 }

 float getBiasFactor() {
  return m_biasFactor;
 }

 float getRelaxationFactor() {
  return m_relaxationFactor;
 }

 public float getTwistAngle() {
  return m_twistAngle;
 }

 public boolean isPastSwingLimit() {
  return m_solveSwingLimit;
 }

 float getDamping() {
  return m_damping;
 }

 public void setDamping(float damping) {
  m_damping = damping;
 }

 void enableMotor(boolean b) {
  m_bMotorEnabled = b;
 }

 public boolean isMotorEnabled() {
  return m_bMotorEnabled;
 }

 float getMaxMotorImpulse() {
  return m_maxMotorImpulse;
 }

 public boolean isMaxMotorImpulseNormalized() {
  return m_bNormalizedMotorStrength;
 }

 public void setMaxMotorImpulse(float maxMotorImpulse) {
  m_maxMotorImpulse = maxMotorImpulse;
  m_bNormalizedMotorStrength = false;
 }

 public void setMaxMotorImpulseNormalized(float maxMotorImpulse) {
  m_maxMotorImpulse = maxMotorImpulse;
  m_bNormalizedMotorStrength = true;
 }

 float getFixThresh() {
  return m_fixThresh;
 }

 public void setFixThresh(float fixThresh) {
  m_fixThresh = fixThresh;
 }

 // setMotorTarget:
 // q: the desired rotation of bodyA wrt bodyB.
 // note: if q violates the joint limits, the internal target is clamped to avoid conflicting impulses (very bad for stability)
 // note: don't forget to enableMotor()
 public void setMotorTarget(final btQuaternion q) {
  //btTransform trACur = m_rbA.getCenterOfMassTransform();
  //btTransform trBCur = m_rbB.getCenterOfMassTransform();
//	btTransform trABCur = trBCur.inverse() * trACur;
//	btQuaternion qABCur = trABCur.getRotation();
//	btTransform trConstraintCur = (trBCur * m_rbBFrame).inverse() * (trACur * m_rbAFrame);
  //btQuaternion qConstraintCur = trConstraintCur.getRotation();
  final btQuaternion qConstraint = new btQuaternion().set(m_rbBFrame).conjugate().mul(q).mul(
   new btQuaternion().set(m_rbAFrame));
  setMotorTargetInConstraintSpace(qConstraint);
 }

 btQuaternion getMotorTarget() {
  return new btQuaternion(m_qTarget);
 }

 // same as above, but q is the desired rotation of frameA wrt frameB in constraint space
 public void setMotorTargetInConstraintSpace(final btQuaternion q) {
  m_qTarget.set(q);
  // clamp motor target to within limits
  {
   float softness = 1.f;//m_limitSoftness;
   // split into twist and cone
   final btVector3 vTwisted = quatRotate(m_qTarget, vTwist);
   final btQuaternion qTargetCone = shortestArcQuat(vTwist, vTwisted);
   qTargetCone.normalize();
   final btQuaternion qTargetTwist = new btQuaternion(qTargetCone).conjugate().mul(m_qTarget);
   qTargetTwist.normalize();
   // clamp cone
   if (m_swingSpan1 >= (0.05f) && m_swingSpan2 >= (0.05f)) {
    float[] swingAngle = new float[1];
    float[] swingLimit = new float[1];
    final btVector3 swingAxis = new btVector3();
    computeConeLimitInfo(qTargetCone, swingAngle, swingAxis, swingLimit);
    if (btFabs(swingAngle[0]) > SIMD_EPSILON) {
     if (swingAngle[0] > swingLimit[0] * softness) {
      swingAngle[0] = swingLimit[0] * softness;
     } else if (swingAngle[0] < -swingLimit[0] * softness) {
      swingAngle[0] = -swingLimit[0] * softness;
     }
     qTargetCone.set(new AxisAngle4f(swingAxis, swingAngle[0]));
    }
   }
   // clamp twist
   if (m_twistSpan >= (0.05f)) {
    float[] twistAngle = new float[1];
    final btVector3 twistAxis = new btVector3();
    computeTwistLimitInfo(qTargetTwist, twistAngle, twistAxis);
    if (btFabs(twistAngle[0]) > SIMD_EPSILON) {
     // eddy todo: limitSoftness used here???
     if (twistAngle[0] > m_twistSpan * softness) {
      twistAngle[0] = m_twistSpan * softness;
     } else if (twistAngle[0] < -m_twistSpan * softness) {
      twistAngle[0] = -m_twistSpan * softness;
     }
     qTargetTwist.set(new AxisAngle4f(twistAxis, twistAngle[0]));
    }
   }
   m_qTarget.set(qTargetCone).mul(qTargetTwist);
  }
 }

 public btVector3 GetPointForAngle(float fAngleInRadians, float fLength) {
  // compute x/y in ellipse using cone angle (0 . 2*PI along surface of cone)
  float xEllipse = btCos(fAngleInRadians);
  float yEllipse = btSin(fAngleInRadians);
  // Use the slope of the vector (using x/yEllipse) and find the length
  // of the line that intersects the ellipse:
  //  x^2   y^2
  //  --- + --- = 1, where a and b are semi-major axes 2 and 1 respectively (ie. the limits)
  //  a^2   b^2
  // Do the math and it should be clear.
  float swingLimit = m_swingSpan1; // if xEllipse == 0, just use axis b (1)
  if (btFabs(xEllipse) > SIMD_EPSILON) {
   float surfaceSlope2 = (yEllipse * yEllipse) / (xEllipse * xEllipse);
   float norm = 1 / (m_swingSpan2 * m_swingSpan2);
   norm += surfaceSlope2 / (m_swingSpan1 * m_swingSpan1);
   float swingLimit2 = (1 + surfaceSlope2) / norm;
   swingLimit = btSqrt(swingLimit2);
  }
  // convert into point in constraint space:
  // note: twist is x-axis, swing 1 and 2 are along the z and y axes respectively
  final btVector3 vSwingAxis = new btVector3(0, xEllipse, -yEllipse);
  final btQuaternion qSwing = new btQuaternion().set(new AxisAngle4f(vSwingAxis, swingLimit));
  final btVector3 vPointInConstraintSpace = new btVector3(fLength, 0, 0);
  return quatRotate(qSwing, vPointInConstraintSpace);
 }

 ///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
 ///If no axis is provided, it uses the default axis for this constraint.
 @Override
 public void setParam(int num, float value, int axis) {
  switch (num) {
   case BT_CONSTRAINT_ERP:
   case BT_CONSTRAINT_STOP_ERP:
    if ((axis >= 0) && (axis < 3)) {
     m_linERP = value;
     m_flags |= BT_CONETWIST_FLAGS_LIN_ERP;
    } else {
     m_biasFactor = value;
    }
    break;
   case BT_CONSTRAINT_CFM:
   case BT_CONSTRAINT_STOP_CFM:
    if ((axis >= 0) && (axis < 3)) {
     m_linCFM = value;
     m_flags |= BT_CONETWIST_FLAGS_LIN_CFM;
    } else {
     m_angCFM = value;
     m_flags |= BT_CONETWIST_FLAGS_ANG_CFM;
    }
    break;
   default:
    assert (false);
    break;
  }
 }

 public void setFrames(final btTransform frameA, final btTransform frameB) {
  m_rbAFrame.set(frameA);
  m_rbBFrame.set(frameB);
  buildJacobian();
  //calculateTransforms();
 }

 btTransform getFrameOffsetA() {
  return new btTransform(m_rbAFrame);
 }

 btTransform getFrameOffsetB() {
  return new btTransform(m_rbBFrame);
 }

 ///return the local value of parameter
 @Override
 public float getParam(int num, int axis) {
  float retVal = 0;
  switch (num) {
   case BT_CONSTRAINT_ERP:
   case BT_CONSTRAINT_STOP_ERP:
    if ((axis >= 0) && (axis < 3)) {
     assert ((m_flags & BT_CONETWIST_FLAGS_LIN_ERP) != 0);
     retVal = m_linERP;
    } else if ((axis >= 3) && (axis < 6)) {
     retVal = m_biasFactor;
    } else {
     assert (false);
    }
    break;
   case BT_CONSTRAINT_CFM:
   case BT_CONSTRAINT_STOP_CFM:
    if ((axis >= 0) && (axis < 3)) {
     assert ((m_flags & BT_CONETWIST_FLAGS_LIN_CFM) != 0);
     retVal = m_linCFM;
    } else if ((axis >= 3) && (axis < 6)) {
     assert ((m_flags & BT_CONETWIST_FLAGS_ANG_CFM) != 0);
     retVal = m_angCFM;
    } else {
     assert (false);
    }
    break;
   default:
    assert (false);
  }
  return retVal;
 }

 int getFlags() {
  return m_flags;
 }

 float computeAngularImpulseDenominator(final btVector3 axis, final btMatrix3x3 invInertiaWorld) {
  final btVector3 vec = invInertiaWorld.transposeTransform(new btVector3(axis));
  return axis.dot(vec);
 }
}
