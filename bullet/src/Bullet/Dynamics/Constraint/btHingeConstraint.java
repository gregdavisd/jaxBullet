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

 /* Hinge Constraint by Dirk Gregorius. Limits added by Marcus Hennix at Starbreeze Studios */
package Bullet.Dynamics.Constraint;

import Bullet.Dynamics.btJacobianEntry;
import Bullet.Dynamics.btRigidBody;
import Bullet.LinearMath.btMatrix3x3;
import Bullet.LinearMath.btQuaternion;
import static Bullet.LinearMath.btQuaternion.quatRotate;
import static Bullet.LinearMath.btQuaternion.shortestArcQuat;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.SIMD_INFINITY;
import static Bullet.LinearMath.btScalar.SIMD_PI;
import static Bullet.LinearMath.btScalar.btAtan2;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.btPlaneSpace1;
import java.io.Serializable;

/**
 * hinge constraint between two rigidbodies each with a pivotpoint that descibes the axis location
 * in local space axis defines the orientation of the hinge axis
 *
 * @author Gregery Barton
 */
public class btHingeConstraint extends btTypedConstraint implements Serializable  {

 public static final boolean HINGE_USE_OBSOLETE_SOLVER = false;
 public static final boolean HINGE_USE_FRAME_OFFSET = true;
 public static final int BT_HINGE_FLAGS_CFM_STOP = 1;
 public static final int BT_HINGE_FLAGS_ERP_STOP = 2;
 public static final int BT_HINGE_FLAGS_CFM_NORM = 4;
 public static final int BT_HINGE_FLAGS_ERP_NORM = 8;
 final btJacobianEntry[] m_jac = {new btJacobianEntry(), new btJacobianEntry(),
  new btJacobianEntry()}; //3 orthogonal linear constraints
 final btJacobianEntry[] m_jacAng = {new btJacobianEntry(), new btJacobianEntry(),
  new btJacobianEntry()}; //2 orthogonal angular constraints+ 1 for limit/motor
 final btTransform m_rbAFrame = new btTransform(); // constraint axii. Assumes z is hinge axis.
 final btTransform m_rbBFrame = new btTransform();
 float m_motorTargetVelocity;
 float m_maxMotorImpulse;
 final btAngularLimit m_limit = new btAngularLimit();
 float m_lowerLimit;
 float m_upperLimit;
 float m_limitSign;
 float m_correction;
 float m_limitSoftness;
 float m_biasFactor;
 float m_relaxationFactor;
 boolean m_solveLimit;
 float m_kHinge;
 float m_accLimitImpulse;
 float m_hingeAngle;
 float m_referenceSign;
 boolean m_angularOnly;
 boolean m_enableAngularMotor;
 boolean m_useSolveConstraintObsolete;
 boolean m_useOffsetForConstraintFrame;
 boolean m_useReferenceFrameA;
 float m_accMotorImpulse;
 int m_flags;
 float m_normalCFM;
 float m_normalERP;
 float m_stopCFM;
 float m_stopERP;

 public btHingeConstraint(btRigidBody rbA, btRigidBody rbB, final btVector3 pivotInA,
  final btVector3 pivotInB, final btVector3 axisInA, final btVector3 axisInB) {
  this(rbA, rbB, pivotInA, pivotInB, axisInA, axisInB, false);
 }

 public btHingeConstraint(btRigidBody rbA, btRigidBody rbB, final btVector3 pivotInA,
  final btVector3 pivotInB, final btVector3 axisInA, final btVector3 axisInB,
  boolean useReferenceFrameA) {
  super(HINGE_CONSTRAINT_TYPE, rbA, rbB);
  m_angularOnly = false;
  m_enableAngularMotor = false;
  m_useSolveConstraintObsolete = HINGE_USE_OBSOLETE_SOLVER;
  m_useOffsetForConstraintFrame = HINGE_USE_FRAME_OFFSET;
  m_useReferenceFrameA = useReferenceFrameA;
  m_flags = 0;
  m_normalCFM = 0f;
  m_normalERP = 0f;
  m_stopCFM = 0f;
  m_stopERP = 0f;
  m_rbAFrame.setOrigin(pivotInA);
  // since no frame is given, assume this to be zero angle and just pick rb transform axis
  final btVector3 rbAxisA1 = rbA.getCenterOfMassTransform().getBasisColumn(0);
  final btVector3 rbAxisA2;
  float projection = axisInA.dot(rbAxisA1);
  if (projection >= 1.0f - SIMD_EPSILON) {
   rbAxisA1.set(rbA.getCenterOfMassTransform().getBasisColumn(2).negate());
   rbAxisA2 = rbA.getCenterOfMassTransform().getBasisColumn(1);
  } else if (projection <= -1.0f + SIMD_EPSILON) {
   rbAxisA1.set(rbA.getCenterOfMassTransform().getBasisColumn(2));
   rbAxisA2 = rbA.getCenterOfMassTransform().getBasisColumn(1);
  } else {
   rbAxisA2 = new btVector3(axisInA).cross(rbAxisA1);
   rbAxisA1.set(new btVector3(rbAxisA2).cross(axisInA));
  }
  m_rbAFrame.set3x3(new btMatrix3x3(
   rbAxisA1.getX(), rbAxisA2.getX(), axisInA.getX(),
   rbAxisA1.getY(), rbAxisA2.getY(), axisInA.getY(),
   rbAxisA1.getZ(), rbAxisA2.getZ(), axisInA.getZ()));
  final btQuaternion rotationArc = shortestArcQuat(axisInA, axisInB);
  final btVector3 rbAxisB1 = quatRotate(rotationArc, rbAxisA1);
  final btVector3 rbAxisB2 = new btVector3(axisInB).cross(rbAxisB1);
  m_rbBFrame.setOrigin(pivotInB);
  m_rbBFrame.set3x3(new btMatrix3x3(rbAxisB1.getX(), rbAxisB2.getX(), axisInB.getX(),
   rbAxisB1.getY(), rbAxisB2.getY(), axisInB.getY(),
   rbAxisB1.getZ(), rbAxisB2.getZ(), axisInB.getZ()));
  //start with free
  m_referenceSign = m_useReferenceFrameA ? (-1.f) : (1.f);
 }

 public btHingeConstraint(btRigidBody rbA, final btVector3 pivotInA, final btVector3 axisInA) {
  this(rbA, pivotInA, axisInA, false);
 }

 public btHingeConstraint(
  btRigidBody rbA, final btVector3 pivotInA, final btVector3 axisInA,
  boolean useReferenceFrameA) {
  super(HINGE_CONSTRAINT_TYPE, rbA);
  m_angularOnly = false;
  m_enableAngularMotor = false;
  m_useSolveConstraintObsolete = HINGE_USE_OBSOLETE_SOLVER;
  m_useOffsetForConstraintFrame = HINGE_USE_FRAME_OFFSET;
  m_useReferenceFrameA = useReferenceFrameA;
  m_flags = 0;
  m_normalCFM = 0;
  m_normalERP = 0;
  m_stopCFM = 0;
  m_stopERP = 0;
  // since no frame is given, assume this to be zero angle and just pick rb transform axis
  // fixed axis in worldspace
  final btVector3 rbAxisA1 = new btVector3();
  final btVector3 rbAxisA2 = new btVector3();
  btPlaneSpace1(axisInA, rbAxisA1, rbAxisA2);
  m_rbAFrame.setOrigin(pivotInA);
  m_rbAFrame.set3x3(new btMatrix3x3(rbAxisA1.getX(), rbAxisA2.getX(), axisInA.getX(),
   rbAxisA1.getY(), rbAxisA2.getY(), axisInA.getY(),
   rbAxisA1.getZ(), rbAxisA2.getZ(), axisInA.getZ()));
  final btVector3 axisInB = rbA.getCenterOfMassTransform().transform3x3(new btVector3(axisInA));
  final btQuaternion rotationArc = shortestArcQuat(axisInA, axisInB);
  final btVector3 rbAxisB1 = quatRotate(rotationArc, rbAxisA1);
  final btVector3 rbAxisB2 = new btVector3(axisInB).cross(rbAxisB1);
  m_rbBFrame.setOrigin(rbA.getCenterOfMassTransform().transform(new btVector3(pivotInA)));
  m_rbBFrame.set3x3(new btMatrix3x3(rbAxisB1.getX(), rbAxisB2.getX(), axisInB.getX(),
   rbAxisB1.getY(), rbAxisB2.getY(), axisInB.getY(),
   rbAxisB1.getZ(), rbAxisB2.getZ(), axisInB.getZ()));
  m_referenceSign = m_useReferenceFrameA ? (-1.f) : (1.f);
 }

 public btHingeConstraint(btRigidBody rbA, btRigidBody rbB, final btTransform rbAFrame,
  final btTransform rbBFrame) {
  this(rbA, rbB, rbAFrame, rbBFrame, false);
 }

 public btHingeConstraint(btRigidBody rbA, btRigidBody rbB, final btTransform rbAFrame,
  final btTransform rbBFrame, boolean useReferenceFrameA) {
  super(HINGE_CONSTRAINT_TYPE, rbA, rbB);
  m_rbAFrame.set(rbAFrame);
  m_rbBFrame.set(rbBFrame);
  m_angularOnly = false;
  m_enableAngularMotor = false;
  m_useSolveConstraintObsolete = HINGE_USE_OBSOLETE_SOLVER;
  m_useOffsetForConstraintFrame = HINGE_USE_FRAME_OFFSET;
  m_useReferenceFrameA = useReferenceFrameA;
  m_flags = 0;
  m_normalCFM = 0;
  m_normalERP = 0;
  m_stopCFM = 0;
  m_stopERP = 0;
  m_referenceSign = m_useReferenceFrameA ? (-1.f) : (1.f);
 }

 public btHingeConstraint(btRigidBody rbA, final btTransform rbAFrame) {
  this(rbA, rbAFrame, false);
 }

 public btHingeConstraint(btRigidBody rbA, final btTransform rbAFrame, boolean useReferenceFrameA) {
  super(HINGE_CONSTRAINT_TYPE, rbA);
  m_rbAFrame.set(rbAFrame);
  m_rbBFrame.set(rbAFrame);
  m_angularOnly = false;
  m_enableAngularMotor = false;
  m_useSolveConstraintObsolete = HINGE_USE_OBSOLETE_SOLVER;
  m_useOffsetForConstraintFrame = HINGE_USE_FRAME_OFFSET;
  m_useReferenceFrameA = useReferenceFrameA;
  m_flags = (0);
  m_normalCFM = 0;
  m_normalERP = 0;
  m_stopCFM = 0;
  m_stopERP = 0;
  ///not providing rigidbody B means implicitly using worldspace for body B
  m_rbBFrame.setOrigin(m_rbA.getCenterOfMassTransform().transform(m_rbAFrame.getOrigin()));
  m_referenceSign = m_useReferenceFrameA ? (-1.f) : (1.f);
 }

 @Override
 public void buildJacobian() {
  if (m_useSolveConstraintObsolete) {
   m_appliedImpulse = (0.f);
   m_accMotorImpulse = (0.f);
   if (!m_angularOnly) {
    final btVector3 pivotAInW = m_rbA.getCenterOfMassTransform().transform(m_rbAFrame.getOrigin());
    final btVector3 pivotBInW = m_rbB.getCenterOfMassTransform().transform(m_rbBFrame.getOrigin());
    final btVector3 relPos = new btVector3(pivotBInW).sub(pivotAInW);
    btVector3[] normal = {new btVector3(), new btVector3(), new btVector3()};
    if (relPos.lengthSquared() > SIMD_EPSILON) {
     normal[0].set(relPos).normalize();
    } else {
     normal[0].set((1.0f), 0f, 0f);
    }
    btPlaneSpace1(normal[0], normal[1], normal[2]);
    for (int i = 0; i < 3; i++) {
     m_jac[i] = new btJacobianEntry(
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
   //calculate two perpendicular jointAxis, orthogonal to hingeAxis
   //these two jointAxis require equal angular velocities for both bodies
   //this is unused for now, it's a todo
   final btVector3 jointAxis0local = new btVector3();
   final btVector3 jointAxis1local = new btVector3();
   btPlaneSpace1(m_rbAFrame.getBasisColumn(2), jointAxis0local, jointAxis1local);
   final btVector3 jointAxis0 = getRigidBodyA()
    .getCenterOfMassTransform()
    .transform3x3(new btVector3(jointAxis0local));
   final btVector3 jointAxis1 = getRigidBodyA().getCenterOfMassTransform().transform3x3(
    new btVector3(jointAxis1local));
   final btVector3 hingeAxisWorld = getRigidBodyA().getCenterOfMassTransform().transform3x3(
    m_rbAFrame.getBasisColumn(2));
   m_jacAng[0] = new btJacobianEntry(jointAxis0,
    m_rbA.getCenterOfMassTransform().getBasis().transpose(),
    m_rbB.getCenterOfMassTransform().getBasis().transpose(),
    m_rbA.getInvInertiaDiagLocal(),
    m_rbB.getInvInertiaDiagLocal());
   m_jacAng[1] = new btJacobianEntry(jointAxis1,
    m_rbA.getCenterOfMassTransform().getBasis().transpose(),
    m_rbB.getCenterOfMassTransform().getBasis().transpose(),
    m_rbA.getInvInertiaDiagLocal(),
    m_rbB.getInvInertiaDiagLocal());
   m_jacAng[2] = new btJacobianEntry(hingeAxisWorld,
    m_rbA.getCenterOfMassTransform().getBasis().transpose(),
    m_rbB.getCenterOfMassTransform().getBasis().transpose(),
    m_rbA.getInvInertiaDiagLocal(),
    m_rbB.getInvInertiaDiagLocal());
   // clear accumulator
   m_accLimitImpulse = (0.f);
   // test angular limit
   testLimit(m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform());
   //Compute K = J*W*J' for hinge axis
   final btVector3 axisA = getRigidBodyA().getCenterOfMassTransform().transform3x3(m_rbAFrame
    .getBasisColumn(2));
   m_kHinge = 1.0f / (getRigidBodyA().computeAngularImpulseDenominator(axisA) +
    getRigidBodyB().computeAngularImpulseDenominator(axisA));
  }
 }

 @Override
 public void getInfo1(btConstraintInfo1 info) {
  if (m_useSolveConstraintObsolete) {
   info.m_numConstraintRows = 0;
   info.nub = 0;
  } else {
   info.m_numConstraintRows = 5; // Fixed 3 linear + 2 angular
   info.nub = 1;
   //always add the row, to avoid computation (data is not available yet)
   //prepare constraint
   testLimit(m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform());
   if (getSolveLimit() != 0 || getEnableAngularMotor()) {
    info.m_numConstraintRows++; // limit 3rd anguar as well
    info.nub--;
   }
  }
 }

 public void getInfo1NonVirtual(btConstraintInfo1 info) {
  if (m_useSolveConstraintObsolete) {
   info.m_numConstraintRows = 0;
   info.nub = 0;
  } else {
   //always add the 'limit' row, to avoid computation (data is not available yet)
   info.m_numConstraintRows = 6; // Fixed 3 linear + 2 angular
   info.nub = 0;
  }
 }

 @Override
 public void getInfo2(btConstraintInfo2 info) {
  if (m_useOffsetForConstraintFrame) {
   getInfo2InternalUsingFrameOffset(info, m_rbA.getCenterOfMassTransform(), m_rbB
    .getCenterOfMassTransform(), m_rbA.getAngularVelocity(), m_rbB.getAngularVelocity());
  } else {
   getInfo2Internal(info, m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform(), m_rbA
    .getAngularVelocity(), m_rbB.getAngularVelocity());
  }
 }

 public void getInfo2NonVirtual(btConstraintInfo2 info, final btTransform transA,
  final btTransform transB,
  final btVector3 angVelA, final btVector3 angVelB) {
  ///the regular ( ) implementation getInfo2 already performs 'testLimit' during getInfo1, so we need to do it now
  testLimit(transA, transB);
  getInfo2Internal(info, transA, transB, angVelA, angVelB);
 }

 public void getInfo2Internal(btConstraintInfo2 info, final btTransform transA,
  final btTransform transB,
  final btVector3 angVelA, final btVector3 angVelB) {
  assert(!m_useSolveConstraintObsolete);
  int i, skip = info.rowskip;
  // transforms in world space
  final btTransform trA = new btTransform(transA).mul(m_rbAFrame);
  final btTransform trB = new btTransform(transB).mul(m_rbBFrame);
  // pivot point
  final btVector3 pivotAInW = trA.getOrigin();
  final btVector3 pivotBInW = trB.getOrigin();
  // linear (all fixed)
  if (!m_angularOnly) {
   info.m_J1linearAxis[0].x = 1f;
   info.m_J1linearAxis[skip].y = 1f;
   info.m_J1linearAxis[2 * skip].z = 1f;
   info.m_J2linearAxis[0].x = -1;
   info.m_J2linearAxis[skip].y = -1;
   info.m_J2linearAxis[2 * skip].z = -1;
  }
  final btVector3 a1 = transA.getOrigin().negate().add(pivotAInW);
  {
   final btVector3 angular0 = (info.m_J1angularAxis[0]);
   final btVector3 angular1 = (info.m_J1angularAxis[skip]);
   final btVector3 angular2 = (info.m_J1angularAxis[2 * skip]);
   final btVector3 a1neg = new btVector3(a1).negate();
   a1neg.getSkewSymmetricMatrix(angular0, angular1, angular2);
  }
  final btVector3 a2 = transB.getOrigin().negate().add(pivotBInW);
  {
   final btVector3 angular0 = (info.m_J2angularAxis[0]);
   final btVector3 angular1 = (info.m_J2angularAxis[skip]);
   final btVector3 angular2 = (info.m_J2angularAxis[2 * skip]);
   a2.getSkewSymmetricMatrix(angular0, angular1, angular2);
  }
  // linear RHS
  float normalErp = (m_flags & BT_HINGE_FLAGS_ERP_NORM) != 0f ? m_normalERP : info.erp;
  float k = info.fps * normalErp;
  if (!m_angularOnly) {
   for (i = 0; i < 3; i++) {
    info.m_constraintError[i * skip].set(k * (pivotBInW.getElement(i) - pivotAInW.getElement(i)));
   }
  }
  // make rotations around X and Y equal
  // the hinge axis should be the only unconstrained
  // rotational axis, the angular velocity of the two bodies perpendicular to
  // the hinge axis should be equal. thus the constraint equations are
  //    p*w1 - p*w2 = 0
  //    q*w1 - q*w2 = 0
  // where p and q are unit vectors normal to the hinge axis, and w1 and w2
  // are the angular velocity vectors of the two bodies.
  // get hinge axis (Z)
  final btVector3 ax1 = trA.getBasisColumn(2);
  // get 2 orthos to hinge axis (X, Y)
  final btVector3 p = trA.getBasisColumn(0);
  final btVector3 q = trA.getBasisColumn(1);
  // set the two hinge angular rows 
  int s3 = 3 * info.rowskip;
  int s4 = 4 * info.rowskip;
  info.m_J1angularAxis[s3].set(p);
  info.m_J1angularAxis[s4].set(q);
  info.m_J2angularAxis[s3].set(p).negate();
  info.m_J2angularAxis[s4].set(q).negate();
  // compute the right hand side of the constraint equation. set relative
  // body velocities along p and q to bring the hinge back into alignment.
  // if ax1,ax2 are the unit length hinge axes as computed from body1 and
  // body2, we need to rotate both bodies along the axis u = (ax1 x ax2).
  // if `theta' is the angle between ax1 and ax2, we need an angular velocity
  // along u to cover angle erp*theta in one step :
  //   |angular_velocity| = angle/time = erp*theta / stepsize
  //                      = (erp*fps) * theta
  //    angular_velocity  = |angular_velocity| * (ax1 x ax2) / |ax1 x ax2|
  //                      = (erp*fps) * theta * (ax1 x ax2) / sin(theta)
  // ...as ax1 and ax2 are unit length. if theta is smallish,
  // theta ~= sin(theta), so
  //    angular_velocity  = (erp*fps) * (ax1 x ax2)
  // ax1 x ax2 is in the plane space of ax1, so we project the angular
  // velocity to p and q to find the right hand side.
  final btVector3 ax2 = trB.getBasisColumn(2);
  final btVector3 u = new btVector3(ax1).cross(ax2);
  info.m_constraintError[s3].set(k * u.dot(p));
  info.m_constraintError[s4].set(k * u.dot(q));
  // check angular limits
  int nrow = 4; // last filled row
  int srow;
  float limit_err = (0.0f);
  int limit = 0;
  if (getSolveLimit() != 0) {
   limit_err = m_limit.getCorrection() * m_referenceSign;
   limit = (limit_err > (0.0f)) ? 1 : 2;
  }
  // if the hinge has joint limits or motor, add in the extra row
  int powered = 0;
  if (getEnableAngularMotor()) {
   powered = 1;
  }
  if (limit != 0 || powered != 0) {
   nrow++;
   srow = nrow * info.rowskip;
   info.m_J1angularAxis[srow].set(ax1);
   info.m_J2angularAxis[srow].set(ax1).negate();
   float lostop = getLowerLimit();
   float histop = getUpperLimit();
   if (limit != 0 && (lostop == histop)) {  // the joint motor is ineffective
    powered = 0;
   }
   info.m_constraintError[srow].set(0.0f);
   float currERP = (m_flags & BT_HINGE_FLAGS_ERP_STOP) != 0 ? m_stopERP : normalErp;
   if (powered != 0) {
    if ((m_flags & BT_HINGE_FLAGS_CFM_NORM) != 0) {
     info.cfm[srow].set(m_normalCFM);
    }
    float mot_fact = getMotorFactor(m_hingeAngle, lostop, histop, m_motorTargetVelocity, info.fps *
     currERP);
    info.m_constraintError[srow].plusEquals(mot_fact * m_motorTargetVelocity * m_referenceSign);
    info.m_lowerLimit[srow].set(-m_maxMotorImpulse);
    info.m_upperLimit[srow].set(m_maxMotorImpulse);
   }
   if (limit != 0) {
    k = info.fps * currERP;
    info.m_constraintError[srow].plusEquals(k * limit_err);
    if ((m_flags & BT_HINGE_FLAGS_CFM_STOP) != 0) {
     info.cfm[srow].set(m_stopCFM);
    }
    if (lostop == histop) {
     // limited low and high simultaneously
     info.m_lowerLimit[srow].set(-SIMD_INFINITY);
     info.m_upperLimit[srow].set(SIMD_INFINITY);
    } else if (limit == 1) { // low limit
     info.m_lowerLimit[srow].set(0);
     info.m_upperLimit[srow].set(SIMD_INFINITY);
    } else { // high limit
     info.m_lowerLimit[srow].set(-SIMD_INFINITY);
     info.m_upperLimit[srow].set(0);
    }
    // bounce (we'll use slider parameter abs(1.0 - m_dampingLimAng) for that)
    float bounce = m_limit.getRelaxationFactor();
    if (bounce > (0.0f)) {
     float vel = angVelA.dot(ax1);
     vel -= angVelB.dot(ax1);
     // only apply bounce if the velocity is incoming, and if the
     // resulting c[] exceeds what we already have.
     if (limit == 1) {	// low limit
      if (vel < 0) {
       float newc = -bounce * vel;
       if (newc > info.m_constraintError[srow].get()) {
        info.m_constraintError[srow].set(newc);
       }
      }
     } else if (vel > 0) {
      // high limit - all those computations are reversed
      float newc = -bounce * vel;
      if (newc < info.m_constraintError[srow].get()) {
       info.m_constraintError[srow].set(newc);
      }
     }
    }
    info.m_constraintError[srow].timesEquals(m_limit.getBiasFactor());
   } // if(limit)
  } // if angular limit or powered
 }

 public void getInfo2InternalUsingFrameOffset(btConstraintInfo2 info, final btTransform transA,
  final btTransform transB, final btVector3 angVelA, final btVector3 angVelB) {
 
  assert(!m_useSolveConstraintObsolete);
  int i, s = info.rowskip;
  // transforms in world space
  final btTransform trA = new btTransform(transA).mul(m_rbAFrame);
  final btTransform trB = new btTransform(transB).mul(m_rbBFrame);
  // pivot point
//	btVector3 pivotAInW = trA.getOrigin();
//	btVector3 pivotBInW = trB.getOrigin();
  // difference between frames in WCS
  final btVector3 ofs = trB.getOrigin().sub(trA.getOrigin());
  // now get weight factors depending on masses
  float miA = getRigidBodyA().getInvMass();
  float miB = getRigidBodyB().getInvMass();
  boolean hasStaticBody = (miA < SIMD_EPSILON) || (miB < SIMD_EPSILON);
  float miS = miA + miB;
  float factA, factB;
  if (miS > (0.f)) {
   factA = miB / miS;
  } else {
   factA = (0.5f);
  }
  factB = (1.0f) - factA;
  // get the desired direction of hinge axis
  // as weighted sum of Z-orthos of frameA and frameB in WCS
  final btVector3 ax1A = trA.getBasisColumn(2);
  final btVector3 ax1B = trB.getBasisColumn(2);
  final btVector3 ax1 = new btVector3(ax1A).scale(factA).add(new btVector3(ax1B).scale(factB));
  ax1.normalize();
  // fill first 3 rows 
  // we want: velA + wA x relA == velB + wB x relB
  	final btTransform bodyA_trans = new btTransform(transA);
	final btTransform bodyB_trans = new btTransform(transB);
  int s0 = 0;
  int s1 = s;
  int s2 = s * 2;
  int nrow = 2; // last filled row
  final btVector3 tmpA=new btVector3();
  final btVector3 tmpB=new btVector3();
  final btVector3 relA=new btVector3();
  final btVector3 relB=new btVector3();
  final btVector3 p=new btVector3();
  final btVector3 q=new btVector3();
  // get vector from bodyB to frameB in WCS
  relB .set( trB.getOrigin().sub(bodyB_trans.getOrigin()));
  // get its projection to hinge axis
  final btVector3 projB = new btVector3(ax1).scale(relB.dot(ax1));
  // get vector directed from bodyB to hinge axis (and orthogonal to it)
  final btVector3 orthoB = new btVector3(relB).sub(projB);
  // same for bodyA
  relA .set(trA.getOrigin().sub(bodyA_trans.getOrigin()));
  final btVector3 projA = new btVector3(ax1).scale(relA.dot(ax1));
  final btVector3 orthoA = new btVector3(relA).sub(projA);
  final btVector3 totalDist = new btVector3(projA).sub(projB);
  // get offset vectors relA and relB
  relA.scaleAdd(factA, totalDist, orthoA);
  relB.scaleAdd(-factB, totalDist, orthoB);
  // now choose average ortho to hinge axis
  p .set(orthoB).scale(factA).add(new btVector3(orthoA).scale(factB));
  float len2 = p.lengthSquared();
  if (len2 > SIMD_EPSILON) {
   p.scale(1.0f / btSqrt(len2));
  } else {
   p.set(trA.getBasisColumn(1));
  }
  // make one more ortho
  q .set(ax1).cross(p);
  // fill three rows
  tmpA .set(relA).cross(p);
  tmpB .set(relB).cross(p);
  info.m_J1angularAxis[s0].set(tmpA);
  info.m_J2angularAxis[s0].set(tmpB).negate();
  tmpA.set(relA).cross(q);
  tmpB.set(relB).cross(q);
  if (hasStaticBody && getSolveLimit() != 0) { // to make constraint between static and dynamic objects more rigid
   // remove wA (or wB) from equation if angular limit is hit
   tmpB.scale(factB);
   tmpA.scale(factA);
  }
  info.m_J1angularAxis[s1].set(tmpA);
  info.m_J2angularAxis[s1].set(tmpB).negate();
  tmpA.set(relA).cross(ax1);
  tmpB.set(relB).cross(ax1);
  if (hasStaticBody) { // to make constraint between static and dynamic objects more rigid
   // remove wA (or wB) from equation
   tmpB.scale(factB);
   tmpA.scale(factA);
  }
  info.m_J1angularAxis[s2].set(tmpA);
  info.m_J2angularAxis[s2].set(tmpB).negate();
  float normalErp = (m_flags & BT_HINGE_FLAGS_ERP_NORM) != 0 ? m_normalERP : info.erp;
  float k = info.fps * normalErp;
  if (!m_angularOnly) {
   info.m_J1linearAxis[s0].set(p);
   info.m_J1linearAxis[s1].set(q);
   info.m_J1linearAxis[s2].set(ax1);
   info.m_J2linearAxis[s0].set( p).negate() ;
   info.m_J2linearAxis[s1].set( q).negate() ;
   info.m_J2linearAxis[s2].set( ax1).negate() ;
   // compute three elements of right hand side
   float rhs = k * p.dot(ofs);
   info.m_constraintError[s0].set(rhs);
   rhs = k * q.dot(ofs);
   info.m_constraintError[s1].set(rhs);
   rhs = k * ax1.dot(ofs);
   info.m_constraintError[s2].set(rhs);
  }
  // the hinge axis should be the only unconstrained
  // rotational axis, the angular velocity of the two bodies perpendicular to
  // the hinge axis should be equal. thus the constraint equations are
  //    p*w1 - p*w2 = 0
  //    q*w1 - q*w2 = 0
  // where p and q are unit vectors normal to the hinge axis, and w1 and w2
  // are the angular velocity vectors of the two bodies.
  int s3 = 3 * s;
  int s4 = 4 * s;
  info.m_J1angularAxis[s3].set(p);
  info.m_J1angularAxis[s4].set(q);
  info.m_J2angularAxis[s3].set(p).negate();
  info.m_J2angularAxis[s4].set(q).negate();
  // compute the right hand side of the constraint equation. set relative
  // body velocities along p and q to bring the hinge back into alignment.
  // if ax1A,ax1B are the unit length hinge axes as computed from bodyA and
  // bodyB, we need to rotate both bodies along the axis u = (ax1 x ax2).
  // if "theta" is the angle between ax1 and ax2, we need an angular velocity
  // along u to cover angle erp*theta in one step :
  //   |angular_velocity| = angle/time = erp*theta / stepsize
  //                      = (erp*fps) * theta
  //    angular_velocity  = |angular_velocity| * (ax1 x ax2) / |ax1 x ax2|
  //                      = (erp*fps) * theta * (ax1 x ax2) / sin(theta)
  // ...as ax1 and ax2 are unit length. if theta is smallish,
  // theta ~= sin(theta), so
  //    angular_velocity  = (erp*fps) * (ax1 x ax2)
  // ax1 x ax2 is in the plane space of ax1, so we project the angular
  // velocity to p and q to find the right hand side.
  k = info.fps * normalErp;//??
  final btVector3 u = new btVector3(ax1A).cross(ax1B);
  info.m_constraintError[s3].set(k * u.dot(p));
  info.m_constraintError[s4].set(k * u.dot(q));
  // check angular limits
  nrow = 4; // last filled row
  int srow;
  float limit_err = (0.0f);
  int limit = 0;
  if (getSolveLimit() != 0) {
   limit_err = m_limit.getCorrection() * m_referenceSign;
   limit = (limit_err > (0.0f)) ? 1 : 2;
  }
  // if the hinge has joint limits or motor, add in the extra row
  int powered = 0;
  if (getEnableAngularMotor()) {
   powered = 1;
  }
  if (limit != 0 || powered != 0) {
   nrow++;
   srow = nrow * info.rowskip;
   info.m_J1angularAxis[srow].set(ax1);
   info.m_J2angularAxis[srow].set( ax1).negate() ;
   float lostop = getLowerLimit();
   float histop = getUpperLimit();
   if (limit != 0 && (lostop == histop)) {  // the joint motor is ineffective
    powered = 0;
   }
   info.m_constraintError[srow].set(0.0f);
   float currERP = (m_flags & BT_HINGE_FLAGS_ERP_STOP) != 0 ? m_stopERP : normalErp;
   if (powered != 0) {
    if ((m_flags & BT_HINGE_FLAGS_CFM_NORM) != 0) {
     info.cfm[srow].set(m_normalCFM);
    }
    float mot_fact = getMotorFactor(m_hingeAngle, lostop, histop, m_motorTargetVelocity, info.fps *
     currERP);
    info.m_constraintError[srow].plusEquals(mot_fact * m_motorTargetVelocity * m_referenceSign);
    info.m_lowerLimit[srow].set(-m_maxMotorImpulse);
    info.m_upperLimit[srow].set(m_maxMotorImpulse);
   }
   if (limit != 0) {
    k = info.fps * currERP;
    info.m_constraintError[srow].plusEquals(k * limit_err);
    if ((m_flags & BT_HINGE_FLAGS_CFM_STOP) != 0) {
     info.cfm[srow].set(m_stopCFM);
    }
    if (lostop == histop) {
     // limited low and high simultaneously
     info.m_lowerLimit[srow].set(-SIMD_INFINITY);
     info.m_upperLimit[srow].set(SIMD_INFINITY);
    } else if (limit == 1) { // low limit
     info.m_lowerLimit[srow].set(0);
     info.m_upperLimit[srow].set(SIMD_INFINITY);
    } else { // high limit
     info.m_lowerLimit[srow].set(-SIMD_INFINITY);
     info.m_upperLimit[srow].set(0);
    }
    // bounce (we'll use slider parameter abs(1.0 - m_dampingLimAng) for that)
    float bounce = m_limit.getRelaxationFactor();
    if (bounce > (0.0f)) {
     float vel = angVelA.dot(ax1);
     vel -= angVelB.dot(ax1);
     // only apply bounce if the velocity is incoming, and if the
     // resulting c[] exceeds what we already have.
     if (limit == 1) {	// low limit
      if (vel < 0) {
       float newc = -bounce * vel;
       if (newc > info.m_constraintError[srow].get()) {
        info.m_constraintError[srow].set(newc);
       }
      }
     } else if (vel > 0) {
      // high limit - all those computations are reversed
      float newc = -bounce * vel;
      if (newc < info.m_constraintError[srow].get()) {
       info.m_constraintError[srow].set(newc);
      }
     }
    }
    info.m_constraintError[srow].timesEquals(m_limit.getBiasFactor());
   } // if(limit)
  } // if angular limit or powered
 
 }

 public void updateRHS(float timeStep) {
 }

 @Override
 public btRigidBody getRigidBodyA() {
  return m_rbA;
 }

 @Override
 public btRigidBody getRigidBodyB() {
  return m_rbB;
 }

 public btTransform getFrameOffsetA() {
  return new btTransform(m_rbAFrame);
 }

 public btTransform getFrameOffsetB() {
  return new btTransform(m_rbBFrame);
 }

 public void setFrames(final btTransform frameA, final btTransform frameB) {
  m_rbAFrame.set(frameA);
  m_rbBFrame.set(frameB);
  buildJacobian();
 }

 public void setAngularOnly(boolean angularOnly) {
  m_angularOnly = angularOnly;
 }

 public void enableAngularMotor(boolean enableMotor, float targetVelocity, float maxMotorImpulse) {
  m_enableAngularMotor = enableMotor;
  m_motorTargetVelocity = targetVelocity;
  m_maxMotorImpulse = maxMotorImpulse;
 }

 // extra motor API, including ability to set a target rotation (as opposed to angular velocity)
 // note: setMotorTarget sets angular velocity under the hood, so you must call it every tick to
 //       maintain a given angular target.
 public void enableMotor(boolean enableMotor) {
  m_enableAngularMotor = enableMotor;
 }

 public void setMaxMotorImpulse(float maxMotorImpulse) {
  m_maxMotorImpulse = maxMotorImpulse;
 }

 public void setMotorTargetVelocity(float motorTargetVelocity) {
  m_motorTargetVelocity = motorTargetVelocity;
 }
 static final btVector3 vHinge = new btVector3(0f, 0f, (1f));

 // qAinB is rotation of body A wrt body B.
 public void setMotorTarget(final btQuaternion qAinB, float dt) {
  // convert target from body to constraint space
  final btQuaternion qConstraint = m_rbBFrame.getRotation().conjugate().mul(qAinB).mul(m_rbAFrame
   .getRotation());
  qConstraint.normalize();
  // extract "pure" hinge component
  final btVector3 vNoHinge = quatRotate(qConstraint, vHinge);
  vNoHinge.normalize();
  final btQuaternion qNoHinge = shortestArcQuat(vHinge, vNoHinge);
  final btQuaternion qHinge = new btQuaternion(qNoHinge).conjugate().mul(qConstraint);
  qHinge.normalize();
  // compute angular target, clamped to limits
  float targetAngle = qHinge.getAngle();
  if (targetAngle > SIMD_PI) // long way around. flip quat and recalculate.
  {
   qHinge.negate();
   targetAngle = qHinge.getAngle();
  }
  if (qHinge.getZ() < 0) {
   targetAngle = -targetAngle;
  }
  setMotorTarget(targetAngle, dt);
 }

 public void setMotorTarget(float targetAngle, float dt) {
  float _targetAngle = m_limit.fit(targetAngle);
  // compute angular velocity
  float curAngle = getHingeAngle(m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform());
  float dAngle = _targetAngle - curAngle;
  m_motorTargetVelocity = dAngle / dt;
 }

 public void setLimit(float low, float high) {
  setLimit(low, high, 0.9f);
 }

 public void setLimit(float low, float high, float _softness) {
  setLimit(low, high, _softness, 0.3f);
 }

 public void setLimit(float low, float high, float _softness, float _biasFactor) {
  setLimit(low, high, _softness, _biasFactor, 1.0f);
 }

 public void setLimit(float low, float high, float _softness, float _biasFactor,
  float _relaxationFactor) {
  m_limit.set(low, high, _softness, _biasFactor, _relaxationFactor);
 }

 public float getLimitSoftness() {
  return m_limit.getSoftness();
 }

 public float getLimitBiasFactor() {
  return m_limit.getBiasFactor();
 }

 public float getLimitRelaxationFactor() {
  return m_limit.getRelaxationFactor();
 }

 public void setAxis(final btVector3 axisInA) {
  final btVector3 rbAxisA1 = new btVector3();
  final btVector3 rbAxisA2 = new btVector3();
  btPlaneSpace1(axisInA, rbAxisA1, rbAxisA2);
  final btVector3 pivotInA = m_rbAFrame.getOrigin();
  m_rbAFrame.set3x3(new btMatrix3x3(rbAxisA1.getX(), rbAxisA2.getX(), axisInA.getX(),
   rbAxisA1.getY(), rbAxisA2.getY(), axisInA.getY(),
   rbAxisA1.getZ(), rbAxisA2.getZ(), axisInA.getZ()));
  final btVector3 axisInB = m_rbA.getCenterOfMassTransform().transform3x3(new btVector3(axisInA));
  final btQuaternion rotationArc = shortestArcQuat(axisInA, axisInB);
  final btVector3 rbAxisB1 = quatRotate(rotationArc, rbAxisA1);
  final btVector3 rbAxisB2 = new btVector3(axisInB).cross(rbAxisB1);
  m_rbBFrame.setOrigin(m_rbB.getCenterOfMassTransform().invert().transform(m_rbA
   .getCenterOfMassTransform().transform(new btVector3(pivotInA))));
  m_rbBFrame.set3x3(new btMatrix3x3(rbAxisB1.getX(), rbAxisB2.getX(), axisInB.getX(),
   rbAxisB1.getY(), rbAxisB2.getY(), axisInB.getY(),
   rbAxisB1.getZ(), rbAxisB2.getZ(), axisInB.getZ()));
  m_rbBFrame.set3x3(new btMatrix3x3(m_rbB.getCenterOfMassTransform().getBasis().invert().mul(
   m_rbBFrame.getBasis())));
 }

 public boolean hasLimit() {
  return m_limit.getHalfRange() > 0;
 }

 public float getLowerLimit() {
  return m_limit.getLow();
 }

 public float getUpperLimit() {
  return m_limit.getHigh();
 }

 ///The getHingeAngle gives the hinge angle in range [-PI,PI]
 final public float getHingeAngle() {
  return getHingeAngle(m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform());
 }

 public float getHingeAngle(final btTransform transA, final btTransform transB) {
  final btVector3 refAxis0 = transA.transform3x3(m_rbAFrame.getBasisColumn(0));
  final btVector3 refAxis1 = transA.transform3x3(m_rbAFrame.getBasisColumn(1));
  final btVector3 swingAxis = transB.transform3x3(m_rbBFrame.getBasisColumn(1));
  float angle = btAtan2(swingAxis.dot(refAxis0), swingAxis.dot(refAxis1));
  return m_referenceSign * angle;
 }

 public void testLimit(final btTransform transA, final btTransform transB) {
  // Compute limit information
  m_hingeAngle = getHingeAngle(transA, transB);
  m_limit.test(m_hingeAngle);
 }

 public btTransform getAFrame() {
  return new btTransform(m_rbAFrame);
 }

 public btTransform getBFrame() {
  return new btTransform(m_rbBFrame);
 }

 public int getSolveLimit() {
  return m_limit.isLimit() ? 1 : 0;
 }

 public float getLimitSign() {
  return m_limit.getSign();
 }

 public boolean getAngularOnly() {
  return m_angularOnly;
 }

 public boolean getEnableAngularMotor() {
  return m_enableAngularMotor;
 }

 float getMotorTargetVelosity() {
  return m_motorTargetVelocity;
 }

 public float getMaxMotorImpulse() {
  return m_maxMotorImpulse;
 }

 // access for UseFrameOffset
 public boolean getUseFrameOffset() {
  return m_useOffsetForConstraintFrame;
 }

 public void setUseFrameOffset(boolean frameOffsetOnOff) {
  m_useOffsetForConstraintFrame = frameOffsetOnOff;
 }

 // access for UseReferenceFrameA
 public boolean getUseReferenceFrameA() {
  return m_useReferenceFrameA;
 }

 public void setUseReferenceFrameA(boolean useReferenceFrameA) {
  m_useReferenceFrameA = useReferenceFrameA;
 }

 ///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
 ///If no axis is provided, it uses the default axis for this constraint.
 @Override
 public void setParam(int num, float value, int axis) {
  if ((axis == -1) || (axis == 5)) {
   switch (num) {
    case BT_CONSTRAINT_STOP_ERP:
     m_stopERP = value;
     m_flags |= BT_HINGE_FLAGS_ERP_STOP;
     break;
    case BT_CONSTRAINT_STOP_CFM:
     m_stopCFM = value;
     m_flags |= BT_HINGE_FLAGS_CFM_STOP;
     break;
    case BT_CONSTRAINT_CFM:
     m_normalCFM = value;
     m_flags |= BT_HINGE_FLAGS_CFM_NORM;
     break;
    case BT_CONSTRAINT_ERP:
     m_normalERP = value;
     m_flags |= BT_HINGE_FLAGS_ERP_NORM;
     break;
    default:
     assert(false);
   }
  } else {
   assert(false);
  }
 }

///return the local value of parameter
 @Override
 public float getParam(int num, int axis) {
  float retVal = 0;
  if ((axis == -1) || (axis == 5)) {
   switch (num) {
    case BT_CONSTRAINT_STOP_ERP:
     assert((m_flags & BT_HINGE_FLAGS_ERP_STOP) != 0);
     retVal = m_stopERP;
     break;
    case BT_CONSTRAINT_STOP_CFM:
     assert((m_flags & BT_HINGE_FLAGS_CFM_STOP) != 0);
     retVal = m_stopCFM;
     break;
    case BT_CONSTRAINT_CFM:
     assert((m_flags & BT_HINGE_FLAGS_CFM_NORM) != 0);
     retVal = m_normalCFM;
     break;
    case BT_CONSTRAINT_ERP:
     assert((m_flags & BT_HINGE_FLAGS_ERP_NORM) != 0);
     retVal = m_normalERP;
     break;
    default:
     assert(false);
   }
  } else {
   assert(false);
  }
  return retVal;
 }

 public int getFlags() {
  return m_flags;
 }
}
