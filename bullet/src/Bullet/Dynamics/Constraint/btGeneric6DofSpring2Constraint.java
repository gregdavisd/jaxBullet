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

 /*
 * 2014 May: btGeneric6DofSpring2Constraint is created from the original (2.82.2712) btGeneric6DofConstraint by Gabor Puhr and Tamas Umenhoffer
 * Pros:
 * - Much more accurate and stable in a lot of situation. (Especially when a sleeping chain of RBs connected with 6dof2 is pulled)
 * - Stable and accurate spring with minimal energy loss that works with all of the solvers. (latter is not true for the original 6dof spring)
 * - Servo motor functionality
 * - Much more accurate bouncing. 0 really means zero bouncing (not true for the original 6odf) and there is only a minimal energy loss when the value is 1 (because of the solvers' precision)
 * - Rotation order for the Euler system can be set. (One axis' freedom is still limited to pi/2)
 *
 * Cons:
 * - It is slower than the original 6dof. There is no exact ratio, but half speed is a good estimation. (with PGS)
 * - At bouncing the correct velocity is calculated, but not the correct position. (it is because of the solver can correct position or velocity, but not both.)
 */
/// 2009 March: btGeneric6DofConstraint refactored by Roman Ponomarev
/// Added support for generic constraint solver through getInfo1/getInfo2 methods

/*
 * 2007-09-09
 * btGeneric6DofConstraint Refactored by Francisco Le?n
 * email: projectileman@yahoo.com
 * http://gimpact.sf.net
 */
package Bullet.Dynamics.Constraint;

import Bullet.Dynamics.CollisionObjects.btRigidBody;
import Bullet.Dynamics.btJacobianEntry;
import Bullet.LinearMath.btMatrix3x3;
import static Bullet.LinearMath.btScalar.BT_ONE;
import static Bullet.LinearMath.btScalar.BT_ZERO;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.SIMD_HALF_PI;
import static Bullet.LinearMath.btScalar.SIMD_INFINITY;
import static Bullet.LinearMath.btScalar.btAsin;
import static Bullet.LinearMath.btScalar.btAtan2;
import static Bullet.LinearMath.btScalar.btNormalizeAngle;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btGeneric6DofSpring2Constraint extends btTypedConstraint implements
 Serializable {
 //enum RotateOrder

 public static final int RO_XYZ = 0;
 public static final int RO_XZY = 1;
 public static final int RO_YXZ = 2;
 public static final int RO_YZX = 3;
 public static final int RO_ZXY = 4;
 public static final int RO_ZYX = 5;
//enum bt6DofFlags2
 public static final int BT_6DOF_FLAGS_CFM_STOP2 = 1;
 public static final int BT_6DOF_FLAGS_ERP_STOP2 = 2;
 public static final int BT_6DOF_FLAGS_CFM_MOTO2 = 4;
 public static final int BT_6DOF_FLAGS_ERP_MOTO2 = 8;
 public static final int BT_6DOF_FLAGS_AXIS_SHIFT2 = 4; // bits per axis
 private static final long serialVersionUID = 1L;
 final btTransform m_frameInA = new btTransform();
 final btTransform m_frameInB = new btTransform();
 final btJacobianEntry[] m_jacLinear = {new btJacobianEntry(),
  new btJacobianEntry(),
  new btJacobianEntry()};
 final btJacobianEntry[] m_jacAng = {new btJacobianEntry(),
  new btJacobianEntry(),
  new btJacobianEntry(),};
 final btTranslationalLimitMotor2 m_linearLimits = new btTranslationalLimitMotor2();
 final btRotationalLimitMotor2[] m_angularLimits = {
  new btRotationalLimitMotor2(),
  new btRotationalLimitMotor2(), new btRotationalLimitMotor2()};
 int m_rotateOrder;
 final btTransform m_calculatedTransformA = new btTransform();
 final btTransform m_calculatedTransformB = new btTransform();
 final btVector3 m_calculatedAxisAngleDiff = new btVector3();
 final btVector3[] m_calculatedAxis = {new btVector3(), new btVector3(),
  new btVector3()};
 final btVector3 m_calculatedLinearDiff = new btVector3();
 float m_factA;
 float m_factB;
 boolean m_hasStaticBody;
 int m_flags;

 public int setAngularLimits(btConstraintInfo2 info, int row_offset,
  final btTransform transA,
  final btTransform transB, final btVector3 linVelA, final btVector3 linVelB,
  final btVector3 angVelA, final btVector3 angVelB) {
  int row = row_offset;
  //order of rotational constraint rows
  int cIdx[] = {0, 1, 2};
  switch (m_rotateOrder) {
   case RO_XYZ: cIdx[0] = 0;
    cIdx[1] = 1;
    cIdx[2] = 2;
    break;
   case RO_XZY: cIdx[0] = 0;
    cIdx[1] = 2;
    cIdx[2] = 1;
    break;
   case RO_YXZ: cIdx[0] = 1;
    cIdx[1] = 0;
    cIdx[2] = 2;
    break;
   case RO_YZX: cIdx[0] = 1;
    cIdx[1] = 2;
    cIdx[2] = 0;
    break;
   case RO_ZXY: cIdx[0] = 2;
    cIdx[1] = 0;
    cIdx[2] = 1;
    break;
   case RO_ZYX: cIdx[0] = 2;
    cIdx[1] = 1;
    cIdx[2] = 0;
    break;
   default: assert (false);
  }
  for (int ii = 0; ii < 3; ii++) {
   int i = cIdx[ii];
   if (m_angularLimits[i].m_currentLimit != 0
    || m_angularLimits[i].m_enableMotor || m_angularLimits[i].m_enableSpring) {
    final btVector3 axis = getAxis(i);
    int flags = m_flags >> ((i + 3) * BT_6DOF_FLAGS_AXIS_SHIFT2);
    if (0 == (flags & BT_6DOF_FLAGS_CFM_STOP2)) {
     m_angularLimits[i].m_stopCFM = info.cfm[0].get();
    }
    if (0 == (flags & BT_6DOF_FLAGS_ERP_STOP2)) {
     m_angularLimits[i].m_stopERP = info.erp;
    }
    if (0 == (flags & BT_6DOF_FLAGS_CFM_MOTO2)) {
     m_angularLimits[i].m_motorCFM = info.cfm[0].get();
    }
    if (0 == (flags & BT_6DOF_FLAGS_ERP_MOTO2)) {
     m_angularLimits[i].m_motorERP = info.erp;
    }
    row += get_limit_motor_info2(m_angularLimits[i], transA, transB, linVelA,
     linVelB, angVelA,
     angVelB, info, row, axis, 1);
   }
  }
  return row;
 }

 public int setLinearLimits(btConstraintInfo2 info, int row,
  final btTransform transA,
  final btTransform transB, final btVector3 linVelA, final btVector3 linVelB,
  final btVector3 angVelA, final btVector3 angVelB) {
  int do_row = row;
  //solve linear limits
  btRotationalLimitMotor2 limot = new btRotationalLimitMotor2();
  for (int i = 0; i < 3; i++) {
   if (m_linearLimits.m_currentLimit[i] != 0 || m_linearLimits.m_enableMotor[i]
    || m_linearLimits.m_enableSpring[i]) { // re-use rotational motor code
    limot.m_bounce = m_linearLimits.m_bounce.getElement(i);
    limot.m_currentLimit = m_linearLimits.m_currentLimit[i];
    limot.m_currentPosition = m_linearLimits.m_currentLinearDiff.getElement(i);
    limot.m_currentLimitError = m_linearLimits.m_currentLimitError.getElement(i);
    limot.m_currentLimitErrorHi = m_linearLimits.m_currentLimitErrorHi
     .getElement(i);
    limot.m_enableMotor = m_linearLimits.m_enableMotor[i];
    limot.m_servoMotor = m_linearLimits.m_servoMotor[i];
    limot.m_servoTarget = m_linearLimits.m_servoTarget.getElement(i);
    limot.m_enableSpring = m_linearLimits.m_enableSpring[i];
    limot.m_springStiffness = m_linearLimits.m_springStiffness.getElement(i);
    limot.m_springStiffnessLimited = m_linearLimits.m_springStiffnessLimited[i];
    limot.m_springDamping = m_linearLimits.m_springDamping.getElement(i);
    limot.m_springDampingLimited = m_linearLimits.m_springDampingLimited[i];
    limot.m_equilibriumPoint = m_linearLimits.m_equilibriumPoint.getElement(i);
    limot.m_hiLimit = m_linearLimits.m_upperLimit.getElement(i);
    limot.m_loLimit = m_linearLimits.m_lowerLimit.getElement(i);
    limot.m_maxMotorForce = m_linearLimits.m_maxMotorForce.getElement(i);
    limot.m_targetVelocity = m_linearLimits.m_targetVelocity.getElement(i);
    final btVector3 axis = m_calculatedTransformA.getBasisColumn(i);
    int flags = m_flags >> (i * BT_6DOF_FLAGS_AXIS_SHIFT2);
    limot.m_stopCFM = (flags & BT_6DOF_FLAGS_CFM_STOP2) != 0 ? m_linearLimits.m_stopCFM
     .getElement(i) : info.cfm[0].get();
    limot.m_stopERP = (flags & BT_6DOF_FLAGS_ERP_STOP2) != 0 ? m_linearLimits.m_stopERP
     .getElement(i) : info.erp;
    limot.m_motorCFM = (flags & BT_6DOF_FLAGS_CFM_MOTO2) != 0 ? m_linearLimits.m_motorCFM
     .getElement(i) : info.cfm[0].get();
    limot.m_motorERP = (flags & BT_6DOF_FLAGS_ERP_MOTO2) != 0 ? m_linearLimits.m_motorERP
     .getElement(i) : info.erp;
    //rotAllowed is a bit of a magic from the original 6dof. The calculation of it here is something that imitates the original behavior as much as possible.
    int indx1 = (i + 1) % 3;
    int indx2 = (i + 2) % 3;
    int rotAllowed = 1; // rotations around orthos to current axis (it is used only when one of the body is static)
    final float D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION = 1.0e-3f;
    boolean indx1Violated = m_angularLimits[indx1].m_currentLimit == 1
     || m_angularLimits[indx1].m_currentLimit == 2
     || (m_angularLimits[indx1].m_currentLimit == 3
     && (m_angularLimits[indx1].m_currentLimitError
     < -D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION
     || m_angularLimits[indx1].m_currentLimitError
     > D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION))
     || (m_angularLimits[indx1].m_currentLimit == 4
     && (m_angularLimits[indx1].m_currentLimitError
     < -D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION
     || m_angularLimits[indx1].m_currentLimitErrorHi
     > D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION));
    boolean indx2Violated = m_angularLimits[indx2].m_currentLimit == 1
     || m_angularLimits[indx2].m_currentLimit == 2
     || (m_angularLimits[indx2].m_currentLimit == 3
     && (m_angularLimits[indx2].m_currentLimitError
     < -D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION
     || m_angularLimits[indx2].m_currentLimitError
     > D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION))
     || (m_angularLimits[indx2].m_currentLimit == 4
     && (m_angularLimits[indx2].m_currentLimitError
     < -D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION
     || m_angularLimits[indx2].m_currentLimitErrorHi
     > D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION));
    if (indx1Violated && indx2Violated) {
     rotAllowed = 0;
    }
    do_row += get_limit_motor_info2(limot, transA, transB, linVelA, linVelB,
     angVelA, angVelB, info,
     do_row, axis, 0, rotAllowed);
   }
  }
  return do_row;
 }

 public void calculateLinearInfo() {
  m_calculatedLinearDiff.set(m_calculatedTransformB.getOrigin()).sub(
   m_calculatedTransformA
    .getOrigin());
  m_calculatedTransformA.getBasis().invert().transform(m_calculatedLinearDiff
   .set(
    m_calculatedLinearDiff));
  for (int i = 0; i < 3; i++) {
   m_linearLimits.m_currentLinearDiff.setElement(i, m_calculatedLinearDiff
    .getElement(i));
   m_linearLimits.testLimitValue(i, m_calculatedLinearDiff.getElement(i));
  }
 }

 public void calculateAngleInfo() {
  final btMatrix3x3 relative_frame = m_calculatedTransformA.getBasis().invert()
   .mul(
    m_calculatedTransformB.getBasis());
  switch (m_rotateOrder) {
   case RO_XYZ: matrixToEulerXYZ(relative_frame, m_calculatedAxisAngleDiff);
    break;
   case RO_XZY: matrixToEulerXZY(relative_frame, m_calculatedAxisAngleDiff);
    break;
   case RO_YXZ: matrixToEulerYXZ(relative_frame, m_calculatedAxisAngleDiff);
    break;
   case RO_YZX: matrixToEulerYZX(relative_frame, m_calculatedAxisAngleDiff);
    break;
   case RO_ZXY: matrixToEulerZXY(relative_frame, m_calculatedAxisAngleDiff);
    break;
   case RO_ZYX: matrixToEulerZYX(relative_frame, m_calculatedAxisAngleDiff);
    break;
   default: assert (false);
  }
  // in euler angle mode we do not actually constrain the angular velocity
  // along the axes axis[0] and axis[2] (although we do use axis[1]) :
  //
  //    to get			constrain w2-w1 along		...not
  //    ------			---------------------		------
  //    d(angle[0])/dt = 0	ax[1] x ax[2]			ax[0]
  //    d(angle[1])/dt = 0	ax[1]
  //    d(angle[2])/dt = 0	ax[0] x ax[1]			ax[2]
  //
  // constraining w2-w1 along an axis 'a' means that a'*(w2-w1)=0.
  // to prove the result for angle[0], write the expression for angle[0] from
  // GetInfo1 then take the derivative. to prove this for angle[2] it is
  // easier to take the euler rate expression for d(angle[2])/dt with respect
  // to the components of w and set that to 0.
  switch (m_rotateOrder) {
   case RO_XYZ: {
    //Is this the "line of nodes" calculation choosing planes YZ (B coordinate system) and xy (A coordinate system)? (http://en.wikipedia.org/wiki/Euler_angles)
    //The two planes are non-homologous, so this is a Tait–Bryan angle formalism and not a proper Euler
    //Extrinsic rotations are equal to the reversed order intrinsic rotations so the above xyz extrinsic rotations (axes are fixed) are the same as the zy'x" intrinsic rotations (axes are refreshed after each rotation)
    //that is why xy and YZ planes are chosen (this will describe a zy'x" intrinsic rotation) (see the figure on the left at http://en.wikipedia.org/wiki/Euler_angles under Tait–Bryan angles)
    // x' = Nperp = N.cross(axis2)
    // y' = N = axis2.cross(axis0)	
    // z' = z
    //
    // x" = X
    // y" = y'
    // z" = ??
    //in other words:
    //first rotate around z
    //second rotate around y'= z.cross(X)
    //third rotate around x" = X
    //Original XYZ extrinsic rotation order. 
    //Planes: xy and YZ normals: z, X.  Plane intersection (N) is z.cross(X)
    final btVector3 axis0 = m_calculatedTransformB.getBasisColumn(0);
    final btVector3 axis2 = m_calculatedTransformA.getBasisColumn(2);
    m_calculatedAxis[1].set(axis2).cross(axis0);
    m_calculatedAxis[0].set(m_calculatedAxis[1]).cross(axis2);
    m_calculatedAxis[2].set(axis0).cross(m_calculatedAxis[1]);
    break;
   }
   case RO_XZY: {
    //planes: xz,ZY normals: y, X
    //first rotate around y
    //second rotate around z'= y.cross(X)
    //third rotate around x" = X
    final btVector3 axis0 = m_calculatedTransformB.getBasisColumn(0);
    final btVector3 axis1 = m_calculatedTransformA.getBasisColumn(1);
    m_calculatedAxis[2].set(axis0).cross(axis1);
    m_calculatedAxis[0].set(axis1).cross(m_calculatedAxis[2]);
    m_calculatedAxis[1].set(m_calculatedAxis[2]).cross(axis0);
    break;
   }
   case RO_YXZ: {
    //planes: yx,XZ normals: z, Y
    //first rotate around z
    //second rotate around x'= z.cross(Y)
    //third rotate around y" = Y
    final btVector3 axis1 = m_calculatedTransformB.getBasisColumn(1);
    final btVector3 axis2 = m_calculatedTransformA.getBasisColumn(2);
    m_calculatedAxis[0].set(axis1).cross(axis2);
    m_calculatedAxis[1].set(axis2).cross(m_calculatedAxis[0]);
    m_calculatedAxis[2].set(m_calculatedAxis[0]).cross(axis1);
    break;
   }
   case RO_YZX: {
    //planes: yz,ZX normals: x, Y
    //first rotate around x
    //second rotate around z'= x.cross(Y)
    //third rotate around y" = Y
    final btVector3 axis0 = m_calculatedTransformA.getBasisColumn(0);
    final btVector3 axis1 = m_calculatedTransformB.getBasisColumn(1);
    m_calculatedAxis[2].set(axis0).cross(axis1);
    m_calculatedAxis[0].set(axis1).cross(m_calculatedAxis[2]);
    m_calculatedAxis[1].set(m_calculatedAxis[2]).cross(axis0);
    break;
   }
   case RO_ZXY: {
    //planes: zx,XY normals: y, Z
    //first rotate around y
    //second rotate around x'= y.cross(Z)
    //third rotate around z" = Z
    final btVector3 axis1 = m_calculatedTransformA.getBasisColumn(1);
    final btVector3 axis2 = m_calculatedTransformB.getBasisColumn(2);
    m_calculatedAxis[0].set(axis1).cross(axis2);
    m_calculatedAxis[1].set(axis2).cross(m_calculatedAxis[0]);
    m_calculatedAxis[2].set(m_calculatedAxis[0]).cross(axis1);
    break;
   }
   case RO_ZYX: {
    //planes: zy,YX normals: x, Z
    //first rotate around x
    //second rotate around y' = x.cross(Z)
    //third rotate around z" = Z
    final btVector3 axis0 = m_calculatedTransformA.getBasisColumn(0);
    final btVector3 axis2 = m_calculatedTransformB.getBasisColumn(2);
    m_calculatedAxis[1].set(axis2).cross(axis0);
    m_calculatedAxis[0].set(m_calculatedAxis[1]).cross(axis2);
    m_calculatedAxis[2].set(axis0).cross(m_calculatedAxis[1]);
    break;
   }
   default:
    assert (false);
  }
  m_calculatedAxis[0].normalize();
  m_calculatedAxis[1].normalize();
  m_calculatedAxis[2].normalize();
 }

 public void testAngularLimitMotor(int axis_index) {
  float angle = m_calculatedAxisAngleDiff.getElement(axis_index);
  angle = btAdjustAngleToLimits(angle, m_angularLimits[axis_index].m_loLimit,
   m_angularLimits[axis_index].m_hiLimit);
  m_angularLimits[axis_index].m_currentPosition = angle;
  m_angularLimits[axis_index].testLimitValue(angle);
 }

 public void calculateJacobi(btRotationalLimitMotor2 limot,
  final btTransform transA,
  final btTransform transB, btConstraintInfo2 info, int srow,
  final btVector3 ax1, int rotational,
  int rotAllowed) {
  btVector3[] J1 = rotational != 0 ? info.m_J1angularAxis : info.m_J1linearAxis;
  btVector3[] J2 = rotational != 0 ? info.m_J2angularAxis : info.m_J2linearAxis;
  J1[srow].set(ax1);
  J2[srow].set(ax1).negate();
  if (0 == rotational) {
   final btVector3 tmpA = new btVector3();
   final btVector3 tmpB = new btVector3();
   final btVector3 relA = new btVector3();
   final btVector3 relB = new btVector3();
   // get vector from bodyB to frameB in WCS
   relB.set(m_calculatedTransformB.getOrigin()).sub(transB.getOrigin());
   // same for bodyA
   relA.set(m_calculatedTransformA.getOrigin()).sub(transA.getOrigin());
   tmpA.set(relA).cross(ax1);
   tmpB.set(relB).cross(ax1);
   if (m_hasStaticBody && (0 == rotAllowed)) {
    tmpA.scale(m_factA);
    tmpB.scale(m_factB);
   }
   info.m_J1angularAxis[srow].set(tmpA);
   info.m_J2angularAxis[srow].set(tmpB).negate();
  }
 }

 public int get_limit_motor_info2(btRotationalLimitMotor2 limot,
  final btTransform transA,
  final btTransform transB, final btVector3 linVelA, final btVector3 linVelB,
  final btVector3 angVelA, final btVector3 angVelB,
  btConstraintInfo2 info, int row, final btVector3 ax1, int rotational,
  int rotAllowed) {
  int count = 0;
  int srow = row * info.rowskip;
  if (limot.m_currentLimit == 4) {
   float vel = rotational != 0 ? angVelA.dot(ax1) - angVelB.dot(ax1) : linVelA
    .dot(ax1) - linVelB
    .dot(ax1);
   calculateJacobi(limot, transA, transB, info, srow, ax1, rotational,
    rotAllowed);
   info.m_constraintError[srow].set(info.fps * limot.m_stopERP
    * limot.m_currentLimitError * (rotational != 0 ? -1 : 1));
   if (rotational != 0) {
    if (info.m_constraintError[srow].get() - vel * limot.m_stopERP > 0) {
     float bounceerror = -limot.m_bounce * vel;
     if (bounceerror > info.m_constraintError[srow].get()) {
      info.m_constraintError[srow].set(bounceerror);
     }
    }
   } else if (info.m_constraintError[srow].get() - vel * limot.m_stopERP < 0) {
    float bounceerror = -limot.m_bounce * vel;
    if (bounceerror < info.m_constraintError[srow].get()) {
     info.m_constraintError[srow].set(bounceerror);
    }
   }
   info.m_lowerLimit[srow].set(rotational != 0 ? 0 : -SIMD_INFINITY);
   info.m_upperLimit[srow].set(rotational != 0 ? SIMD_INFINITY : 0);
   info.cfm[srow].set(limot.m_stopCFM);
   srow += info.rowskip;
   ++count;
   calculateJacobi(limot, transA, transB, info, srow, ax1, rotational,
    rotAllowed);
   info.m_constraintError[srow].set(info.fps * limot.m_stopERP
    * limot.m_currentLimitErrorHi * (rotational != 0 ? -1 : 1));
   if (rotational != 0) {
    if (info.m_constraintError[srow].get() - vel * limot.m_stopERP < 0) {
     float bounceerror = -limot.m_bounce * vel;
     if (bounceerror < info.m_constraintError[srow].get()) {
      info.m_constraintError[srow].set(bounceerror);
     }
    }
   } else if (info.m_constraintError[srow].get() - vel * limot.m_stopERP > 0) {
    float bounceerror = -limot.m_bounce * vel;
    if (bounceerror > info.m_constraintError[srow].get()) {
     info.m_constraintError[srow].set(bounceerror);
    }
   }
   info.m_lowerLimit[srow].set(rotational != 0 ? -SIMD_INFINITY : 0);
   info.m_upperLimit[srow].set(rotational != 0 ? 0 : SIMD_INFINITY);
   info.cfm[srow].set(limot.m_stopCFM);
   srow += info.rowskip;
   ++count;
  } else if (limot.m_currentLimit == 3) {
   calculateJacobi(limot, transA, transB, info, srow, ax1, rotational,
    rotAllowed);
   info.m_constraintError[srow].set(info.fps * limot.m_stopERP
    * limot.m_currentLimitError * (rotational != 0 ? -1 : 1));
   info.m_lowerLimit[srow].set(-SIMD_INFINITY);
   info.m_upperLimit[srow].set(SIMD_INFINITY);
   info.cfm[srow].set(limot.m_stopCFM);
   srow += info.rowskip;
   ++count;
  }
  if (limot.m_enableMotor && !limot.m_servoMotor) {
   calculateJacobi(limot, transA, transB, info, srow, ax1, rotational,
    rotAllowed);
   float tag_vel = rotational != 0 ? limot.m_targetVelocity : -limot.m_targetVelocity;
   float mot_fact = getMotorFactor(limot.m_currentPosition,
    limot.m_loLimit,
    limot.m_hiLimit,
    tag_vel,
    info.fps * limot.m_motorERP);
   info.m_constraintError[srow].set(mot_fact * limot.m_targetVelocity);
   info.m_lowerLimit[srow].set(-limot.m_maxMotorForce);
   info.m_upperLimit[srow].set(limot.m_maxMotorForce);
   info.cfm[srow].set(limot.m_motorCFM);
   srow += info.rowskip;
   ++count;
  }
  if (limot.m_enableMotor && limot.m_servoMotor) {
   float error = limot.m_currentPosition - limot.m_servoTarget;
   calculateJacobi(limot, transA, transB, info, srow, ax1, rotational,
    rotAllowed);
   float targetvelocity = error < 0 ? -limot.m_targetVelocity : limot.m_targetVelocity;
   float tag_vel = -targetvelocity;
   float mot_fact;
   if (error != 0) {
    float lowLimit;
    float hiLimit;
    if (limot.m_loLimit > limot.m_hiLimit) {
     lowLimit = error > 0 ? limot.m_servoTarget : -SIMD_INFINITY;
     hiLimit = error < 0 ? limot.m_servoTarget : SIMD_INFINITY;
    } else {
     lowLimit = error > 0 && limot.m_servoTarget > limot.m_loLimit ? limot.m_servoTarget
      : limot.m_loLimit;
     hiLimit = error < 0 && limot.m_servoTarget < limot.m_hiLimit ? limot.m_servoTarget
      : limot.m_hiLimit;
    }
    mot_fact = getMotorFactor(limot.m_currentPosition, lowLimit, hiLimit,
     tag_vel, info.fps * limot.m_motorERP);
   } else {
    mot_fact = 0;
   }
   info.m_constraintError[srow].set(mot_fact * targetvelocity
    * (rotational != 0 ? -1 : 1));
   info.m_lowerLimit[srow].set(-limot.m_maxMotorForce);
   info.m_upperLimit[srow].set(limot.m_maxMotorForce);
   info.cfm[srow].set(limot.m_motorCFM);
   srow += info.rowskip;
   ++count;
  }
  if (limot.m_enableSpring) {
   float error = limot.m_currentPosition - limot.m_equilibriumPoint;
   calculateJacobi(limot, transA, transB, info, srow, ax1, rotational,
    rotAllowed);
   //float cfm = 1.0 / ((1.0/info.fps)*limot.m_springStiffness+ limot.m_springDamping);
   //if(cfm > 0.99999)
   //	cfm = 0.99999;
   //float erp = (1.0/info.fps)*limot.m_springStiffness / ((1.0/info.fps)*limot.m_springStiffness + limot.m_springDamping);
   //info.m_constraintError[srow] = info.fps * erp * error * (rotational ? -1.0 : 1.0);
   //info.m_lowerLimit[srow] = -SIMD_INFINITY;
   //info.m_upperLimit[srow] = SIMD_INFINITY;
   float dt = BT_ONE / info.fps;
   float kd = limot.m_springDamping;
   float ks = limot.m_springStiffness;
   float vel = rotational != 0 ? angVelA.dot(ax1) - angVelB.dot(ax1) : linVelA
    .dot(ax1) - linVelB
    .dot(ax1);
//		float erp = 0.1;
   float cfm = BT_ZERO;
   float mA = BT_ONE / m_rbA.getInvMass();
   float mB = BT_ONE / m_rbB.getInvMass();
   float m = mA > mB ? mB : mA;
   float angularfreq = btSqrt(ks / m);
   //limit stiffness (the spring should not be sampled faster that the quarter of its angular frequency)
   if (limot.m_springStiffnessLimited && 0.25 < angularfreq * dt) {
    ks = BT_ONE / dt / dt / (16.0f) * m;
   }
   //avoid damping that would blow up the spring
   if (limot.m_springDampingLimited && kd * dt > m) {
    kd = m / dt;
   }
   float fs = ks * error * dt;
   float fd = -kd * (vel) * (rotational != 0 ? -1 : 1) * dt;
   float f = (fs + fd);
   info.m_constraintError[srow].set((vel + f * (rotational != 0 ? -1 : 1)));
   float minf = f < fd ? f : fd;
   float maxf = f < fd ? fd : f;
   if (0 == rotational) {
    info.m_lowerLimit[srow].set(minf > 0 ? 0 : minf);
    info.m_upperLimit[srow].set(maxf < 0 ? 0 : maxf);
   } else {
    info.m_lowerLimit[srow].set(-maxf > 0 ? 0 : -maxf);
    info.m_upperLimit[srow].set(-minf < 0 ? 0 : -minf);
   }
   info.cfm[srow].set(cfm);
   srow += info.rowskip;
   ++count;
  }
  return count;
 }

 public int get_limit_motor_info2(btRotationalLimitMotor2 limot,
  final btTransform transA,
  final btTransform transB, final btVector3 linVelA, final btVector3 linVelB,
  final btVector3 angVelA, final btVector3 angVelB,
  btConstraintInfo2 info, int row, final btVector3 ax1, int rotational) {
  return get_limit_motor_info2(limot, transA, transB, linVelA, linVelB, angVelA,
   angVelB, info, row,
   ax1,
   rotational, 0);
 }

 public btGeneric6DofSpring2Constraint(btRigidBody rbA, btRigidBody rbB,
  final btTransform frameInA,
  final btTransform frameInB) {
  this(rbA, rbB, frameInA, frameInB, RO_XYZ);
 }

 public btGeneric6DofSpring2Constraint(btRigidBody rbA, btRigidBody rbB,
  final btTransform frameInA,
  final btTransform frameInB, int rotOrder) {
  super(D6_SPRING_2_CONSTRAINT_TYPE, rbA, rbB);
  m_frameInA.set(frameInA);
  m_frameInB.set(frameInB);
  m_rotateOrder = rotOrder;
  m_flags = 0;
  calculateTransforms();
 }

 public btGeneric6DofSpring2Constraint(btRigidBody rbB,
  final btTransform frameInB) {
  this(rbB, frameInB, RO_XYZ);
 }

 public btGeneric6DofSpring2Constraint(btRigidBody rbB,
  final btTransform frameInB, int rotOrder) {
  super(D6_SPRING_2_CONSTRAINT_TYPE, getFixedBody(), rbB);
  m_frameInB.set(frameInB);
  m_rotateOrder = rotOrder;
  m_flags = 0;
  ///not providing rigidbody A means implicitly using worldspace for body A
  m_frameInA.set(rbB.getCenterOfMassTransform()).mul(m_frameInB);
  calculateTransforms();
 }

 @Override
 public void buildJacobian() {
 }

 @Override
 public void getInfo1(btConstraintInfo1 info) {
  //prepare constraint
  calculateTransforms(m_rbA.getCenterOfMassTransform(), m_rbB
   .getCenterOfMassTransform());
  info.m_numConstraintRows = 0;
  info.nub = 0;
  int i;
  //test linear limits
  for (i = 0; i < 3; i++) {
   if (m_linearLimits.m_currentLimit[i] == 4) {
    info.m_numConstraintRows += 2;
   } else if (m_linearLimits.m_currentLimit[i] != 0) {
    info.m_numConstraintRows += 1;
   }
   if (m_linearLimits.m_enableMotor[i]) {
    info.m_numConstraintRows += 1;
   }
   if (m_linearLimits.m_enableSpring[i]) {
    info.m_numConstraintRows += 1;
   }
  }
  //test angular limits
  for (i = 0; i < 3; i++) {
   testAngularLimitMotor(i);
   if (m_angularLimits[i].m_currentLimit == 4) {
    info.m_numConstraintRows += 2;
   } else if (m_angularLimits[i].m_currentLimit != 0) {
    info.m_numConstraintRows += 1;
   }
   if (m_angularLimits[i].m_enableMotor) {
    info.m_numConstraintRows += 1;
   }
   if (m_angularLimits[i].m_enableSpring) {
    info.m_numConstraintRows += 1;
   }
  }
 }

 @Override
 public void getInfo2(btConstraintInfo2 info) {
  final btTransform transA = m_rbA.getCenterOfMassTransformPtr();
  final btTransform transB = m_rbB.getCenterOfMassTransformPtr();
  final btVector3 linVelA = m_rbA.getLinearVelocityPtr();
  final btVector3 linVelB = m_rbB.getLinearVelocityPtr();
  final btVector3 angVelA = m_rbA.getAngularVelocityPtr();
  final btVector3 angVelB = m_rbB.getAngularVelocityPtr();
  // for stability better to solve angular limits first
  int row = setAngularLimits(info, 0, transA, transB, linVelA, linVelB, angVelA,
   angVelB);
  setLinearLimits(info, row, transA, transB, linVelA, linVelB, angVelA, angVelB);
 }

 public btRotationalLimitMotor2 getRotationalLimitMotor(int index) {
  return m_angularLimits[index];
 }

 public btTranslationalLimitMotor2 getTranslationalLimitMotor() {
  return m_linearLimits;
 }

 // Calculates the global transform for the joint offset for body A an B, and also calculates the angle differences between the bodies.
 public void calculateTransforms(final btTransform transA,
  final btTransform transB) {
  m_calculatedTransformA.set(transA).mul(m_frameInA);
  m_calculatedTransformB.set(transB).mul(m_frameInB);
  calculateLinearInfo();
  calculateAngleInfo();
  float miA = getRigidBodyA().getInvMass();
  float miB = getRigidBodyB().getInvMass();
  m_hasStaticBody = (miA < SIMD_EPSILON) || (miB < SIMD_EPSILON);
  float miS = miA + miB;
  if (miS > (0.f)) {
   m_factA = miB / miS;
  } else {
   m_factA = (0.5f);
  }
  m_factB = (1.0f) - m_factA;
 }

 public final void calculateTransforms() {
  calculateTransforms(m_rbA.getCenterOfMassTransform(), m_rbB
   .getCenterOfMassTransform());
 }

 // Gets the global transform of the offset for body A
 public btTransform getCalculatedTransformA() {
  return new btTransform(m_calculatedTransformA);
 }
 // Gets the global transform of the offset for body B

 public btTransform getCalculatedTransformB() {
  return new btTransform(m_calculatedTransformB);
 }

 public btTransform getFrameOffsetA() {
  return new btTransform(m_frameInA);
 }

 public btTransform getFrameOffsetB() {
  return new btTransform(m_frameInB);
 }

 btTransform getFrameOffsetAPtr() {
  return m_frameInA;
 }

 btTransform getFrameOffsetBPtr() {
  return m_frameInB;
 }

 // Get the rotation axis in global coordinates ( btGeneric6DofSpring2Constraint.calculateTransforms() must be called previously )
 public btVector3 getAxis(int axis_index) {
  return new btVector3(m_calculatedAxis[axis_index]);
 }

 // Get the relative Euler angle ( btGeneric6DofSpring2Constraint.calculateTransforms() must be called previously )
 public float getAngle(int axis_index) {
  return m_calculatedAxisAngleDiff.getElement(axis_index);
 }

 // Get the relative position of the constraint pivot ( btGeneric6DofSpring2Constraint.calculateTransforms() must be called previously )
 public float getRelativePivotPosition(int axis_index) {
  return m_calculatedLinearDiff.getElement(axis_index);
 }

 public void setFrames(final btTransform frameA, final btTransform frameB) {
  m_frameInA.set(frameA);
  m_frameInB.set(frameB);
  buildJacobian();
  calculateTransforms();
 }

 final public void setLinearLowerLimit(final btVector3 linearLower) {
  m_linearLimits.m_lowerLimit.set(linearLower);
 }

 public void getLinearLowerLimit(final btVector3 linearLower) {
  linearLower.set(m_linearLimits.m_lowerLimit);
 }

 final public void setLinearUpperLimit(final btVector3 linearUpper) {
  m_linearLimits.m_upperLimit.set(linearUpper);
 }

 public void getLinearUpperLimit(final btVector3 linearUpper) {
  linearUpper.set(m_linearLimits.m_upperLimit);
 }

 final public void setAngularLowerLimit(final btVector3 angularLower) {
  for (int i = 0; i < 3; i++) {
   m_angularLimits[i].m_loLimit = btNormalizeAngle(angularLower.getElement(i));
  }
 }

 public void setAngularLowerLimitReversed(final btVector3 angularLower) {
  for (int i = 0; i < 3; i++) {
   m_angularLimits[i].m_hiLimit = btNormalizeAngle(-angularLower.getElement(i));
  }
 }

 public void getAngularLowerLimit(final btVector3 angularLower) {
  for (int i = 0; i < 3; i++) {
   angularLower.setElement(i, m_angularLimits[i].m_loLimit);
  }
 }

 public void getAngularLowerLimitReversed(final btVector3 angularLower) {
  for (int i = 0; i < 3; i++) {
   angularLower.setElement(i, -m_angularLimits[i].m_hiLimit);
  }
 }

 final public void setAngularUpperLimit(final btVector3 angularUpper) {
  for (int i = 0; i < 3; i++) {
   m_angularLimits[i].m_hiLimit = btNormalizeAngle(angularUpper.getElement(i));
  }
 }

 public void setAngularUpperLimitReversed(final btVector3 angularUpper) {
  for (int i = 0; i < 3; i++) {
   m_angularLimits[i].m_loLimit = btNormalizeAngle(-angularUpper.getElement(i));
  }
 }

 public void getAngularUpperLimit(final btVector3 angularUpper) {
  for (int i = 0; i < 3; i++) {
   angularUpper.setElement(i, m_angularLimits[i].m_hiLimit);
  }
 }

 public void getAngularUpperLimitReversed(final btVector3 angularUpper) {
  for (int i = 0; i < 3; i++) {
   angularUpper.setElement(i, -m_angularLimits[i].m_loLimit);
  }
 }

 //first 3 are linear, next 3 are angular
 public void setLimit(int axis, float lo, float hi) {
  if (axis < 3) {
   m_linearLimits.m_lowerLimit.setElement(axis, lo);
   m_linearLimits.m_upperLimit.setElement(axis, hi);
  } else {
   float _lo = btNormalizeAngle(lo);
   float _hi = btNormalizeAngle(hi);
   m_angularLimits[axis - 3].m_loLimit = _lo;
   m_angularLimits[axis - 3].m_hiLimit = _hi;
  }
 }

 public void setLimitReversed(int axis, float lo, float hi) {
  if (axis < 3) {
   m_linearLimits.m_lowerLimit.setElement(axis, lo);
   m_linearLimits.m_upperLimit.setElement(axis, hi);
  } else {
   float _lo = btNormalizeAngle(lo);
   float _hi = btNormalizeAngle(hi);
   m_angularLimits[axis - 3].m_hiLimit = -_lo;
   m_angularLimits[axis - 3].m_loLimit = -_hi;
  }
 }

 public boolean isLimited(int limitIndex) {
  if (limitIndex < 3) {
   return m_linearLimits.isLimited(limitIndex);
  }
  return m_angularLimits[limitIndex - 3].isLimited();
 }

 public void setRotationOrder(int order) {
  m_rotateOrder = order;
 }

 public int getRotationOrder() {
  return m_rotateOrder;
 }

 public void setAxis(final btVector3 axis1, final btVector3 axis2) {
  final btVector3 zAxis = new btVector3(axis1).normalize();
  final btVector3 yAxis = new btVector3(axis2).normalize();
  final btVector3 xAxis = new btVector3(yAxis).cross(zAxis); // we want right coordinate system
  final btTransform frameInW = new btTransform();
  frameInW.setIdentity();
  frameInW.set3x3(new btMatrix3x3(xAxis.x, yAxis.y, zAxis.z,
   xAxis.y, yAxis.y, zAxis.y,
   xAxis.z, yAxis.z, zAxis.z));
  // now get constraint frame in local coordinate systems
  m_frameInA.set(m_rbA.getCenterOfMassTransform().invert().mul(frameInW));
  m_frameInB.set(m_rbB.getCenterOfMassTransform().invert().mul(frameInW));
  calculateTransforms();
 }

 public void setBounce(int index, float bounce) {
  assert ((index >= 0) && (index < 6));
  if (index < 3) {
   m_linearLimits.m_bounce.setElement(index, bounce);
  } else {
   m_angularLimits[index - 3].m_bounce = bounce;
  }
 }

 public void enableMotor(int index, boolean onOff) {
  assert ((index >= 0) && (index < 6));
  if (index < 3) {
   m_linearLimits.m_enableMotor[index] = onOff;
  } else {
   m_angularLimits[index - 3].m_enableMotor = onOff;
  }
 }
// set the type of the motor (servo or not) (the motor has to be turned on for servo also)

 public void setServo(int index, boolean onOff) {
  assert ((index >= 0) && (index < 6));
  if (index < 3) {
   m_linearLimits.m_servoMotor[index] = onOff;
  } else {
   m_angularLimits[index - 3].m_servoMotor = onOff;
  }
 }

 public void setTargetVelocity(int index, float velocity) {
  assert ((index >= 0) && (index < 6));
  if (index < 3) {
   m_linearLimits.m_targetVelocity.setElement(index, velocity);
  } else {
   m_angularLimits[index - 3].m_targetVelocity = velocity;
  }
 }

 public void setServoTarget(int index, float target) {
  assert ((index >= 0) && (index < 6));
  if (index < 3) {
   m_linearLimits.m_servoTarget.setElement(index, target);
  } else {
   m_angularLimits[index - 3].m_servoTarget = target;
  }
 }

 public void setMaxMotorForce(int index, float force) {
  assert ((index >= 0) && (index < 6));
  if (index < 3) {
   m_linearLimits.m_maxMotorForce.setElement(index, force);
  } else {
   m_angularLimits[index - 3].m_maxMotorForce = force;
  }
 }

 final public void enableSpring(int index, boolean onOff) {
  assert ((index >= 0) && (index < 6));
  if (index < 3) {
   m_linearLimits.m_enableSpring[index] = onOff;
  } else {
   m_angularLimits[index - 3].m_enableSpring = onOff;
  }
 }

 // if limitIfNeeded is true the system will automatically limit the stiffness in necessary situations where otherwise the spring would move unrealistically too widely	
 final public void setStiffness(int index, float stiffness) {
  setStiffness(index, stiffness, true);
 }

 public void setStiffness(int index, float stiffness, boolean limitIfNeeded) {
  assert ((index >= 0) && (index < 6));
  if (index < 3) {
   m_linearLimits.m_springStiffness.setElement(index, stiffness);
   m_linearLimits.m_springStiffnessLimited[index] = limitIfNeeded;
  } else {
   m_angularLimits[index - 3].m_springStiffness = stiffness;
   m_angularLimits[index - 3].m_springStiffnessLimited = limitIfNeeded;
  }
 }

 final public void setDamping(int index, float damping) {
  setDamping(index, damping, true);
 }

// if limitIfNeeded is true the system will automatically limit the damping in necessary situations where otherwise the spring would blow up	void setDamping(int index, float damping, boolean limitIfNeeded = true); 
// set the current constraint position/orientation as an equilibrium point for all DOF
 public void setDamping(int index, float damping, boolean limitIfNeeded) {
  assert ((index >= 0) && (index < 6));
  if (index < 3) {
   m_linearLimits.m_springDamping.setElement(index, damping);
   m_linearLimits.m_springDampingLimited[index] = limitIfNeeded;
  } else {
   m_angularLimits[index - 3].m_springDamping = damping;
   m_angularLimits[index - 3].m_springDampingLimited = limitIfNeeded;
  }
 }

 final public void setEquilibriumPoint() {
  calculateTransforms();
  int i;
  m_linearLimits.m_equilibriumPoint.set(m_calculatedLinearDiff);
  for (i = 0; i < 3; i++) {
   m_angularLimits[i].m_equilibriumPoint = m_calculatedAxisAngleDiff.getElement(
    i);
  }
 }

// set the current constraint position/orientation as an equilibrium point for given DOF
 public void setEquilibriumPoint(int index) {
  assert ((index >= 0) && (index < 6));
  calculateTransforms();
  if (index < 3) {
   m_linearLimits.m_equilibriumPoint.setElement(index, m_calculatedLinearDiff
    .getElement(index));
  } else {
   m_angularLimits[index - 3].m_equilibriumPoint = m_calculatedAxisAngleDiff
    .getElement(index - 3);
  }
 }

 public void setEquilibriumPoint(int index, float val) {
  assert ((index >= 0) && (index < 6));
  if (index < 3) {
   m_linearLimits.m_equilibriumPoint.setElement(index, val);
  } else {
   m_angularLimits[index - 3].m_equilibriumPoint = val;
  }
 }

 //override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
 //If no axis is provided, it uses the default axis for this constraint.
 @Override
 public void setParam(int num, float value, int axis) {
  if ((axis >= 0) && (axis < 3)) {
   switch (num) {
    case BT_CONSTRAINT_STOP_ERP:
     m_linearLimits.m_stopERP.setElement(axis, value);
     m_flags |= BT_6DOF_FLAGS_ERP_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
     break;
    case BT_CONSTRAINT_STOP_CFM:
     m_linearLimits.m_stopCFM.setElement(axis, value);
     m_flags |= BT_6DOF_FLAGS_CFM_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
     break;
    case BT_CONSTRAINT_ERP:
     m_linearLimits.m_motorERP.setElement(axis, value);
     m_flags |= BT_6DOF_FLAGS_ERP_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
     break;
    case BT_CONSTRAINT_CFM:
     m_linearLimits.m_motorCFM.setElement(axis, value);
     m_flags |= BT_6DOF_FLAGS_CFM_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
     break;
    default:
     assert (false);
   }
  } else if ((axis >= 3) && (axis < 6)) {
   switch (num) {
    case BT_CONSTRAINT_STOP_ERP:
     m_angularLimits[axis - 3].m_stopERP = value;
     m_flags |= BT_6DOF_FLAGS_ERP_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
     break;
    case BT_CONSTRAINT_STOP_CFM:
     m_angularLimits[axis - 3].m_stopCFM = value;
     m_flags |= BT_6DOF_FLAGS_CFM_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
     break;
    case BT_CONSTRAINT_ERP:
     m_angularLimits[axis - 3].m_motorERP = value;
     m_flags |= BT_6DOF_FLAGS_ERP_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
     break;
    case BT_CONSTRAINT_CFM:
     m_angularLimits[axis - 3].m_motorCFM = value;
     m_flags |= BT_6DOF_FLAGS_CFM_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
     break;
    default:
     assert (false);
   }
  } else {
   assert (false);
  }
 }

 @Override
 public float getParam(int num, int axis) {
  float retVal = 0;
  if ((axis >= 0) && (axis < 3)) {
   switch (num) {
    case BT_CONSTRAINT_STOP_ERP:
     assert ((m_flags & (BT_6DOF_FLAGS_ERP_STOP2 << (axis
      * BT_6DOF_FLAGS_AXIS_SHIFT2))) != 0);
     retVal = m_linearLimits.m_stopERP.getElement(axis);
     break;
    case BT_CONSTRAINT_STOP_CFM:
     assert ((m_flags & (BT_6DOF_FLAGS_CFM_STOP2 << (axis
      * BT_6DOF_FLAGS_AXIS_SHIFT2))) != 0);
     retVal = m_linearLimits.m_stopCFM.getElement(axis);
     break;
    case BT_CONSTRAINT_ERP:
     assert ((m_flags & (BT_6DOF_FLAGS_ERP_MOTO2 << (axis
      * BT_6DOF_FLAGS_AXIS_SHIFT2))) != 0);
     retVal = m_linearLimits.m_motorERP.getElement(axis);
     break;
    case BT_CONSTRAINT_CFM:
     assert ((m_flags & (BT_6DOF_FLAGS_CFM_MOTO2 << (axis
      * BT_6DOF_FLAGS_AXIS_SHIFT2))) != 0);
     retVal = m_linearLimits.m_motorCFM.getElement(axis);
     break;
    default:
     assert (false);
   }
  } else if ((axis >= 3) && (axis < 6)) {
   switch (num) {
    case BT_CONSTRAINT_STOP_ERP:
     assert ((m_flags & (BT_6DOF_FLAGS_ERP_STOP2 << (axis
      * BT_6DOF_FLAGS_AXIS_SHIFT2))) != 0);
     retVal = m_angularLimits[axis - 3].m_stopERP;
     break;
    case BT_CONSTRAINT_STOP_CFM:
     assert ((m_flags & (BT_6DOF_FLAGS_CFM_STOP2 << (axis
      * BT_6DOF_FLAGS_AXIS_SHIFT2))) != 0);
     retVal = m_angularLimits[axis - 3].m_stopCFM;
     break;
    case BT_CONSTRAINT_ERP:
     assert ((m_flags & (BT_6DOF_FLAGS_ERP_MOTO2 << (axis
      * BT_6DOF_FLAGS_AXIS_SHIFT2))) != 0);
     retVal = m_angularLimits[axis - 3].m_motorERP;
     break;
    case BT_CONSTRAINT_CFM:
     assert ((m_flags & (BT_6DOF_FLAGS_CFM_MOTO2 << (axis
      * BT_6DOF_FLAGS_AXIS_SHIFT2))) != 0);
     retVal = m_angularLimits[axis - 3].m_motorCFM;
     break;
    default:
     assert (false);
   }
  } else {
   assert (false);
  }
  return retVal;
 }

 public static float btGetMatrixElem(final btMatrix3x3 mat, int index) {
  int i = index % 3;
  int j = index / 3;
  return mat.getElement(i, j);
 }

 public static boolean matrixToEulerXYZ(final btMatrix3x3 mat,
  final btVector3 xyz) {
  // rot =  cy*cz          -cy*sz           sy
  //        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
  //       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy
  float fi = btGetMatrixElem(mat, 2);
  if (fi < (1.0f)) {
   if (fi > (-1.0f)) {
    xyz.x = btAtan2(-btGetMatrixElem(mat, 5), btGetMatrixElem(mat, 8));
    xyz.y = btAsin(btGetMatrixElem(mat, 2));
    xyz.z = btAtan2(-btGetMatrixElem(mat, 1), btGetMatrixElem(mat, 0));
    return true;
   } else {
    // WARNING.  Not unique.  XA - ZA = -atan2(r10,r11)
    xyz.x = -btAtan2(btGetMatrixElem(mat, 3), btGetMatrixElem(mat, 4));
    xyz.y = -SIMD_HALF_PI;
    xyz.z = (0.0f);
    return false;
   }
  } else {
   // WARNING.  Not unique.  XAngle + ZAngle = atan2(r10,r11)
   xyz.x = btAtan2(btGetMatrixElem(mat, 3), btGetMatrixElem(mat, 4));
   xyz.y = SIMD_HALF_PI;
   xyz.z = 0.0f;
  }
  return false;
 }

 public static boolean matrixToEulerXZY(final btMatrix3x3 mat,
  final btVector3 xyz) {
  // rot =  cy*cz          -sz           sy*cz
  //        cy*cx*sz+sx*sy  cx*cz        sy*cx*sz-cy*sx
  //        cy*sx*sz-cx*sy  sx*cz        sy*sx*sz+cx*cy
  float fi = btGetMatrixElem(mat, 1);
  if (fi < (1.0f)) {
   if (fi > (-1.0f)) {
    xyz.x = btAtan2(btGetMatrixElem(mat, 7), btGetMatrixElem(mat, 4));
    xyz.y = btAtan2(btGetMatrixElem(mat, 2), btGetMatrixElem(mat, 0));
    xyz.z = btAsin(-btGetMatrixElem(mat, 1));
    return true;
   } else {
    xyz.x = -btAtan2(-btGetMatrixElem(mat, 6), btGetMatrixElem(mat, 8));
    xyz.y = (0.0f);
    xyz.z = SIMD_HALF_PI;
    return false;
   }
  } else {
   xyz.x = btAtan2(-btGetMatrixElem(mat, 6), btGetMatrixElem(mat, 8));
   xyz.y = 0.0f;
   xyz.z = -SIMD_HALF_PI;
  }
  return false;
 }

 public static boolean matrixToEulerYXZ(final btMatrix3x3 mat,
  final btVector3 xyz) {
  // rot =  cy*cz+sy*sx*sz  cz*sy*sx-cy*sz  cx*sy
  //        cx*sz           cx*cz           -sx
  //        cy*sx*sz-cz*sy  sy*sz+cy*cz*sx  cy*cx
  float fi = btGetMatrixElem(mat, 5);
  if (fi < (1.0f)) {
   if (fi > (-1.0f)) {
    xyz.x = btAsin(-btGetMatrixElem(mat, 5));
    xyz.y = btAtan2(btGetMatrixElem(mat, 2), btGetMatrixElem(mat, 8));
    xyz.z = btAtan2(btGetMatrixElem(mat, 3), btGetMatrixElem(mat, 4));
    return true;
   } else {
    xyz.x = SIMD_HALF_PI;
    xyz.y = -btAtan2(-btGetMatrixElem(mat, 1), btGetMatrixElem(mat, 0));
    xyz.z = (0.0f);
    return false;
   }
  } else {
   xyz.x = -SIMD_HALF_PI;
   xyz.y = btAtan2(-btGetMatrixElem(mat, 1), btGetMatrixElem(mat, 0));
   xyz.z = 0.0f;
  }
  return false;
 }

 public static boolean matrixToEulerYZX(final btMatrix3x3 mat,
  final btVector3 xyz) {
  // rot =  cy*cz   sy*sx-cy*cx*sz   cx*sy+cy*sz*sx
  //        sz           cz*cx           -cz*sx
  //        -cz*sy  cy*sx+cx*sy*sz   cy*cx-sy*sz*sx
  float fi = btGetMatrixElem(mat, 3);
  if (fi < (1.0f)) {
   if (fi > (-1.0f)) {
    xyz.x = btAtan2(-btGetMatrixElem(mat, 5), btGetMatrixElem(mat, 4));
    xyz.y = btAtan2(-btGetMatrixElem(mat, 6), btGetMatrixElem(mat, 0));
    xyz.z = btAsin(btGetMatrixElem(mat, 3));
    return true;
   } else {
    xyz.x = (0.0f);
    xyz.y = -btAtan2(btGetMatrixElem(mat, 7), btGetMatrixElem(mat, 8));
    xyz.z = -SIMD_HALF_PI;
    return false;
   }
  } else {
   xyz.x = (0.0f);
   xyz.y = btAtan2(btGetMatrixElem(mat, 7), btGetMatrixElem(mat, 8));
   xyz.z = SIMD_HALF_PI;
  }
  return false;
 }

 public static boolean matrixToEulerZXY(final btMatrix3x3 mat,
  final btVector3 xyz) {
  // rot =  cz*cy-sz*sx*sy    -cx*sz   cz*sy+cy*sz*sx
  //        cy*sz+cz*sx*sy     cz*cx   sz*sy-cz*xy*sx
  //        -cx*sy              sx     cx*cy
  float fi = btGetMatrixElem(mat, 7);
  if (fi < (1.0f)) {
   if (fi > (-1.0f)) {
    xyz.x = btAsin(btGetMatrixElem(mat, 7));
    xyz.y = btAtan2(-btGetMatrixElem(mat, 6), btGetMatrixElem(mat, 8));
    xyz.z = btAtan2(-btGetMatrixElem(mat, 1), btGetMatrixElem(mat, 4));
    return true;
   } else {
    xyz.x = -SIMD_HALF_PI;
    xyz.y = (0.0f);
    xyz.z = -btAtan2(btGetMatrixElem(mat, 2), btGetMatrixElem(mat, 0));
    return false;
   }
  } else {
   xyz.x = SIMD_HALF_PI;
   xyz.y = 0.0f;
   xyz.z = btAtan2(btGetMatrixElem(mat, 2), btGetMatrixElem(mat, 0));
  }
  return false;
 }

 public static boolean matrixToEulerZYX(final btMatrix3x3 mat,
  final btVector3 xyz) {
  // rot =  cz*cy   cz*sy*sx-cx*sz   sz*sx+cz*cx*sy
  //        cy*sz   cz*cx+sz*sy*sx   cx*sz*sy-cz*sx
  //        -sy          cy*sx         cy*cx
  float fi = btGetMatrixElem(mat, 6);
  if (fi < (1.0f)) {
   if (fi > (-1.0f)) {
    xyz.x = btAtan2(btGetMatrixElem(mat, 7), btGetMatrixElem(mat, 8));
    xyz.y = btAsin(-btGetMatrixElem(mat, 6));
    xyz.z = btAtan2(btGetMatrixElem(mat, 3), btGetMatrixElem(mat, 0));
    return true;
   } else {
    xyz.x = (0.0f);
    xyz.y = SIMD_HALF_PI;
    xyz.z = -btAtan2(btGetMatrixElem(mat, 1), btGetMatrixElem(mat, 2));
    return false;
   }
  } else {
   xyz.x = (0.0f);
   xyz.y = -SIMD_HALF_PI;
   xyz.z = btAtan2(-btGetMatrixElem(mat, 1), -btGetMatrixElem(mat, 2));
  }
  return false;
 }

};
