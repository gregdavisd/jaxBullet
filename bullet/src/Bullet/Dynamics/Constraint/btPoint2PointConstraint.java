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

package Bullet.Dynamics.Constraint;

import static Bullet.Dynamics.Constraint.btPoint2PointFlags.BT_P2P_FLAGS_CFM;
import static Bullet.Dynamics.Constraint.btPoint2PointFlags.BT_P2P_FLAGS_ERP;
import Bullet.Dynamics.btJacobianEntry;
import Bullet.Dynamics.btRigidBody;
import static Bullet.LinearMath.btScalar.SIMD_INFINITY;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btPoint2PointConstraint extends btTypedConstraint  implements Serializable {

 final btJacobianEntry[] m_jac = new btJacobianEntry[3]; //3 orthogonal linear constraints
 final btVector3 m_pivotInA = new btVector3();
 final btVector3 m_pivotInB = new btVector3();
 int m_flags;
 float m_erp;
 float m_cfm;
 ///for backwards compatibility during the transition to 'getInfo/getInfo2'
 public boolean m_useSolveConstraintObsolete;
 public final btConstraintSetting m_setting = new btConstraintSetting();

 public btPoint2PointConstraint(btRigidBody rbA, btRigidBody rbB, final btVector3 pivotInA,
  final btVector3 pivotInB) {
  super(POINT2POINT_CONSTRAINT_TYPE, rbA, rbB);
  m_pivotInA.set(pivotInA);
  m_pivotInB.set(pivotInB);
  m_flags = 0;
  m_useSolveConstraintObsolete = false;
 }

 public btPoint2PointConstraint(btRigidBody rbA, final btVector3 pivotInA) {
  super(POINT2POINT_CONSTRAINT_TYPE, rbA);
  m_pivotInA.set(pivotInA);
  rbA.getCenterOfMassTransform().transform(m_pivotInB.set(pivotInA));
  m_flags = 0;
  m_useSolveConstraintObsolete = false;
 }

 @Override
 public void buildJacobian() {
  ///we need it for both methods
  {
   m_appliedImpulse = (0.f);
   final btVector3 normal = new btVector3();
   for (int i = 0; i < 3; i++) {
    normal.setElement(i, 1);
    m_jac[i] = new btJacobianEntry(
     m_rbA.getCenterOfMassTransform().getBasis().transpose(),
     m_rbB.getCenterOfMassTransform().getBasis().transpose(),
     m_rbA.getCenterOfMassTransform().transform(new btVector3(m_pivotInA)).sub(m_rbA
     .getCenterOfMassPosition()),
     m_rbB.getCenterOfMassTransform().transform(new btVector3(m_pivotInB)).sub(m_rbB
     .getCenterOfMassPosition()),
     normal,
     m_rbA.getInvInertiaDiagLocal(),
     m_rbA.getInvMass(),
     m_rbB.getInvInertiaDiagLocal(),
     m_rbB.getInvMass());
    normal.setElement(i, 0f);
   }
  }
 }

 @Override
 public void getInfo1(btConstraintInfo1 info) {
  getInfo1NonVirtual(info);
 }

 public final void getInfo1NonVirtual(btConstraintInfo1 info) {
  if (m_useSolveConstraintObsolete) {
   info.m_numConstraintRows = 0;
   info.nub = 0;
  } else {
   info.m_numConstraintRows = 3;
   info.nub = 3;
  }
 }

 @Override
 public void getInfo2(btConstraintInfo2 info) {
  getInfo2NonVirtual(info, m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform());
 }

 public final void getInfo2NonVirtual(btConstraintInfo2 info, final btTransform body0_trans,
  final btTransform body1_trans) {
  assert(!m_useSolveConstraintObsolete);
  //retrieve matrices
  // anchor points in global coordinates with respect to body PORs.
  // set jacobian
  info.m_J1linearAxis[0].x = 1f;
  info.m_J1linearAxis[info.rowskip].y = 1f;
  info.m_J1linearAxis[2 * info.rowskip].z = 1f;
  final btVector3 a1 = body0_trans.transform3x3(getPivotInA());
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
  final btVector3 a2 = body1_trans.transform3x3(getPivotInB());
  {
   //	btVector3 a2n = -a2;
   final btVector3 angular0 = (info.m_J2angularAxis[0]);
   final btVector3 angular1 = (info.m_J2angularAxis[info.rowskip]);
   final btVector3 angular2 = (info.m_J2angularAxis[2 * info.rowskip]);
   a2.getSkewSymmetricMatrix(angular0, angular1, angular2);
  }
  // set right hand side
  float currERP = (m_flags & BT_P2P_FLAGS_ERP) != 0 ? m_erp : info.erp;
  float k = info.fps * currERP;
  int j;
  final btVector3 body0_origin = body0_trans.getOrigin();
  final btVector3 body1_origin = body1_trans.getOrigin();
  for (j = 0; j < 3; j++) {
   info.m_constraintError[j * info.rowskip].set(
    k * (a2.getElement(j) + body1_origin.getElement(j) - a1.getElement(j) - body0_origin.getElement(
    j)));
  }
  if ((m_flags & BT_P2P_FLAGS_CFM) != 0) {
   for (j = 0; j < 3; j++) {
    info.cfm[j * info.rowskip].set(m_cfm);
   }
  }
  float impulseClamp = m_setting.m_impulseClamp;//
  for (j = 0; j < 3; j++) {
   if (m_setting.m_impulseClamp > 0) {
    info.m_lowerLimit[j * info.rowskip].set(-impulseClamp);
    info.m_upperLimit[j * info.rowskip].set(impulseClamp);
   }
  }
  info.m_damping = m_setting.m_damping;
 }

 public void updateRHS(float timeStep) {
 }

 public void setPivotA(final btVector3 pivotA) {
  m_pivotInA.set(pivotA);
 }

 public void setPivotB(final btVector3 pivotB) {
  m_pivotInB.set(pivotB);
 }

 public btVector3 getPivotInA() {
  return new btVector3(m_pivotInA);
 }

 public btVector3 getPivotInB() {
  return new btVector3(m_pivotInB);
 }

 ///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
 ///If no axis is provided, it uses the default axis for this constraint.
 @Override
 public void setParam(int num, float value, int axis) {
  if (axis != -1) {
   assert(false);
  } else {
   switch (num) {
    case BT_CONSTRAINT_ERP:
    case BT_CONSTRAINT_STOP_ERP:
     m_erp = value;
     m_flags |= BT_P2P_FLAGS_ERP;
     break;
    case BT_CONSTRAINT_CFM:
    case BT_CONSTRAINT_STOP_CFM:
     m_cfm = value;
     m_flags |= BT_P2P_FLAGS_CFM;
     break;
    default:
     assert(false);
   }
  }
 }

 ///return the local value of parameter
 @Override
 public float getParam(int num, int axis) {
  float retVal = SIMD_INFINITY;
  if (axis != -1) {
   assert(false);
  } else {
   switch (num) {
    case BT_CONSTRAINT_ERP:
    case BT_CONSTRAINT_STOP_ERP:
     assert((m_flags & BT_P2P_FLAGS_ERP) != 0);
     retVal = m_erp;
     break;
    case BT_CONSTRAINT_CFM:
    case BT_CONSTRAINT_STOP_CFM:
     assert((m_flags & BT_P2P_FLAGS_CFM) != 0);
     retVal = m_cfm;
     break;
    default:
     assert(false);
   }
  }
  return retVal;
 }

 public int getFlags() {
  return m_flags;
 }
};
