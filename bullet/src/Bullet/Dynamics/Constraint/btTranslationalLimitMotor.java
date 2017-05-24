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
/// 2009 March: btGeneric6DofConstraint refactored by Roman Ponomarev
/// Added support for generic constraint solver through getInfo1/getInfo2 methods

/*
2007-09-09
btGeneric6DofConstraint Refactored by Francisco Le?n
email: projectileman@yahoo.com
http://gimpact.sf.net
 */
package Bullet.Dynamics.Constraint;

import Bullet.Dynamics.btRigidBody;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btTranslationalLimitMotor  implements Serializable {

 public final btVector3 m_lowerLimit = new btVector3();//!< the constraint lower limits
public  final btVector3 m_upperLimit = new btVector3();//!< the constraint upper limits
public  final btVector3 m_accumulatedImpulse = new btVector3();
 //! Linear_Limit_parameters
 //!@{
public  float m_limitSoftness;//!< Softness for linear limit
public  float m_damping;//!< Damping for linear limit
public  float m_restitution;//! Bounce parameter for linear limit
public  final btVector3 m_normalCFM = new btVector3();//!< Constraint force mixing factor
public  final btVector3 m_stopERP = new btVector3();//!< Error tolerance factor when joint is at limit
public  final btVector3 m_stopCFM = new btVector3();//!< Constraint force mixing factor when joint is at limit
 //!@}
public  final boolean[] m_enableMotor = new boolean[3];
public  final btVector3 m_targetVelocity = new btVector3();//!< target motor velocity
public  final btVector3 m_maxMotorForce = new btVector3();//!< max force on motor
public  final btVector3 m_currentLimitError = new btVector3();//!  How much is violated this limit
public  final btVector3 m_currentLinearDiff = new btVector3();//!  Current relative offset of constraint frames
public  final int[] m_currentLimit = new int[3];//!< 0=free, 1=at lower limit, 2=at upper limit

public  btTranslationalLimitMotor() {
  m_lowerLimit.set(0.f, 0.f, 0.f);
  m_upperLimit.set(0.f, 0.f, 0.f);
  m_accumulatedImpulse.set(0.f, 0.f, 0.f);
  m_normalCFM.set(0.f, 0.f, 0.f);
  m_stopERP.set(0.2f, 0.2f, 0.2f);
  m_stopCFM.set(0.f, 0.f, 0.f);
  m_limitSoftness = 0.7f;
  m_damping = (1.0f);
  m_restitution = (0.5f);
 }

public  btTranslationalLimitMotor(btTranslationalLimitMotor other) {
  m_lowerLimit.set(other.m_lowerLimit);
  m_upperLimit.set(other.m_upperLimit);
  m_accumulatedImpulse.set(other.m_accumulatedImpulse);
  m_limitSoftness = other.m_limitSoftness;
  m_damping = other.m_damping;
  m_restitution = other.m_restitution;
  m_normalCFM.set(other.m_normalCFM);
  m_stopERP.set(other.m_stopERP);
  m_stopCFM.set(other.m_stopCFM);
  m_targetVelocity.set(other.m_targetVelocity);
  m_maxMotorForce.set(other.m_maxMotorForce);
  for (int i = 0; i < 3; i++) {
   m_enableMotor[i] = other.m_enableMotor[i];
  }
 }

 //! Test limit
 /*!
    - free means upper < lower,
    - locked means upper == lower
    - limited means upper > lower
    - limitIndex: first 3 are linear, next 3 are angular
  */
public  boolean isLimited(int limitIndex) {
  return (m_upperLimit.getElement(limitIndex) >= m_lowerLimit.getElement(limitIndex));
 }

public  boolean needApplyForce(int limitIndex) {
  return !(m_currentLimit[limitIndex] == 0 && !m_enableMotor[limitIndex]);
 }

public  int testLimitValue(final int limitIndex, final float test_value) {
  float loLimit = m_lowerLimit.getElement(limitIndex);
  float hiLimit = m_upperLimit.getElement(limitIndex);
  if (loLimit > hiLimit) {
   m_currentLimit[limitIndex] = 0;//Free from violation
   m_currentLimitError.setElement(limitIndex, 0.f);
   return 0;
  }
  if (test_value < loLimit) {
   m_currentLimit[limitIndex] = 2;//low limit violation
   m_currentLimitError.setElement(limitIndex, test_value - loLimit);
   return 2;
  } else if (test_value > hiLimit) {
   m_currentLimit[limitIndex] = 1;//High limit violation
   m_currentLimitError.setElement(limitIndex, test_value - hiLimit);
   return 1;
  }
  m_currentLimit[limitIndex] = 0;//Free from violation
  m_currentLimitError.setElement(limitIndex, 0.f);
  return 0;
 }

public  float solveLinearAxis(
  float timeStep,
  float jacDiagABInv,
  btRigidBody body1, final btVector3 pointInA,
  btRigidBody body2, final btVector3 pointInB,
  int limit_index, final btVector3 axis_normal_on_a, final btVector3 anchorPos) {
  ///find relative velocity
  //    btVector3 rel_pos1 = pointInA - body1.getCenterOfMassPosition();
  //    btVector3 rel_pos2 = pointInB - body2.getCenterOfMassPosition();
  final btVector3 rel_pos1 = new btVector3(anchorPos).sub(body1.getCenterOfMassPosition());
  final btVector3 rel_pos2 = new btVector3(anchorPos).sub(body2.getCenterOfMassPosition());
  final btVector3 vel1 = body1.getVelocityInLocalPoint(rel_pos1);
  final btVector3 vel2 = body2.getVelocityInLocalPoint(rel_pos2);
  final btVector3 vel = new btVector3(vel1).sub(vel2);
  float rel_vel = axis_normal_on_a.dot(vel);
  /// apply displacement correction
  //positional error (zeroth order error)
  float depth = -(new btVector3(pointInA).sub(pointInB)).dot(axis_normal_on_a);
  float lo = (-BT_LARGE_FLOAT);
  float hi = (BT_LARGE_FLOAT);
  float minLimit = m_lowerLimit.getElement(limit_index);
  float maxLimit = m_upperLimit.getElement(limit_index);
  //handle the limits
  if (minLimit < maxLimit) {
   {
    if (depth > maxLimit) {
     depth -= maxLimit;
     lo = (0.f);
    } else if (depth < minLimit) {
     depth -= minLimit;
     hi = (0.f);
    } else {
     return 0.0f;
    }
   }
  }
  float normalImpulse = m_limitSoftness * (m_restitution * depth / timeStep - m_damping * rel_vel) *
   jacDiagABInv;
  float oldNormalImpulse = m_accumulatedImpulse.getElement(limit_index);
  float sum = oldNormalImpulse + normalImpulse;
  m_accumulatedImpulse.setElement(limit_index, sum > hi ? (0.f) : sum < lo ? (0.f) : sum);
  normalImpulse = m_accumulatedImpulse.getElement(limit_index) - oldNormalImpulse;
  final btVector3 impulse_vector = new btVector3(axis_normal_on_a).scale(normalImpulse);
  body1.applyImpulse(impulse_vector, rel_pos1);
  body2.applyImpulse(new btVector3(impulse_vector).negate(), rel_pos2);
  return normalImpulse;
 }
};
