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
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import static Bullet.LinearMath.btScalar.SIMD_2_PI;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.SIMD_PI;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 * Rotation Limit structure for generic joints
 *
 * @author Gregery Barton
 */
public class btRotationalLimitMotor implements Serializable {

 //! limit_parameters
 //!@{
 public float m_loLimit;//!< joint limit
 public float m_hiLimit;//!< joint limit
 public float m_targetVelocity;//!< target motor velocity
 public float m_maxMotorForce;//!< max force on motor
 public float m_maxLimitForce;//!< max force on limit
 public float m_damping;//!< Damping.
 public float m_limitSoftness;//! Relaxation factor
 public float m_normalCFM;//!< Constraint force mixing factor
 public float m_stopERP;//!< Error tolerance factor when joint is at limit
 public float m_stopCFM;//!< Constraint force mixing factor when joint is at limit
 public float m_bounce;//!< restitution factor
 public boolean m_enableMotor;
 //!@}
 //! temp_variables
 //!@{
 public float m_currentLimitError;//!  How much is violated this limit
 public float m_currentPosition;     //!  current value of angle 
 public int m_currentLimit;//!< 0=free, 1=at lo limit, 2=at hi limit
 public float m_accumulatedImpulse;
 //!@}

 btRotationalLimitMotor() {
  m_accumulatedImpulse = 0.f;
  m_targetVelocity = 0;
  m_maxMotorForce = 0.1f;
  m_maxLimitForce = 300.0f;
  m_loLimit = 1.0f;
  m_hiLimit = -1.0f;
  m_normalCFM = 0.f;
  m_stopERP = 0.2f;
  m_stopCFM = 0.f;
  m_bounce = 0.0f;
  m_damping = 1.0f;
  m_limitSoftness = 0.5f;
  m_currentLimit = 0;
  m_currentLimitError = 0;
  m_enableMotor = false;
 }

 btRotationalLimitMotor(btRotationalLimitMotor limot) {
  m_targetVelocity = limot.m_targetVelocity;
  m_maxMotorForce = limot.m_maxMotorForce;
  m_limitSoftness = limot.m_limitSoftness;
  m_loLimit = limot.m_loLimit;
  m_hiLimit = limot.m_hiLimit;
  m_normalCFM = limot.m_normalCFM;
  m_stopERP = limot.m_stopERP;
  m_stopCFM = limot.m_stopCFM;
  m_bounce = limot.m_bounce;
  m_currentLimit = limot.m_currentLimit;
  m_currentLimitError = limot.m_currentLimitError;
  m_enableMotor = limot.m_enableMotor;
 }

 //! Is limited
 boolean isLimited() {
  return m_loLimit <= m_hiLimit;
 }

 //! Need apply correction
 boolean needApplyTorques() {
  return !(m_currentLimit == 0 && m_enableMotor == false);
 }

 //! calculates  error
 /*
  * !
  * calculates m_currentLimit and m_currentLimitError.
  */
 int testLimitValue(float test_value) {
  if (m_loLimit > m_hiLimit) {
   m_currentLimit = 0;//Free from violation
   return 0;
  }
  if (test_value < m_loLimit) {
   m_currentLimit = 1;//low limit violation
   m_currentLimitError = test_value - m_loLimit;
   if (m_currentLimitError > SIMD_PI) {
    m_currentLimitError -= SIMD_2_PI;
   } else if (m_currentLimitError < -SIMD_PI) {
    m_currentLimitError += SIMD_2_PI;
   }
   return 1;
  } else if (test_value > m_hiLimit) {
   m_currentLimit = 2;//High limit violation
   m_currentLimitError = test_value - m_hiLimit;
   if (m_currentLimitError > SIMD_PI) {
    m_currentLimitError -= SIMD_2_PI;
   } else if (m_currentLimitError < -SIMD_PI) {
    m_currentLimitError += SIMD_2_PI;
   }
   return 2;
  }
  m_currentLimit = 0;//Free from violation
  return 0;
 }

 //! apply the correction impulses for two bodies
 float solveAngularLimits(float timeStep, final btVector3 axis,
  float jacDiagABInv,
  btRigidBody body0, btRigidBody body1) {
  if (needApplyTorques() == false) {
   return 0.0f;
  }
  float target_velocity = m_targetVelocity;
  float maxMotorForce = m_maxMotorForce;
  //current error correction
  if (m_currentLimit != 0) {
   target_velocity = -m_stopERP * m_currentLimitError / (timeStep);
   maxMotorForce = m_maxLimitForce;
  }
  maxMotorForce *= timeStep;
  // current velocity difference
  final btVector3 angVelA = body0.getAngularVelocity();
  final btVector3 angVelB = body1.getAngularVelocity();
  final btVector3 vel_diff = new btVector3(angVelA).sub(angVelB);
  float rel_vel = axis.dot(vel_diff);
  // correction velocity
  float motor_relvel = m_limitSoftness * (target_velocity - m_damping * rel_vel);
  if (motor_relvel < SIMD_EPSILON && motor_relvel > -SIMD_EPSILON) {
   return 0.0f;//no need for applying force
  }
  // correction impulse
  float unclippedMotorImpulse = (1 + m_bounce) * motor_relvel * jacDiagABInv;
  // clip correction impulse
  float clippedMotorImpulse;
  ///@todo: should clip against accumulated impulse
  if (unclippedMotorImpulse > 0.0f) {
   clippedMotorImpulse = unclippedMotorImpulse > maxMotorForce ? maxMotorForce
    : unclippedMotorImpulse;
  } else {
   clippedMotorImpulse = unclippedMotorImpulse < -maxMotorForce ? -maxMotorForce
    : unclippedMotorImpulse;
  }
  // sort with accumulated impulses
  float lo = (-BT_LARGE_FLOAT);
  float hi = (BT_LARGE_FLOAT);
  float oldaccumImpulse = m_accumulatedImpulse;
  float sum = oldaccumImpulse + clippedMotorImpulse;
  m_accumulatedImpulse = sum > hi ? (0.f) : sum < lo ? (0.f) : sum;
  clippedMotorImpulse = m_accumulatedImpulse - oldaccumImpulse;
  final btVector3 motorImp = new btVector3(axis).scale(clippedMotorImpulse);
  body0.applyTorqueImpulse(motorImp);
  body1.applyTorqueImpulse(new btVector3(motorImp).negate());
  return clippedMotorImpulse;
 }

}
