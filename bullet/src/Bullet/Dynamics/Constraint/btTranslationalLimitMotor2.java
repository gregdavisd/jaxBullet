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

 /*
2014 May: btGeneric6DofSpring2Constraint is created from the original (2.82.2712) btGeneric6DofConstraint by Gabor Puhr and Tamas Umenhoffer
Pros:
- Much more accurate and stable in a lot of situation. (Especially when a sleeping chain of RBs connected with 6dof2 is pulled)
- Stable and accurate spring with minimal energy loss that works with all of the solvers. (latter is not true for the original 6dof spring)
- Servo motor functionality
- Much more accurate bouncing. 0 really means zero bouncing (not true for the original 6odf) and there is only a minimal energy loss when the value is 1 (because of the solvers' precision)
- Rotation order for the Euler system can be set. (One axis' freedom is still limited to pi/2)

Cons:
- It is slower than the original 6dof. There is no exact ratio, but half speed is a good estimation.
- At bouncing the correct velocity is calculated, but not the correct position. (it is because of the solver can correct position or velocity, but not both.)
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

import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btTranslationalLimitMotor2 implements Serializable {

// upper < lower means free
// upper == lower means locked
// upper > lower means limited
 public final btVector3 m_lowerLimit = new btVector3();
 public final btVector3 m_upperLimit = new btVector3();
 public final btVector3 m_bounce = new btVector3();
 public final btVector3 m_stopERP = new btVector3();
 public final btVector3 m_stopCFM = new btVector3();
 public final btVector3 m_motorERP = new btVector3();
 public final btVector3 m_motorCFM = new btVector3();
 public final boolean[] m_enableMotor = new boolean[3];
 public final boolean[] m_servoMotor = new boolean[3];
 public final boolean[] m_enableSpring = new boolean[3];
 public final btVector3 m_servoTarget = new btVector3();
 public final btVector3 m_springStiffness = new btVector3();
 public boolean[] m_springStiffnessLimited = new boolean[3];
 public final btVector3 m_springDamping = new btVector3();
 public final boolean[] m_springDampingLimited = new boolean[3];
 public final btVector3 m_equilibriumPoint = new btVector3();
 public final btVector3 m_targetVelocity = new btVector3();
 public final btVector3 m_maxMotorForce = new btVector3();
 public final btVector3 m_currentLimitError = new btVector3();
 public final btVector3 m_currentLimitErrorHi = new btVector3();
 public final btVector3 m_currentLinearDiff = new btVector3();
 public final int[] m_currentLimit = new int[3];

 btTranslationalLimitMotor2() {
  m_stopERP.set(0.2f, 0.2f, 0.2f);
  m_motorERP.set(0.9f, 0.9f, 0.9f);
 }

 btTranslationalLimitMotor2(btTranslationalLimitMotor2 other) {
  m_lowerLimit.set(other.m_lowerLimit);
  m_upperLimit.set(other.m_upperLimit);
  m_bounce.set(other.m_bounce);
  m_stopERP.set(other.m_stopERP);
  m_stopCFM.set(other.m_stopCFM);
  m_motorERP.set(other.m_motorERP);
  m_motorCFM.set(other.m_motorCFM);
  m_currentLimitError.set(other.m_currentLimitError);
  m_currentLimitErrorHi.set(other.m_currentLimitErrorHi);
  m_currentLinearDiff.set(other.m_currentLinearDiff);
  m_servoTarget.set(other.m_servoTarget);
  m_springStiffness.set(other.m_springStiffness);
  m_springDamping.set(other.m_springDamping);
  m_equilibriumPoint.set(other.m_equilibriumPoint);
  m_targetVelocity.set(other.m_targetVelocity);
  m_maxMotorForce.set(other.m_maxMotorForce);
  for (int i = 0; i < 3; i++) {
   m_enableMotor[i] = other.m_enableMotor[i];
   m_servoMotor[i] = other.m_servoMotor[i];
   m_enableSpring[i] = other.m_enableSpring[i];
   m_springStiffnessLimited[i] = other.m_springStiffnessLimited[i];
   m_springDampingLimited[i] = other.m_springDampingLimited[i];
   m_currentLimit[i] = other.m_currentLimit[i];
  }
 }

 boolean isLimited(int limitIndex) {
  return (m_upperLimit.getElement(limitIndex) >= m_lowerLimit.getElement(limitIndex));
 }

 void testLimitValue(int limitIndex, float test_value) {
  float loLimit = m_lowerLimit.getElement(limitIndex);
  float hiLimit = m_upperLimit.getElement(limitIndex);
  if (loLimit > hiLimit) {
   m_currentLimitError.setElement(limitIndex, 0);
   m_currentLimit[limitIndex] = 0;
  } else if (loLimit == hiLimit) {
   m_currentLimitError.setElement(limitIndex, test_value - loLimit);
   m_currentLimit[limitIndex] = 3;
  } else {
   m_currentLimitError.setElement(limitIndex, test_value - loLimit);
   m_currentLimitErrorHi.setElement(limitIndex, test_value - hiLimit);
   m_currentLimit[limitIndex] = 4;
  }
 }
};
