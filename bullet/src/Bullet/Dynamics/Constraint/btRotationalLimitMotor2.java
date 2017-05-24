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

import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btRotationalLimitMotor2 implements Serializable  {

// upper < lower means free
// upper == lower means locked
// upper > lower means limited
public  float m_loLimit;
public  float m_hiLimit;
public  float m_bounce;
public  float m_stopERP;
public  float m_stopCFM;
public  float m_motorERP;
public  float m_motorCFM;
public  boolean m_enableMotor;
public  float m_targetVelocity;
public  float m_maxMotorForce;
public  boolean m_servoMotor;
public  float m_servoTarget;
public  boolean m_enableSpring;
public  float m_springStiffness;
public  boolean m_springStiffnessLimited;
public  float m_springDamping;
public  boolean m_springDampingLimited;
public  float m_equilibriumPoint;
public  float m_currentLimitError;
public  float m_currentLimitErrorHi;
public  float m_currentPosition;
public  int m_currentLimit;

 btRotationalLimitMotor2() {
  m_loLimit = 1.0f;
  m_hiLimit = -1.0f;
  m_bounce = 0.0f;
  m_stopERP = 0.2f;
  m_stopCFM = 0.f;
  m_motorERP = 0.9f;
  m_motorCFM = 0.f;
  m_enableMotor = false;
  m_targetVelocity = 0;
  m_maxMotorForce = 0.1f;
  m_servoMotor = false;
  m_servoTarget = 0;
  m_enableSpring = false;
  m_springStiffness = 0;
  m_springStiffnessLimited = false;
  m_springDamping = 0;
  m_springDampingLimited = false;
  m_equilibriumPoint = 0;
  m_currentLimitError = 0;
  m_currentLimitErrorHi = 0;
  m_currentPosition = 0;
  m_currentLimit = 0;
 }

 btRotationalLimitMotor2(btRotationalLimitMotor2 limot) {
  m_loLimit = limot.m_loLimit;
  m_hiLimit = limot.m_hiLimit;
  m_bounce = limot.m_bounce;
  m_stopERP = limot.m_stopERP;
  m_stopCFM = limot.m_stopCFM;
  m_motorERP = limot.m_motorERP;
  m_motorCFM = limot.m_motorCFM;
  m_enableMotor = limot.m_enableMotor;
  m_targetVelocity = limot.m_targetVelocity;
  m_maxMotorForce = limot.m_maxMotorForce;
  m_servoMotor = limot.m_servoMotor;
  m_servoTarget = limot.m_servoTarget;
  m_enableSpring = limot.m_enableSpring;
  m_springStiffness = limot.m_springStiffness;
  m_springStiffnessLimited = limot.m_springStiffnessLimited;
  m_springDamping = limot.m_springDamping;
  m_springDampingLimited = limot.m_springDampingLimited;
  m_equilibriumPoint = limot.m_equilibriumPoint;
  m_currentLimitError = limot.m_currentLimitError;
  m_currentLimitErrorHi = limot.m_currentLimitErrorHi;
  m_currentPosition = limot.m_currentPosition;
  m_currentLimit = limot.m_currentLimit;
 }

 boolean isLimited() {
  if (m_loLimit > m_hiLimit) {
   return false;
  }
  return true;
 }

 void testLimitValue(float test_value) {
  //we can't normalize the angles here because we would lost the sign that we use later, but it doesn't seem to be a problem
  if (m_loLimit > m_hiLimit) {
   m_currentLimit = 0;
   m_currentLimitError = (0.f);
  } else if (m_loLimit == m_hiLimit) {
   m_currentLimitError = test_value - m_loLimit;
   m_currentLimit = 3;
  } else {
   m_currentLimitError = test_value - m_loLimit;
   m_currentLimitErrorHi = test_value - m_hiLimit;
   m_currentLimit = 4;
  }
 }
};
