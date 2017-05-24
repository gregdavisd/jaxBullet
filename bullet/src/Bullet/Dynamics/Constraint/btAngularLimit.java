/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2010 Erwin Coumans  http://continuousphysics.com/Bullet/

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

import static Bullet.LinearMath.btScalar.btEqual;
import static Bullet.LinearMath.btScalar.btNormalizeAngle;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btAngularLimit  implements Serializable {

 float m_center,
  m_halfRange,
  m_softness,
  m_biasFactor,
  m_relaxationFactor,
  m_correction,
  m_sign;
 boolean m_solveLimit;

 /// Default constructor initializes limit as inactive, allowing free constraint movement
 btAngularLimit() {
  m_center = 0.0f;
  m_halfRange = -1.0f;
  m_softness = 0.9f;
  m_biasFactor = 0.3f;
  m_relaxationFactor = 1.0f;
  m_correction = 0.0f;
  m_sign = 0.0f;
  m_solveLimit = false;
 }

 /// Sets all limit's parameters.
 /// When low > high limit becomes inactive.
 /// When high - low > 2PI limit is ineffective too becouse no angle can exceed the limit
 final void set(float low, float high) {
  set(low, high, 0.9f);
 }

 final void set(float low, float high, float _softness) {
  set(low, high, _softness, 0.3f);
 }

 final void set(float low, float high, float _softness, float _biasFactor) {
  set(low, high, _softness, _biasFactor, 1.0f);
 }

 void set(float low, float high, float _softness, float _biasFactor, float _relaxationFactor) {
  m_halfRange = (high - low) / 2.0f;
  m_center = btNormalizeAngle(low + m_halfRange);
  m_softness = _softness;
  m_biasFactor = _biasFactor;
  m_relaxationFactor = _relaxationFactor;
 }

 /// Checks conastaint angle against limit. If limit is active and the angle violates the limit
 /// correction is calculated.
 void test(float angle) {
  m_correction = 0.0f;
  m_sign = 0.0f;
  m_solveLimit = false;
  if (m_halfRange >= 0.0f) {
   float deviation = btNormalizeAngle(angle - m_center);
   if (deviation < -m_halfRange) {
    m_solveLimit = true;
    m_correction = -(deviation + m_halfRange);
    m_sign = +1.0f;
   } else if (deviation > m_halfRange) {
    m_solveLimit = true;
    m_correction = m_halfRange - deviation;
    m_sign = -1.0f;
   }
  }
 }

 /// Returns limit's softness
 float getSoftness() {
  return m_softness;
 }

 /// Returns limit's bias factor
 float getBiasFactor() {
  return m_biasFactor;
 }

 /// Returns limit's relaxation factor
 float getRelaxationFactor() {
  return m_relaxationFactor;
 }

 /// Returns correction value evaluated when test() was invoked 
 float getCorrection() {
  return m_correction;
 }

 /// Returns sign value evaluated when test() was invoked 
 float getSign() {
  return m_sign;
 }

 /// Gives half of the distance between min and max limit angle
 float getHalfRange() {
  return m_halfRange;
 }

 /// Returns true when the last test() invocation recognized limit violation
 boolean isLimit() {
  return m_solveLimit;
 }

 /// Checks given angle against limit. If limit is active and angle doesn't fit it, the angle
 /// returned is modified so it equals to the limit closest to given angle.
 float fit(float angle) {
  if (m_halfRange > 0.0f) {
   float relativeAngle = btNormalizeAngle(angle - m_center);
   if (!btEqual(relativeAngle, m_halfRange)) {
    if (relativeAngle > 0.0f) {
     return getHigh();
    } else {
     return getLow();
    }
   }
  }
  return angle;
 }

 /// Returns correction value multiplied by sign value
 float getError() {
  return m_correction * m_sign;
 }

 float getLow() {
  return btNormalizeAngle(m_center - m_halfRange);
 }

 float getHigh() {
  return btNormalizeAngle(m_center + m_halfRange);
 }
};
