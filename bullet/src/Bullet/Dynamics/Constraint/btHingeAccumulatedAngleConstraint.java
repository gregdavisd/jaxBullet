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

 /* Hinge Constraint by Dirk Gregorius. Limits added by Marcus Hennix at Starbreeze Studios */
package Bullet.Dynamics.Constraint;

import Bullet.Dynamics.CollisionObjects.btRigidBody;
import static Bullet.LinearMath.btScalar.SIMD_PI;
import static Bullet.LinearMath.btScalar.btFabs;
import static Bullet.LinearMath.btScalar.btFmod;
import static Bullet.LinearMath.btScalar.btNormalizeAngle;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;

/**
 * The getAccumulatedHingeAngle returns the accumulated hinge angle, taking
 * rotation across the -PI/PI boundary into account
 *
 * @author Gregery Barton
 */
public class btHingeAccumulatedAngleConstraint extends btHingeConstraint {

 private static final long serialVersionUID = 5127670227845110667L;
 float m_accumulatedAngle;

 public btHingeAccumulatedAngleConstraint(btRigidBody rbA, btRigidBody rbB,
  final btVector3 pivotInA,
  final btVector3 pivotInB, final btVector3 axisInA, final btVector3 axisInB) {
  this(rbA, rbB, pivotInA, pivotInB, axisInA, axisInB, false);
 }

 public btHingeAccumulatedAngleConstraint(btRigidBody rbA, btRigidBody rbB,
  final btVector3 pivotInA,
  final btVector3 pivotInB, final btVector3 axisInA, final btVector3 axisInB,
  boolean useReferenceFrameA) {
  super(rbA, rbB, pivotInA, pivotInB, axisInA, axisInB, useReferenceFrameA);
  m_accumulatedAngle = getHingeAngle();
 }

 public btHingeAccumulatedAngleConstraint(btRigidBody rbA,
  final btVector3 pivotInA,
  final btVector3 axisInA) {
  this(rbA, pivotInA, axisInA, false);
 }

 public btHingeAccumulatedAngleConstraint(btRigidBody rbA,
  final btVector3 pivotInA,
  final btVector3 axisInA, boolean useReferenceFrameA) {
  super(rbA, pivotInA, axisInA, useReferenceFrameA);
  m_accumulatedAngle = getHingeAngle();
 }

 public btHingeAccumulatedAngleConstraint(btRigidBody rbA, btRigidBody rbB,
  final btTransform rbAFrame,
  final btTransform rbBFrame, boolean useReferenceFrameA) {
  super(rbA, rbB, rbAFrame, rbBFrame, useReferenceFrameA);
  m_accumulatedAngle = getHingeAngle();
 }

 public btHingeAccumulatedAngleConstraint(btRigidBody rbA,
  final btTransform rbAFrame) {
  this(rbA, rbAFrame, false);
 }

 public btHingeAccumulatedAngleConstraint(btRigidBody rbA,
  final btTransform rbAFrame,
  boolean useReferenceFrameA) {
  super(rbA, rbAFrame, useReferenceFrameA);
  m_accumulatedAngle = getHingeAngle();
 }

 public float getAccumulatedHingeAngle() {
  float hingeAngle = getHingeAngle();
  m_accumulatedAngle = btShortestAngleUpdate(m_accumulatedAngle, hingeAngle);
  return m_accumulatedAngle;
 }

 public void setAccumulatedHingeAngle(float accAngle) {
  m_accumulatedAngle = accAngle;
 }

 @Override
 public void getInfo1(btConstraintInfo1 info) {
  //update m_accumulatedAngle
  float curHingeAngle = getHingeAngle();
  m_accumulatedAngle = btShortestAngleUpdate(m_accumulatedAngle, curHingeAngle);
  super.getInfo1(info);
 }

 static float btShortestAngleUpdate(float accAngle, float curAngle) {
  float tol = 0.3f;
  float result = btShortestAngularDistance(accAngle, curAngle);
  if (btFabs(result) > tol) {
   return curAngle;
  } else {
   return accAngle + result;
  }
 }

 static float btShortestAngularDistance(float accAngle, float curAngle) {
  float result = btNormalizeAngle(btNormalizeAnglePositive(
   btNormalizeAnglePositive(curAngle) - btNormalizeAnglePositive(accAngle)));
  return result;
 }

 static float btNormalizeAnglePositive(float angle) {
  return btFmod(btFmod(angle, (2.0f * SIMD_PI)) + (2.0f * SIMD_PI), (2.0f
   * SIMD_PI));
 }

};
