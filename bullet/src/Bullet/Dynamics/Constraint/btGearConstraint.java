/*
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2012 Advanced Micro Devices, Inc.  http://bulletphysics.org
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
package Bullet.Dynamics.Constraint;

import Bullet.Dynamics.CollisionObjects.btRigidBody;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 * The btGeatConstraint will couple the angular velocity for two bodies around
 * given local axis and ratio. See Bullet/Demos/ConstraintDemo for an example
 * use.
 *
 * @author Gregery Barton
 */
public class btGearConstraint extends btTypedConstraint implements Serializable {

 final btVector3 m_axisInA = new btVector3();
 final btVector3 m_axisInB = new btVector3();
 boolean m_useFrameA;
 float m_ratio;

 public btGearConstraint(btRigidBody rbA, btRigidBody rbB,
  final btVector3 axisInA,
  final btVector3 axisInB) {
  this(rbA, rbB, axisInA, axisInB, 1.0f);
 }

 public btGearConstraint(btRigidBody rbA, btRigidBody rbB,
  final btVector3 axisInA,
  final btVector3 axisInB, float ratio) {
  super(GEAR_CONSTRAINT_TYPE, rbA, rbB);
  m_axisInA.set(axisInA);
  m_axisInB.set(axisInB);
  m_ratio = ratio;
 }

 ///internal method used by the constraint solver, don't use them directly
 public void getInfo1(btConstraintInfo1 info) {
  info.m_numConstraintRows = 1;
  info.nub = 1;
 }

 ///internal method used by the constraint solver, don't use them directly
 public void getInfo2(btConstraintInfo2 info) {
  final btVector3 globalAxisA = m_rbA.getWorldTransformPtr().transform3x3(
   new btVector3(m_axisInA));
  final btVector3 globalAxisB = m_rbB.getWorldTransformPtr().transform3x3(
   new btVector3(m_axisInB));
  info.m_J1angularAxis[0].set(globalAxisA);
  info.m_J2angularAxis[0].set(globalAxisB).scale(m_ratio);
 }

 public void setAxisA(final btVector3 axisA) {
  m_axisInA.set(axisA);
 }

 public void setAxisB(final btVector3 axisB) {
  m_axisInB.set(axisB);
 }

 public void setRatio(float ratio) {
  m_ratio = ratio;
 }

 public btVector3 getAxisA() {
  return new btVector3(m_axisInA);
 }

 public btVector3 getAxisB() {
  return new btVector3(m_axisInB);
 }

 public float getRatio() {
  return m_ratio;
 }

 public void setParam(int num, float value, int axis) {
  assert (false);
 }

 ///return the local value of parameter
 public float getParam(int num, int axis) {
  assert (false);
  return 0.f;
 }

};
