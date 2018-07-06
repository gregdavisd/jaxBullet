/*
 * Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
 * Copyright (C) 2006, 2007 Sony Computer Entertainment Inc.  *
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
import Bullet.LinearMath.btMatrix3x3;
import static Bullet.LinearMath.btScalar.SIMD_HALF_PI;
import static Bullet.LinearMath.btScalar.SIMD_PI;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btUniversalConstraint extends btGeneric6DofConstraint implements
 Serializable {

 public static final float UNIV_EPS = (0.01f);
 final btVector3 m_anchor = new btVector3();
 final btVector3 m_axis1 = new btVector3();
 final btVector3 m_axis2 = new btVector3();

 // constructor
 // anchor, axis1 and axis2 are in world coordinate system
 // axis1 must be orthogonal to axis2
 public btUniversalConstraint(btRigidBody rbA, btRigidBody rbB,
  final btVector3 anchor,
  final btVector3 axis1, final btVector3 axis2) {
  super(rbA, rbB, btTransform.getIdentity(), btTransform.getIdentity(), true);
  m_anchor.set(anchor);
  m_axis1.set(axis1).normalize();
  m_axis2.set(axis2).normalize();
  // build frame basis
  // 6DOF constraint uses Euler angles and to define limits
  // it is assumed that rotational order is :
  // Z - first, allowed limits are (-PI,PI);
  // new position of Y - second (allowed limits are (-PI/2 + epsilon, PI/2 - epsilon), where epsilon is a small positive number 
  // used to prevent constraint from instability on poles;
  // new position of X, allowed limits are (-PI,PI);
  // So to simulate ODE Universal joint we should use parent axis as Z, child axis as Y and limit all other DOFs
  // Build the frame in world coordinate system first
  final btVector3 zAxis = m_axis1;
  final btVector3 yAxis = m_axis2;
  final btVector3 xAxis = new btVector3(yAxis).cross(zAxis); // we want right coordinate system
  final btTransform frameInW = new btTransform();
  frameInW.setIdentity();
  frameInW.setBasis(new btMatrix3x3(xAxis.x, yAxis.x, zAxis.x,
   xAxis.y, yAxis.y, zAxis.y,
   xAxis.z, yAxis.z, zAxis.z));
  frameInW.setOrigin(anchor);
  // now get constraint frame in local coordinate systems
  m_frameInA.set(rbA.getCenterOfMassTransform().invert().mul(frameInW));
  m_frameInB.set(rbB.getCenterOfMassTransform().invert().mul(frameInW));
  // sei limits
  setLinearLowerLimit(new btVector3());
  setLinearUpperLimit(new btVector3());
  setAngularLowerLimit(new btVector3(0.f, -SIMD_HALF_PI + UNIV_EPS, -SIMD_PI
   + UNIV_EPS));
  setAngularUpperLimit(new btVector3(0.f, SIMD_HALF_PI - UNIV_EPS, SIMD_PI
   - UNIV_EPS));
 }

 // access
 public btVector3 getAnchor() {
  return m_calculatedTransformA.getOrigin();
 }

 public btVector3 getAnchor2() {
  return m_calculatedTransformB.getOrigin();
 }

 public btVector3 getAxis1() {
  return new btVector3(m_axis1);
 }

 public btVector3 getAxis2() {
  return new btVector3(m_axis2);
 }

 public float getAngle1() {
  return getAngle(2);
 }

 public float getAngle2() {
  return getAngle(1);
 }
 // limits

 public void setUpperLimit(float ang1max, float ang2max) {
  setAngularUpperLimit(new btVector3(0.f, ang1max, ang2max));
 }

 public void setLowerLimit(float ang1min, float ang2min) {
  setAngularLowerLimit(new btVector3(0.f, ang1min, ang2min));
 }

 public void setAxis(final btVector3 axis1, final btVector3 axis2) {
  m_axis1.set(axis1);
  m_axis2.set(axis2);
  final btVector3 zAxis = new btVector3(axis1).normalize();
  final btVector3 yAxis = new btVector3(axis2).normalize();
  final btVector3 xAxis = new btVector3(yAxis).cross(zAxis); // we want right coordinate system
  final btTransform frameInW = new btTransform();
  frameInW.setIdentity();
  frameInW.setBasis(new btMatrix3x3(xAxis.x, yAxis.x, zAxis.x,
   xAxis.y, yAxis.y, zAxis.y,
   xAxis.z, yAxis.z, zAxis.z));
  frameInW.setOrigin(m_anchor);
  // now get constraint frame in local coordinate systems
  m_frameInA.set(m_rbA.getCenterOfMassTransform().invert().mul(frameInW));
  m_frameInB.set(m_rbB.getCenterOfMassTransform().invert().mul(frameInW));
  calculateTransforms();
 }

}
