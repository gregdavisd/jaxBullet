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
 * Constraint similar to ODE Hinge2 Joint has 3 degrees of freedom: 2 rotational
 * degrees of freedom, similar to Euler rotations around Z (axis 1) and X (axis
 * 2) 1 translational (along axis Z) with suspension spring
 *
 * @author Gregery Barton
 */
public class btHinge2Constraint extends btGeneric6DofSpring2Constraint
 implements Serializable {

 final btVector3 m_anchor = new btVector3();
 final btVector3 m_axis1 = new btVector3();
 final btVector3 m_axis2 = new btVector3();

 // constructor
 // anchor, axis1 and axis2 are in world coordinate system
 // axis1 must be orthogonal to axis2
 public btHinge2Constraint(btRigidBody rbA, btRigidBody rbB,
  final btVector3 anchor,
  final btVector3 axis1,
  final btVector3 axis2) {
  super(rbA, rbB, btTransform.getIdentity(), btTransform.getIdentity(), RO_XYZ);
  m_anchor.set(anchor);
  m_axis1.set(axis1);
  m_axis2.set(axis2);
  // build frame basis
  // 6DOF constraint uses Euler angles and to define limits
  // it is assumed that rotational order is :
  // Z - first, allowed limits are (-PI,PI);
  // new position of Y - second (allowed limits are (-PI/2 + epsilon, PI/2 - epsilon), where epsilon is a small positive number 
  // used to prevent constraint from instability on poles;
  // new position of X, allowed limits are (-PI,PI);
  // So to simulate ODE Universal joint we should use parent axis as Z, child axis as Y and limit all other DOFs
  // Build the frame in world coordinate system first
  final btVector3 zAxis = new btVector3(axis1).normalize();
  final btVector3 xAxis = new btVector3(axis2).normalize();
  final btVector3 yAxis = new btVector3(zAxis).cross(xAxis); // we want right coordinate system
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
  setLinearLowerLimit(new btVector3(0.f, 0.f, -1.f));
  setLinearUpperLimit(new btVector3(0.f, 0.f, 1.f));
  // like front wheels of a car
  setAngularLowerLimit(new btVector3(1.f, 0.f, -SIMD_HALF_PI * 0.5f));
  setAngularUpperLimit(new btVector3(-1.f, 0.f, SIMD_HALF_PI * 0.5f));
  // enable suspension
  enableSpring(2, true);
  setStiffness(2, SIMD_PI * SIMD_PI * 4.f);
  setDamping(2, 0.01f);
  setEquilibriumPoint();
 }

 // access
 public final btVector3 getAnchor() {
  return m_calculatedTransformA.getOrigin();
 }

 public final btVector3 getAnchor2() {
  return m_calculatedTransformB.getOrigin();
 }

 public final btVector3 getAxis1() {
  return m_axis1;
 }

 public final btVector3 getAxis2() {
  return m_axis2;
 }

 public final float getAngle1() {
  return getAngle(2);
 }

 public final float getAngle2() {
  return getAngle(0);
 }
 // limits

 public final void setUpperLimit(float ang1max) {
  setAngularUpperLimit(new btVector3(-1.f, 0.f, ang1max));
 }

 public final void setLowerLimit(float ang1min) {
  setAngularLowerLimit(new btVector3(1.f, 0.f, ang1min));
 }

}
