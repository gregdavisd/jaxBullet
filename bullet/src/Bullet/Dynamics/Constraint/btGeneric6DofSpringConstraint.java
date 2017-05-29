/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006, 2007 Sony Computer Entertainment Inc. 

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

import Bullet.Dynamics.btRigidBody;
import Bullet.LinearMath.btMatrix3x3;
import static Bullet.LinearMath.btScalar.btFabs;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btGeneric6DofSpringConstraint extends btGeneric6DofConstraint implements Serializable {

 public final boolean[] m_springEnabled = new boolean[6];
 public final float[] m_equilibriumPoint = new float[6];
 public final float[] m_springStiffness = new float[6];
 public final float[] m_springDamping = new float[6]; // between 0 and 1 (1 == no damping)

 void internalUpdateSprings(btConstraintInfo2 info) {
  // it is assumed that calculateTransforms() have been called before this call
  int i;
  //btVector3 relVel = m_rbB.getLinearVelocity() - m_rbA.getLinearVelocity();
  for (i = 0; i < 3; i++) {
   if (m_springEnabled[i]) {
    // get current position of constraint
    float currPos = m_calculatedLinearDiff.getElement(i);
    // calculate difference
    float delta = currPos - m_equilibriumPoint[i];
    // spring force is (delta * m_stiffness) according to Hooke's Law
    float force = delta * m_springStiffness[i];
    float velFactor = info.fps * m_springDamping[i] / (info.m_numIterations);
    m_linearLimits.m_targetVelocity.setElement(i, velFactor * force);
    m_linearLimits.m_maxMotorForce.setElement(i, btFabs(force) / info.fps);
   }
  }
  for (i = 0; i < 3; i++) {
   if (m_springEnabled[i + 3]) {
    // get current position of constraint
    float currPos = m_calculatedAxisAngleDiff.getElement(i);
    // calculate difference
    float delta = currPos - m_equilibriumPoint[i + 3];
    // spring force is (-delta * m_stiffness) according to Hooke's Law
    float force = -delta * m_springStiffness[i + 3];
    float velFactor = info.fps * m_springDamping[i + 3] / (info.m_numIterations);
    m_angularLimits[i].m_targetVelocity = velFactor * force;
    m_angularLimits[i].m_maxMotorForce = btFabs(force) / info.fps;
   }
  }
 }

 public btGeneric6DofSpringConstraint(btRigidBody rbA, btRigidBody rbB, final btTransform frameInA,
  final btTransform frameInB, boolean useLinearReferenceFrameA) {
  super(D6_SPRING_CONSTRAINT_TYPE, rbA, rbB, frameInA, frameInB, useLinearReferenceFrameA);
 }

 public btGeneric6DofSpringConstraint(btRigidBody rbB, final btTransform frameInB,
  boolean useLinearReferenceFrameB) {
  super(D6_SPRING_CONSTRAINT_TYPE, rbB, frameInB, useLinearReferenceFrameB);
 }

 public void enableSpring(int index, boolean onOff) {
  assert ((index >= 0) && (index < 6));
  m_springEnabled[index] = onOff;
  if (index < 3) {
   m_linearLimits.m_enableMotor[index] = onOff;
  } else {
   m_angularLimits[index - 3].m_enableMotor = onOff;
  }
 }

 public void setStiffness(int index, float stiffness) {
  assert ((index >= 0) && (index < 6));
  m_springStiffness[index] = stiffness;
 }

 public void setDamping(int index, float damping) {
  assert ((index >= 0) && (index < 6));
  m_springDamping[index] = damping;
 }

 // set the current constraint position/orientation as an equilibrium point for all DOF
 public void setEquilibriumPoint() {
  calculateTransforms();
  int i;
  for (i = 0; i < 3; i++) {
   m_equilibriumPoint[i] = m_calculatedLinearDiff.getElement(i);
  }
  for (i = 0; i < 3; i++) {
   m_equilibriumPoint[i + 3] = m_calculatedAxisAngleDiff.getElement(i);
  }
 }

 // set the current constraint position/orientation as an equilibrium point for given DOF
 public void setEquilibriumPoint(int index) {
  assert ((index >= 0) && (index < 6));
  calculateTransforms();
  if (index < 3) {
   m_equilibriumPoint[index] = m_calculatedLinearDiff.getElement(index);
  } else {
   m_equilibriumPoint[index] = m_calculatedAxisAngleDiff.getElement(index - 3);
  }
 }

 public void setEquilibriumPoint(int index, float val) {
  assert ((index >= 0) && (index < 6));
  m_equilibriumPoint[index] = val;
 }

 public boolean isSpringEnabled(int index) {
  return m_springEnabled[index];
 }

 public float getStiffness(int index) {
  return m_springStiffness[index];
 }

 public float getDamping(int index) {
  return m_springDamping[index];
 }

 public float getEquilibriumPoint(int index) {
  return m_equilibriumPoint[index];
 }

 public void setAxis(final btVector3 axis1, final btVector3 axis2) {
  final btVector3 zAxis = new btVector3(axis1).normalize();
  final btVector3 yAxis = new btVector3(axis2).normalize();
  final btVector3 xAxis = new btVector3(yAxis).cross(zAxis); // we want right coordinate system
  final btTransform frameInW = new btTransform();
  frameInW.setIdentity();
  frameInW.setBasis(new btMatrix3x3(xAxis.x, yAxis.x, zAxis.x,
   xAxis.y, yAxis.y, zAxis.y,
   xAxis.z, yAxis.z, zAxis.z));
  // now get constraint frame in local coordinate systems
  m_frameInA.set(m_rbA.getCenterOfMassTransform().invert().mul(frameInW));
  m_frameInB.set(m_rbB.getCenterOfMassTransform().invert().mul(frameInW));
  calculateTransforms();
 }

 public void getInfo2(btConstraintInfo2 info) {
  // this will be called by constraint solver at the constraint setup stage
  // set current motor parameters
  internalUpdateSprings(info);
  // do the rest of job for constraint setup
  super.getInfo2(info);
 }
};
