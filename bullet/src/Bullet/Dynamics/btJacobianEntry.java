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
package Bullet.Dynamics;

import Bullet.LinearMath.btMatrix3x3;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 * notes: Another memory optimization would be to store m_1MinvJt in the remaining 3 w components
 * which makes the btJacobianEntry memory layout 16 bytes if you only are interested in angular
 * part, just feed massInvA and massInvB zero
 *
 * Jacobian entry is an abstraction that allows to describe constraints it can be used in
 * combination with a constraint solver Can be used to relate the effect of an impulse to the
 * constraint error
 *
 * @author Gregery Barton
 */
public class btJacobianEntry implements Serializable {

 public final btVector3 m_linearJointAxis = new btVector3();
 public final btVector3 m_aJ = new btVector3();
 public final btVector3 m_bJ = new btVector3();
 public final btVector3 m_0MinvJt = new btVector3();
 public final btVector3 m_1MinvJt = new btVector3();
 //Optimization: can be stored in the w/last component of one of the vectors
 float m_Adiag;

 public btJacobianEntry() {
 }

 //constraint between two different rigidbodies
 public btJacobianEntry(
  final btMatrix3x3 world2A, final btMatrix3x3 world2B, final btVector3 rel_pos1,
  final btVector3 rel_pos2, final btVector3 jointAxis, final btVector3 inertiaInvA,
  float massInvA, final btVector3 inertiaInvB,
  float massInvB) {
  m_linearJointAxis.set(jointAxis);
  m_aJ.set(world2A.transform(new btVector3(rel_pos1).cross(m_linearJointAxis)));
  m_bJ.set(world2B.transform(new btVector3(rel_pos2)
   .cross(new btVector3(m_linearJointAxis).negate())));
  m_0MinvJt.set(inertiaInvA).mul(m_aJ);
  m_1MinvJt.set(inertiaInvB).mul(m_bJ);
  m_Adiag = massInvA + m_0MinvJt.dot(m_aJ) + massInvB + m_1MinvJt.dot(m_bJ);
  assert (m_Adiag > (0.0f));
 }

 //angular constraint between two different rigidbodies
 public btJacobianEntry(final btVector3 jointAxis, final btMatrix3x3 world2A,
  final btMatrix3x3 world2B, final btVector3 inertiaInvA, final btVector3 inertiaInvB) {
  m_linearJointAxis.setZero();
  m_aJ.set(world2A.transform(new btVector3(jointAxis)));
  m_bJ.set(world2B.transform(new btVector3(jointAxis).negate()));
  m_0MinvJt.set(inertiaInvA).mul(m_aJ);
  m_1MinvJt.set(inertiaInvB).mul(m_bJ);
  m_Adiag = m_0MinvJt.dot(m_aJ) + m_1MinvJt.dot(m_bJ);
  assert (m_Adiag > (0.0f));
 }

 //angular constraint between two different rigidbodies
 public btJacobianEntry(final btVector3 axisInA, final btVector3 axisInB,
  final btVector3 inertiaInvA, final btVector3 inertiaInvB) {
  m_linearJointAxis.setZero();
  m_aJ.set(axisInA);
  m_bJ.set(axisInB).negate();
  m_0MinvJt.set(inertiaInvA).mul(m_aJ);
  m_1MinvJt.set(inertiaInvB).mul(m_bJ);
  m_Adiag = m_0MinvJt.dot(m_aJ) + m_1MinvJt.dot(m_bJ);
  assert (m_Adiag > (0.0f));
 }

 //constraint on one rigidbody
 public btJacobianEntry(
  final btMatrix3x3 world2A, final btVector3 rel_pos1, final btVector3 rel_pos2,
  final btVector3 jointAxis, final btVector3 inertiaInvA,
  float massInvA) {
  m_linearJointAxis.set(jointAxis);
  world2A.transform(m_aJ.set(rel_pos1).cross(jointAxis));
  world2A.transform(m_bJ.set(rel_pos2).cross(new btVector3(jointAxis).negate()));
  m_0MinvJt.set(inertiaInvA).mul(m_aJ);
  m_1MinvJt.setZero();
  m_Adiag = massInvA + m_0MinvJt.dot(m_aJ);
  assert (m_Adiag > (0.0f));
 }

 public float getDiagonal() {
  return m_Adiag;
 }

 // for two constraints on the same rigidbody (for example vehicle friction)
 public float getNonDiagonal(btJacobianEntry jacB, float massInvA) {
  btJacobianEntry jacA = this;
  float lin = massInvA * jacA.m_linearJointAxis.dot(jacB.m_linearJointAxis);
  float ang = jacA.m_0MinvJt.dot(jacB.m_aJ);
  return lin + ang;
 }

 // for two constraints on sharing two same rigidbodies (for example two contact points between two rigidbodies)
 public float getNonDiagonal(btJacobianEntry jacB, float massInvA, float massInvB) {
  btJacobianEntry jacA = this;
  final btVector3 lin = new btVector3(jacA.m_linearJointAxis).mul(jacB.m_linearJointAxis);
  final btVector3 ang0 = new btVector3(jacA.m_0MinvJt).mul(jacB.m_aJ);
  final btVector3 ang1 = new btVector3(jacA.m_1MinvJt).mul(jacB.m_bJ);
  final btVector3 lin0 = new btVector3(lin).scale(massInvA);
  final btVector3 lin1 = new btVector3(lin).scale(massInvB);
  final btVector3 sum = new btVector3(ang0).add(ang1).add(lin0).add(lin1);
  return sum.x + sum.y + sum.z;
 }

 public float getRelativeVelocity(final btVector3 linvelA, final btVector3 angvelA,
  final btVector3 linvelB, final btVector3 angvelB) {
  final btVector3 linrel = new btVector3(linvelA).sub(linvelB);
  final btVector3 angvela = new btVector3(angvelA).mul(m_aJ);
  final btVector3 angvelb = new btVector3(angvelB).mul(m_bJ);
  linrel.mul(m_linearJointAxis);
  angvela.add(angvelb);
  angvela.add(linrel);
  float rel_vel2 = angvela.x + angvela.y + angvela.z;
  return rel_vel2 + SIMD_EPSILON;
 }
}
