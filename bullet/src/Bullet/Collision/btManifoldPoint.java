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

package Bullet.Collision;

import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btManifoldPoint  implements Serializable {

 public static final int BT_CONTACT_FLAG_LATERAL_FRICTION_INITIALIZED = 1;
 public static final int BT_CONTACT_FLAG_HAS_CONTACT_CFM = 2;
 public static final int BT_CONTACT_FLAG_HAS_CONTACT_ERP = 4;
 public static final int BT_CONTACT_FLAG_CONTACT_STIFFNESS_DAMPING = 8;
 public final btVector3 m_localPointA = new btVector3();
 public final btVector3 m_localPointB = new btVector3();
 public final btVector3 m_positionWorldOnB = new btVector3();
 ///m_positionWorldOnA is redundant information, see getPositionWorldOnA(), but for clarity
 public final btVector3 m_positionWorldOnA = new btVector3();
 public final btVector3 m_normalWorldOnB = new btVector3();
 public float m_distance1;
 public float m_combinedFriction;
 public float m_combinedRollingFriction;//torsional friction orthogonal to contact normal, useful to make spheres stop rolling forever
 public float m_combinedSpinningFriction;//torsional friction around contact normal, useful for grasping objects
 public float m_combinedRestitution;
 //BP mod, store contact triangles.
 public int m_partId0;
 public int m_partId1;
 public int m_index0;
 public int m_index1;
 public Object m_userPersistentData;
 //boolean			m_lateralFrictionInitialized;
 public int m_contactPointFlags;
 public float m_appliedImpulse;
 public float m_appliedImpulseLateral1;
 public float m_appliedImpulseLateral2;
 public float m_contactMotion1;
 public float m_contactMotion2;
 // can't union in Java so have to pick on of these
//			union
//			{
 public float m_contactCFM;
 public float m_combinedContactStiffness1;
//			};
//			union
//			{
 public float m_contactERP;
 public float m_combinedContactDamping1;
//			};
 public float m_frictionCFM;
 public int m_lifeTime;//lifetime of the contactpoint in frames
 public final btVector3 m_lateralFrictionDir1 = new btVector3();
 public final btVector3 m_lateralFrictionDir2 = new btVector3();

 public btManifoldPoint() {
  m_userPersistentData = null;
  m_contactPointFlags = 0;
  m_appliedImpulse = 0.f;
  m_appliedImpulseLateral1 = 0.f;
  m_appliedImpulseLateral2 = 0.f;
  m_contactMotion1 = 0.f;
  m_contactMotion2 = 0.f;
  m_contactCFM = 0.f;
  m_contactERP = 0.f;
  m_frictionCFM = 0.f;
  m_lifeTime = 0;
 }

 public btManifoldPoint(final btVector3 pointA, final btVector3 pointB, final btVector3 normal,
  float distance) {
  m_localPointA.set(pointA);
  m_localPointB.set(pointB);
  m_normalWorldOnB.set(normal);
  m_distance1 = distance;
  m_combinedFriction = 0.f;
  m_combinedRollingFriction = 0f;
  m_combinedSpinningFriction = 0f;
  m_combinedRestitution = 0f;
  m_userPersistentData = null;
  m_contactPointFlags = 0;
  m_appliedImpulse = 0f;
  m_appliedImpulseLateral1 = 0f;
  m_appliedImpulseLateral2 = 0.f;
  m_contactMotion1 = 0.f;
  m_contactMotion2 = 0.f;
  m_contactCFM = 0.f;
  m_contactERP = 0.f;
  m_frictionCFM = 0.f;
  m_lifeTime = 0;
 }

 public float getDistance() {
  return m_distance1;
 }

 public int getLifeTime() {
  return m_lifeTime;
 }

 public btVector3 getPositionWorldOnA() {
  return new btVector3(m_positionWorldOnA);
//				return m_positionWorldOnB + m_normalWorldOnB * m_distance1;
 }

 public btVector3 getPositionWorldOnB() {
  return new btVector3(m_positionWorldOnB);
 }

 public void setDistance(float dist) {
  m_distance1 = dist;
 }

 ///this returns the most recent applied impulse, to satisfy contact constraints by the constraint solver
 public float getAppliedImpulse() {
  return m_appliedImpulse;
 }
}
