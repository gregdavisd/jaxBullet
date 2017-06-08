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

import Bullet.Collision.Algorithm.Detector.btDiscreteCollisionDetectorInterface;
import static Bullet.Collision.CollisionFlags.CF_CUSTOM_MATERIAL_CALLBACK;
import static Bullet.Collision.CollisionFlags.CF_HAS_CONTACT_STIFFNESS_DAMPING;
import static Bullet.Collision.btManifoldPoint.BT_CONTACT_FLAG_CONTACT_STIFFNESS_DAMPING;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.btPlaneSpace1;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btManifoldResult extends btDiscreteCollisionDetectorInterface.Result implements
 Serializable {

 static ContactAddedCallback gContactAddedCallback;

 /// in the future we can let the user override the methods to combine restitution and friction
 public static float calculateCombinedRestitution(btCollisionObject body0, btCollisionObject body1) {
  return body0.getRestitution() * body1.getRestitution();
 }

 public static float calculateCombinedFriction(btCollisionObject body0, btCollisionObject body1) {
  float friction = body0.getFriction() * body1.getFriction();
  float MAX_FRICTION = (10.f);
  if (friction < -MAX_FRICTION) {
   friction = -MAX_FRICTION;
  }
  if (friction > MAX_FRICTION) {
   friction = MAX_FRICTION;
  }
  return friction;
 }

 static float calculateCombinedRollingFriction(btCollisionObject body0, btCollisionObject body1) {
  float friction = body0.getRollingFriction() * body1.getFriction() + body1.getRollingFriction() *
   body0.getFriction();
  float MAX_FRICTION = (BT_LARGE_FLOAT);
  if (friction < -MAX_FRICTION) {
   friction = -MAX_FRICTION;
  }
  if (friction > MAX_FRICTION) {
   friction = MAX_FRICTION;
  }
  return friction;
 }

 static float calculateCombinedSpinningFriction(btCollisionObject body0, btCollisionObject body1) {
  float friction = body0.getSpinningFriction() * body1.getFriction() + body1.getSpinningFriction() *
   body0.getFriction();
  float MAX_FRICTION = (10.f);
  if (friction < -MAX_FRICTION) {
   friction = -MAX_FRICTION;
  }
  if (friction > MAX_FRICTION) {
   friction = MAX_FRICTION;
  }
  return friction;
 }

 static float calculateCombinedContactDamping(btCollisionObject body0, btCollisionObject body1) {
  return body0.getContactDamping() + body1.getContactDamping();
 }

 static float calculateCombinedContactStiffness(btCollisionObject body0, btCollisionObject body1) {
  float s0 = body0.getContactStiffness();
  float s1 = body1.getContactStiffness();
  float tmp0 = (1f) / s0;
  float tmp1 = (1f) / s1;
  float combinedStiffness = (1f) / (tmp0 + tmp1);
  return combinedStiffness;
 }
 public btPersistentManifold m_manifoldPtr;
 public btCollisionObjectWrapper m_body0Wrap;
 public btCollisionObjectWrapper m_body1Wrap;
 public int m_partId0;
 public int m_partId1;
 public int m_index0;
 public int m_index1;
 public float m_closestPointDistanceThreshold;

 public btManifoldResult(btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap) {
  m_manifoldPtr = null;
  m_body0Wrap = body0Wrap;
  m_body1Wrap = body1Wrap;
 }

 public btManifoldResult() {
 }

 public void setPersistentManifold(btPersistentManifold manifoldPtr) {
  m_manifoldPtr = manifoldPtr;
 }

 public btPersistentManifold getPersistentManifold() {
  return m_manifoldPtr;
 }

 @Override
 public void setShapeIdentifiersA(int partId0, int index0) {
  m_partId0 = partId0;
  m_index0 = index0;
 }

 @Override
 public void setShapeIdentifiersB(int partId1, int index1) {
  m_partId1 = partId1;
  m_index1 = index1;
 }

 @Override
 public void addContactPoint(final btVector3 normalOnBInWorld, final btVector3 pointInWorld,
  float depth) {
//System.out.println(pointInWorld.toString()) ;
  assert (m_manifoldPtr != null);
  //order in manifold needs to match
  if (depth > m_manifoldPtr.getContactBreakingThreshold()) {
   return;
  }
  boolean isSwapped = m_manifoldPtr.getBody0() != m_body0Wrap.getCollisionObject();
  final btVector3 pointA = new btVector3().scaleAdd(depth, normalOnBInWorld, pointInWorld);
  final btVector3 localA;
  final btVector3 localB;
  if (isSwapped) {
   localA = m_body1Wrap.getCollisionObject().getWorldTransformPtr().invXform(pointA);
   localB = m_body0Wrap.getCollisionObject().getWorldTransformPtr().invXform(pointInWorld);
  } else {
   localA = m_body0Wrap.getCollisionObject().getWorldTransformPtr().invXform(pointA);
   localB = m_body1Wrap.getCollisionObject().getWorldTransformPtr().invXform(pointInWorld);
  }
  btManifoldPoint newPt = new btManifoldPoint(localA, localB, normalOnBInWorld, depth);
  newPt.m_positionWorldOnA.set(pointA);
  newPt.m_positionWorldOnB.set(pointInWorld);
  int insertIndex = m_manifoldPtr.getCacheEntry(newPt);
  newPt.m_combinedFriction = calculateCombinedFriction(m_body0Wrap.getCollisionObject(), m_body1Wrap
   .getCollisionObject());
  newPt.m_combinedRestitution = calculateCombinedRestitution(m_body0Wrap.getCollisionObject(),
   m_body1Wrap.getCollisionObject());
  newPt.m_combinedRollingFriction = calculateCombinedRollingFriction(m_body0Wrap
   .getCollisionObject(), m_body1Wrap.getCollisionObject());
  newPt.m_combinedSpinningFriction = calculateCombinedSpinningFriction(m_body0Wrap
   .getCollisionObject(), m_body1Wrap.getCollisionObject());
  if ((m_body0Wrap.getCollisionObject().getCollisionFlags() & CF_HAS_CONTACT_STIFFNESS_DAMPING) != 0 ||
   (m_body1Wrap.getCollisionObject().getCollisionFlags() & CF_HAS_CONTACT_STIFFNESS_DAMPING) != 0) {
   newPt.m_combinedContactDamping1 = calculateCombinedContactDamping(m_body0Wrap
    .getCollisionObject(), m_body1Wrap.getCollisionObject());
   newPt.m_combinedContactStiffness1 = calculateCombinedContactStiffness(m_body0Wrap
    .getCollisionObject(), m_body1Wrap.getCollisionObject());
   newPt.m_contactPointFlags |= BT_CONTACT_FLAG_CONTACT_STIFFNESS_DAMPING;
  }
  btPlaneSpace1(newPt.m_normalWorldOnB, newPt.m_lateralFrictionDir1, newPt.m_lateralFrictionDir2);
  //BP mod, store contact triangles.
  if (isSwapped) {
   newPt.m_partId0 = m_partId1;
   newPt.m_partId1 = m_partId0;
   newPt.m_index0 = m_index1;
   newPt.m_index1 = m_index0;
  } else {
   newPt.m_partId0 = m_partId0;
   newPt.m_partId1 = m_partId1;
   newPt.m_index0 = m_index0;
   newPt.m_index1 = m_index1;
  }
  //printf("depth=%f\n",depth);
  ///@todo, check this for any side effects
  if (insertIndex >= 0) {
   //  btManifoldPoint& oldPoint = m_manifoldPtr.getContactPoint(insertIndex);
   m_manifoldPtr.replaceContactPoint(newPt, insertIndex);
  } else {
   insertIndex = m_manifoldPtr.addManifoldPoint(newPt);
  }
  //User can override friction and/or restitution
  if (gContactAddedCallback != null && //and if either of the two bodies requires custom material
   ((m_body0Wrap.getCollisionObject().getCollisionFlags() & CF_CUSTOM_MATERIAL_CALLBACK) != 0 ||
   (m_body1Wrap.getCollisionObject().getCollisionFlags() & CF_CUSTOM_MATERIAL_CALLBACK) != 0)) {
   //experimental feature info, for per-triangle material etc.
   btCollisionObjectWrapper obj0Wrap = isSwapped ? m_body1Wrap : m_body0Wrap;
   btCollisionObjectWrapper obj1Wrap = isSwapped ? m_body0Wrap : m_body1Wrap;
   (gContactAddedCallback).callback(m_manifoldPtr.getContactPoint(insertIndex), obj0Wrap,
    newPt.m_partId0, newPt.m_index0, obj1Wrap, newPt.m_partId1, newPt.m_index1);
  }
 }

 public void refreshContactPoints() {
  assert (m_manifoldPtr != null);
  if (m_manifoldPtr.getNumContacts() == 0) {
   return;
  }
  boolean isSwapped = m_manifoldPtr.getBody0() != m_body0Wrap.getCollisionObject();
  if (isSwapped) {
   m_manifoldPtr.refreshContactPoints(m_body1Wrap.getCollisionObject().getWorldTransformPtr(),
    m_body0Wrap.getCollisionObject().getWorldTransformPtr());
  } else {
   m_manifoldPtr.refreshContactPoints(m_body0Wrap.getCollisionObject().getWorldTransformPtr(),
    m_body1Wrap.getCollisionObject().getWorldTransformPtr());
  }
 }

 btCollisionObjectWrapper getBody0Wrap() {
  return m_body0Wrap;
 }

 btCollisionObjectWrapper getBody1Wrap() {
  return m_body1Wrap;
 }

 void setBody0Wrap(btCollisionObjectWrapper obj0Wrap) {
  m_body0Wrap = obj0Wrap;
 }

 void setBody1Wrap(btCollisionObjectWrapper obj1Wrap) {
  m_body1Wrap = obj1Wrap;
 }

 btCollisionObject getBody0Internal() {
  return m_body0Wrap.getCollisionObject();
 }

 btCollisionObject getBody1Internal() {
  return m_body1Wrap.getCollisionObject();
 }
};
