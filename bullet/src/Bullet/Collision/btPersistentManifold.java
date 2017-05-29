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

import static Bullet.Dynamics.btContactManifoldTypes.BT_PERSISTENT_MANIFOLD_TYPE;
import static Bullet.Extras.btMinMax.btMax;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btTypedObject;
import Bullet.LinearMath.btVector3;
import Bullet.LinearMath.btVector4;
import java.io.Serializable;

/**
 * btPersistentManifold is a contact point cache, it stays persistent as long as objects are
 * overlapping in the broadphase. Those contact points are created by the collision narrow phase.
 * The cache can be empty, or hold 1,2,3 or 4 points. Some collision algorithms (GJK) might only add
 * one point at a time. updates/refreshes old contact points, and throw them away if necessary
 * (distance becomes too large) reduces the cache to 4 points, when more then 4 points are added,
 * using following rules: the contact point with deepest penetration is always kept, and it tries to
 * maximuze the area covered by the points note that some pairs of objects might have more then one
 * contact manifold.
 *
 * @author Gregery Barton
 */
public class btPersistentManifold extends btTypedObject implements Serializable {

 public static final int MANIFOLD_CACHE_SIZE = 4;
 private static final long serialVersionUID = 1L;
 final btManifoldPoint[] m_pointCache = new btManifoldPoint[MANIFOLD_CACHE_SIZE];
 /// this two body pointers can point to the physics rigidbody class.
 public btCollisionObject m_body0;
 public btCollisionObject m_body1;
 public int m_cachedPoints;
 public float m_contactBreakingThreshold;
 public float m_contactProcessingThreshold;
 public static float gContactBreakingThreshold = 0.02f;
 static ContactDestroyedCallback gContactDestroyedCallback = null;
 static ContactProcessedCallback gContactProcessedCallback = null;
///gContactCalcArea3Points will approximate the convex hull area using 3 points
///when setting it to false, it will use 4 points to compute the area: it is more accurate but slower
 static boolean gContactCalcArea3Points = true;

 /// sort cached points so most isolated points come first
 int sortCachedPoints(btManifoldPoint pt) {
  //calculate 4 possible cases areas, and take biggest area
  //also need to keep 'deepest'
  int maxPenetrationIndex = -1;
  float maxPenetration = pt.getDistance();
  for (int i = 0; i < 4; i++) {
   if (m_pointCache[i].getDistance() < maxPenetration) {
    maxPenetrationIndex = i;
    maxPenetration = m_pointCache[i].getDistance();
   }
  }
  float res0 = 0, res1 = 0, res2 = 0, res3 = 0;
  if (gContactCalcArea3Points) {
   if (maxPenetrationIndex != 0) {
    final btVector3 a0 = new btVector3(pt.m_localPointA).sub(m_pointCache[1].m_localPointA);
    final btVector3 b0 = new btVector3(m_pointCache[3].m_localPointA).sub(
     m_pointCache[2].m_localPointA);
    final btVector3 cross = a0.cross(b0);
    res0 = cross.lengthSquared();
   }
   if (maxPenetrationIndex != 1) {
    final btVector3 a1 = new btVector3(pt.m_localPointA).sub(m_pointCache[0].m_localPointA);
    final btVector3 b1 = new btVector3(m_pointCache[3].m_localPointA).sub(
     m_pointCache[2].m_localPointA);
    final btVector3 cross = a1.cross(b1);
    res1 = cross.lengthSquared();
   }
   if (maxPenetrationIndex != 2) {
    final btVector3 a2 = new btVector3(pt.m_localPointA).sub(m_pointCache[0].m_localPointA);
    final btVector3 b2 = new btVector3(m_pointCache[3].m_localPointA).sub(
     m_pointCache[1].m_localPointA);
    final btVector3 cross = a2.cross(b2);
    res2 = cross.lengthSquared();
   }
   if (maxPenetrationIndex != 3) {
    final btVector3 a3 = new btVector3(pt.m_localPointA).sub(m_pointCache[0].m_localPointA);
    final btVector3 b3 = new btVector3(m_pointCache[2].m_localPointA).sub(
     m_pointCache[1].m_localPointA);
    final btVector3 cross = a3.cross(b3);
    res3 = cross.lengthSquared();
   }
  } else {
   if (maxPenetrationIndex != 0) {
    res0 = calcArea4Points(pt.m_localPointA, m_pointCache[1].m_localPointA,
     m_pointCache[2].m_localPointA, m_pointCache[3].m_localPointA);
   }
   if (maxPenetrationIndex != 1) {
    res1 = calcArea4Points(pt.m_localPointA, m_pointCache[0].m_localPointA,
     m_pointCache[2].m_localPointA, m_pointCache[3].m_localPointA);
   }
   if (maxPenetrationIndex != 2) {
    res2 = calcArea4Points(pt.m_localPointA, m_pointCache[0].m_localPointA,
     m_pointCache[1].m_localPointA, m_pointCache[3].m_localPointA);
   }
   if (maxPenetrationIndex != 3) {
    res3 = calcArea4Points(pt.m_localPointA, m_pointCache[0].m_localPointA,
     m_pointCache[1].m_localPointA, m_pointCache[2].m_localPointA);
   }
  }
  btVector4 maxvec = new btVector4(res0, res1, res2, res3);
  int biggestarea = maxvec.closestAxis4();
  return biggestarea;
 }
 //int		findContactPoint(  btManifoldPoint  unUsed, int numUnused,  btManifoldPoint  pt) 
 int m_companionIdA;
 int m_companionIdB;
 int m_index1a;

 btPersistentManifold() {
  super(BT_PERSISTENT_MANIFOLD_TYPE);
  for (int i = 0; i < m_pointCache.length; ++i) {
   m_pointCache[i] = new btManifoldPoint();
  }
 }

 btPersistentManifold(btCollisionObject body0, btCollisionObject body1, int a,
  float contactBreakingThreshold, float contactProcessingThreshold) {
  this();
  assert (body0 != null);
  assert (body1 != null);
  m_body0 = body0;
  m_body1 = body1;
  m_cachedPoints = 0;
  m_contactBreakingThreshold = contactBreakingThreshold;
  m_contactProcessingThreshold = contactProcessingThreshold;
 }

 public btCollisionObject getBody0() {
  return m_body0;
 }

 public btCollisionObject getBody1() {
  return m_body1;
 }

 public void setBodies(btCollisionObject body0, btCollisionObject body1) {
  assert (body0 != null);
  assert (body1 != null);
  m_body0 = body0;
  m_body1 = body1;
 }

 void clearUserCache(btManifoldPoint pt) {
  Object oldPtr = pt.m_userPersistentData;
  if (oldPtr != null) {
   if (gContactDestroyedCallback != null) {
    (gContactDestroyedCallback).callback(pt.m_userPersistentData);
    pt.m_userPersistentData = null;
   }
  }
 }

 public int getNumContacts() {
  return m_cachedPoints;
 }
 /// the setNumContacts API is usually not used, except when you gather/fill all contacts manually

 void setNumContacts(int cachedPoints) {
  m_cachedPoints = cachedPoints;
 }

 public btManifoldPoint getContactPoint(int index) {
  assert (index < m_cachedPoints);
  return m_pointCache[index];
 }

 ///@todo: get this margin from the current physics / collision environment
 public float getContactBreakingThreshold() {
  return m_contactBreakingThreshold;
 }

 public float getContactProcessingThreshold() {
  return m_contactProcessingThreshold;
 }

 public void setContactBreakingThreshold(float contactBreakingThreshold) {
  m_contactBreakingThreshold = contactBreakingThreshold;
 }

 public void setContactProcessingThreshold(float contactProcessingThreshold) {
  m_contactProcessingThreshold = contactProcessingThreshold;
 }

 int getCacheEntry(btManifoldPoint newPoint) {
  float shortestDist = getContactBreakingThreshold() * getContactBreakingThreshold();
  int size = getNumContacts();
  int nearestPoint = -1;
  for (int i = 0; i < size; i++) {
   btManifoldPoint mp = m_pointCache[i];
   final btVector3 diffA = new btVector3(mp.m_localPointA).sub(newPoint.m_localPointA);
   float distToManiPoint = diffA.dot(diffA);
   if (distToManiPoint < shortestDist) {
    shortestDist = distToManiPoint;
    nearestPoint = i;
   }
  }
  return nearestPoint;
 }

 public int addManifoldPoint(btManifoldPoint newPoint) {
  return addManifoldPoint(newPoint, false);
 }

 public int addManifoldPoint(btManifoldPoint newPoint, boolean isPredictive) {
  if (!isPredictive) {
   assert (validContactDistance(newPoint));
  }
  int insertIndex = getNumContacts();
  if (insertIndex == MANIFOLD_CACHE_SIZE) {
   if (MANIFOLD_CACHE_SIZE >= 4) //sort cache so best points come first, based on area
   {
    insertIndex = sortCachedPoints(newPoint);
   } else {
    insertIndex = 0;
   }
   clearUserCache(m_pointCache[insertIndex]);
  } else {
   m_cachedPoints++;
  }
  if (insertIndex < 0) {
   insertIndex = 0;
  }
  assert (m_pointCache[insertIndex].m_userPersistentData == null);
  m_pointCache[insertIndex] = newPoint;
  return insertIndex;
 }

 public void removeContactPoint(int index) {
  clearUserCache(m_pointCache[index]);
  int lastUsedIndex = getNumContacts() - 1;
//		m_pointCache[index] = m_pointCache[lastUsedIndex];
  if (index != lastUsedIndex) {
   m_pointCache[index] = m_pointCache[lastUsedIndex];
   //get rid of duplicated userPersistentData pointer
   m_pointCache[lastUsedIndex].m_userPersistentData = null;
   m_pointCache[lastUsedIndex].m_appliedImpulse = 0.f;
   m_pointCache[lastUsedIndex].m_contactPointFlags = 0;
   m_pointCache[lastUsedIndex].m_appliedImpulseLateral1 = 0.f;
   m_pointCache[lastUsedIndex].m_appliedImpulseLateral2 = 0.f;
   m_pointCache[lastUsedIndex].m_lifeTime = 0;
  }
  assert (m_pointCache[lastUsedIndex].m_userPersistentData == null);
  m_cachedPoints--;
 }

 public void replaceContactPoint(btManifoldPoint newPoint, int insertIndex) {
  assert (validContactDistance(newPoint));
  int lifeTime = m_pointCache[insertIndex].getLifeTime();
  float appliedImpulse = m_pointCache[insertIndex].m_appliedImpulse;
  float appliedLateralImpulse1 = m_pointCache[insertIndex].m_appliedImpulseLateral1;
  float appliedLateralImpulse2 = m_pointCache[insertIndex].m_appliedImpulseLateral2;
  assert (lifeTime >= 0);
  Object cache = m_pointCache[insertIndex].m_userPersistentData;
  m_pointCache[insertIndex] = newPoint;
  m_pointCache[insertIndex].m_userPersistentData = cache;
  m_pointCache[insertIndex].m_appliedImpulse = appliedImpulse;
  m_pointCache[insertIndex].m_appliedImpulseLateral1 = appliedLateralImpulse1;
  m_pointCache[insertIndex].m_appliedImpulseLateral2 = appliedLateralImpulse2;
  m_pointCache[insertIndex].m_lifeTime = lifeTime;
 }

 boolean validContactDistance(btManifoldPoint pt) {
  return pt.m_distance1 <= getContactBreakingThreshold();
 }
 /// calculated new worldspace coordinates and depth, and reject points that exceed the collision margin

 void refreshContactPoints(final btTransform trA, final btTransform trB) {
  int i;
  /// first refresh worldspace positions and distance
  for (i = getNumContacts() - 1; i >= 0; i--) {
   btManifoldPoint manifoldPoint = m_pointCache[i];
   trA.transform(manifoldPoint.m_positionWorldOnA.set(manifoldPoint.m_localPointA));
   trB.transform(manifoldPoint.m_positionWorldOnB.set(manifoldPoint.m_localPointB));
   manifoldPoint.m_distance1 = (new btVector3(manifoldPoint.m_positionWorldOnA).sub(
    manifoldPoint.m_positionWorldOnB)).dot(manifoldPoint.m_normalWorldOnB);
   manifoldPoint.m_lifeTime++;
  }
  /// then 
  float distance2d;
  final btVector3 projectedDifference = new btVector3();
  final btVector3 projectedPoint = new btVector3();
  for (i = getNumContacts() - 1; i >= 0; i--) {
   btManifoldPoint manifoldPoint = m_pointCache[i];
   //contact becomes invalid when signed distance exceeds margin (projected on contactnormal direction)
   if (!validContactDistance(manifoldPoint)) {
    removeContactPoint(i);
   } else {
    //contact also becomes invalid when relative movement orthogonal to normal exceeds margin
    projectedPoint.scaleAdd(-manifoldPoint.m_distance1, manifoldPoint.m_normalWorldOnB,
     manifoldPoint.m_positionWorldOnA);
    projectedDifference.set(manifoldPoint.m_positionWorldOnB).sub(projectedPoint);
    distance2d = projectedDifference.dot(projectedDifference);
    if (distance2d > getContactBreakingThreshold() * getContactBreakingThreshold()) {
     removeContactPoint(i);
    } else //contact point processed callback
     if (gContactProcessedCallback != null) {
      (gContactProcessedCallback).callback(manifoldPoint, (Object) m_body0, (Object) m_body1);
     }
   }
  }
 }

 public void clearManifold() {
  int i;
  for (i = 0; i < m_cachedPoints; i++) {
   clearUserCache(m_pointCache[i]);
  }
  m_cachedPoints = 0;
 }

 public static float calcArea4Points(final btVector3 p0, final btVector3 p1, final btVector3 p2,
  final btVector3 p3) {
  // It calculates possible 3 area constructed from random 4 points and returns the biggest one.
  btVector3[] a = {new btVector3(p0), new btVector3(p0), new btVector3(p0)};
  btVector3[] b = {new btVector3(p2), new btVector3(p1), new btVector3(p1)};
  a[0].sub(p1);
  a[1].sub(p2);
  a[2].sub(p3);
  b[0].sub(p3);
  b[1].sub(p3);
  b[2].sub(p2);
  //todo: Following 3 cross production can be easily optimized by SIMD.
  a[0].cross(b[0]);
  a[1].cross(b[1]);
  a[2].cross(b[2]);
  return btMax(btMax(a[0].lengthSquared(), a[1].lengthSquared()), a[2].lengthSquared());
 }

 static int getIslandId(btPersistentManifold lhs) {
  int islandId;
  btCollisionObject rcolObj0 = (lhs.getBody0());
  btCollisionObject rcolObj1 = (lhs.getBody1());
  islandId = rcolObj0.getIslandTag() >= 0 ? rcolObj0.getIslandTag() : rcolObj1.getIslandTag();
  return islandId;
 }
}
