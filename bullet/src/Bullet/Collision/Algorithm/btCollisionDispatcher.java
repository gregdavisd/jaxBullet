/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.Collision.Algorithm;

import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.MAX_BROADPHASE_COLLISION_TYPES;
import Bullet.Collision.Broadphase.btBroadphasePair;
import Bullet.Collision.Broadphase.btDispatcher;
import Bullet.Collision.Broadphase.btDispatcherInfo;
import Bullet.Collision.Broadphase.btOverlappingPairCache;
import Bullet.Collision.btCollisionConfiguration;
import Bullet.Collision.btCollisionObject;
import Bullet.Collision.btCollisionObjectWrapper;
import Bullet.Collision.btManifoldResult;
import Bullet.Collision.btNearCallback;
import Bullet.Collision.btPersistentManifold;
import static Bullet.Collision.btPersistentManifold.gContactBreakingThreshold;
import static Bullet.Collision.ebtDispatcherQueryType.BT_CONTACT_POINT_ALGORITHMS;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collections;
import static javax.vecmath.VecMath.DEBUG_BLOCKS;
import static Bullet.Extras.btMinMax.btMin;

/**
 *
 * @author Gregery Barton
 */
public class btCollisionDispatcher implements btDispatcher, Serializable {

 int m_dispatcherFlags;
 final ArrayList<btPersistentManifold> m_manifoldsPtr = new ArrayList<>(0);
 //final btManifoldResult m_defaultManifoldResult= new btManifoldResult();
 btNearCallback m_nearCallback;
 final btCollisionAlgorithmCreateFunc[][] m_doubleDispatchContactPoints =
  new btCollisionAlgorithmCreateFunc[MAX_BROADPHASE_COLLISION_TYPES][MAX_BROADPHASE_COLLISION_TYPES];
 final btCollisionAlgorithmCreateFunc[][] m_doubleDispatchClosestPoints =
  new btCollisionAlgorithmCreateFunc[MAX_BROADPHASE_COLLISION_TYPES][MAX_BROADPHASE_COLLISION_TYPES];
 btCollisionConfiguration m_collisionConfiguration;
 public static final int CD_STATIC_STATIC_REPORTED = 1;
 public static final int CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD = 2;
 public static final int CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION = 4;

 int getDispatcherFlags() {
  return m_dispatcherFlags;
 }

 void setDispatcherFlags(int flags) {
  m_dispatcherFlags = flags;
 }

 @Override
 public int getNumManifolds() {
  return m_manifoldsPtr.size();
 }

 @Override
 public ArrayList<btPersistentManifold> getInternalManifoldPointer() {
  return m_manifoldsPtr;
 }

 @Override
 public btPersistentManifold getManifoldByIndexInternal(int index) {
  return m_manifoldsPtr.get(index);
 }

 static class DefaultNearCallback implements btNearCallback {

  @Override
  public void callback(btBroadphasePair collisionPair, btCollisionDispatcher dispatcher,
   btDispatcherInfo dispatchInfo) {
   btCollisionObject colObj0 = (btCollisionObject) collisionPair.m_pProxy0.m_clientObject;
   btCollisionObject colObj1 = (btCollisionObject) collisionPair.m_pProxy1.m_clientObject;
   if (dispatcher.needsCollision(colObj0, colObj1)) {
    btCollisionObjectWrapper obj0Wrap = new btCollisionObjectWrapper(null, colObj0
     .getCollisionShape(), colObj0, colObj0.getWorldTransformPtr(), -1, -1);
    btCollisionObjectWrapper obj1Wrap = new btCollisionObjectWrapper(null, colObj1
     .getCollisionShape(), colObj1, colObj1.getWorldTransformPtr(), -1, -1);
    //dispatcher will keep algorithms persistent in the collision pair
    if (collisionPair.m_algorithm == null) {
     collisionPair.m_algorithm = dispatcher.findAlgorithm(obj0Wrap, obj1Wrap, null,
      BT_CONTACT_POINT_ALGORITHMS);
    }
    if (collisionPair.m_algorithm != null) {
     btManifoldResult contactPointResult = new btManifoldResult(obj0Wrap, obj1Wrap);
     if (dispatchInfo.m_dispatchFunc == btDispatcherInfo.DISPATCH_DISCRETE) {
      //discrete collision detection query
      collisionPair.m_algorithm.processCollision(obj0Wrap, obj1Wrap, dispatchInfo,
       contactPointResult);
     } else {
      //continuous collision detection query, time of impact (toi)
      float toi = collisionPair.m_algorithm.calculateTimeOfImpact(colObj0, colObj1, dispatchInfo,
       contactPointResult);
      if (dispatchInfo.m_timeOfImpact > toi) {
       dispatchInfo.m_timeOfImpact = toi;
      }
     }
    }
   }
  }
 }

 public btCollisionDispatcher(btCollisionConfiguration collisionConfiguration) {
  m_dispatcherFlags = CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD;
  m_collisionConfiguration = collisionConfiguration;
  int i;
  setNearCallback(new DefaultNearCallback());
  for (i = 0; i < MAX_BROADPHASE_COLLISION_TYPES; i++) {
   for (int j = 0; j < MAX_BROADPHASE_COLLISION_TYPES; j++) {
    m_doubleDispatchContactPoints[i][j] = m_collisionConfiguration
     .getCollisionAlgorithmCreateFunc(i, j);
    assert (m_doubleDispatchContactPoints[i][j] != null);
    m_doubleDispatchClosestPoints[i][j] = m_collisionConfiguration
     .getClosestPointsAlgorithmCreateFunc(i, j);
   }
  }
 }
 static int gNumManifold = 0;

 @Override
 public btPersistentManifold getNewManifold(btCollisionObject body0, btCollisionObject body1) {
  gNumManifold++;
  //assert(gNumManifold < 65535);
  //optional relative contact breaking threshold, turned on by default (use setDispatcherFlags to switch off feature for improved performance)
  float contactBreakingThreshold = (m_dispatcherFlags &
   btCollisionDispatcher.CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD) != 0 ?
    btMin(body0.getCollisionShape().getContactBreakingThreshold(gContactBreakingThreshold), body1
     .getCollisionShape().getContactBreakingThreshold(gContactBreakingThreshold)) :
    gContactBreakingThreshold;
  float contactProcessingThreshold = btMin(body0.getContactProcessingThreshold(), body1
   .getContactProcessingThreshold());
  btPersistentManifold manifold =
   new btPersistentManifold(body0, body1, 0, contactBreakingThreshold, contactProcessingThreshold);
  manifold.m_index1a = m_manifoldsPtr.size();
  m_manifoldsPtr.add(manifold);
  return manifold;
 }

 @Override
 public void releaseManifold(btPersistentManifold manifold) {
  gNumManifold--;
  if (DEBUG_BLOCKS) {
   for (int i = 0; i < m_manifoldsPtr.size(); ++i) {
    btPersistentManifold mani = m_manifoldsPtr.get(i);
    assert (mani.m_index1a == i);
   }
   boolean found = false;
   for (int i = 0; i < m_manifoldsPtr.size(); ++i) {
    btPersistentManifold mani = m_manifoldsPtr.get(i);
    if (mani == manifold) {
     found = true;
     break;
    }
   }
   assert (found);
  }
  assert (m_manifoldsPtr.get(manifold.m_index1a) == manifold);
  clearManifold(manifold);
  int findIndex = manifold.m_index1a;
  assert (findIndex < m_manifoldsPtr.size());
  Collections.swap(m_manifoldsPtr, findIndex, m_manifoldsPtr.size() - 1);
  assert (m_manifoldsPtr.get(findIndex).m_index1a == m_manifoldsPtr.size() - 1);
  m_manifoldsPtr.get(findIndex).m_index1a = findIndex;
  m_manifoldsPtr.remove(m_manifoldsPtr.size() - 1);
 }

 @Override
 public void clearManifold(btPersistentManifold manifold) {
  manifold.clearManifold();
 }

 @Override
 public btCollisionAlgorithm findAlgorithm(btCollisionObjectWrapper body0Wrap,
  btCollisionObjectWrapper body1Wrap, btPersistentManifold sharedManifold, int algoType) {
  btCollisionAlgorithmConstructionInfo ci = new btCollisionAlgorithmConstructionInfo();
  ci.m_dispatcher1 = this;
  ci.m_manifold = sharedManifold;
  btCollisionAlgorithm algo = null;
  if (algoType == BT_CONTACT_POINT_ALGORITHMS) {
   algo = m_doubleDispatchContactPoints[body0Wrap.getCollisionShape().getShapeType()][body1Wrap
    .getCollisionShape().getShapeType()].CreateCollisionAlgorithm(ci, body0Wrap, body1Wrap);
  } else {
   algo = m_doubleDispatchClosestPoints[body0Wrap.getCollisionShape().getShapeType()][body1Wrap
    .getCollisionShape().getShapeType()].CreateCollisionAlgorithm(ci, body0Wrap, body1Wrap);
  }
  return algo;
 }

 @Override
 public boolean needsCollision(btCollisionObject body0, btCollisionObject body1) {
  assert (body0 != null);
  assert (body1 != null);
  boolean needsCollision = true;
  if ((!body0.isActive()) && (!body1.isActive())) {
   needsCollision = false;
  } else if ((!body0.checkCollideWith(body1)) || (!body1.checkCollideWith(body0))) {
   needsCollision = false;
  }
  return needsCollision;
 }

 @Override
 public boolean needsResponse(btCollisionObject body0, btCollisionObject body1) {
  //here you can do filtering
  boolean hasResponse =
   (body0.hasContactResponse() && body1.hasContactResponse());
  //no response between two static/kinematic bodies:
  hasResponse = hasResponse &&
   ((!body0.isStaticOrKinematicObject()) || (!body1.isStaticOrKinematicObject()));
  return hasResponse;
 }

 @Override
 public void dispatchAllCollisionPairs(btOverlappingPairCache pairCache,
  btDispatcherInfo dispatchInfo, btDispatcher dispatcher) {
  //m_blockedForChanges = true;
  btCollisionPairCallback collisionCallback = new btCollisionPairCallback(dispatchInfo, this);
  pairCache.processAllOverlappingPairs(collisionCallback, dispatcher);
  //m_blockedForChanges = false;
 }

 final void setNearCallback(btNearCallback nearCallback) {
  m_nearCallback = nearCallback;
 }

 btNearCallback getNearCallback() {
  return m_nearCallback;
 }

 //by default, Bullet will use this near callback
 static void defaultNearCallback(btBroadphasePair collisionPair, btCollisionDispatcher dispatcher,
  btDispatcherInfo dispatchInfo) {
  btCollisionObject colObj0 = (btCollisionObject) collisionPair.m_pProxy0.m_clientObject;
  btCollisionObject colObj1 = (btCollisionObject) collisionPair.m_pProxy1.m_clientObject;
  if (dispatcher.needsCollision(colObj0, colObj1)) {
   btCollisionObjectWrapper obj0Wrap = new btCollisionObjectWrapper(null, colObj0
    .getCollisionShape(), colObj0, colObj0.getWorldTransformPtr(), -1, -1);
   btCollisionObjectWrapper obj1Wrap = new btCollisionObjectWrapper(null, colObj1
    .getCollisionShape(), colObj1, colObj1.getWorldTransformPtr(), -1, -1);
   //dispatcher will keep algorithms persistent in the collision pair
   if (collisionPair.m_algorithm == null) {
    collisionPair.m_algorithm = dispatcher.findAlgorithm(obj0Wrap, obj1Wrap, null,
     BT_CONTACT_POINT_ALGORITHMS);
   }
   if (collisionPair.m_algorithm != null) {
    btManifoldResult contactPointResult = new btManifoldResult(obj0Wrap, obj1Wrap);
    if (dispatchInfo.m_dispatchFunc == btDispatcherInfo.DISPATCH_DISCRETE) {
     //discrete collision detection query
     collisionPair.m_algorithm
      .processCollision(obj0Wrap, obj1Wrap, dispatchInfo, contactPointResult);
    } else {
     //continuous collision detection query, time of impact (toi)
     float toi = collisionPair.m_algorithm.calculateTimeOfImpact(colObj0, colObj1, dispatchInfo,
      contactPointResult);
     if (dispatchInfo.m_timeOfImpact > toi) {
      dispatchInfo.m_timeOfImpact = toi;
     }
    }
   }
  }
 }

 btCollisionConfiguration getCollisionConfiguration() {
  return m_collisionConfiguration;
 }

 void setCollisionConfiguration(btCollisionConfiguration config) {
  m_collisionConfiguration = config;
 }
};
