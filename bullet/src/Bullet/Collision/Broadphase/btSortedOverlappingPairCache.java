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
package Bullet.Collision.Broadphase;

import static Bullet.common.btAlignedObjectArray.findLinearSearch;
import java.io.Serializable;
import java.util.ArrayList;
import static java.util.Collections.swap;

/**
 *
 * @author Gregery Barton
 */
public class btSortedOverlappingPairCache implements btOverlappingPairCache, Serializable {

 //avoid brute-force finding all the time
 final ArrayList<btBroadphasePair> m_overlappingPairArray = new ArrayList<>(0);
 //during the dispatch, check that user doesn't destroy/create proxy
 boolean m_blockedForChanges;
 ///by default, do the removal during the pair traversal
 boolean m_hasDeferredRemoval;
 //if set, use the callback instead of the built in filter in needBroadphaseCollision
 btOverlapFilterCallback m_overlapFilterCallback;
 btOverlappingPairCallback m_ghostPairCallback;

 btSortedOverlappingPairCache() {
  m_blockedForChanges = false;
  m_hasDeferredRemoval = true;
  m_overlapFilterCallback = null;
  m_ghostPairCallback = null;
 }

 @Override
 public void processAllOverlappingPairs(btOverlapCallback callback, btDispatcher dispatcher) {
  int i;
  for (i = 0; i < m_overlappingPairArray.size();) {
   btBroadphasePair pair = m_overlappingPairArray.get(i);
   if (callback.processOverlap(pair)) {
    cleanOverlappingPair(pair, dispatcher);
    swap(m_overlappingPairArray, i, m_overlappingPairArray.size() - 1);
    m_overlappingPairArray.remove(m_overlappingPairArray.size() - 1);
   } else {
    i++;
   }
  }
 }

 /**
  *
  * @param proxy0
  * @param proxy1
  * @param dispatcher
  * @return
  */
 @Override
 public Object removeOverlappingPair(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1,
  btDispatcher dispatcher) {
  btBroadphaseProxy do_proxy0 = proxy0;
  btBroadphaseProxy do_proxy1 = proxy1;
  if (do_proxy0.m_uniqueId > do_proxy1.m_uniqueId) {
   btBroadphaseProxy swapper = do_proxy0;
   do_proxy0 = do_proxy1;
   do_proxy1 = swapper;
  }
  if (!hasDeferredRemoval()) {
   btBroadphasePair findPair = new btBroadphasePair(do_proxy0, do_proxy1);
   int findIndex = findLinearSearch(m_overlappingPairArray, findPair);
   if (findIndex < m_overlappingPairArray.size()) {
    btBroadphasePair pair = m_overlappingPairArray.get(findIndex);
//    Object userData = pair.m_internalInfo1;
    cleanOverlappingPair(pair, dispatcher);
    if (m_ghostPairCallback != null) {
     m_ghostPairCallback.removeOverlappingPair(do_proxy0, do_proxy1, dispatcher);
    }
    swap(m_overlappingPairArray, findIndex, m_overlappingPairArray.size() - 1);
    m_overlappingPairArray.remove(m_overlappingPairArray.size() - 1);
    return null;
   }
  }
  return null;
 }

 @Override
 public void cleanOverlappingPair(btBroadphasePair pair, btDispatcher dispatcher) {
  pair.m_algorithm = null;
 }

 /**
  *
  * @param proxy0
  * @param proxy1
  * @return
  */
 @Override
 public btBroadphasePair addOverlappingPair(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1) {
  btBroadphaseProxy do_proxy0 = proxy0;
  btBroadphaseProxy do_proxy1 = proxy1;
  if (do_proxy0.m_uniqueId > do_proxy1.m_uniqueId) {
   btBroadphaseProxy swapper = do_proxy0;
   do_proxy0 = do_proxy1;
   do_proxy1 = swapper;
  }
//don't add overlap with own
  assert (proxy0 != proxy1);
  if (!needsBroadphaseCollision(do_proxy0, do_proxy1)) {
   return null;
  }
  btBroadphasePair pair = new btBroadphasePair(do_proxy0, do_proxy1);
  m_overlappingPairArray.add(pair);
  if (m_ghostPairCallback != null) {
   m_ghostPairCallback.addOverlappingPair(do_proxy0, do_proxy1);
  }
  return pair;
 }

 @Override
 public btBroadphasePair findPair(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1) {
  btBroadphaseProxy do_proxy0 = proxy0;
  btBroadphaseProxy do_proxy1 = proxy1;
  if (do_proxy0.m_uniqueId > do_proxy1.m_uniqueId) {
   btBroadphaseProxy swapper = do_proxy0;
   do_proxy0 = do_proxy1;
   do_proxy1 = swapper;
  }
  if (!needsBroadphaseCollision(do_proxy0, do_proxy1)) {
   return null;
  }
  btBroadphasePair tmpPair = new btBroadphasePair(do_proxy0, do_proxy1);
  int findIndex = findLinearSearch(m_overlappingPairArray, (tmpPair));
  if (findIndex < m_overlappingPairArray.size()) {
   //assert(it != m_overlappingPairSet.end());
   btBroadphasePair pair = m_overlappingPairArray.get(findIndex);
   return pair;
  }
  return null;
 }

 @Override
 public void cleanProxyFromPairs(btBroadphaseProxy proxy, btDispatcher dispatcher) {
  CleanPairCallback cleanPairs = new CleanPairCallback(proxy, this, dispatcher);
  processAllOverlappingPairs(cleanPairs, dispatcher);
 }

 /**
  *
  * @param proxy
  * @param dispatcher
  */
 @Override
 public void removeOverlappingPairsContainingProxy(btBroadphaseProxy proxy, btDispatcher dispatcher) {
  RemovePairCallback removeCallback = new RemovePairCallback(proxy);
  processAllOverlappingPairs(removeCallback, dispatcher);
 }

 public boolean needsBroadphaseCollision(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1) {
  if (m_overlapFilterCallback != null) {
   return m_overlapFilterCallback.needBroadphaseCollision(proxy0, proxy1);
  }
  boolean collides = (proxy0.m_collisionFilterGroup & proxy1.m_collisionFilterMask) != 0;
  collides = collides && ((proxy1.m_collisionFilterGroup & proxy0.m_collisionFilterMask) != 0);
  return collides;
 }

 @Override
 public ArrayList<btBroadphasePair> getOverlappingPairArray() {
  return m_overlappingPairArray;
 }

 @Override
 public ArrayList<btBroadphasePair> getOverlappingPairArrayPtr() {
  return m_overlappingPairArray;
 }

 @Override
 public int getNumOverlappingPairs() {
  return m_overlappingPairArray.size();
 }

 btOverlapFilterCallback getOverlapFilterCallback() {
  return m_overlapFilterCallback;
 }

 @Override
 public void setOverlapFilterCallback(btOverlapFilterCallback callback) {
  m_overlapFilterCallback = callback;
 }

 @Override
 public boolean hasDeferredRemoval() {
  return m_hasDeferredRemoval;
 }

 @Override
 public void setInternalGhostPairCallback(btOverlappingPairCallback ghostPairCallback) {
  m_ghostPairCallback = ghostPairCallback;
 }

 @Override
 public void sortOverlappingPairs(btDispatcher dispatcher) {
  //should already be sorted
 }

 @Override
 public void incrementalCleanup(int ni, btDispatcher dispatcher) {
  assert (false);
 }
};
