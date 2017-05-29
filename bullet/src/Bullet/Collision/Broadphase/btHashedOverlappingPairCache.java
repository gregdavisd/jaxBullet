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
package Bullet.Collision.Broadphase;
/// Hash-space based Pair Cache, thanks to Erin Catto, Box2D, http://www.box2d.org, and Pierre Terdiman, Codercorner, http://codercorner.com

import static Bullet.LinearMath.btQuickprof.BT_PROFILE;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map.Entry;
import javax.vecmath.Tuple2i;

/**
 *
 * @author Gregery Barton
 */
public class btHashedOverlappingPairCache implements btOverlappingPairCache, Serializable {

 private static final long serialVersionUID = 1L;

 final HashMap<BroadphasePairKey, btBroadphasePair> m_overlappingPairMap = new HashMap<>();
 btOverlapFilterCallback m_overlapFilterCallback;
 btOverlappingPairCallback m_ghostPairCallback;
 Iterator<Entry<BroadphasePairKey, btBroadphasePair>> cleaner;

 public btHashedOverlappingPairCache() {
  m_overlapFilterCallback = null;
  m_ghostPairCallback = null;
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
//gRemovePairs++;
  btBroadphaseProxy do_proxy0 = proxy0;
  btBroadphaseProxy do_proxy1 = proxy1;
  if (do_proxy0.m_uniqueId > do_proxy1.m_uniqueId) {
   btBroadphaseProxy swapper = do_proxy0;
   do_proxy0 = do_proxy1;
   do_proxy1 = swapper;
   //btSwap(proxy0,proxy1);
  }
  int proxyId1 = do_proxy0.getUid();
  int proxyId2 = do_proxy1.getUid();
  btBroadphasePair pair = m_overlappingPairMap.remove(new BroadphasePairKey(proxyId1, proxyId2));
  if (pair == null) {
   return null;
  }
  cleaner = null;
  cleanOverlappingPair(pair, dispatcher);
  assert (pair.m_pProxy0.getUid() == proxyId1);
  assert (pair.m_pProxy1.getUid() == proxyId2);
  if (m_ghostPairCallback != null) {
   m_ghostPairCallback.removeOverlappingPair(do_proxy0, do_proxy1, dispatcher);
  }
  return null;
 }

 void removeOverlappingPair(btBroadphasePair pair, btDispatcher dispatcher, Iterator iter) {
//gRemovePairs++;
  iter.remove();
  cleanOverlappingPair(pair, dispatcher);
  if (m_ghostPairCallback != null) {
   m_ghostPairCallback.removeOverlappingPair(pair.m_pProxy0, pair.m_pProxy1, dispatcher);
  }
 }

 boolean needsBroadphaseCollision(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1) {
  if (m_overlapFilterCallback != null) {
   return m_overlapFilterCallback.needBroadphaseCollision(proxy0, proxy1);
  }
  boolean collides = (proxy0.m_collisionFilterGroup & proxy1.m_collisionFilterMask) != 0;
  collides = collides && ((proxy1.m_collisionFilterGroup & proxy0.m_collisionFilterMask) != 0);
  return collides;
 }

 // Add a pair and return the new pair. If the pair already exists,
 // no new pair is created and the old one is returned.
 /**
  *
  * @param proxy0
  * @param proxy1
  * @return
  */
 @Override
 public btBroadphasePair addOverlappingPair(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1) {
  if (!needsBroadphaseCollision(proxy0, proxy1)) {
   return null;
  }
  return internalAddPair(proxy0, proxy1);
 }

 /**
  *
  * @param proxy
  * @param dispatcher
  */
 @Override
 public void cleanProxyFromPairs(btBroadphaseProxy proxy, btDispatcher dispatcher) {
  CleanPairCallback cleanPairs = new CleanPairCallback(proxy, this, dispatcher);
  processAllOverlappingPairs(cleanPairs, dispatcher);
 }

 @Override
 public void processAllOverlappingPairs(btOverlapCallback callback, btDispatcher dispatcher) {
  BT_PROFILE("btHashedOverlappingPairCache.processAllOverlappingPairs");
//	printf("m_overlappingPairArray.size()=%d\n",m_overlappingPairArray.size());
  for (Iterator<Entry<BroadphasePairKey, btBroadphasePair>> i = m_overlappingPairMap.entrySet()
   .iterator(); i.hasNext();) {
   btBroadphasePair pair = i.next().getValue();
   if (callback.processOverlap(pair)) {
    removeOverlappingPair(pair, dispatcher, i);
   }
  }
 }

 @Override
 public ArrayList<btBroadphasePair> getOverlappingPairArrayPtr() {
  assert (false);
  return new ArrayList<>(0);
 }

 @Override
 public Collection<btBroadphasePair> getOverlappingPairArray() {
  return m_overlappingPairMap.values();
 }

 @Override
 public void cleanOverlappingPair(btBroadphasePair pair, btDispatcher dispatcher) {
  if ((pair.m_algorithm != null) && (dispatcher != null)) {
   {
    pair.m_algorithm.destroy();
    pair.m_algorithm = null;
   }
  }
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
  int proxyId1 = do_proxy0.getUid();
  int proxyId2 = do_proxy1.getUid();
  return m_overlappingPairMap.get(new BroadphasePairKey(proxyId1, proxyId2));
 }

 public int getCount() {
  return m_overlappingPairMap.size();
 }
//	btBroadphasePair* GetPairs() { return m_pairs; }

 public btOverlapFilterCallback
  getOverlapFilterCallback() {
  return m_overlapFilterCallback;
 }

 @Override
 public void setOverlapFilterCallback(btOverlapFilterCallback callback) {
  m_overlapFilterCallback = callback;
 }

 @Override
 public int getNumOverlappingPairs() {
  return m_overlappingPairMap.size();
 }

 btBroadphasePair internalAddPair(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1) {
  btBroadphaseProxy do_proxy0 = proxy0;
  btBroadphaseProxy do_proxy1 = proxy1;
  if (do_proxy0.m_uniqueId > do_proxy1.m_uniqueId) {
   btBroadphaseProxy swapper = do_proxy0;
   do_proxy0 = do_proxy1;
   do_proxy1 = swapper;
  }
  int proxyId1 = do_proxy0.getUid();
  int proxyId2 = do_proxy1.getUid();
  BroadphasePairKey key = new BroadphasePairKey(proxyId1, proxyId2);
  {
   btBroadphasePair pair = m_overlappingPairMap.get(key);
   if (pair != null) {
    return pair;
   }
  }
  //this is where we add an actual pair, so also call the 'ghost'
  if (m_ghostPairCallback != null) {
   m_ghostPairCallback.addOverlappingPair(do_proxy0, do_proxy1);
  }
  btBroadphasePair pair = new btBroadphasePair(do_proxy0, do_proxy1);
  m_overlappingPairMap.put(key, pair);
  cleaner = null;
  pair.m_algorithm = null;
  return pair;
 }

 boolean equalsPair(btBroadphasePair pair, int proxyId1, int proxyId2) {
  return pair.m_pProxy0.getUid() == proxyId1 && pair.m_pProxy1.getUid() == proxyId2;
 }

 @Override
 public boolean hasDeferredRemoval() {
  return false;
 }

 @Override
 public void setInternalGhostPairCallback(btOverlappingPairCallback ghostPairCallback) {
  m_ghostPairCallback = ghostPairCallback;
 }

 @Override
 public void sortOverlappingPairs(btDispatcher dispatcher) {
  assert (false);
 }

 @Override
 public void incrementalCleanup(int ni, btDispatcher dispatcher) {
  if (cleaner == null || !cleaner.hasNext()) {
   cleaner = m_overlappingPairMap.entrySet().iterator();
  }
  int do_ni = ni;
  while (do_ni > 0) {
   while ((do_ni > 0) && cleaner != null && cleaner.hasNext()) {
    btBroadphasePair p = cleaner.next().getValue();
    btBroadphaseProxy pa = p.m_pProxy0;
    btBroadphaseProxy pb = p.m_pProxy1;
    if (!pa.intersect(pb)) {
     removeOverlappingPair(p, dispatcher, cleaner);
    }
    --do_ni;
   }
   if (do_ni > 0) {
    cleaner = m_overlappingPairMap.entrySet().iterator();
   }
  }
 }

 static class BroadphasePairKey extends Tuple2i implements Serializable {

  static final long serialVersionUID = 1;

  public BroadphasePairKey(int x, int y) {
   super(x, y);
  }

  @Override
  public int hashCode() {
   /**
    * Thomas Wang's hash, see: http://www.concentric.net/~Ttwang/tech/inthash.htm This assumes
    * proxyId1 and proxyId2 are 16-bit.
    */
   int key = ((x) | ((y) << 16));
   // Thomas Wang's hash
   key += ~(key << 15);
   key ^= (key >>> 10);
   key += (key << 3);
   key ^= (key >>> 6);
   key += ~(key << 11);
   key ^= (key >>> 16);
   return (int) Math.abs(key);
  }
 }
}
