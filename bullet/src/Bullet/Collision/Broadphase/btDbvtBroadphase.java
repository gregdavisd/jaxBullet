/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
///btDbvtBroadphase implementation by Nathanael Presson
package Bullet.Collision.Broadphase;

import static Bullet.Collision.Broadphase.btDbvt.Merge;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import java.util.ArrayList;
import static Bullet.Collision.Broadphase.btDbvt.Intersect;
import static Bullet.Extras.btMinMax.btMax;
import static Bullet.Extras.btMinMax.btMin;

/**
 *
 * @author Gregery Barton
 */
public class btDbvtBroadphase extends btBroadphaseInterface implements Serializable {

 /* Config		*/
 static final int DYNAMIC_SET = 0;
 /* Dynamic set index	*/
 static final int FIXED_SET = 1;
 /* Fixed set index		*/
 static final int STAGECOUNT = 2;
 /* Number of stages		*/

 /* Fields		*/
 final btDbvt[] m_sets = new btDbvt[2];					// Dbvt sets
 final ArrayList<btDbvtProxy>[] m_stageRoots = new ArrayList[STAGECOUNT + 1];	// Stages list
 btOverlappingPairCache m_paircache;				// Pair cache
 float m_prediction;				// Velocity prediction
 int m_stageCurrent;				// Current stage
 int m_fupdates;					// % of fixed updates per frame
 int m_dupdates;					// % of dynamic updates per frame
 int m_cupdates;					// % of cleanup updates per frame
 int m_newpairs;					// Number of pairs created
 int m_fixedleft;				// Fixed optimization left
 int m_updates_call;				// Number of updates call
 int m_updates_done;				// Number of updates done
 float m_updates_ratio;			// m_updates_done/m_updates_call
 int m_pid;						// Parse id
 int m_cid;						// Cleanup index
 int m_gid;						// Gen id
 boolean m_releasepaircache;			// Release pair cache on delete
 boolean m_deferedcollide;			// Defere dynamic/static collision to collide call
 boolean m_needcleanup;				// Need to run cleanup?
 final ArrayList< ArrayList<  btDbvtNode>> m_rayTestStacks = new ArrayList<>(0);

 /* Methods		*/
 public btDbvtBroadphase() {
  this(null);
 }

 btDbvtBroadphase(btOverlappingPairCache paircache) {
  m_deferedcollide = false;
  m_needcleanup = true;
  m_releasepaircache = (paircache == null);
  m_prediction = 0;
  m_stageCurrent = 0;
  m_fixedleft = 0;
  m_fupdates = 1;
  m_dupdates = 0;
  m_cupdates = 3;
  m_newpairs = 1;
  m_updates_call = 0;
  m_updates_done = 0;
  m_updates_ratio = 0;
  m_paircache = paircache != null ? paircache : new btHashedOverlappingPairCache();
  m_gid = 0;
  m_pid = 0;
  m_cid = 0;
  for (int i = 0; i < m_sets.length; i++) {
   m_sets[i] = new btDbvt();
  }
  for (int i = 0; i < m_stageRoots.length; i++) {
   m_stageRoots[i] = new ArrayList<>(0);
  }
  m_rayTestStacks.add(new ArrayList<btDbvtNode>(0));
 }

 void collide(btDispatcher dispatcher) {
  //SPC(m_profiling.m_total);
  /* optimize				*/
  m_sets[0].optimizeIncremental(1 + (m_sets[0].m_leaves * m_dupdates) / 100);
  if (m_fixedleft != 0) {
   int count = 1 + (m_sets[1].m_leaves * m_fupdates) / 100;
   m_sets[1].optimizeIncremental(1 + (m_sets[1].m_leaves * m_fupdates) / 100);
   m_fixedleft = btMax(0, m_fixedleft - count);
  }
  /* dynamic . fixed set	*/
  m_stageCurrent = (m_stageCurrent + 1) % STAGECOUNT;
  ArrayList<btDbvtProxy> stage_list = m_stageRoots[m_stageCurrent];
  //btDbvtTreeCollider	collider= new btDbvtTreeCollider(this);
  while (!stage_list.isEmpty()) {
   btDbvtProxy current = stage_list.get(stage_list.size() - 1);
   assert (current.stage == m_stageCurrent);
   m_stageRoots[current.stage].remove(stage_list.size() - 1);
   m_stageRoots[STAGECOUNT].add(current);
   m_sets[0].remove(current.leaf);
   btDbvtAabbMm curAabb = btDbvtAabbMm.fromMM(current.m_aabbMin, current.m_aabbMax);
   current.leaf = m_sets[1].insert(curAabb, 0, current);
   current.stage = STAGECOUNT;
  }
  m_fixedleft = m_sets[1].m_leaves;
  m_needcleanup = true;

  /* collide dynamics		*/
  {
   btDbvtTreeCollider collider = new btDbvtTreeCollider(this);
   if (m_deferedcollide) {
    m_sets[0].collideTTpersistentStack(m_sets[0].m_root, m_sets[1].m_root, collider);
   }
   if (m_deferedcollide) {
    m_sets[0].collideTTpersistentStack(m_sets[0].m_root, m_sets[0].m_root, collider);
   }
  }
  /* clean up				*/
  if (m_needcleanup) {
   //SPC(m_profiling.m_cleanup);
   if (m_paircache.getNumOverlappingPairs() > 0) {
    int ni = btMin(m_paircache.getNumOverlappingPairs(), btMax(m_newpairs, m_paircache
     .getNumOverlappingPairs() * m_cupdates) / 100);
    m_paircache.incrementalCleanup(ni, dispatcher);
   }
  }
  ++m_pid;
  m_newpairs = 1;
  m_needcleanup = false;
  if (m_updates_call > 0) {
   m_updates_ratio = m_updates_done / (float) m_updates_call;
  } else {
   m_updates_ratio = 0;
  }
  m_updates_done /= 2;
  m_updates_call /= 2;
 }

 void optimize() {
  m_sets[0].optimizeTopDown();
  m_sets[1].optimizeTopDown();
 }

 /* btBroadphaseInterface Implementation	*/
 @Override
 public btBroadphaseProxy createProxy(final btVector3 aabbMin, final btVector3 aabbMax,
  int shapeType,
  Object userPtr, int collisionFilterGroup, int collisionFilterMask, btDispatcher dispatcher) {
  btDbvtProxy proxy = new btDbvtProxy(aabbMin, aabbMax, userPtr,
   collisionFilterGroup,
   collisionFilterMask);
  btDbvtAabbMm aabb = btDbvtAabbMm.fromMM(aabbMin, aabbMax);
  //bproxy.aabb			=	btDbvtAabbMm.FromMM(aabbMin,aabbMax);
  proxy.stage = m_stageCurrent;
  proxy.m_uniqueId = ++m_gid;
  proxy.leaf = m_sets[0].insert(aabb, 0, proxy);
  m_stageRoots[m_stageCurrent].add(proxy);
  if (!m_deferedcollide) {
   btDbvtTreeCollider collider = new btDbvtTreeCollider(this);
   collider.proxy = proxy;
   m_sets[0].collideTV(m_sets[0].m_root, aabb, collider);
   m_sets[1].collideTV(m_sets[1].m_root, aabb, collider);
  }
  return (proxy);
 }

 @Override
 public void destroyProxy(btBroadphaseProxy absproxy, btDispatcher dispatcher) {
  btDbvtProxy proxy = (btDbvtProxy) absproxy;
  if (proxy.stage == STAGECOUNT) {
   m_sets[1].remove(proxy.leaf);
  } else {
   m_sets[0].remove(proxy.leaf);
  }
  m_stageRoots[proxy.stage].remove(proxy);
  m_paircache.removeOverlappingPairsContainingProxy(proxy, dispatcher);
  m_needcleanup = true;
 }
 static final float DBVT_BP_MARGIN = 0.05f;

 @Override
 public void setAabb(btBroadphaseProxy absproxy, final btVector3 aabbMin, final btVector3 aabbMax,
  btDispatcher dispatcher) {
  btDbvtProxy proxy = (btDbvtProxy) absproxy;
  btDbvtAabbMm aabb = btDbvtAabbMm.fromMM(aabbMin, aabbMax);
  {
   boolean docollide = false;
   if (proxy.stage == STAGECOUNT) {/* fixed . dynamic set	*/
    m_sets[1].remove(proxy.leaf);
    proxy.leaf = m_sets[0].insert(aabb, 0, proxy);
    docollide = true;
   } else {/* dynamic set				*/
    ++m_updates_call;
    if (Intersect(proxy.leaf.volume(), aabb)) {/* Moving				*/

     final btVector3 delta = new btVector3(aabbMin).sub(proxy.m_aabbMin);
     final btVector3 velocity = new btVector3(proxy.m_aabbMax).sub(proxy.m_aabbMin).scale(1.0f /
      2.0f)
      .scale(m_prediction);
     if (delta.x < 0) {
      velocity.x = -velocity.x;
     }
     if (delta.y < 0) {
      velocity.y = -velocity.y;
     }
     if (delta.z < 0) {
      velocity.z = -velocity.z;
     }
     if (m_sets[0].update(proxy.leaf, aabb, velocity, DBVT_BP_MARGIN)) {
      ++m_updates_done;
      docollide = true;
     }
    } else {/* Teleporting			*/
     m_sets[0].update(proxy.leaf, aabb);
     ++m_updates_done;
     docollide = true;
    }
   }
   m_stageRoots[proxy.stage].remove(proxy);
   proxy.m_aabbMin.set(aabbMin);
   proxy.m_aabbMax.set(aabbMax);
   proxy.stage = m_stageCurrent;
   m_stageRoots[m_stageCurrent].add(proxy);
   if (docollide) {
    m_needcleanup = true;
    if (!m_deferedcollide) {
     btDbvtTreeCollider collider = new btDbvtTreeCollider(this);
     m_sets[1].collideTTpersistentStack(m_sets[1].m_root, proxy.leaf, collider);
     m_sets[0].collideTTpersistentStack(m_sets[0].m_root, proxy.leaf, collider);
    }
   }
  }
 }

 @Override
 public void rayTest(final btVector3 rayFrom, final btVector3 rayTo,
  btBroadphaseRayCallback rayCallback, final btVector3 aabbMin, final btVector3 aabbMax) {
  BroadphaseRayTester callback = new BroadphaseRayTester(rayCallback);
  ArrayList<  btDbvtNode> stack = m_rayTestStacks.get(0);
  m_sets[0].rayTestInternal(m_sets[0].m_root,
   rayFrom,
   rayTo,
   rayCallback.m_rayDirectionInverse,
   rayCallback.m_signs,
   rayCallback.m_lambda_max,
   aabbMin,
   aabbMax,
   stack,
   callback);
  m_sets[1].rayTestInternal(m_sets[1].m_root,
   rayFrom,
   rayTo,
   rayCallback.m_rayDirectionInverse,
   rayCallback.m_signs,
   rayCallback.m_lambda_max,
   aabbMin,
   aabbMax,
   stack,
   callback);
 }

 @Override
 public void aabbTest(final btVector3 aabbMin, final btVector3 aabbMax,
  btBroadphaseAabbCallback aabbCallback) {
  BroadphaseAabbTester callback = new BroadphaseAabbTester(aabbCallback);
  btDbvtAabbMm bounds = btDbvtAabbMm.fromMM(aabbMin, aabbMax);
  //process all children, that overlap with  the given AABB bounds
  m_sets[0].collideTV(m_sets[0].m_root, bounds, callback);
  m_sets[1].collideTV(m_sets[1].m_root, bounds, callback);
 }

 @Override
 public void getAabb(btBroadphaseProxy absproxy, final btVector3 aabbMin, final btVector3 aabbMax) {
  btDbvtProxy proxy = (btDbvtProxy) absproxy;
  aabbMin.set(proxy.m_aabbMin);
  aabbMax.set(proxy.m_aabbMax);
 }

 @Override
 public void calculateOverlappingPairs(btDispatcher dispatcher) {
  collide(dispatcher);
  performDeferredRemoval(dispatcher);
 }

 @Override
 public btOverlappingPairCache getOverlappingPairCache() {
  return (m_paircache);
 }

 @Override
 public void getBroadphaseAabb(final btVector3 aabbMin, final btVector3 aabbMax) {
  btDbvtAabbMm bounds = new btDbvtAabbMm();
  if (!m_sets[0].empty()) {
   if (!m_sets[1].empty()) {
    Merge(m_sets[0].m_root.volume(),
     m_sets[1].m_root.volume(), bounds);
   } else {
    bounds = m_sets[0].m_root.volume();
   }
  } else if (!m_sets[1].empty()) {
   bounds = m_sets[1].m_root.volume();
  } else {
   bounds = btDbvtAabbMm.fromCR(new btVector3(), 0);
  }
  aabbMin.set(bounds.mins());
  aabbMax.set(bounds.maxs());
 }

 @Override
 public void printStats() {
 }

 ///reset broadphase internal structures, to ensure determinism/reproducability
 @Override
 public void resetPool(btDispatcher dispatcher) {
  int totalObjects = m_sets[0].m_leaves + m_sets[1].m_leaves;
  if (totalObjects != 0) {
   //reset internal dynamic tree data structures
   m_sets[0].clear();
   m_sets[1].clear();
   m_deferedcollide = false;
   m_needcleanup = true;
   m_stageCurrent = 0;
   m_fixedleft = 0;
   m_fupdates = 1;
   m_dupdates = 0;
   m_cupdates = 3;
   m_newpairs = 1;
   m_updates_call = 0;
   m_updates_done = 0;
   m_updates_ratio = 0;
   m_gid = 0;
   m_pid = 0;
   m_cid = 0;
   for (int i = 0; i <= STAGECOUNT; ++i) {
    m_stageRoots[i] = null;
   }
  }
 }

 void performDeferredRemoval(btDispatcher dispatcher) {
  if (m_paircache.hasDeferredRemoval()) {
   ArrayList<btBroadphasePair> overlappingPairArray = m_paircache.getOverlappingPairArrayPtr();
   //perform a sort, to find duplicates and to sort 'invalid' pairs to the end
   overlappingPairArray.sort(new btBroadphasePairSortPredicate());
   int invalidPair = 0;
   int i;
   btBroadphasePair previousPair = new btBroadphasePair();
   previousPair.m_pProxy0 = null;
   previousPair.m_pProxy1 = null;
   previousPair.m_algorithm = null;
   for (i = 0; i < overlappingPairArray.size(); i++) {
    btBroadphasePair pair = overlappingPairArray.get(i);
    boolean isDuplicate = (pair == previousPair);
    previousPair = pair;
    boolean needsRemoval = false;
    if (!isDuplicate) {
     //important to perform AABB check that is consistent with the broadphase
     btDbvtProxy pa = (btDbvtProxy) pair.m_pProxy0;
     btDbvtProxy pb = (btDbvtProxy) pair.m_pProxy1;
     boolean hasOverlap = Intersect(pa.leaf.volume(), pb.leaf.volume());
     needsRemoval = !hasOverlap;
    } else {
     //remove duplicate
     needsRemoval = true;
     //should have no algorithm
     assert (null == pair.m_algorithm);
    }
    if (needsRemoval) {
     m_paircache.cleanOverlappingPair(pair, dispatcher);
     pair.m_pProxy0 = null;
     pair.m_pProxy1 = null;
     invalidPair++;
    }
   }
   //perform a sort, to sort 'invalid' pairs to the end
   overlappingPairArray.sort(new btBroadphasePairSortPredicate());
   {
    int new_size = overlappingPairArray.size() - invalidPair;
    while (overlappingPairArray.size() > new_size) {
     assert (overlappingPairArray.get(overlappingPairArray.size() - 1).m_pProxy0 == null);
     assert (overlappingPairArray.get(overlappingPairArray.size() - 1).m_pProxy1 == null);
     overlappingPairArray.remove(overlappingPairArray.size() - 1);
    }
   }
  }
 }

 void setVelocityPrediction(float prediction) {
  m_prediction = prediction;
 }

 float getVelocityPrediction() {
  return m_prediction;
 }

 ///this setAabbForceUpdate is similar to setAabb but always forces the aabb update. 
 ///it is not part of the btBroadphaseInterface but specific to btDbvtBroadphase.
 ///it bypasses certain optimizations that prevent aabb updates (when the aabb shrinks), see
 ///http://code.google.com/p/bullet/issues/detail?id=223
 void setAabbForceUpdate(btBroadphaseProxy absproxy, final btVector3 aabbMin,
  final btVector3 aabbMax,
  btDispatcher dispatcher) {
  btDbvtProxy proxy = (btDbvtProxy) absproxy;
  btDbvtAabbMm aabb = btDbvtAabbMm.fromMM(aabbMin, aabbMax);
  boolean docollide;
  if (proxy.stage == STAGECOUNT) {/* fixed . dynamic set	*/
   m_sets[1].remove(proxy.leaf);
   proxy.leaf = m_sets[0].insert(aabb, 0, proxy);
   docollide = true;
  } else {/* dynamic set				*/
   ++m_updates_call;
   /* Teleporting			*/
   m_sets[0].update(proxy.leaf, aabb);
   ++m_updates_done;
   docollide = true;
  }
  m_stageRoots[proxy.stage].remove(proxy);
  proxy.m_aabbMin.set(aabbMin);
  proxy.m_aabbMax.set(aabbMax);
  proxy.stage = m_stageCurrent;
  m_stageRoots[m_stageCurrent].add(proxy);
  if (docollide) {
   m_needcleanup = true;
   if (!m_deferedcollide) {
    btDbvtTreeCollider collider = new btDbvtTreeCollider(this);
    m_sets[1].collideTTpersistentStack(m_sets[1].m_root, proxy.leaf, collider);
    m_sets[0].collideTTpersistentStack(m_sets[0].m_root, proxy.leaf, collider);
   }
  }
 }

 static void benchmark(btBroadphaseInterface broadphase) {
 }
}
