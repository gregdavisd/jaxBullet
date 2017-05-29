//Bullet Continuous Collision Detection and Physics Library
//Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
// btAxisSweep3.h
//
// Copyright (c) 2006 Simon Hobbs
//
// This software is provided 'as-is', without any express or implied warranty. In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose, including commercial applications, and to alter it and redistribute it freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.
package Bullet.Collision.Broadphase;

import static Bullet.LinearMath.btAabbUtil2.TestAabbAgainstAabb2;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
import static javax.vecmath.VecMath.DEBUG_BLOCKS;

/**
 * The bt32BitAxisSweep3 allows higher precision quantization and more objects compared to the
 * btAxisSweep3 sweep and prune. This comes at the cost of more memory per handle, and a bit slower
 * performance. It uses arrays rather then lists for storage of the 3 axis.
 *
 * @author Gregery Barton
 */
public class btAxisSweep3 extends btBroadphaseInterface implements Serializable {

 protected short m_bpHandleMask;
 protected short m_handleSentinel;

 public static class Edge {

  public short m_pos;			// low bit is min/max
  public short m_handle;

  short IsMax() {
   return (short) (m_pos & 1);
  }

  public Edge() {
  }
 }

 public class Handle extends btBroadphaseProxy {

  // indexes into the edge arrays
  public short[] m_minEdges = new short[3];
  public short[] m_maxEdges = new short[3];		// 6 * 2 = 12
//		short m_uniqueId;
  public btBroadphaseProxy m_dbvtProxy;//for faster raycast
  //void* m_pOwner; this is now in btBroadphaseProxy.m_clientObject

  public void SetNextFree(short next) {
   m_minEdges[0] = next;
   assert (m_minEdges[0] >= 0);
  }

  public short GetNextFree() {
   return m_minEdges[0];
  }

  @Override
  public boolean intersect(btBroadphaseProxy other) {
   return testAabbOverlap(this, other);
  }
 } 		// 24 bytes + 24 for Edge structures = 44 bytes total per entry
 protected final btVector3 m_worldAabbMin = new btVector3();						// overall system bounds
 protected final btVector3 m_worldAabbMax = new btVector3();						// overall system bounds
 protected final btVector3 m_quantize = new btVector3();						// scaling factor for quantization
 protected short m_numHandles;						// number of active handles
 protected short m_maxHandles;						// max number of handles
 protected Handle[] m_pHandles;						// handles pool
 protected short m_firstFreeHandle;		// free handles list
 protected Edge[][] m_pEdges = new Edge[3][];						// edge arrays for the 3 axes (each array has m_maxHandles * 2 + 2 sentinel entries)
 protected btOverlappingPairCache m_pairCache;
 ///btOverlappingPairCallback is an additional optional user callback for adding/removing overlapping pairs, similar interface to btOverlappingPairCache.
 protected btOverlappingPairCallback m_userPairCallback;
 protected boolean m_ownsPairCache;
 protected int m_invalidPair;
 ///additional dynamic aabb structure, used to accelerate ray cast queries.
 ///can be disabled using a optional argument in the constructor
 protected btDbvtBroadphase m_raycastAccelerator;
 protected btOverlappingPairCache m_nullPairCache;

 // allocation/deallocation
 protected short allocHandle() {
  assert (m_firstFreeHandle != 0);
  short handle = m_firstFreeHandle;
  m_firstFreeHandle = getHandle(handle).GetNextFree();
  m_numHandles++;
  return handle;
 }

 protected void freeHandle(short handle) {
  assert (handle > 0 && handle < m_maxHandles);
  getHandle(handle).SetNextFree(m_firstFreeHandle);
  m_firstFreeHandle = handle;
  m_numHandles--;
 }

 protected boolean testOverlap2D(Handle pHandleA, Handle pHandleB, short axis0, short axis1) {
  //optimization 1: check the array index (memory address), instead of the m_pos
  if (pHandleA.m_maxEdges[axis0] < pHandleB.m_minEdges[axis0] ||
   pHandleB.m_maxEdges[axis0] < pHandleA.m_minEdges[axis0] ||
   pHandleA.m_maxEdges[axis1] < pHandleB.m_minEdges[axis1] ||
   pHandleB.m_maxEdges[axis1] < pHandleA.m_minEdges[axis1]) {
   return false;
  }
  return true;
 }

 protected void sortMinDown(short axis, short edge, btDispatcher dispatcher, boolean updateOverlaps) {
  int i_pEdge = edge;
  int i_pPrev = (edge - 1);
  assert (i_pEdge > 0 && i_pPrev >= 0);
  Edge pEdge = m_pEdges[axis][i_pEdge];
  Edge pPrev = m_pEdges[axis][i_pPrev];
  Handle pHandleEdge = getHandle(pEdge.m_handle);
  if (DEBUG_BLOCKS) {
   verify_edges();
  }
  while (pEdge.m_pos < pPrev.m_pos) {
   assert (pPrev.m_pos != 0);
   Handle pHandlePrev = getHandle(pPrev.m_handle);
   if (pPrev.IsMax() != 0) {
    // if previous edge is a maximum check the bounds and add an overlap if necessary
    short axis1 = (short) (((short) 1 << axis) & 3);
    short axis2 = (short) (((short) 1 << axis1) & 3);
    if (updateOverlaps && testOverlap2D(pHandleEdge, pHandlePrev, axis1, axis2)) {
     m_pairCache.addOverlappingPair(pHandleEdge, pHandlePrev);
     if (m_userPairCallback != null) {
      m_userPairCallback.addOverlappingPair(pHandleEdge, pHandlePrev);
     }
    }
    // update edge reference in other handle
    pHandlePrev.m_maxEdges[axis]++;
   } else {
    pHandlePrev.m_minEdges[axis]++;
   }
   pHandleEdge.m_minEdges[axis]--;
   assert (pHandleEdge.m_minEdges[axis] >= 0);
   // swap the edges
   m_pEdges[axis][i_pEdge] = pPrev;
   m_pEdges[axis][i_pPrev] = pEdge;
   // decrement
   i_pEdge--;
   i_pPrev--;
   pEdge = m_pEdges[axis][i_pEdge];
   pPrev = m_pEdges[axis][i_pPrev];
  }
  if (DEBUG_BLOCKS) {
   verify_edges();
  }
 }

 protected void sortMinUp(short axis, short edge, btDispatcher dispatcher, boolean updateOverlaps) {
  int i_pEdge = edge;
  int i_pNext = (i_pEdge + 1);
  assert (i_pEdge >= 0);
  Edge pEdge = m_pEdges[axis][i_pEdge];
  Edge pNext = m_pEdges[axis][i_pNext];
  Handle pHandleEdge = getHandle(pEdge.m_handle);
  if (DEBUG_BLOCKS) {
   verify_edges();
  }
  while (pNext.m_handle != 0 && (pEdge.m_pos > pNext.m_pos)) {
   Handle pHandleNext = getHandle(pNext.m_handle);
   if (pNext.IsMax() != 0) {
    Handle handle0 = getHandle(pEdge.m_handle);
    Handle handle1 = getHandle(pNext.m_handle);
    final short axis1 = (short) (((short) 1 << axis) & (short) 3);
    final short axis2 = (short) (((short) 1 << axis1) & (short) 3);
    // if next edge is maximum remove any overlap between the two handles
    if (updateOverlaps &&
     testOverlap2D(handle0, handle1, axis1, axis2)) {
     m_pairCache.removeOverlappingPair(handle0, handle1, dispatcher);
     if (m_userPairCallback != null) {
      m_userPairCallback.removeOverlappingPair(handle0, handle1, dispatcher);
     }
    }
    // update edge reference in other handle
    pHandleNext.m_maxEdges[axis]--;
    assert (pHandleNext.m_maxEdges[axis] >= 0);
   } else {
    pHandleNext.m_minEdges[axis]--;
   }
   pHandleEdge.m_minEdges[axis]++;
   assert (pHandleNext.m_minEdges[axis] >= 0);
   // swap the edges
   m_pEdges[axis][i_pEdge] = pNext;
   m_pEdges[axis][i_pNext] = pEdge;
   // increment
   i_pEdge++;
   i_pNext++;
   pEdge = m_pEdges[axis][i_pEdge];
   pNext = m_pEdges[axis][i_pNext];
  }
  if (DEBUG_BLOCKS) {
   verify_edges();
  }
 }

 protected void sortMaxDown(short axis, short edge, btDispatcher dispatcher, boolean updateOverlaps) {
  int i_pEdge = edge;
  int i_pPrev = (edge - 1);
  assert (i_pEdge > 0 && i_pPrev >= 0);
  Edge pEdge = m_pEdges[axis][i_pEdge];
  Edge pPrev = m_pEdges[axis][i_pPrev];
  Handle pHandleEdge = getHandle(pEdge.m_handle);
  if (DEBUG_BLOCKS) {
   verify_edges();
  }
  while (pEdge.m_pos < pPrev.m_pos) {
   assert (pPrev.m_pos != 0);
   Handle pHandlePrev = getHandle(pPrev.m_handle);
   if (0 == pPrev.IsMax()) {
    // if previous edge was a minimum remove any overlap between the two handles
    Handle handle0 = getHandle(pEdge.m_handle);
    Handle handle1 = getHandle(pPrev.m_handle);
    final short axis1 = (short) (((short) 1 << axis) & (short) 3);
    final short axis2 = (short) (((short) 1 << axis1) & (short) 3);
    if (updateOverlaps &&
     testOverlap2D(handle0, handle1, axis1, axis2)) {
     //this is done during the overlappingpairarray iteration/narrowphase collision
     m_pairCache.removeOverlappingPair(handle0, handle1, dispatcher);
     if (m_userPairCallback != null) {
      m_userPairCallback.removeOverlappingPair(handle0, handle1, dispatcher);
     }
    }
    // update edge reference in other handle
    pHandlePrev.m_minEdges[axis]++;;
   } else {
    pHandlePrev.m_maxEdges[axis]++;
   }
   pHandleEdge.m_maxEdges[axis]--;
   // swap the edges
   m_pEdges[axis][i_pEdge] = pPrev;
   m_pEdges[axis][i_pPrev] = pEdge;
   // decrement
   i_pEdge--;
   i_pPrev--;
   assert (i_pEdge > 0 && i_pPrev >= 0);
   pEdge = m_pEdges[axis][i_pEdge];
   pPrev = m_pEdges[axis][i_pPrev];
  }
  if (DEBUG_BLOCKS) {
   verify_edges();
  }
 }

 private void verify_edges() {
  if (DEBUG_BLOCKS) {
   for (short axis = 0; axis < 3; ++axis) {
    assert (m_pEdges[axis][0].m_handle == 0);
    assert (m_pEdges[axis][0].m_pos == 0);
    boolean found_sentinel = false;
    for (int i = m_numHandles * 2 + 2; i-- > 0;) {
     if (m_pEdges[axis][i].m_pos != 0) {
      if (m_pEdges[axis][i].m_pos == m_handleSentinel) {
       found_sentinel = true;
      } else if (!found_sentinel) {
       assert (false);
      } else {
       break;
      }
     }
    }
   }
  }
 }

 private void verify_edges_sorted() {
  if (DEBUG_BLOCKS) {
   for (short axis = 0; axis < 3; ++axis) {
    for (int i = 1; i < m_numHandles * 2; ++i) {
     assert (m_pEdges[axis][i].m_pos >= m_pEdges[axis][i - 1].m_pos);
    }
   }
  }
 }

 protected void sortMaxUp(short axis, short edge, btDispatcher dispatcher, boolean updateOverlaps) {
  int i_pEdge = edge;
  int i_pNext = (edge + 1);
  assert (i_pEdge > 0 && i_pNext < m_pEdges[axis].length);
  Edge pEdge = m_pEdges[axis][i_pEdge];
  Edge pNext = m_pEdges[axis][i_pNext];
  Handle pHandleEdge = getHandle(pEdge.m_handle);
  if (DEBUG_BLOCKS) {
   verify_edges();
  }
  while (pNext.m_handle != 0 && (pEdge.m_pos > pNext.m_pos)) {
   Handle pHandleNext = getHandle(pNext.m_handle);
   final short axis1 = (short) ((1 << axis) & 3);
   final short axis2 = (short) ((1 << axis1) & 3);
   if (0 == pNext.IsMax()) {
    // if next edge is a minimum check the bounds and add an overlap if necessary
    if (updateOverlaps && testOverlap2D(pHandleEdge, pHandleNext, axis1, axis2)) {
     Handle handle0 = getHandle(pEdge.m_handle);
     Handle handle1 = getHandle(pNext.m_handle);
     m_pairCache.addOverlappingPair(handle0, handle1);
     if (m_userPairCallback != null) {
      m_userPairCallback.addOverlappingPair(handle0, handle1);
     }
    }
    // update edge reference in other handle
    pHandleNext.m_minEdges[axis]--;
    if (pHandleNext.m_minEdges[axis] < 0) {
     assert (pHandleNext.m_minEdges[axis] >= 0);
    }
   } else {
    pHandleNext.m_maxEdges[axis]--;
    assert (pHandleNext.m_maxEdges[axis] >= 0);
   }
   pHandleEdge.m_maxEdges[axis]++;
   // swap the edges
   m_pEdges[axis][i_pEdge] = pNext;
   m_pEdges[axis][i_pNext] = pEdge;
   // increment
   i_pEdge++;
   i_pNext++;
   pEdge = m_pEdges[axis][i_pEdge];
   pNext = m_pEdges[axis][i_pNext];
  }
  if (DEBUG_BLOCKS) {
   verify_edges();
  }
 }

 public btAxisSweep3(final btVector3 worldAabbMin, final btVector3 worldAabbMax) {
  this(worldAabbMin, worldAabbMax, 0xfffe, 0x7fff);
 }

 public btAxisSweep3(final btVector3 worldAabbMin, final btVector3 worldAabbMax,
  int handleMask, int handleSentinel) {
  this(worldAabbMin, worldAabbMax, handleMask, handleSentinel, (short) 16384);
 }

 public btAxisSweep3(final btVector3 worldAabbMin, final btVector3 worldAabbMax,
  int handleMask, int handleSentinel, int maxHandles) {
  this(worldAabbMin, worldAabbMax, handleMask, handleSentinel, maxHandles, null);
 }

 public btAxisSweep3(final btVector3 worldAabbMin, final btVector3 worldAabbMax,
  int handleMask, int handleSentinel, int maxHandles, btOverlappingPairCache pairCache) {
  this(worldAabbMin, worldAabbMax, handleMask, handleSentinel, maxHandles, pairCache, false);
 }

 public btAxisSweep3(final btVector3 worldAabbMin, final btVector3 worldAabbMax,
  int handleMask, int handleSentinel, int userMaxHandles, btOverlappingPairCache pairCache,
  boolean disableRaycastAccelerator) {
  m_bpHandleMask = (short) handleMask;
  m_handleSentinel = (short) handleSentinel;
  m_pairCache = pairCache;
  m_userPairCallback = null;
  m_ownsPairCache = false;
  m_invalidPair = 0;
  m_raycastAccelerator = null;
  short maxHandles = (short) (userMaxHandles + 1);//need to add one sentinel handle
  if (null == m_pairCache) {
   m_pairCache = new btSortedOverlappingPairCache();
   //m_pairCache = new btHashedOverlappingPairCache();
   m_ownsPairCache = true;
  }
  if (!disableRaycastAccelerator) {
   m_nullPairCache = new btNullPairCache();
   m_raycastAccelerator = new btDbvtBroadphase(m_nullPairCache);//m_pairCache);
   m_raycastAccelerator.m_deferedcollide = true;//don't add/remove pairs
  }
  //btAssert(bounds.HasVolume());
  // init bounds
  m_worldAabbMin.set(worldAabbMin);
  m_worldAabbMax.set(worldAabbMax);
  final btVector3 aabbSize = new btVector3(m_worldAabbMax).sub(m_worldAabbMin);
  int maxInt = m_handleSentinel;
  m_quantize.set(maxInt, maxInt, maxInt).div(aabbSize);
  // allocate handles buffer, using btAlignedAlloc, and put all handles on free list
  m_pHandles = new Handle[maxHandles * 2];
  for (int i = 0; i < m_pHandles.length; ++i) {
   m_pHandles[i] = new Handle();
  }
  m_maxHandles = maxHandles;
  m_numHandles = 0;
  // handle 0 is reserved as the null index, and is also used as the sentinel
  m_firstFreeHandle = 1;
  {
   for (short i = m_firstFreeHandle; i < maxHandles; i++) {
    m_pHandles[i].SetNextFree((short) (i + 1));
   }
   m_pHandles[maxHandles - 1].SetNextFree((short) 0);
  }
  {
   // allocate edge buffers
   for (short i = 0; i < 3; i++) {
    m_pEdges[i] = new Edge[maxHandles * 2];
    for (int j = 0; j < m_pEdges[i].length; ++j) {
     m_pEdges[i][j] = new Edge();
    }
   }
  }
  //removed overlap management
  // make boundary sentinels
  m_pHandles[0].m_clientObject = null;
  for (short axis = 0; axis < 3; axis++) {
   m_pHandles[0].m_minEdges[axis] = 0;
   m_pHandles[0].m_maxEdges[axis] = 1;
   m_pEdges[axis][0].m_pos = 0;
   m_pEdges[axis][0].m_handle = 0;
   m_pEdges[axis][1].m_pos = m_handleSentinel;
   m_pEdges[axis][1].m_handle = 0;
  }
 }

 public short getNumHandles() {
  return m_numHandles;
 }

 public void calculateOverlappingPairs(btDispatcher dispatcher) {
  if (m_pairCache.hasDeferredRemoval()) {
   ArrayList<btBroadphasePair> overlappingPairArray = m_pairCache.getOverlappingPairArrayPtr();
   //perform a sort, to find duplicates and to sort 'invalid' pairs to the end
   overlappingPairArray.sort(new btBroadphasePairSortPredicate());
   if (m_invalidPair > 0) {
    remove_invalid_pairs(overlappingPairArray, m_invalidPair);
    m_invalidPair = 0;
   }
   int i;
   btBroadphasePair previousPair = new btBroadphasePair();
   previousPair.m_pProxy0 = null;
   previousPair.m_pProxy1 = null;
   previousPair.m_algorithm = null;
   for (i = 0; i < overlappingPairArray.size(); i++) {
    btBroadphasePair pair = overlappingPairArray.get(i);
    boolean isDuplicate = (pair.equals(previousPair));
    previousPair = pair;
    boolean needsRemoval;
    if (!isDuplicate) {
     ///important to use an AABB test that is consistent with the broadphase
     boolean hasOverlap = testAabbOverlap(pair.m_pProxy0, pair.m_pProxy1);
     needsRemoval = !hasOverlap; //callback.processOverlap(pair);
    } else {
     //remove duplicate
     needsRemoval = true;
     //should have no algorithm
     assert (null == pair.m_algorithm);
    }
    if (needsRemoval) {
     m_pairCache.cleanOverlappingPair(pair, dispatcher);
     pair.m_pProxy0 = null;
     pair.m_pProxy1 = null;
     m_invalidPair++;
    }
   }
   //perform a sort, to sort 'invalid' pairs to the end
   if (m_invalidPair > 0) {
    //perform a sort, to find duplicates and to sort 'invalid' pairs to the end
    overlappingPairArray.sort(new btBroadphasePairSortPredicate());
    remove_invalid_pairs(overlappingPairArray, m_invalidPair);
    m_invalidPair = 0;
   }
  } else {
   m_pairCache.incrementalCleanup(m_pairCache.getNumOverlappingPairs(), dispatcher);
  }
 }

 private void remove_invalid_pairs(ArrayList<btBroadphasePair> overlappingPairArray, int invalidPair) {
  int new_size = (overlappingPairArray.size() - invalidPair);
  while (overlappingPairArray.size() > new_size) {
   assert (overlappingPairArray.get(overlappingPairArray.size() - 1).m_pProxy0 == null);
   assert (overlappingPairArray.get(overlappingPairArray.size() - 1).m_pProxy1 == null);
   overlappingPairArray.remove(overlappingPairArray.size() - 1);
  }
  if (DEBUG_BLOCKS) {
   Set<btBroadphasePair> pair_set = new HashSet<>();
   for (btBroadphasePair broadphasePair : overlappingPairArray) {
    assert (!pair_set.contains(broadphasePair));
    pair_set.add(broadphasePair);
   }
  }
 }

 protected short addHandle(final btVector3 aabbMin, final btVector3 aabbMax, Object pOwner,
  int collisionFilterGroup, int collisionFilterMask, btDispatcher dispatcher) {
  // quantize the bounds
  short[] min = new short[3];
  short[] max = new short[3];
  quantize(min, aabbMin, (short) 0);
  quantize(max, aabbMax, (short) 1);
  // allocate a handle
  short handle = allocHandle();
  Handle pHandle = getHandle(handle);
  pHandle.m_uniqueId = (handle);
  //pHandle.m_pOverlaps = 0;
  pHandle.m_clientObject = pOwner;
  pHandle.m_collisionFilterGroup = collisionFilterGroup;
  pHandle.m_collisionFilterMask = collisionFilterMask;
  // compute current limit of edge arrays
  short limit = (short) (m_numHandles * 2);
  // insert new edges just inside the max boundary edge
  for (short axis = 0; axis < 3; axis++) {
   assert (min[axis] <= max[axis]);
   m_pHandles[0].m_maxEdges[axis] += 2;
   Edge swapper = m_pEdges[axis][limit + 1];
   m_pEdges[axis][limit + 1] = m_pEdges[axis][limit - 1];
   m_pEdges[axis][limit - 1] = swapper;
   m_pEdges[axis][limit - 1].m_pos = min[axis];
   assert (m_pEdges[axis][limit - 1].m_pos >= 0);
   m_pEdges[axis][limit - 1].m_handle = handle;
   m_pEdges[axis][limit].m_pos = max[axis];
   assert (m_pEdges[axis][limit].m_pos >= 0);
   m_pEdges[axis][limit].m_handle = handle;
   pHandle.m_minEdges[axis] = (short) (limit - 1);
   assert (pHandle.m_minEdges[axis] >= 0);
   pHandle.m_maxEdges[axis] = limit;
  }
  if (DEBUG_BLOCKS) {
   verify_edges();
  }
  // now sort the new edges to their correct position
  sortMinDown((short) 0, pHandle.m_minEdges[0], dispatcher, false);
  sortMaxDown((short) 0, pHandle.m_maxEdges[0], dispatcher, false);
  sortMinDown((short) 1, pHandle.m_minEdges[1], dispatcher, false);
  sortMaxDown((short) 1, pHandle.m_maxEdges[1], dispatcher, false);
  sortMinDown((short) 2, pHandle.m_minEdges[2], dispatcher, true);
  sortMaxDown((short) 2, pHandle.m_maxEdges[2], dispatcher, true);
  if (DEBUG_BLOCKS) {
   verify_edges();
   verify_edges_sorted();
  }
  return handle;
 }

 public void removeHandle(short handle, btDispatcher dispatcher) {
  Handle pHandle = getHandle(handle);
  //explicitly remove the pairs containing the proxy
  //we could do it also in the sortMinUp (passing true)
  ///@todo: compare performance
  if (!m_pairCache.hasDeferredRemoval()) {
   m_pairCache.removeOverlappingPairsContainingProxy(pHandle, dispatcher);
  }
  // compute current limit of edge arrays
  short limit = (short) (m_numHandles * 2);
  short axis;
  for (axis = 0; axis < 3; axis++) {
   m_pHandles[0].m_maxEdges[axis] -= 2;
  }
  // remove the edges by sorting them up to the end of the list
  for (axis = 0; axis < 3; axis++) {
   Edge[] pEdges = m_pEdges[axis];
   short max = pHandle.m_maxEdges[axis];
   pEdges[max].m_pos = m_handleSentinel;
   sortMaxUp(axis, max, dispatcher, false);
   short i = pHandle.m_minEdges[axis];
   pEdges[i].m_pos = m_handleSentinel;
   sortMinUp(axis, i, dispatcher, false);
   pEdges[limit - 1].m_handle = 0;
   pEdges[limit - 1].m_pos = m_handleSentinel;
  }
  // free the handle
  freeHandle(handle);
  if (DEBUG_BLOCKS) {
   verify_edges();
  }
 }

 public void updateHandle(int handle, final btVector3 aabbMin, final btVector3 aabbMax,
  btDispatcher dispatcher) {
//	btAssert(bounds.IsFinite());
  //btAssert(bounds.HasVolume());
  Handle pHandle = getHandle(handle);
  // quantize the new bounds
  short[] min = new short[3];
  short[] max = new short[3];
  quantize(min, aabbMin, (short) 0);
  quantize(max, aabbMax, (short) 1);
  // update changed edges
  for (short axis = 0; axis < 3; axis++) {
   short emin = pHandle.m_minEdges[axis];
   short emax = pHandle.m_maxEdges[axis];
   if (DEBUG_BLOCKS) {
    assert (axis >= 0);
    assert (emin >= 0);
    assert (emax >= 0);
   }
   int dmin = (min[axis] - m_pEdges[axis][emin].m_pos);
   int dmax = (max[axis] - m_pEdges[axis][emax].m_pos);
   m_pEdges[axis][emin].m_pos = min[axis];
   assert (m_pEdges[axis][emin].m_pos >= 0);
   m_pEdges[axis][emax].m_pos = max[axis];
   assert (m_pEdges[axis][emax].m_pos >= 0);
   // expand (only adds overlaps)
   if (dmin < 0) {
    sortMinDown(axis, emin, dispatcher, true);
   }
   if (dmax > 0) {
    sortMaxUp(axis, emax, dispatcher, true);
   }
   // shrink (only removes overlaps)
   if (dmin > 0) {
    sortMinUp(axis, emin, dispatcher, true);
   }
   if (dmax < 0) {
    sortMaxDown(axis, emax, dispatcher, true);
   }
  }
  if (DEBUG_BLOCKS) {
   verify_edges();
   verify_edges_sorted();
  }
 }

 public Handle getHandle(int index) {
  return m_pHandles[index];
 }

 public void resetPool(btDispatcher dispatcher) {
  if (m_numHandles == 0) {
   m_firstFreeHandle = 1;
   {
    for (short i = m_firstFreeHandle; i < m_maxHandles; i++) {
     m_pHandles[i].SetNextFree((short) (i + 1));
    }
    m_pHandles[m_maxHandles - 1].SetNextFree((short) 0);
   }
  }
 }

 public void processAllOverlappingPairs(btOverlapCallback callback) {
  assert (false);
  // can't find c++ source code
 }

 //Broadphase Interface
 @Override
 public btBroadphaseProxy createProxy(final btVector3 aabbMin, final btVector3 aabbMax,
  int shapeType, Object userPtr, int collisionFilterGroup, int collisionFilterMask,
  btDispatcher dispatcher) {
  short handleId = addHandle(aabbMin, aabbMax, userPtr, collisionFilterGroup, collisionFilterMask,
   dispatcher);
  Handle handle = getHandle(handleId);
  if (m_raycastAccelerator != null) {
   btBroadphaseProxy rayProxy = m_raycastAccelerator.createProxy(aabbMin, aabbMax, shapeType,
    userPtr, collisionFilterGroup, collisionFilterMask, dispatcher);
   handle.m_dbvtProxy = rayProxy;
  }
  return handle;
 }

 @Override
 public void destroyProxy(btBroadphaseProxy proxy, btDispatcher dispatcher) {
  Handle handle = (Handle) (proxy);
  if (m_raycastAccelerator != null) {
   m_raycastAccelerator.destroyProxy(handle.m_dbvtProxy, dispatcher);
  }
  removeHandle((short) handle.m_uniqueId, dispatcher);
 }

 public void setAabb(btBroadphaseProxy proxy, final btVector3 aabbMin, final btVector3 aabbMax,
  btDispatcher dispatcher) {
  Handle handle = (Handle) (proxy);
  handle.m_aabbMin.set(aabbMin);
  handle.m_aabbMax.set(aabbMax);
  updateHandle(handle.m_uniqueId, aabbMin, aabbMax, dispatcher);
  if (m_raycastAccelerator != null) {
   m_raycastAccelerator.setAabb(handle.m_dbvtProxy, aabbMin, aabbMax, dispatcher);
  }
 }

 @Override
 public void getAabb(btBroadphaseProxy proxy, final btVector3 aabbMin, final btVector3 aabbMax) {
  Handle pHandle = (Handle) (proxy);
  aabbMin.set(pHandle.m_aabbMin);
  aabbMax.set(pHandle.m_aabbMax);
 }

 @Override
 public void rayTest(final btVector3 rayFrom, final btVector3 rayTo,
  btBroadphaseRayCallback rayCallback) {
  rayTest(rayFrom, rayTo, rayCallback, new btVector3(), new btVector3());
 }

 public void rayTest(final btVector3 rayFrom, final btVector3 rayTo,
  btBroadphaseRayCallback rayCallback, final btVector3 aabbMin, final btVector3 aabbMax) {
  if (m_raycastAccelerator != null) {
   m_raycastAccelerator.rayTest(rayFrom, rayTo, rayCallback, aabbMin, aabbMax);
  } else {
   //choose axis?
   short axis = 0;
   //for each proxy
   for (short i = 1; i < m_numHandles * 2 + 1; i++) {
    if (m_pEdges[axis][i].IsMax() != 0) {
     rayCallback.process(getHandle(m_pEdges[axis][i].m_handle));
    }
   }
  }
 }

 public void aabbTest(final btVector3 aabbMin, final btVector3 aabbMax,
  btBroadphaseAabbCallback callback) {
  if (m_raycastAccelerator != null) {
   m_raycastAccelerator.aabbTest(aabbMin, aabbMax, callback);
  } else {
   //choose axis?
   short axis = 0;
   //for each proxy
   for (short i = 1; i < m_numHandles * 2 + 1; i++) {
    if (m_pEdges[axis][i].IsMax() != 0) {
     Handle handle = getHandle(m_pEdges[axis][i].m_handle);
     if (TestAabbAgainstAabb2(aabbMin, aabbMax, handle.m_aabbMin, handle.m_aabbMax)) {
      callback.process(handle);
     }
    }
   }
  }
 }

 public void quantize(short[] out, final btVector3 point, short isMax) {
  final btVector3 v = (new btVector3(point).sub(m_worldAabbMin)).mul(m_quantize);
  out[0] = (short) ((v.x <= 0) ? isMax : (v.x >= m_handleSentinel) ? ((m_handleSentinel &
   m_bpHandleMask) | isMax) : (((short) v.x & m_bpHandleMask) | isMax));
  out[1] = (short) ((v.y <= 0) ? isMax : (v.y >= m_handleSentinel) ? ((m_handleSentinel &
   m_bpHandleMask) | isMax) : (((short) v.y & m_bpHandleMask) | isMax));
  out[2] = (short) ((v.z <= 0) ? isMax : (v.z >= m_handleSentinel) ? ((m_handleSentinel &
   m_bpHandleMask) | isMax) : (((short) v.z & m_bpHandleMask) | isMax));
 }
 ///unQuantize should be conservative: aabbMin/aabbMax should be larger then 'getAabb' result

 public void unQuantize(btBroadphaseProxy proxy, final btVector3 aabbMin, final btVector3 aabbMax) {
  Handle pHandle = (Handle) (proxy);
  short[] vecInMin = new short[3];
  short[] vecInMax = new short[3];
  vecInMin[0] = m_pEdges[0][pHandle.m_minEdges[0]].m_pos;
  vecInMax[0] = (short) (m_pEdges[0][pHandle.m_maxEdges[0]].m_pos + 1);
  vecInMin[1] = m_pEdges[1][pHandle.m_minEdges[1]].m_pos;
  vecInMax[1] = (short) (m_pEdges[1][pHandle.m_maxEdges[1]].m_pos + 1);
  vecInMin[2] = m_pEdges[2][pHandle.m_minEdges[2]].m_pos;
  vecInMax[2] = (short) (m_pEdges[2][pHandle.m_maxEdges[2]].m_pos + 1);
  aabbMin.set((float) (vecInMin[0]) / (m_quantize.getX()), (float) (vecInMin[1]) / (m_quantize
   .getY()), (float) (vecInMin[2]) / (m_quantize.getZ()));
  aabbMin.add(m_worldAabbMin);
  aabbMax.set((float) (vecInMax[0]) / (m_quantize.getX()), (float) (vecInMax[1]) / (m_quantize
   .getY()), (float) (vecInMax[2]) / (m_quantize.getZ()));
  aabbMax.add(m_worldAabbMin);
 }

 public boolean testAabbOverlap(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1) {
  final Handle pHandleA = (Handle) (proxy0);
  final Handle pHandleB = (Handle) (proxy1);
  //optimization 1: check the array index (memory address), instead of the m_pos
  for (short axis = 0; axis < 3; axis++) {
   if (pHandleA.m_maxEdges[axis] < pHandleB.m_minEdges[axis] ||
    pHandleB.m_maxEdges[axis] < pHandleA.m_minEdges[axis]) {
    assert (!TestAabbAgainstAabb2(proxy0.m_aabbMin, proxy0.m_aabbMax, proxy1.m_aabbMin,
     proxy1.m_aabbMax));
    return false;
   }
  }
  if (DEBUG_BLOCKS) {
   for (short axis = 0; axis < 3; axis++) {
    if (pHandleB.m_maxEdges[axis] < pHandleA.m_minEdges[axis] ||
     pHandleA.m_maxEdges[axis] < pHandleB.m_minEdges[axis]) {
     assert (false);
    }
   }
  }
  return true;
 }

 public btOverlappingPairCache getOverlappingPairCache() {
  return m_pairCache;
 }

 public void setOverlappingPairUserCallback(btOverlappingPairCallback pairCallback) {
  m_userPairCallback = pairCallback;
 }

 public btOverlappingPairCallback getOverlappingPairUserCallback() {
  return m_userPairCallback;
 }

 ///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
 ///will add some transform later
 @Override
 public void getBroadphaseAabb(final btVector3 aabbMin, final btVector3 aabbMax) {
  aabbMin.set(m_worldAabbMin);
  aabbMax.set(m_worldAabbMax);
 }

 public void printStats() {
  /*		printf("btAxisSweep3.h\n");
		printf("numHandles = %d, maxHandles = %d\n",m_numHandles,m_maxHandles);
		printf("aabbMin=%f,%f,%f,aabbMax=%f,%f,%f\n",m_worldAabbMin.getX(),m_worldAabbMin.getY(),m_worldAabbMin.getZ(),
			m_worldAabbMax.getX(),m_worldAabbMax.getY(),m_worldAabbMax.getZ());
   */
 }
};
