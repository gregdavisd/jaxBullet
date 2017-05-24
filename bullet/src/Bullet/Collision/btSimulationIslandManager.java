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

import Bullet.Collision.Broadphase.btBroadphasePair;
import Bullet.Collision.Broadphase.btDispatcher;
import Bullet.Collision.Broadphase.btOverlappingPairCache;
import static Bullet.Collision.btCollisionObject.ACTIVE_TAG;
import static Bullet.Collision.btCollisionObject.DISABLE_DEACTIVATION;
import static Bullet.Collision.btCollisionObject.ISLAND_SLEEPING;
import static Bullet.Collision.btCollisionObject.WANTS_DEACTIVATION;
import static Bullet.Collision.btPersistentManifold.getIslandId;
import static Bullet.LinearMath.btQuickprof.BT_PROFILE;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * SimulationIslandManager creates and handles simulation islands, using btUnionFind
 *
 * @author Gregery Barton
 */
public class btSimulationIslandManager  implements Serializable {

 btUnionFind m_unionFind = new btUnionFind();
 final ArrayList<btPersistentManifold> m_islandmanifold = new ArrayList<>(0);
 final ArrayList<btCollisionObject> m_islandBodies = new ArrayList<>(0);
 boolean m_splitIslands;

 public btSimulationIslandManager() {
  m_splitIslands = true;
 }

 public void initUnionFind(int n) {
  m_unionFind.reset(n);
 }

 public btUnionFind getUnionFind() {
  return m_unionFind;
 }

 public void updateActivationState(btCollisionWorld colWorld, btDispatcher dispatcher) {
  // put the index into m_controllers into m_tag   
  int index = 0;
  {
   int i;
   for (i = 0; i < colWorld.getCollisionObjectArray().size(); i++) {
    btCollisionObject collisionObject = colWorld.getCollisionObjectArray().get(i);
    //Adding filtering here
    if (!collisionObject.isStaticOrKinematicObject()) {
     collisionObject.setIslandTag(index++);
    }
    collisionObject.setCompanion(null);
    collisionObject.setHitFraction((1.f));
   }
  }
  // do the union find
  initUnionFind(index);
  findUnions(dispatcher, colWorld);
 }

 public void storeIslandActivationState(btCollisionWorld colWorld) {
  // put the islandId ('find' value) into m_tag   
  {
   int index = 0;
   int i;
   for (i = 0; i < colWorld.getCollisionObjectArray().size(); i++) {
    btCollisionObject collisionObject = colWorld.getCollisionObjectArray().get(i);
    if (!collisionObject.isStaticOrKinematicObject()) {
     collisionObject.setIslandTag(m_unionFind.find(index));
     //Set the correct object offset in Collision Object Array
     m_unionFind.getElement(index).m_sz = i;
     collisionObject.setCompanion(null);
     index++;
    } else {
     collisionObject.setIslandTag(-1);
     collisionObject.setCompanion(null);
    }
   }
  }
 }

 void findUnions(btDispatcher dispatcher, btCollisionWorld colWorld) {
  {
   btOverlappingPairCache pairCachePtr = colWorld.getPairCache();
   int numOverlappingPairs = pairCachePtr.getNumOverlappingPairs();
   if (numOverlappingPairs != 0) {
    Collection<btBroadphasePair> pairPtr = pairCachePtr.getOverlappingPairArray ();
    for (btBroadphasePair collisionPair : pairPtr) {
     btCollisionObject colObj0 = (btCollisionObject) collisionPair.m_pProxy0.m_clientObject;
     btCollisionObject colObj1 = (btCollisionObject) collisionPair.m_pProxy1.m_clientObject;
     if (((colObj0 != null) && ((colObj0).mergesSimulationIslands())) && ((colObj1 != null) &&
      ((colObj1).mergesSimulationIslands()))) {
      m_unionFind.unite((colObj0).getIslandTag(),
       (colObj1).getIslandTag());
     }
    }
   }
  }
 }

 public static abstract class IslandCallback {

  public abstract void processIsland(List<btCollisionObject> bodies, int numBodies,
   List<btPersistentManifold> manifolds, int numManifolds, int islandId);
 };

 public void buildAndProcessIslands(btDispatcher dispatcher, btCollisionWorld collisionWorld,
  IslandCallback callback) {
  ArrayList<btCollisionObject> collisionObjects = collisionWorld.getCollisionObjectArray();
  buildIslands(dispatcher, collisionWorld);
  int endIslandIndex = 1;
  int startIslandIndex;
  int numElem = getUnionFind().getNumElements();
  BT_PROFILE("processIslands");
  if (!m_splitIslands) {
   ArrayList<btPersistentManifold> manifold = dispatcher.getInternalManifoldPointer();
   int maxNumManifolds = dispatcher.getNumManifolds();
   callback.processIsland(collisionObjects, collisionObjects.size(), manifold, maxNumManifolds, -1);
  } else {
   // Sort manifolds, based on islands
   // Sort the vector using predicate and std.sort
   //std.sort(islandmanifold.begin(), islandmanifold.end(), btPersistentManifoldSortPredicate);
   int numManifolds = (m_islandmanifold.size());
   //tried a radix sort, but quicksort/heapsort seems still faster
   //@todo rewrite island management
   m_islandmanifold.sort(new btPersistentManifoldSortPredicate());
   //m_islandmanifold.heapSort(btPersistentManifoldSortPredicate());
   //now process all active islands (sets of manifolds for now)
   int startManifoldIndex = 0;
   int endManifoldIndex = 1;
   //int islandId;
   //	printf("Start Islands\n");
   //traverse the simulation islands, and call the solver, unless all objects are sleeping/deactivated
   for (startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex) {
    int islandId = getUnionFind().getElement(startIslandIndex).m_id;
    boolean islandSleeping = true;
    for (endIslandIndex = startIslandIndex; (endIslandIndex < numElem) && (getUnionFind()
     .getElement(endIslandIndex).m_id == islandId); endIslandIndex++) {
     int i = getUnionFind().getElement(endIslandIndex).m_sz;
     btCollisionObject colObj0 = collisionObjects.get(i);
     m_islandBodies.add(colObj0);
     if (colObj0.isActive()) {
      islandSleeping = false;
     }
    }
    //find the accompanying contact manifold for this islandId
    int numIslandManifolds = 0;
    int startManifold = 0;
    if (startManifoldIndex < numManifolds) {
     int curIslandId = getIslandId(m_islandmanifold.get(startManifoldIndex));
     if (curIslandId == islandId) {
      startManifold = startManifoldIndex;
      for (endManifoldIndex = startManifoldIndex + 1; (endManifoldIndex < numManifolds) &&
       (islandId ==
       getIslandId(m_islandmanifold.get(endManifoldIndex))); endManifoldIndex++) {
      }
      /// Process the actual simulation, only if not sleeping/deactivated
      numIslandManifolds = endManifoldIndex - startManifoldIndex;
     }
    }
    if (!islandSleeping) {
     callback.processIsland(m_islandBodies, m_islandBodies.size(), m_islandmanifold.subList(
      startManifold,
      startManifold + numIslandManifolds), numIslandManifolds, islandId);
    }
    if (numIslandManifolds != 0) {
     startManifoldIndex = endManifoldIndex;
    }
    m_islandBodies.clear();
   }
  }
 }

 void buildIslands(btDispatcher dispatcher, btCollisionWorld collisionWorld) {
  BT_PROFILE("islandUnionFindAndQuickSort");
  ArrayList<btCollisionObject> collisionObjects = collisionWorld.getCollisionObjectArray();
  m_islandmanifold.clear();
  //we are going to sort the unionfind array, and store the element id in the size
  //afterwards, we clean unionfind, to make sure no-one uses it anymore
  getUnionFind().sortIslands();
  int numElem = getUnionFind().getNumElements();
  int endIslandIndex = 1;
  int startIslandIndex;
  //update the sleeping state for bodies, if all are sleeping
  for (startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex) {
   int islandId = getUnionFind().getElement(startIslandIndex).m_id;
   for (endIslandIndex = startIslandIndex + 1; (endIslandIndex < numElem) && (getUnionFind()
    .getElement(endIslandIndex).m_id == islandId); endIslandIndex++) {
   }
   //int numSleeping = 0;
   boolean allSleeping = true;
   for (int idx = startIslandIndex; idx < endIslandIndex; idx++) {
    int i = getUnionFind().getElement(idx).m_sz;
    btCollisionObject colObj0 = collisionObjects.get(i);
    if ((colObj0.getIslandTag() != islandId) && (colObj0.getIslandTag() != -1)) {
//				printf("error in island management\n");
    }
    assert((colObj0.getIslandTag() == islandId) || (colObj0.getIslandTag() == -1));
    if (colObj0.getIslandTag() == islandId) {
     if (colObj0.getActivationState() == ACTIVE_TAG) {
      allSleeping = false;
     }
     if (colObj0.getActivationState() == DISABLE_DEACTIVATION) {
      allSleeping = false;
     }
    }
   }
   if (allSleeping) {
    for (int idx = startIslandIndex; idx < endIslandIndex; idx++) {
     int i = getUnionFind().getElement(idx).m_sz;
     btCollisionObject colObj0 = collisionObjects.get(i);
     if ((colObj0.getIslandTag() != islandId) && (colObj0.getIslandTag() != -1)) {
//					printf("error in island management\n");
     }
     assert((colObj0.getIslandTag() == islandId) || (colObj0.getIslandTag() == -1));
     if (colObj0.getIslandTag() == islandId) {
      colObj0.setActivationState(ISLAND_SLEEPING);
     }
    }
   } else {
    int idx;
    for (idx = startIslandIndex; idx < endIslandIndex; idx++) {
     int i = getUnionFind().getElement(idx).m_sz;
     btCollisionObject colObj0 = collisionObjects.get(i);
     if ((colObj0.getIslandTag() != islandId) && (colObj0.getIslandTag() != -1)) {
//					printf("error in island management\n");
     }
     assert((colObj0.getIslandTag() == islandId) || (colObj0.getIslandTag() == -1));
     if (colObj0.getIslandTag() == islandId) {
      if (colObj0.getActivationState() == ISLAND_SLEEPING) {
       colObj0.setActivationState(WANTS_DEACTIVATION);
       colObj0.setDeactivationTime(0.f);
      }
     }
    }
   }
  }
  int i;
  int maxNumManifolds = dispatcher.getNumManifolds();
//#define SPLIT_ISLANDS 1
//#ifdef SPLIT_ISLANDS
//#endif //SPLIT_ISLANDS
  for (i = 0; i < maxNumManifolds; i++) {
   btPersistentManifold manifold = dispatcher.getManifoldByIndexInternal(i);
   btCollisionObject colObj0 = (manifold.getBody0());
   btCollisionObject colObj1 = (manifold.getBody1());
   ///@todo: check sleeping conditions!
   if ((colObj0.getActivationState() != ISLAND_SLEEPING) || (colObj1.getActivationState() !=
    ISLAND_SLEEPING)) {
    //kinematic objects don't merge islands, but wake up all connected objects
    if (colObj0.isKinematicObject() && colObj0.getActivationState() != ISLAND_SLEEPING) {
     if (colObj0.hasContactResponse()) {
      colObj1.activate();
     }
    }
    if (colObj1.isKinematicObject() && colObj1.getActivationState() != ISLAND_SLEEPING) {
     if (colObj1.hasContactResponse()) {
      colObj0.activate();
     }
    }
    if (m_splitIslands) {
     //filtering for response
     if (dispatcher.needsResponse(colObj0, colObj1)) {
      m_islandmanifold.add(manifold);
     }
    }
   }
  }
 }

 boolean getSplitIslands() {
  return m_splitIslands;
 }

 void setSplitIslands(boolean doSplitIslands) {
  m_splitIslands = doSplitIslands;
 }
};
