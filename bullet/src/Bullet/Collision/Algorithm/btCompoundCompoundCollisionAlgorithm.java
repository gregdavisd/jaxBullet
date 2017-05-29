/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

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

import Bullet.Collision.Broadphase.btDbvt;
import Bullet.Collision.Broadphase.btDbvtAabbMm;
import static Bullet.Collision.Broadphase.btDbvtAabbMm.intersect;
import Bullet.Collision.Broadphase.btDbvtNode;
import Bullet.Collision.Broadphase.btDispatcherInfo;
import Bullet.Collision.Broadphase.btHashedSimplePairCache;
import Bullet.Collision.Shape.btCollisionShape;
import Bullet.Collision.Shape.btCompoundShape;
import Bullet.Collision.btCollisionObject;
import Bullet.Collision.btCollisionObjectWrapper;
import Bullet.Collision.btCompoundCompoundLeafCallback;
import Bullet.Collision.btManifoldResult;
import Bullet.Collision.btPersistentManifold;
import Bullet.Collision.btSimplePair;
import static Bullet.LinearMath.btAabbUtil2.TestAabbAgainstAabb2;
import static Bullet.LinearMath.btAabbUtil2.btTransformAabb;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collection;

/**
 *
 * @author Gregery Barton
 */
public class btCompoundCompoundCollisionAlgorithm extends btCompoundCollisionAlgorithm implements
 Serializable {

 final btHashedSimplePairCache m_childCollisionAlgorithmCache;
 final ArrayList<btSimplePair> m_removePairs = new ArrayList<>(0);
 int m_compoundShapeRevision0;//to keep track of changes, so that childAlgorithm array can be updated
 int m_compoundShapeRevision1;

 @Override
 void removeChildAlgorithms() {
  Collection<btSimplePair> pairs = m_childCollisionAlgorithmCache.getOverlappingPairArray();
  int numChildren = pairs.size();
  int i;
  for (btSimplePair pair : pairs) {
   if (pair.m_userPointer != null) {
    btCollisionAlgorithm algo = (btCollisionAlgorithm) pair.m_userPointer;
    algo.destroy();
   }
  }
  m_childCollisionAlgorithmCache.removeAllPairs();
 }

//	void	preallocateChildAlgorithms(  btCollisionObjectWrapper* body0Wrap,  btCollisionObjectWrapper* body1Wrap);
 btCompoundCompoundCollisionAlgorithm(btCollisionAlgorithmConstructionInfo ci,
  btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap, boolean isSwapped) {
  super(ci, body0Wrap, body1Wrap, isSwapped);
  m_childCollisionAlgorithmCache = new btHashedSimplePairCache();
  btCollisionObjectWrapper col0ObjWrap = body0Wrap;
  assert (col0ObjWrap.getCollisionShape().isCompound());
  btCollisionObjectWrapper col1ObjWrap = body1Wrap;
  assert (col1ObjWrap.getCollisionShape().isCompound());
  btCompoundShape compoundShape0 = (btCompoundShape) (col0ObjWrap.getCollisionShape());
  m_compoundShapeRevision0 = compoundShape0.getUpdateRevision();
  btCompoundShape compoundShape1 = (btCompoundShape) (col1ObjWrap.getCollisionShape());
  m_compoundShapeRevision1 = compoundShape1.getUpdateRevision();
 }

 @Override
 public void destroy() {
  removeChildAlgorithms();
 }

 @Override
 public void processCollision(btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap,
  btDispatcherInfo dispatchInfo, btManifoldResult resultOut) {
  btCollisionObjectWrapper col0ObjWrap = body0Wrap;
  btCollisionObjectWrapper col1ObjWrap = body1Wrap;
  assert (col0ObjWrap.getCollisionShape().isCompound());
  assert (col1ObjWrap.getCollisionShape().isCompound());
  btCompoundShape compoundShape0 = (btCompoundShape) (col0ObjWrap.getCollisionShape());
  btCompoundShape compoundShape1 = (btCompoundShape) (col1ObjWrap.getCollisionShape());
  btDbvt tree0 = compoundShape0.getDynamicAabbTree();
  btDbvt tree1 = compoundShape1.getDynamicAabbTree();
  if (tree0 == null || tree1 == null) {
   super.processCollision(body0Wrap, body1Wrap, dispatchInfo, resultOut);
   return;
  }
  ///btCompoundShape might have changed:
  ////make sure the internal child collision algorithm caches are still valid
  if ((compoundShape0.getUpdateRevision() != m_compoundShapeRevision0) || (compoundShape1
   .getUpdateRevision() != m_compoundShapeRevision1)) {
   ///clear all
   removeChildAlgorithms();
   m_compoundShapeRevision0 = compoundShape0.getUpdateRevision();
   m_compoundShapeRevision1 = compoundShape1.getUpdateRevision();
  }
  ///we need to refresh all contact manifolds
  ///note that we should actually recursively traverse all children, btCompoundShape can nested more then 1 level deep
  ///so we should add a 'refreshManifolds' in the btCollisionAlgorithm
  {
   ArrayList<btPersistentManifold> manifold = new ArrayList<>(0);
   Collection<btSimplePair> pairs = m_childCollisionAlgorithmCache.getOverlappingPairArray();
   for (btSimplePair pair : pairs) {
    if (pair.m_userPointer != null) {
     btCollisionAlgorithm algo = (btCollisionAlgorithm) pair.m_userPointer;
     algo.getAllContactManifolds(manifold);
     for (int m = 0; m < manifold.size(); m++) {
      if (manifold.get(m).getNumContacts() > 0) {
       resultOut.setPersistentManifold(manifold.get(m));
       resultOut.refreshContactPoints();
       resultOut.setPersistentManifold(null);
      }
     }
     manifold.clear();
    }
   }
  }
  btCompoundCompoundLeafCallback callback = new btCompoundCompoundLeafCallback(col0ObjWrap,
   col1ObjWrap, this.m_dispatcher, dispatchInfo, resultOut, this.m_childCollisionAlgorithmCache,
   m_sharedManifold);
  final btTransform xform = col0ObjWrap.getWorldTransform().invert().mul(col1ObjWrap
   .getWorldTransform());
  MycollideTT(tree0.m_root, tree1.m_root, xform, callback, resultOut.m_closestPointDistanceThreshold);
  //printf("#compound-compound child/leaf overlap =%d                      \r",callback.m_numOverlapPairs);
  //remove non-overlapping child pairs
  {
   assert (m_removePairs.isEmpty());
   //iterate over all children, perform an AABB check inside ProcessChildShape
   Collection<btSimplePair> pairs = m_childCollisionAlgorithmCache.getOverlappingPairArray();
   //ArrayList<btPersistentManifold> manifoldArray = new ArrayList<>(0);
   final btVector3 aabbMin0 = new btVector3();
   final btVector3 aabbMax0 = new btVector3();
   final btVector3 aabbMin1 = new btVector3();
   final btVector3 aabbMax1 = new btVector3();
   for (btSimplePair pair : pairs) {
    if (pair.m_userPointer != null) {
     btCollisionAlgorithm algo = (btCollisionAlgorithm) pair.m_userPointer;
     {
      final btTransform orgTrans0;
      btCollisionShape childShape0;
      childShape0 = compoundShape0.getChildShape(pair.m_indexA);
      orgTrans0 = col0ObjWrap.getWorldTransform();
      final btTransform childTrans0 = compoundShape0.getChildTransform(pair.m_indexA);
      final btTransform newChildWorldTrans0 = orgTrans0.mul(childTrans0);
      childShape0.getAabb(newChildWorldTrans0, aabbMin0, aabbMax0);
     }
     final btVector3 thresholdVec = new btVector3(resultOut.m_closestPointDistanceThreshold,
      resultOut.m_closestPointDistanceThreshold, resultOut.m_closestPointDistanceThreshold);
     aabbMin0.sub(thresholdVec);
     aabbMax0.add(thresholdVec);
     {
      btCollisionShape childShape1;
      final btTransform orgTrans1;
      childShape1 = compoundShape1.getChildShape(pair.m_indexB);
      orgTrans1 = col1ObjWrap.getWorldTransform();
      final btTransform childTrans1 = compoundShape1.getChildTransform(pair.m_indexB);
      final btTransform newChildWorldTrans1 = orgTrans1.mul(childTrans1);
      childShape1.getAabb(newChildWorldTrans1, aabbMin1, aabbMax1);
     }
     aabbMin1.sub(thresholdVec);
     aabbMax1.add(thresholdVec);
     if (!TestAabbAgainstAabb2(aabbMin0, aabbMax0, aabbMin1, aabbMax1)) {
      algo.destroy();
      m_removePairs.add(new btSimplePair(pair.m_indexA, pair.m_indexB));
     }
    }
   }
   for (int i = 0; i < m_removePairs.size(); i++) {
    m_childCollisionAlgorithmCache.removeOverlappingPair(m_removePairs.get(i).m_indexA,
     m_removePairs.get(i).m_indexB);
   }
   m_removePairs.clear();
  }
 }

 @Override
 public float calculateTimeOfImpact(btCollisionObject body0, btCollisionObject body1,
  btDispatcherInfo dispatchInfo, btManifoldResult resultOut) {
  assert (false);
  return 0.f;
 }

 @Override
 public void getAllContactManifolds(ArrayList<btPersistentManifold> manifoldArray) {
  Collection<btSimplePair> pairs = m_childCollisionAlgorithmCache.getOverlappingPairArray();
  for (btSimplePair pair : pairs) {
   if (pair.m_userPointer != null) {
    ((btCollisionAlgorithm) pair.m_userPointer).getAllContactManifolds(manifoldArray);
   }
  }
 }

 public static class CreateFunc extends btCollisionAlgorithmCreateFunc {

  @Override
  public btCollisionAlgorithm CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo ci,
   btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap) {
   return new btCompoundCompoundCollisionAlgorithm(ci, body0Wrap, body1Wrap, false);
  }
 }

 public static class SwappedCreateFunc extends btCollisionAlgorithmCreateFunc {

  @Override
  public btCollisionAlgorithm CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo ci,
   btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap) {
   return new btCompoundCompoundCollisionAlgorithm(ci, body0Wrap, body1Wrap, true);
  }
 };

 static void MycollideTT(btDbvtNode root0,
  btDbvtNode root1, final btTransform xform,
  btCompoundCompoundLeafCallback callback, float distanceThreshold) {
  if (root0 != null && root1 != null) {
   ArrayDeque<btDbvt.sStkNN> stkStack = new ArrayDeque<>();
   stkStack.push(new btDbvt.sStkNN(root0, root1));
   do {
    btDbvt.sStkNN p = stkStack.pop();
    if (MyIntersect(p.a.volume(), p.b.volume(), xform, distanceThreshold)) {
     if (p.a.isinternal()) {
      if (p.b.isinternal()) {
       stkStack.push(new btDbvt.sStkNN(p.a.child0(), p.b.child0()));
       stkStack.push(new btDbvt.sStkNN(p.a.child1(), p.b.child0()));
       stkStack.push(new btDbvt.sStkNN(p.a.child0(), p.b.child1()));
       stkStack.push(new btDbvt.sStkNN(p.a.child1(), p.b.child1()));
      } else {
       stkStack.push(new btDbvt.sStkNN(p.a.child0(), p.b));
       stkStack.push(new btDbvt.sStkNN(p.a.child1(), p.b));
      }
     } else if (p.b.isinternal()) {
      stkStack.push(new btDbvt.sStkNN(p.a, p.b.child0()));
      stkStack.push(new btDbvt.sStkNN(p.a, p.b.child1()));
     } else {
      callback.process(p.a, p.b);
     }
    }
   } while (!stkStack.isEmpty());
  }
 }

 static boolean MyIntersect(btDbvtAabbMm a,
  btDbvtAabbMm b, final btTransform xform, float distanceThreshold) {
  final btVector3 newmin = new btVector3();
  final btVector3 newmax = new btVector3();
  btTransformAabb(b.mins(), b.maxs(), 0.f, xform, newmin, newmax);
  newmin.sub(new btVector3(distanceThreshold, distanceThreshold, distanceThreshold));
  newmax.add(new btVector3(distanceThreshold, distanceThreshold, distanceThreshold));
  btDbvtAabbMm newb = btDbvtAabbMm.fromMM(newmin, newmax);
  return intersect(a, newb);
 }
}
