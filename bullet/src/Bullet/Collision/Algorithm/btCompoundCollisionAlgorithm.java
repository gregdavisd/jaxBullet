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
 *
 */
package Bullet.Collision.Algorithm;

import Bullet.Collision.Broadphase.btDbvt;
import Bullet.Collision.Broadphase.btDbvtAabbMm;
import Bullet.Collision.Broadphase.btDbvtNode;
import Bullet.Collision.Broadphase.btDispatcherInfo;
import Bullet.Collision.Shape.btCollisionShape;
import Bullet.Collision.Shape.btCompoundShape;
import Bullet.Collision.btCollisionObject;
import Bullet.Collision.btCollisionObjectWrapper;
import Bullet.Collision.btCompoundLeafCallback;
import Bullet.Collision.btManifoldResult;
import Bullet.Collision.btPersistentManifold;
import static Bullet.Collision.ebtDispatcherQueryType.BT_CONTACT_POINT_ALGORITHMS;
import static Bullet.LinearMath.btAabbUtil2.TestAabbAgainstAabb2;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import java.util.ArrayList;

/**
 *
 * @author Gregery Barton
 */
public class btCompoundCollisionAlgorithm extends btActivatingCollisionAlgorithm
 implements
 Serializable {

 final ArrayList<btDbvtNode> stack2 = new ArrayList<>(0);
 final ArrayList<btPersistentManifold> manifoldArray = new ArrayList<>(0);
 final ArrayList<btCollisionAlgorithm> m_childCollisionAlgorithms = new ArrayList<>(
  0);
 boolean m_isSwapped;
 final btPersistentManifold m_sharedManifold;
 boolean m_ownsManifold;
 int m_compoundShapeRevision;//to keep track of changes, so that childAlgorithm array can be updated

 void removeChildAlgorithms() {
  int numChildren = m_childCollisionAlgorithms.size();
  int i;
  for (i = 0; i < numChildren; i++) {
   if (m_childCollisionAlgorithms.get(i) != null) {
    m_childCollisionAlgorithms.get(i).destroy();
   }
  }
 }

 final void preallocateChildAlgorithms(btCollisionObjectWrapper body0Wrap,
  btCollisionObjectWrapper body1Wrap) {
  btCollisionObjectWrapper colObjWrap = m_isSwapped ? body1Wrap : body0Wrap;
  btCollisionObjectWrapper otherObjWrap = m_isSwapped ? body0Wrap : body1Wrap;
  assert (colObjWrap.getCollisionShape().isCompound());
  btCompoundShape compoundShape = (btCompoundShape) (colObjWrap
   .getCollisionShape());
  int numChildren = compoundShape.getNumChildShapes();
  int i;
  m_childCollisionAlgorithms.ensureCapacity(numChildren);
  for (i = 0; i < numChildren; i++) {
   if (compoundShape.getDynamicAabbTree() != null) {
    m_childCollisionAlgorithms.add(null);
   } else {
    btCollisionShape childShape = compoundShape.getChildShape(i);
    btCollisionObjectWrapper childWrap = new btCollisionObjectWrapper(colObjWrap,
     childShape,
     colObjWrap.getCollisionObject(), colObjWrap.getWorldTransform(), -1, i);//wrong child trans, but unused (hopefully)
    m_childCollisionAlgorithms.add(m_dispatcher.findAlgorithm(childWrap,
     otherObjWrap,
     m_sharedManifold, BT_CONTACT_POINT_ALGORITHMS));
   }
  }
 }

 btCompoundCollisionAlgorithm(btCollisionAlgorithmConstructionInfo ci,
  btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap,
  boolean isSwapped) {
  super(ci, body0Wrap, body1Wrap);
  m_isSwapped = isSwapped;
  m_sharedManifold = (ci.m_manifold);
  m_ownsManifold = false;
  btCollisionObjectWrapper colObjWrap = m_isSwapped ? body1Wrap : body0Wrap;
  assert (colObjWrap.getCollisionShape().isCompound());
  btCompoundShape compoundShape = (btCompoundShape) (colObjWrap
   .getCollisionShape());
  m_compoundShapeRevision = compoundShape.getUpdateRevision();
  preallocateChildAlgorithms(body0Wrap, body1Wrap);
 }

 @Override
 public void destroy() {
  removeChildAlgorithms();
 }

 btCollisionAlgorithm getChildAlgorithm(int n) {
  return m_childCollisionAlgorithms.get(n);
 }

 @Override
 public void processCollision(btCollisionObjectWrapper body0Wrap,
  btCollisionObjectWrapper body1Wrap,
  btDispatcherInfo dispatchInfo, btManifoldResult resultOut) {
  btCollisionObjectWrapper colObjWrap = m_isSwapped ? body1Wrap : body0Wrap;
  btCollisionObjectWrapper otherObjWrap = m_isSwapped ? body0Wrap : body1Wrap;
  assert (colObjWrap.getCollisionShape().isCompound());
  btCompoundShape compoundShape = (btCompoundShape) (colObjWrap
   .getCollisionShape());
  ///btCompoundShape might have changed:
  ////make sure the internal child collision algorithm caches are still valid
  if (compoundShape.getUpdateRevision() != m_compoundShapeRevision) {
   ///clear and update all
   removeChildAlgorithms();
   preallocateChildAlgorithms(body0Wrap, body1Wrap);
   m_compoundShapeRevision = compoundShape.getUpdateRevision();
  }
  if (m_childCollisionAlgorithms.isEmpty()) {
   return;
  }
  btDbvt tree = compoundShape.getDynamicAabbTree();
  //use a dynamic aabb tree to cull potential child-overlaps
  btCompoundLeafCallback callback = new btCompoundLeafCallback(colObjWrap,
   otherObjWrap,
   m_dispatcher, dispatchInfo, resultOut, m_childCollisionAlgorithms,
   m_sharedManifold);
  ///we need to refresh all contact manifolds
  ///note that we should actually recursively traverse all children, btCompoundShape can nested more then 1 level deep
  ///so we should add a 'refreshManifolds' in the btCollisionAlgorithm
  {
   int i;
   manifoldArray.clear();
   for (i = 0; i < m_childCollisionAlgorithms.size(); i++) {
    if (m_childCollisionAlgorithms.get(i) != null) {
     m_childCollisionAlgorithms.get(i).getAllContactManifolds(manifoldArray);
     for (int m = 0; m < manifoldArray.size(); m++) {
      if (manifoldArray.get(m).getNumContacts() != 0) {
       resultOut.setPersistentManifold(manifoldArray.get(m));
       resultOut.refreshContactPoints();
       resultOut.setPersistentManifold(null);//??necessary?
      }
     }
     manifoldArray.clear();
    }
   }
  }
  if (tree != null) {
   final btVector3 localAabbMin = new btVector3();
   final btVector3 localAabbMax = new btVector3();
   final btTransform otherInCompoundSpace = new btTransform(
    colObjWrap.getWorldTransform().invert()
     .mul(otherObjWrap.getWorldTransform()));
   otherObjWrap.getCollisionShape().getAabb(otherInCompoundSpace, localAabbMin,
    localAabbMax);
   final btVector3 extraExtends = new btVector3(
    resultOut.m_closestPointDistanceThreshold,
    resultOut.m_closestPointDistanceThreshold,
    resultOut.m_closestPointDistanceThreshold);
   localAabbMin.sub(extraExtends);
   localAabbMax.sub(extraExtends);
   btDbvtAabbMm bounds = btDbvtAabbMm.fromMM(localAabbMin, localAabbMax);
   //process all children, that overlap with  the given AABB bounds
   tree.collideTVNoStackAlloc(tree.m_root, bounds, stack2, callback);
  } else {
   //iterate over all children, perform an AABB check inside ProcessChildShape
   int numChildren = m_childCollisionAlgorithms.size();
   int i;
   for (i = 0; i < numChildren; i++) {
    callback.processChildShape(compoundShape.getChildShape(i), i);
   }
  }
  {
   //iterate over all children, perform an AABB check inside ProcessChildShape
   int numChildren = m_childCollisionAlgorithms.size();
   int i;
   manifoldArray.clear();
   btCollisionShape childShape = null;
   final btTransform orgTrans = new btTransform();
   final btTransform newChildWorldTrans = new btTransform();
   final btVector3 aabbMin0 = new btVector3();
   final btVector3 aabbMax0 = new btVector3();
   final btVector3 aabbMin1 = new btVector3();
   final btVector3 aabbMax1 = new btVector3();
   for (i = 0; i < numChildren; i++) {
    if (m_childCollisionAlgorithms.get(i) != null) {
     childShape = compoundShape.getChildShape(i);
     //if not longer overlapping, remove the algorithm
     orgTrans.set(colObjWrap.getWorldTransform());
     final btTransform childTrans = compoundShape.getChildTransform(i);
     newChildWorldTrans.set(orgTrans).mul(childTrans);
     //perform an AABB check first
     childShape.getAabb(newChildWorldTrans, aabbMin0, aabbMax0);
     otherObjWrap.getCollisionShape().getAabb(otherObjWrap.getWorldTransform(),
      aabbMin1, aabbMax1);
     if (!TestAabbAgainstAabb2(aabbMin0, aabbMax0, aabbMin1, aabbMax1)) {
      m_childCollisionAlgorithms.get(i).destroy();
      m_childCollisionAlgorithms.set(i, null);
     }
    }
   }
  }
 }

 @Override
 public float calculateTimeOfImpact(btCollisionObject body0,
  btCollisionObject body1,
  btDispatcherInfo dispatchInfo, btManifoldResult resultOut) {
  assert (false);
  return 0;
  /*
   * //needs to be fixed, using btCollisionObjectWrapper and NOT modifying
   * internal data structures btCollisionObject* colObj = m_isSwapped? body1 :
   * body0; btCollisionObject* otherObj = m_isSwapped? body0 : body1;
   *
   * assert (colObj.getCollisionShape().isCompound());
   *
   * btCompoundShape* compoundShape =
   * static_cast<btCompoundShape*>(colObj.getCollisionShape());
   *
   * //We will use the OptimizedBVH, AABB tree to cull potential child-overlaps
   * //If both proxies are Compound, we will deal with that directly, by
   * performing sequential/parallel tree traversals //given Proxy0 and Proxy1,
   * if both have a tree, Tree0 and Tree1, this means: //determine overlapping
   * nodes of Proxy1 using Proxy0 AABB against Tree1 //then use each overlapping
   * node AABB against Tree0 //and vise versa.
   *
   * float hitFraction = float(1.);
   *
   * int numChildren = m_childCollisionAlgorithms.size(); int i; btTransform
   * orgTrans; float frac; for (i=0;i<numChildren;i++) { //btCollisionShape*
   * childShape = compoundShape.getChildShape(i);
   *
   * //backup orgTrans = colObj.getWorldTransform();
   *
   * btTransform& childTrans = compoundShape.getChildTransform(i); //btTransform
   * newChildWorldTrans = orgTrans*childTrans ; colObj.setWorldTransform(
   * orgTrans*childTrans );
   *
   * //btCollisionShape* tmpShape = colObj.getCollisionShape();
   * //colObj.internalSetTemporaryCollisionShape( childShape ); frac =
   * m_childCollisionAlgorithms[i].calculateTimeOfImpact(colObj,otherObj,dispatchInfo,resultOut);
   * if (frac<hitFraction) { hitFraction = frac; } //revert back
   * //colObj.internalSetTemporaryCollisionShape( tmpShape);
   * colObj.setWorldTransform( orgTrans); } return hitFraction;
   */
 }

 @Override
 public void getAllContactManifolds(
  ArrayList<btPersistentManifold> manifoldArray) {
  int i;
  for (i = 0; i < m_childCollisionAlgorithms.size(); i++) {
   if (m_childCollisionAlgorithms.get(i) != null) {
    m_childCollisionAlgorithms.get(i).getAllContactManifolds(manifoldArray);
   }
  }
 }

 public static class CreateFunc extends btCollisionAlgorithmCreateFunc {

  @Override
  public btCollisionAlgorithm CreateCollisionAlgorithm(
   btCollisionAlgorithmConstructionInfo ci,
   btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap) {
   return new btCompoundCollisionAlgorithm(ci, body0Wrap, body1Wrap, false);
  }

 };

 public static class SwappedCreateFunc extends btCollisionAlgorithmCreateFunc {

  @Override
  public btCollisionAlgorithm CreateCollisionAlgorithm(
   btCollisionAlgorithmConstructionInfo ci,
   btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap) {
   return new btCompoundCollisionAlgorithm(ci, body0Wrap, body1Wrap, true);
  }

 };
};
