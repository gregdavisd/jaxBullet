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
package Bullet.Collision;

import Bullet.Collision.Algorithm.btCollisionAlgorithm;
import Bullet.Collision.Broadphase.btDbvt;
import Bullet.Collision.Broadphase.btDbvtNode;
import Bullet.Collision.Broadphase.btDispatcher;
import Bullet.Collision.Broadphase.btDispatcherInfo;
import Bullet.Collision.Broadphase.btHashedSimplePairCache;
import Bullet.Collision.Shape.btCollisionShape;
import Bullet.Collision.Shape.btCompoundShape;
import static Bullet.Collision.ebtDispatcherQueryType.BT_CLOSEST_POINT_ALGORITHMS;
import static Bullet.Collision.ebtDispatcherQueryType.BT_CONTACT_POINT_ALGORITHMS;
import static Bullet.LinearMath.btAabbUtil2.TestAabbAgainstAabb2;
import static Bullet.LinearMath.btQuickprof.BT_PROFILE;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btCompoundCompoundLeafCallback extends btDbvt.ICollide  implements Serializable {

 int m_numOverlapPairs;
 static btShapePairCallback gCompoundCompoundChildShapePairCallback = null;
 final btCollisionObjectWrapper m_compound0ColObjWrap;
 final btCollisionObjectWrapper m_compound1ColObjWrap;
 final btDispatcher m_dispatcher;
 final btDispatcherInfo m_dispatchInfo;
 final btManifoldResult m_resultOut;
 final btHashedSimplePairCache m_childCollisionAlgorithmCache;
 final btPersistentManifold m_sharedManifold;

 public btCompoundCompoundLeafCallback(btCollisionObjectWrapper compound1ObjWrap,
  btCollisionObjectWrapper compound0ObjWrap,
  btDispatcher dispatcher,
  btDispatcherInfo dispatchInfo,
  btManifoldResult resultOut,
  btHashedSimplePairCache childAlgorithmsCache,
  btPersistentManifold sharedManifold) {
  m_numOverlapPairs = 0;
  m_compound0ColObjWrap = compound1ObjWrap;
  m_compound1ColObjWrap = compound0ObjWrap;
  m_dispatcher = dispatcher;
  m_dispatchInfo = dispatchInfo;
  m_resultOut = resultOut;
  m_childCollisionAlgorithmCache = childAlgorithmsCache;
  m_sharedManifold = sharedManifold;
 }

 @Override
 public  void process(btDbvtNode leaf0, btDbvtNode leaf1) {
  BT_PROFILE("btCompoundCompoundLeafCallback.Process");
  m_numOverlapPairs++;
  int childIndex0 = leaf0.dataAsInt;
  int childIndex1 = leaf1.dataAsInt;
  assert(childIndex0 >= 0);
  assert(childIndex1 >= 0);
  btCompoundShape compoundShape0 = (btCompoundShape) (m_compound0ColObjWrap.getCollisionShape());
  assert(childIndex0 < compoundShape0.getNumChildShapes());
  btCompoundShape compoundShape1 = (btCompoundShape) (m_compound1ColObjWrap.getCollisionShape());
  assert(childIndex1 < compoundShape1.getNumChildShapes());
  btCollisionShape childShape0 = compoundShape0.getChildShape(childIndex0);
  btCollisionShape childShape1 = compoundShape1.getChildShape(childIndex1);
  //backup
  final btTransform orgTrans0 = m_compound0ColObjWrap.getWorldTransform();
  final btTransform childTrans0 = compoundShape0.getChildTransform(childIndex0);
  final btTransform newChildWorldTrans0 = new btTransform(orgTrans0).mul(childTrans0);
  final btTransform orgTrans1 = m_compound1ColObjWrap.getWorldTransform();
  final btTransform childTrans1 = compoundShape1.getChildTransform(childIndex1);
  final btTransform newChildWorldTrans1 = new btTransform(orgTrans1).mul(childTrans1);
  //perform an AABB check first
  final btVector3 aabbMin0 = new btVector3();
  final btVector3 aabbMax0 = new btVector3();
  final btVector3 aabbMin1 = new btVector3();
  final btVector3 aabbMax1 = new btVector3();
  childShape0.getAabb(newChildWorldTrans0, aabbMin0, aabbMax0);
  childShape1.getAabb(newChildWorldTrans1, aabbMin1, aabbMax1);
  final btVector3 thresholdVec = new btVector3(m_resultOut.m_closestPointDistanceThreshold,
   m_resultOut.m_closestPointDistanceThreshold, m_resultOut.m_closestPointDistanceThreshold);
  aabbMin0.sub(thresholdVec);
  aabbMax0.add(thresholdVec);
  if (gCompoundCompoundChildShapePairCallback != null) {
   if (!gCompoundCompoundChildShapePairCallback.callback(childShape0, childShape1)) {
    return;
   }
  }
  if (TestAabbAgainstAabb2(aabbMin0, aabbMax0, aabbMin1, aabbMax1)) {
   btCollisionObjectWrapper compoundWrap0 = new btCollisionObjectWrapper(this.m_compound0ColObjWrap,
    childShape0, m_compound0ColObjWrap.getCollisionObject(), newChildWorldTrans0, -1, childIndex0);
   btCollisionObjectWrapper compoundWrap1 = new btCollisionObjectWrapper(this.m_compound1ColObjWrap,
    childShape1, m_compound1ColObjWrap.getCollisionObject(), newChildWorldTrans1, -1, childIndex1);
   btSimplePair pair = m_childCollisionAlgorithmCache.findPair(childIndex0, childIndex1);
   btCollisionAlgorithm colAlgo;
   if (m_resultOut.m_closestPointDistanceThreshold > 0) {
    colAlgo = m_dispatcher.findAlgorithm(compoundWrap0, compoundWrap1, null,
     BT_CLOSEST_POINT_ALGORITHMS);
   } else if (pair != null) {
    colAlgo = (btCollisionAlgorithm) pair.m_userPointer;
   } else {
    colAlgo = m_dispatcher.findAlgorithm(compoundWrap0, compoundWrap1, m_sharedManifold,
     BT_CONTACT_POINT_ALGORITHMS);
    pair = m_childCollisionAlgorithmCache.addOverlappingPair(childIndex0, childIndex1);
    assert(pair != null);
    pair.m_userPointer = colAlgo;
   }
   assert(colAlgo != null);
   btCollisionObjectWrapper tmpWrap0;
   btCollisionObjectWrapper tmpWrap1;
   tmpWrap0 = m_resultOut.getBody0Wrap();
   tmpWrap1 = m_resultOut.getBody1Wrap();
   m_resultOut.setBody0Wrap(compoundWrap0);
   m_resultOut.setBody1Wrap(compoundWrap1);
   m_resultOut.setShapeIdentifiersA(-1, childIndex0);
   m_resultOut.setShapeIdentifiersB(-1, childIndex1);
   colAlgo.processCollision(compoundWrap0, compoundWrap1, m_dispatchInfo, m_resultOut);
   m_resultOut.setBody0Wrap(tmpWrap0);
   m_resultOut.setBody1Wrap(tmpWrap1);
  }
 }
};
