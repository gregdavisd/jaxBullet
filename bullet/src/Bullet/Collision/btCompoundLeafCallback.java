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

import Bullet.Collision.Algorithm.btCollisionAlgorithm;
import Bullet.Collision.Broadphase.btDbvt;
import Bullet.Collision.Broadphase.btDbvtNode;
import Bullet.Collision.Broadphase.btDispatcher;
import Bullet.Collision.Broadphase.btDispatcherInfo;
import Bullet.Collision.Shape.btCollisionShape;
import Bullet.Collision.Shape.btCompoundShape;
import static Bullet.Collision.ebtDispatcherQueryType.BT_CLOSEST_POINT_ALGORITHMS;
import static Bullet.Collision.ebtDispatcherQueryType.BT_CONTACT_POINT_ALGORITHMS;
import static Bullet.LinearMath.btAabbUtil2.TestAabbAgainstAabb2;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import java.util.List;

/**
 *
 * @author Gregery Barton
 */
public class btCompoundLeafCallback extends btDbvt.ICollide  implements Serializable {

 public static btShapePairCallback gCompoundChildShapePairCallback;
 final btCollisionObjectWrapper m_compoundColObjWrap;
 final btCollisionObjectWrapper m_otherObjWrap;
 final btDispatcher m_dispatcher;
 final btDispatcherInfo m_dispatchInfo;
 final btManifoldResult m_resultOut;
 final List<btCollisionAlgorithm> m_childCollisionAlgorithms;
 final btPersistentManifold m_sharedManifold;

 public btCompoundLeafCallback(btCollisionObjectWrapper compoundObjWrap,
  btCollisionObjectWrapper otherObjWrap, btDispatcher dispatcher, btDispatcherInfo dispatchInfo,
  btManifoldResult resultOut, List<btCollisionAlgorithm> childCollisionAlgorithms,
  btPersistentManifold sharedManifold) {
  m_compoundColObjWrap = compoundObjWrap;
  m_otherObjWrap = otherObjWrap;
  m_dispatcher = dispatcher;
  m_dispatchInfo = dispatchInfo;
  m_resultOut = resultOut;
  m_childCollisionAlgorithms = childCollisionAlgorithms;
  m_sharedManifold = sharedManifold;
 }

 public void processChildShape(btCollisionShape childShape, int index) {
  assert(index >= 0);
  btCompoundShape compoundShape = (btCompoundShape) (m_compoundColObjWrap.getCollisionShape());
  assert(index < compoundShape.getNumChildShapes());
  //backup
  final btTransform orgTrans = m_compoundColObjWrap.getWorldTransform();
  final btTransform childTrans = compoundShape.getChildTransform(index);
  final btTransform newChildWorldTrans = new btTransform(orgTrans).mul(childTrans);
  //perform an AABB check first
  final btVector3 aabbMin0 = new btVector3();
  final btVector3 aabbMax0 = new btVector3();
  childShape.getAabb(newChildWorldTrans, aabbMin0, aabbMax0);
  final btVector3 extendAabb = new btVector3(m_resultOut.m_closestPointDistanceThreshold,
   m_resultOut.m_closestPointDistanceThreshold, m_resultOut.m_closestPointDistanceThreshold);
  aabbMin0.sub(extendAabb);
  aabbMax0.add(extendAabb);
  final btVector3 aabbMin1 = new btVector3();
  final btVector3 aabbMax1 = new btVector3();
  m_otherObjWrap.getCollisionShape().getAabb(m_otherObjWrap.getWorldTransform(), aabbMin1, aabbMax1);
  if (gCompoundChildShapePairCallback != null) {
   if (!gCompoundChildShapePairCallback.callback(m_otherObjWrap.getCollisionShape(), childShape)) {
    return;
   }
  }
  if (TestAabbAgainstAabb2(aabbMin0, aabbMax0, aabbMin1, aabbMax1)) {
   btCollisionObjectWrapper compoundWrap = new btCollisionObjectWrapper(this.m_compoundColObjWrap,
    childShape, m_compoundColObjWrap.getCollisionObject(), newChildWorldTrans, -1, index);
   btCollisionAlgorithm algo;
   if (m_resultOut.m_closestPointDistanceThreshold > 0) {
    algo = m_dispatcher.findAlgorithm(compoundWrap, m_otherObjWrap, null,
     BT_CLOSEST_POINT_ALGORITHMS);
   } else {
    //the contactpoint is still projected back using the original inverted worldtrans
    if (m_childCollisionAlgorithms.get(index) == null) {
     m_childCollisionAlgorithms.set(index, m_dispatcher.findAlgorithm(compoundWrap, m_otherObjWrap,
      m_sharedManifold, BT_CONTACT_POINT_ALGORITHMS));
    }
    algo = m_childCollisionAlgorithms.get(index);
   }
   btCollisionObjectWrapper tmpWrap;
   ///detect swapping case
   if (m_resultOut.getBody0Internal() == m_compoundColObjWrap.getCollisionObject()) {
    tmpWrap = m_resultOut.getBody0Wrap();
    m_resultOut.setBody0Wrap(compoundWrap);
    m_resultOut.setShapeIdentifiersA(-1, index);
   } else {
    tmpWrap = m_resultOut.getBody1Wrap();
    m_resultOut.setBody1Wrap(compoundWrap);
    m_resultOut.setShapeIdentifiersB(-1, index);
   }
   algo.processCollision(compoundWrap, m_otherObjWrap, m_dispatchInfo, m_resultOut);
   if (m_resultOut.getBody0Internal() == m_compoundColObjWrap.getCollisionObject()) {
    m_resultOut.setBody0Wrap(tmpWrap);
   } else {
    m_resultOut.setBody1Wrap(tmpWrap);
   }
  }
 }

 @Override
 public void process(btDbvtNode leaf) {
  int index = leaf.dataAsInt;
  btCompoundShape compoundShape = (btCompoundShape) (m_compoundColObjWrap.getCollisionShape());
  btCollisionShape childShape = compoundShape.getChildShape(index);
  processChildShape(childShape, index);
 }
};
