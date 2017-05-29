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
import Bullet.Collision.Algorithm.btCollisionAlgorithmConstructionInfo;
import Bullet.Collision.Broadphase.btDispatcher;
import Bullet.Collision.Broadphase.btDispatcherInfo;
import Bullet.Collision.Shape.btCollisionShape;
import Bullet.Collision.Shape.btTriangleShape;
import static Bullet.Collision.ebtDispatcherQueryType.BT_CLOSEST_POINT_ALGORITHMS;
import static Bullet.Collision.ebtDispatcherQueryType.BT_CONTACT_POINT_ALGORITHMS;
import static Bullet.LinearMath.btAabbUtil2.TestTriangleAgainstAabb2;
import static Bullet.LinearMath.btQuickprof.BT_PROFILE;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 * For each triangle in the concave mesh that overlaps with the AABB of a convex (m_convexProxy),
 * processTriangle is called.
 *
 * @author Gregery Barton
 */
public class btConvexTriangleCallback implements btTriangleCallback, Serializable {

 public final btVector3 m_aabbMin = new btVector3();
 public final btVector3 m_aabbMax = new btVector3();
 public btCollisionObjectWrapper m_convexBodyWrap;
 public btCollisionObjectWrapper m_triBodyWrap;
 public btManifoldResult m_resultOut;
 public final btDispatcher m_dispatcher;
 public btDispatcherInfo m_dispatchInfoPtr;
 public float m_collisionMarginTriangle;
 public int m_triangleCount;
 public final btPersistentManifold m_manifoldPtr;

 public btConvexTriangleCallback(btDispatcher dispatcher, btCollisionObjectWrapper body0Wrap,
  btCollisionObjectWrapper body1Wrap, boolean isSwapped) {
  m_dispatcher = dispatcher;
  m_dispatchInfoPtr = (null);
  m_convexBodyWrap = isSwapped ? body1Wrap : body0Wrap;
  m_triBodyWrap = isSwapped ? body0Wrap : body1Wrap;
  //
  // create the manifold from the dispatcher 'manifold pool'
  //
  m_manifoldPtr = m_dispatcher.getNewManifold(m_convexBodyWrap.getCollisionObject(), m_triBodyWrap
   .getCollisionObject());
  clearCache();
 }

 public void setTimeStepAndCounters(float collisionMarginTriangle, btDispatcherInfo dispatchInfo,
  btCollisionObjectWrapper convexBodyWrap, btCollisionObjectWrapper triBodyWrap,
  btManifoldResult resultOut) {
  m_convexBodyWrap = convexBodyWrap;
  m_triBodyWrap = triBodyWrap;
  m_dispatchInfoPtr = dispatchInfo;
  m_collisionMarginTriangle = collisionMarginTriangle;
  m_resultOut = resultOut;
  //recalc aabbs
  final btTransform convexInTriangleSpace =
   m_triBodyWrap.getWorldTransform().invert().mul(m_convexBodyWrap.getWorldTransform());
  btCollisionShape convexShape = (btCollisionShape) (m_convexBodyWrap.getCollisionShape());
  //CollisionShape* triangleShape = static_cast<btCollisionShape*>(triBody.m_collisionShape);
  convexShape.getAabb(convexInTriangleSpace, m_aabbMin, m_aabbMax);
  float extraMargin = collisionMarginTriangle + resultOut.m_closestPointDistanceThreshold;
  final btVector3 extra = new btVector3(extraMargin, extraMargin, extraMargin);
  m_aabbMax.add(extra);
  m_aabbMin.sub(extra);
 }

 public void clearWrapperData() {
  m_convexBodyWrap = null;
  m_triBodyWrap = null;
 }

 public void destroy() {
  clearCache();
  m_dispatcher.releaseManifold(m_manifoldPtr);
 }

 @Override
 public boolean processTriangle(btVector3[] triangle, int partId, int triangleIndex) {
  BT_PROFILE("btConvexTriangleCallback.processTriangle");
  if (!TestTriangleAgainstAabb2(triangle, m_aabbMin, m_aabbMax)) {
   return true;
  }
  btCollisionAlgorithmConstructionInfo ci = new btCollisionAlgorithmConstructionInfo();
  ci.m_dispatcher1 = m_dispatcher;
  if (m_convexBodyWrap.getCollisionShape().isConvex()) {
   btTriangleShape tm = new btTriangleShape(triangle[0], triangle[1], triangle[2]);
   tm.setMargin(m_collisionMarginTriangle);
   btCollisionObjectWrapper triObWrap = new btCollisionObjectWrapper(m_triBodyWrap, tm,
    m_triBodyWrap.getCollisionObject(), m_triBodyWrap.getWorldTransform(), partId, triangleIndex);//correct transform?
   btCollisionAlgorithm colAlgo;
   if (m_resultOut.m_closestPointDistanceThreshold > 0) {
    colAlgo = ci.m_dispatcher1.findAlgorithm(m_convexBodyWrap, triObWrap, null,
     BT_CLOSEST_POINT_ALGORITHMS);
   } else {
    colAlgo = ci.m_dispatcher1.findAlgorithm(m_convexBodyWrap, triObWrap, m_manifoldPtr,
     BT_CONTACT_POINT_ALGORITHMS);
   }
   btCollisionObjectWrapper tmpWrap;
   if (m_resultOut.getBody0Internal() == m_triBodyWrap.getCollisionObject()) {
    tmpWrap = m_resultOut.getBody0Wrap();
    m_resultOut.setBody0Wrap(triObWrap);
    m_resultOut.setShapeIdentifiersA(partId, triangleIndex);
   } else {
    tmpWrap = m_resultOut.getBody1Wrap();
    m_resultOut.setBody1Wrap(triObWrap);
    m_resultOut.setShapeIdentifiersB(partId, triangleIndex);
   }
   colAlgo.processCollision(m_convexBodyWrap, triObWrap, m_dispatchInfoPtr, m_resultOut);
   if (m_resultOut.getBody0Internal() == m_triBodyWrap.getCollisionObject()) {
    m_resultOut.setBody0Wrap(tmpWrap);
   } else {
    m_resultOut.setBody1Wrap(tmpWrap);
   }
   colAlgo.destroy();
  }
  return true;
 }

 public final void clearCache() {
  m_dispatcher.clearManifold(m_manifoldPtr);
 }

 public btVector3 getAabbMin() {
  return new btVector3(m_aabbMin);
 }

 public btVector3 getAabbMax() {
  return new btVector3(m_aabbMax);
 }
}
