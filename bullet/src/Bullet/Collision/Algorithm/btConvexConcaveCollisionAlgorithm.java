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
package Bullet.Collision.Algorithm;

import Bullet.Collision.Broadphase.btDispatcherInfo;
import Bullet.Collision.LocalTriangleSphereCastCallback;
import Bullet.Collision.Shape.btConcaveShape;
import Bullet.Collision.btCollisionObject;
import Bullet.Collision.btCollisionObjectWrapper;
import Bullet.Collision.btConvexTriangleCallback;
import Bullet.Collision.btManifoldResult;
import Bullet.Collision.btPersistentManifold;
import static Bullet.LinearMath.btQuickprof.BT_PROFILE;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import java.util.ArrayList;

/**
 *
 *
 * @author Gregery Barton
 */
public class btConvexConcaveCollisionAlgorithm extends btActivatingCollisionAlgorithm
 implements
 Serializable {

 final btConvexTriangleCallback m_btConvexTriangleCallback;
 boolean m_isSwapped;

 btConvexConcaveCollisionAlgorithm(btCollisionAlgorithmConstructionInfo ci,
  btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap,
  boolean isSwapped) {
  super(ci, body0Wrap, body1Wrap);
  m_btConvexTriangleCallback = new btConvexTriangleCallback(ci.m_dispatcher1,
   body0Wrap, body1Wrap,
   isSwapped);
  m_isSwapped = isSwapped;
 }

 @Override
 public void processCollision(btCollisionObjectWrapper body0Wrap,
  btCollisionObjectWrapper body1Wrap,
  btDispatcherInfo dispatchInfo, btManifoldResult resultOut) {
  BT_PROFILE("btConvexConcaveCollisionAlgorithm.processCollision");
  btCollisionObjectWrapper convexBodyWrap = m_isSwapped ? body1Wrap : body0Wrap;
  btCollisionObjectWrapper triBodyWrap = m_isSwapped ? body0Wrap : body1Wrap;
  if (triBodyWrap.getCollisionShape().isConcave()) {
   btConcaveShape concaveShape = (btConcaveShape) (triBodyWrap
    .getCollisionShape());
   if (convexBodyWrap.getCollisionShape().isConvex()) {
    float collisionMarginTriangle = concaveShape.getMargin();
    resultOut.setPersistentManifold(m_btConvexTriangleCallback.m_manifoldPtr);
    m_btConvexTriangleCallback.setTimeStepAndCounters(collisionMarginTriangle,
     dispatchInfo,
     convexBodyWrap, triBodyWrap, resultOut);
    m_btConvexTriangleCallback.m_manifoldPtr.setBodies(convexBodyWrap
     .getCollisionObject(),
     triBodyWrap.getCollisionObject());
    concaveShape.processAllTriangles(m_btConvexTriangleCallback,
     m_btConvexTriangleCallback
      .getAabbMin(), m_btConvexTriangleCallback.getAabbMax());
    resultOut.refreshContactPoints();
    m_btConvexTriangleCallback.clearWrapperData();
   }
  }
 }

 @Override
 public float calculateTimeOfImpact(btCollisionObject body0,
  btCollisionObject body1,
  btDispatcherInfo dispatchInfo, btManifoldResult resultOut) {
  btCollisionObject convexbody = m_isSwapped ? body1 : body0;
  btCollisionObject triBody = m_isSwapped ? body0 : body1;
  //quick approximation using raycast, todo: hook up to the continuous collision detection (one of the btConvexCast)
  //only perform CCD above a certain threshold, this prevents blocking on the long run
  //because object in a blocked ccd state (hitfraction<1) get their linear velocity halved each frame...
  float squareMot0 = (convexbody.getInterpolationWorldTransform().getOrigin()
   .sub(convexbody
    .getWorldTransformPtr().getOrigin())).lengthSquared();
  if (squareMot0 < convexbody.getCcdSquareMotionThreshold()) {
   return (1.f);
  }
  //  btVector3& from = convexbody.m_worldTransform.getOrigin();
  //btVector3 to = convexbody.m_interpolationWorldTransform.getOrigin();
  //todo: only do if the motion exceeds the 'radius'
  final btTransform triInv = triBody.getWorldTransform().invert();
  final btTransform convexFromLocal = new btTransform(triInv).mul(convexbody
   .getWorldTransformPtr());
  final btTransform convexToLocal = new btTransform(triInv).mul(convexbody
   .getInterpolationWorldTransform());
  if (triBody.getCollisionShape().isConcave()) {
   final btVector3 rayAabbMin = convexFromLocal.getOrigin();
   rayAabbMin.setMin(convexToLocal.getOrigin());
   final btVector3 rayAabbMax = convexFromLocal.getOrigin();
   rayAabbMax.setMax(convexToLocal.getOrigin());
   float ccdRadius0 = convexbody.getCcdSweptSphereRadius();
   rayAabbMin.sub(new btVector3(ccdRadius0, ccdRadius0, ccdRadius0));
   rayAabbMax.add(new btVector3(ccdRadius0, ccdRadius0, ccdRadius0));
   float curHitFraction = (1.f); //is this available?
   LocalTriangleSphereCastCallback raycastCallback = new LocalTriangleSphereCastCallback(
    convexFromLocal, convexToLocal,
    convexbody.getCcdSweptSphereRadius(), curHitFraction);
   raycastCallback.m_hitFraction = convexbody.getHitFraction();
   btCollisionObject concavebody = triBody;
   btConcaveShape triangleMesh = (btConcaveShape) concavebody
    .getCollisionShape();
   if (triangleMesh != null) {
    triangleMesh.processAllTriangles(raycastCallback, rayAabbMin, rayAabbMax);
   }
   if (raycastCallback.m_hitFraction < convexbody.getHitFraction()) {
    convexbody.setHitFraction(raycastCallback.m_hitFraction);
    return raycastCallback.m_hitFraction;
   }
  }
  return (1.f);
 }

 @Override
 public void getAllContactManifolds(
  ArrayList<btPersistentManifold> manifoldArray) {
  if (m_btConvexTriangleCallback.m_manifoldPtr != null) {
   manifoldArray.add(m_btConvexTriangleCallback.m_manifoldPtr);
  }
 }

 void clearCache() {
  m_btConvexTriangleCallback.clearCache();
 }

 @Override
 public void destroy() {
  m_btConvexTriangleCallback.destroy();
 }

 public static class CreateFunc extends btCollisionAlgorithmCreateFunc {

  @Override
  public btCollisionAlgorithm CreateCollisionAlgorithm(
   btCollisionAlgorithmConstructionInfo ci,
   btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap) {
   return new btConvexConcaveCollisionAlgorithm(ci, body0Wrap, body1Wrap, false);
  }

 }

 public static class SwappedCreateFunc extends btCollisionAlgorithmCreateFunc {

  @Override
  public btCollisionAlgorithm CreateCollisionAlgorithm(
   btCollisionAlgorithmConstructionInfo ci,
   btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap) {
   return new btConvexConcaveCollisionAlgorithm(ci, body0Wrap, body1Wrap, true);
  }

 }
}
