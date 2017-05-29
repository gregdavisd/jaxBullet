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
package Bullet.Collision.Algorithm;

import Bullet.Collision.Algorithm.Detector.SphereTriangleDetector;
import Bullet.Collision.Algorithm.Detector.btDiscreteCollisionDetectorInterface;
import Bullet.Collision.Broadphase.btDispatcherInfo;
import Bullet.Collision.Shape.btSphereShape;
import Bullet.Collision.Shape.btTriangleShape;
import Bullet.Collision.btCollisionObject;
import Bullet.Collision.btCollisionObjectWrapper;
import Bullet.Collision.btManifoldResult;
import Bullet.Collision.btPersistentManifold;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import java.io.Serializable;
import java.util.ArrayList;

/**
 *
 * @author Gregery Barton
 */
public class btSphereTriangleCollisionAlgorithm extends btActivatingCollisionAlgorithm implements
 Serializable {

 boolean m_ownManifold;
 btPersistentManifold m_manifoldPtr;
 boolean m_swapped;

 btSphereTriangleCollisionAlgorithm(btPersistentManifold mf, btCollisionAlgorithmConstructionInfo ci,
  btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap, boolean swapped) {
  super(ci, body0Wrap, body1Wrap);
  m_ownManifold = false;
  m_manifoldPtr = mf;
  m_swapped = swapped;
  if (m_manifoldPtr == null) {
   m_manifoldPtr = m_dispatcher.getNewManifold(body0Wrap.getCollisionObject(), body1Wrap
    .getCollisionObject());
   m_ownManifold = true;
  }
 }

 btSphereTriangleCollisionAlgorithm(btCollisionAlgorithmConstructionInfo ci) {
  super(ci);
 }

 @Override
 public void processCollision(btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap,
  btDispatcherInfo dispatchInfo, btManifoldResult resultOut) {
  if (m_manifoldPtr == null) {
   return;
  }
  btCollisionObjectWrapper sphereObjWrap = m_swapped ? body1Wrap : body0Wrap;
  btCollisionObjectWrapper triObjWrap = m_swapped ? body0Wrap : body1Wrap;
  btSphereShape sphere = (btSphereShape) sphereObjWrap.getCollisionShape();
  btTriangleShape triangle = (btTriangleShape) triObjWrap.getCollisionShape();
  /// report a contact. internally this will be kept persistent, and contact reduction is done
  resultOut.setPersistentManifold(m_manifoldPtr);
  SphereTriangleDetector detector = new SphereTriangleDetector(sphere, triangle, m_manifoldPtr
   .getContactBreakingThreshold() + resultOut.m_closestPointDistanceThreshold);
  btDiscreteCollisionDetectorInterface.ClosestPointInput input =
   new btDiscreteCollisionDetectorInterface.ClosestPointInput();
  input.m_maximumDistanceSquared = (BT_LARGE_FLOAT);///@todo: tighter bounds
  input.m_transformA.set(sphereObjWrap.getWorldTransform());
  input.m_transformB.set(triObjWrap.getWorldTransform());
  boolean swapResults = m_swapped;
  detector.getClosestPoints(input, resultOut, dispatchInfo.m_debugDraw, swapResults);
  if (m_ownManifold) {
   resultOut.refreshContactPoints();
  }
 }

 @Override
 public float calculateTimeOfImpact(btCollisionObject body0, btCollisionObject body1,
  btDispatcherInfo dispatchInfo, btManifoldResult resultOut) {
  //not yet
  return (1.f);
 }

 @Override
 public void getAllContactManifolds(ArrayList<btPersistentManifold> manifoldArray) {
  if (m_manifoldPtr != null && m_ownManifold) {
   manifoldArray.add(m_manifoldPtr);
  }
 }

 @Override
 public void destroy() {
  {
   if (m_ownManifold) {
    if (m_manifoldPtr != null) {
     m_dispatcher.releaseManifold(m_manifoldPtr);
    }
   }
  }
 }

 public static class CreateFunc extends btCollisionAlgorithmCreateFunc {

  @Override
  public btCollisionAlgorithm CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo ci,
   btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap) {
   return new btSphereTriangleCollisionAlgorithm(ci.m_manifold, ci, body0Wrap, body1Wrap, m_swapped);
  }
 };
};
