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

import Bullet.Collision.Broadphase.btDispatcherInfo;
import Bullet.Collision.Shape.btSphereShape;
import Bullet.Collision.btCollisionObject;
import Bullet.Collision.btCollisionObjectWrapper;
import Bullet.Collision.btManifoldResult;
import Bullet.Collision.btPersistentManifold;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import java.util.ArrayList;

/**
 * btSphereSphereCollisionAlgorithm provides sphere-sphere collision detection. Other features are
 * frame-coherency (persistent data) and collision response. Also provides the most basic sample for
 * custom/user btCollisionAlgorithm
 *
 * @author Gregery Barton
 */
public class btSphereSphereCollisionAlgorithm extends btActivatingCollisionAlgorithm implements
 Serializable {

 boolean m_ownManifold;
 btPersistentManifold m_manifoldPtr;

 btSphereSphereCollisionAlgorithm(btPersistentManifold mf, btCollisionAlgorithmConstructionInfo ci,
  btCollisionObjectWrapper col0Wrap, btCollisionObjectWrapper col1Wrap) {
  super(ci, col0Wrap, col1Wrap);
  m_ownManifold = false;
  m_manifoldPtr = mf;
  if (m_manifoldPtr == null) {
   m_manifoldPtr = m_dispatcher.getNewManifold(col0Wrap.getCollisionObject(), col1Wrap
    .getCollisionObject());
   m_ownManifold = true;
  }
 }

 btSphereSphereCollisionAlgorithm(btCollisionAlgorithmConstructionInfo ci) {
  super(ci);
 }

 @Override
 public void processCollision(btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap,
  btDispatcherInfo dispatchInfo, btManifoldResult resultOut) {
  if (m_manifoldPtr == null) {
   return;
  }
  resultOut.setPersistentManifold(m_manifoldPtr);
  btSphereShape sphere0 = (btSphereShape) body0Wrap.getCollisionShape();
  btSphereShape sphere1 = (btSphereShape) body1Wrap.getCollisionShape();
  final btVector3 diff = body0Wrap.getWorldTransform().getOrigin().sub(body1Wrap.getWorldTransform()
   .getOrigin());
  float len = diff.length();
  float radius0 = sphere0.getRadius();
  float radius1 = sphere1.getRadius();
  m_manifoldPtr.clearManifold(); //don't do this, it disables warmstarting
  ///iff distance positive, don't generate a new contact
  if (len > (radius0 + radius1 + resultOut.m_closestPointDistanceThreshold)) {
   return;
  }
  ///distance (negative means penetration)
  float dist = len - (radius0 + radius1);
  final btVector3 normalOnSurfaceB = new btVector3(1f, 0f, 0f);
  if (len > SIMD_EPSILON) {
   normalOnSurfaceB.set(diff).scale(1.0f / len);
  }
  ///point on A (worldspace)
  ///btVector3 pos0 = col0.getWorldTransform().getOrigin() - radius0 * normalOnSurfaceB;
  ///point on B (worldspace)
  final btVector3 pos1 = new btVector3().scaleAdd(radius1, normalOnSurfaceB, body1Wrap
   .getWorldTransform()
   .getOrigin());
  /// report a contact. internally this will be kept persistent, and contact reduction is done
  resultOut.addContactPoint(normalOnSurfaceB, pos1, dist);
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
  if (m_ownManifold) {
   if (m_manifoldPtr != null) {
    m_dispatcher.releaseManifold(m_manifoldPtr);
   }
  }
 }

 public static class CreateFunc extends btCollisionAlgorithmCreateFunc {

  @Override
  public btCollisionAlgorithm CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo ci,
   btCollisionObjectWrapper col0Wrap, btCollisionObjectWrapper col1Wrap) {
   return new btSphereSphereCollisionAlgorithm(null, ci, col0Wrap, col1Wrap);
  }
 };
};
