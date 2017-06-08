/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.Dynamics.CollisionObjects;

import Bullet.Collision.Broadphase.btBroadphaseProxy;
import Bullet.Collision.Broadphase.btDispatcher;
import static Bullet.Collision.CollisionObjectTypes.CO_GHOST_OBJECT;
import Bullet.Collision.ConvexResultCallback;
import Bullet.Collision.RayResultCallback;
import Bullet.Collision.Shape.btConvexShape;
import Bullet.Collision.btCollisionObject;
import Bullet.Collision.btCollisionWorld;
import static Bullet.LinearMath.btAabbUtil2.AabbExpand;
import static Bullet.LinearMath.btAabbUtil2.btRayAabb;
import Bullet.LinearMath.btTransform;
import static Bullet.LinearMath.btTransformUtil.calculateVelocity;
import Bullet.LinearMath.btVector3;
import static Bullet.common.btAlignedObjectArray.findLinearSearch;
import java.util.ArrayList;

/**
 *
 * @author Gregery Barton
 */
public class btGhostObject extends btCollisionObject {

 protected ArrayList<btCollisionObject> m_overlappingObjects = new ArrayList<>(1);

 public btGhostObject() {
  m_internalType = CO_GHOST_OBJECT;
 }

 public void convexSweepTest(btConvexShape castShape, final btTransform convexFromWorld,
  final btTransform convexToWorld, ConvexResultCallback resultCallback) {
  convexSweepTest(castShape, convexFromWorld, convexToWorld, resultCallback, 0);
 }

 public void convexSweepTest(btConvexShape castShape, final btTransform convexFromWorld,
  final btTransform convexToWorld, ConvexResultCallback resultCallback, float allowedCcdPenetration) {
  final btTransform convexFromTrans = new btTransform(convexFromWorld);
  final btTransform convexToTrans = new btTransform(convexToWorld);
  final btVector3 castShapeAabbMin = new btVector3();
  final btVector3 castShapeAabbMax = new btVector3();
  /* Compute AABB that encompasses angular movement */
  {
   final btVector3 linVel = new btVector3();
   final btVector3 angVel = new btVector3();
   calculateVelocity(convexFromTrans, convexToTrans, 1.0f, linVel, angVel);
   final btTransform R = new btTransform();
   R.setIdentity();
   R.set3x3(convexFromTrans.getRotation());
   castShape.calculateTemporalAabb(R, linVel, angVel, 1.0f, castShapeAabbMin, castShapeAabbMax);
  }
  /// go over all objects, and if the ray intersects their aabb + cast shape aabb,
  // do a ray-shape query using convexCaster (CCD)
  int i;
  for (i = 0; i < m_overlappingObjects.size(); i++) {
   btCollisionObject collisionObject = m_overlappingObjects.get(i);
   //only perform raycast if filterMask matches
   if (resultCallback.needsCollision(collisionObject.getBroadphaseHandle())) {
    //RigidcollisionObject* collisionObject = ctrl.GetRigidcollisionObject();
    final btVector3 collisionObjectAabbMin = new btVector3();
    final btVector3 collisionObjectAabbMax = new btVector3();
    collisionObject.getCollisionShape().getAabb(collisionObject.getWorldTransform(),
     collisionObjectAabbMin, collisionObjectAabbMax);
    AabbExpand(collisionObjectAabbMin, collisionObjectAabbMax, castShapeAabbMin, castShapeAabbMax);
    float[] hitLambda = {1.f}; //could use resultCallback.m_closestHitFraction, but needs testing
    final btVector3 hitNormal = new btVector3();
    if (btRayAabb(convexFromWorld.getOrigin(), convexToWorld.getOrigin(), collisionObjectAabbMin,
     collisionObjectAabbMax, hitLambda, hitNormal)) {
     btCollisionWorld.objectQuerySingle(castShape, convexFromTrans, convexToTrans,
      collisionObject,
      collisionObject.getCollisionShape(),
      collisionObject.getWorldTransform(),
      resultCallback,
      allowedCcdPenetration);
    }
   }
  }
 }

 public void rayTest(final btVector3 rayFromWorld, final btVector3 rayToWorld,
  RayResultCallback resultCallback) {
  final btTransform rayFromTrans = new btTransform();
  rayFromTrans.setIdentity();
  rayFromTrans.setOrigin(rayFromWorld);
  final btTransform rayToTrans = new btTransform();
  rayToTrans.setIdentity();
  rayToTrans.setOrigin(rayToWorld);
  int i;
  for (i = 0; i < m_overlappingObjects.size(); i++) {
   btCollisionObject collisionObject = m_overlappingObjects.get(i);
   //only perform raycast if filterMask matches
   if (resultCallback.needsCollision(collisionObject.getBroadphaseHandle())) {
    btCollisionWorld.rayTestSingle(rayFromTrans, rayToTrans,
     collisionObject,
     collisionObject.getCollisionShape(),
     collisionObject.getWorldTransform(),
     resultCallback);
   }
  }
 }

 ///this method is mainly for expert/internal use only.
 public final void addOverlappingObjectInternal(btBroadphaseProxy otherProxy) {
  addOverlappingObjectInternal(otherProxy, null);
 }

 public void addOverlappingObjectInternal(btBroadphaseProxy otherProxy, btBroadphaseProxy thisProxy) {
  btCollisionObject otherObject = (btCollisionObject) otherProxy.m_clientObject;
  assert (otherObject != null);
  ///if this linearSearch becomes too slow (too many overlapping objects) we should add a more appropriate data structure
  int index = findLinearSearch(m_overlappingObjects, otherObject);
  if (index == m_overlappingObjects.size()) {
   //not found
   m_overlappingObjects.add(otherObject);
  }
 }

 ///this method is mainly for expert/internal use only.
 public final  void removeOverlappingObjectInternal(btBroadphaseProxy otherProxy, btDispatcher dispatcher) {
  removeOverlappingObjectInternal(otherProxy, dispatcher, null);
 }

 public void removeOverlappingObjectInternal(btBroadphaseProxy otherProxy, btDispatcher dispatcher,
  btBroadphaseProxy thisProxy) {
  btCollisionObject otherObject = (btCollisionObject) otherProxy.m_clientObject;
  assert (otherObject != null);
  int index = findLinearSearch(m_overlappingObjects, otherObject);
  if (index < m_overlappingObjects.size()) {
   m_overlappingObjects.set(index, m_overlappingObjects.get(m_overlappingObjects.size() - 1));
   m_overlappingObjects.remove(m_overlappingObjects.size() - 1);
  }
 }

 public int getNumOverlappingObjects() {
  return m_overlappingObjects.size();
 }

 public btCollisionObject getOverlappingObject(int index) {
  return m_overlappingObjects.get(index);
 }

 public ArrayList<btCollisionObject> getOverlappingPairs() {
  return m_overlappingObjects;
 }

 //
 // internal cast
 //
 public static btGhostObject upcast(btCollisionObject colObj) {
  if (colObj.getInternalType() == CO_GHOST_OBJECT) {
   return (btGhostObject) colObj;
  }
  return null;
 }
}
