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
package Bullet.Collision.Broadphase;

import Bullet.Collision.ConvexResultCallback;
import Bullet.Collision.Shape.btConvexShape;
import Bullet.Collision.btCollisionObject;
import Bullet.Collision.btCollisionWorld;
import static Bullet.Collision.btCollisionWorld.objectQuerySingle;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btSingleSweepCallback extends btBroadphaseRayCallback implements
 Serializable {

 final btTransform m_convexFromTrans = new btTransform();
 final btTransform m_convexToTrans = new btTransform();
 final btVector3 m_hitNormal = new btVector3();
 final btCollisionWorld m_world;
 final ConvexResultCallback m_resultCallback;
 float m_allowedCcdPenetration;
 final btConvexShape m_castShape;

 public btSingleSweepCallback(btConvexShape castShape,
  final btTransform convexFromTrans,
  final btTransform convexToTrans, btCollisionWorld world,
  ConvexResultCallback resultCallback,
  float allowedPenetration) {
  m_convexFromTrans.set(convexFromTrans);
  m_convexToTrans.set(convexToTrans);
  m_world = world;
  m_resultCallback = resultCallback;
  m_allowedCcdPenetration = allowedPenetration;
  m_castShape = castShape;
  final btVector3 unnormalizedRayDir = m_convexToTrans.getOrigin()
   .sub(m_convexFromTrans.getOrigin());
  final btVector3 rayDir = new btVector3(unnormalizedRayDir).normalize();
  ///what about division by zero? -. just set rayDirection[i] to INF/BT_LARGE_FLOAT
  m_rayDirectionInverse.x = rayDir.x == (0.0f) ? (BT_LARGE_FLOAT) : (1.0f)
   / rayDir.x;
  m_rayDirectionInverse.y = rayDir.y == (0.0f) ? (BT_LARGE_FLOAT) : (1.0f)
   / rayDir.y;
  m_rayDirectionInverse.z = rayDir.z == (0.0f) ? (BT_LARGE_FLOAT) : (1.0f)
   / rayDir.z;
  m_signs[0] = (m_rayDirectionInverse.x < 0.0) ? 1 : 0;
  m_signs[1] = (m_rayDirectionInverse.y < 0.0) ? 1 : 0;
  m_signs[2] = (m_rayDirectionInverse.z < 0.0) ? 1 : 0;
  m_lambda_max = rayDir.dot(unnormalizedRayDir);
 }

 @Override
 public boolean process(btBroadphaseProxy proxy) {
  ///terminate further convex sweep tests, once the closestHitFraction reached zero
  if (m_resultCallback.m_closestHitFraction == (0.f)) {
   return false;
  }
  btCollisionObject collisionObject = (btCollisionObject) proxy.m_clientObject;
  //only perform raycast if filterMask matches
  if (m_resultCallback.needsCollision(collisionObject.getBroadphaseHandle())) {
   //RigidcollisionObject* collisionObject = ctrl.GetRigidcollisionObject();
   objectQuerySingle(m_castShape, m_convexFromTrans, m_convexToTrans,
    collisionObject,
    collisionObject.getCollisionShape(),
    collisionObject.getWorldTransformPtr(),
    m_resultCallback,
    m_allowedCcdPenetration);
  }
  return true;
 }

};
