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
package Bullet.Collision.Broadphase;

import Bullet.Collision.RayResultCallback;
import Bullet.Collision.btCollisionObject;
import Bullet.Collision.btCollisionWorld;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btSingleRayCallback extends btBroadphaseRayCallback implements Serializable {

 final btVector3 m_rayFromWorld = new btVector3();
 final btVector3 m_rayToWorld = new btVector3();
 final btTransform m_rayFromTrans = new btTransform();
 final btTransform m_rayToTrans = new btTransform();
 final btVector3 m_hitNormal = new btVector3();
 final btCollisionWorld m_world;
 final RayResultCallback m_resultCallback;

 public btSingleRayCallback(final btVector3 rayFromWorld, final btVector3 rayToWorld,
  btCollisionWorld world,
  RayResultCallback resultCallback) {
  m_rayFromWorld.set(rayFromWorld);
  m_rayToWorld.set(rayToWorld);
  m_world = world;
  m_resultCallback = resultCallback;
  m_rayFromTrans.setIdentity();
  m_rayFromTrans.setOrigin(m_rayFromWorld);
  m_rayToTrans.setIdentity();
  m_rayToTrans.setOrigin(m_rayToWorld);
  final btVector3 rayDir = new btVector3(rayToWorld).sub(rayFromWorld);
  rayDir.normalize();
  ///what about division by zero? -. just set rayDirection[i] to INF/BT_LARGE_FLOAT
  m_rayDirectionInverse.x = rayDir.x == (0.0f) ? (BT_LARGE_FLOAT) : (1.0f) / rayDir.x;
  m_rayDirectionInverse.y = rayDir.y == (0.0f) ? (BT_LARGE_FLOAT) : (1.0f) / rayDir.y;
  m_rayDirectionInverse.z = rayDir.z == (0.0f) ? (BT_LARGE_FLOAT) : (1.0f) / rayDir.z;
  m_signs[0] = (m_rayDirectionInverse.x < 0.0) ? 1 : 0;
  m_signs[1] = (m_rayDirectionInverse.y < 0.0) ? 1 : 0;
  m_signs[2] = (m_rayDirectionInverse.z < 0.0) ? 1 : 0;
  m_lambda_max = rayDir.dot(new btVector3(m_rayToWorld).sub(m_rayFromWorld));
 }

 @Override
 public boolean process(btBroadphaseProxy proxy) {
  ///terminate further ray tests, once the closestHitFraction reached zero
  if (m_resultCallback.m_closestHitFraction == (0.f)) {
   return false;
  }
  btCollisionObject collisionObject = (btCollisionObject) proxy.m_clientObject;
  //only perform raycast if filterMask matches
  if (m_resultCallback.needsCollision(collisionObject.getBroadphaseHandle())) {
   {
    btCollisionWorld.rayTestSingle(m_rayFromTrans, m_rayToTrans,
     collisionObject,
     collisionObject.getCollisionShape(),
     collisionObject.getWorldTransformPtr(),
     m_resultCallback);
   }
  }
  return true;
 }
}
