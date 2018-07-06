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

import Bullet.Collision.Algorithm.btCollisionAlgorithm;
import Bullet.Collision.ContactResultCallback;
import Bullet.Collision.btBridgedManifoldResult;
import Bullet.Collision.btCollisionObject;
import Bullet.Collision.btCollisionObjectWrapper;
import Bullet.Collision.btCollisionWorld;
import static Bullet.Collision.ebtDispatcherQueryType.BT_CLOSEST_POINT_ALGORITHMS;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btSingleContactCallback implements btBroadphaseAabbCallback,
 Serializable {

 private static final long serialVersionUID = 1L;
 final btCollisionObject m_collisionObject;
 final btCollisionWorld m_world;
 public final ContactResultCallback m_resultCallback;

 public btSingleContactCallback(btCollisionObject collisionObject,
  btCollisionWorld world,
  ContactResultCallback resultCallback) {
  m_collisionObject = collisionObject;
  m_world = world;
  m_resultCallback = resultCallback;
 }

 @Override
 public boolean process(btBroadphaseProxy proxy) {
  btCollisionObject collisionObject = (btCollisionObject) proxy.m_clientObject;
  if (collisionObject == m_collisionObject) {
   return true;
  }
  //only perform raycast if filterMask matches
  if (m_resultCallback.needsCollision(collisionObject.getBroadphaseHandle())) {
   btCollisionObjectWrapper ob0 = new btCollisionObjectWrapper(null,
    m_collisionObject
     .getCollisionShape(), m_collisionObject, m_collisionObject
     .getWorldTransformPtr(), -1, -1);
   btCollisionObjectWrapper ob1 = new btCollisionObjectWrapper(null,
    collisionObject
     .getCollisionShape(), collisionObject, collisionObject
     .getWorldTransformPtr(), -1, -1);
   btCollisionAlgorithm algorithm = m_world.getDispatcher().findAlgorithm(ob0,
    ob1, null,
    BT_CLOSEST_POINT_ALGORITHMS);
   if (algorithm != null) {
    btBridgedManifoldResult contactPointResult = new btBridgedManifoldResult(ob0,
     ob1,
     m_resultCallback);
    //discrete collision detection query
    algorithm.processCollision(ob0, ob1, m_world.getDispatchInfo(),
     contactPointResult);
   }
  }
  return true;
 }

};
