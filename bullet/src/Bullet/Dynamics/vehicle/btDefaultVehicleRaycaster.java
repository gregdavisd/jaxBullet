/*
 * Copyright (c) 2005 Erwin Coumans http://continuousphysics.com/Bullet/
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
 */
package Bullet.Dynamics.vehicle;

import Bullet.Collision.ClosestRayResultCallback;
import Bullet.Dynamics.CollisionObjects.btRigidBody;
import Bullet.Dynamics.btDynamicsWorld;
import Bullet.Dynamics.vehicle.btVehicleRaycaster.btVehicleRaycasterResult;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btDefaultVehicleRaycaster extends btVehicleRaycaster implements
 Serializable {

 private static final long serialVersionUID = 1L;
 private btDynamicsWorld m_dynamicsWorld;

 public btDefaultVehicleRaycaster(btDynamicsWorld world) {
  m_dynamicsWorld = world;
 }

 public Object castRay(final btVector3 from, final btVector3 to,
  btVehicleRaycasterResult result) {
//	RayResultCallback& resultCallback;
  ClosestRayResultCallback rayCallback = new ClosestRayResultCallback(from, to);
  m_dynamicsWorld.rayTest(from, to, rayCallback);
  if (rayCallback.hasHit()) {
   btRigidBody body = btRigidBody.upcast(rayCallback.m_collisionObject);
   if (body != null && body.hasContactResponse()) {
    result.m_hitPointInWorld.set(rayCallback.m_hitPointWorld);
    result.m_hitNormalInWorld.set(rayCallback.m_hitNormalWorld);
    result.m_hitNormalInWorld.normalize();
    result.m_distFraction = rayCallback.m_closestHitFraction;
    return body;
   }
  }
  return null;
 }

}
