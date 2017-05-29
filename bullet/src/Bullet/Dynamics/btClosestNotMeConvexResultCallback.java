/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.Dynamics;

import Bullet.Collision.Broadphase.btBroadphaseProxy;
import Bullet.Collision.Broadphase.btDispatcher;
import Bullet.Collision.Broadphase.btOverlappingPairCache;
import Bullet.Collision.ClosestConvexResultCallback;
import Bullet.Collision.LocalConvexResult;
import Bullet.Collision.btCollisionObject;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btClosestNotMeConvexResultCallback extends ClosestConvexResultCallback implements
 Serializable {

 final btCollisionObject m_me;
 float m_allowedPenetration;
 final btOverlappingPairCache m_pairCache;
 final btDispatcher m_dispatcher;

 btClosestNotMeConvexResultCallback(btCollisionObject me, final btVector3 fromA, final btVector3 toA,
  btOverlappingPairCache pairCache, btDispatcher dispatcher) {
  super(fromA, toA);
  m_me = me;
  m_allowedPenetration = 0.0f;
  m_pairCache = pairCache;
  m_dispatcher = dispatcher;
 }

 @Override
 public float addSingleResult(LocalConvexResult convexResult, boolean normalInWorldSpace) {
  if (convexResult.m_hitCollisionObject == m_me) {
   return 1.0f;
  }
  //ignore result if there is no contact response
  if (!convexResult.m_hitCollisionObject.hasContactResponse()) {
   return 1.0f;
  }
  final btVector3 linVelA = new btVector3(m_convexToWorld).sub(m_convexFromWorld);
  final btVector3 linVelB = new btVector3();//toB.getOrigin()-fromB.getOrigin();
  final btVector3 relativeVelocity = new btVector3(linVelA).sub(linVelB);
  //don't report time of impact for motion away from the contact normal (or causes minor penetration)
  if (convexResult.m_hitNormalLocal.dot(relativeVelocity) >= -m_allowedPenetration) {
   return 1.f;
  }
  return super.addSingleResult(convexResult, normalInWorldSpace);
 }

 @Override
 public boolean needsCollision(btBroadphaseProxy proxy0) {
  //don't collide with itself
  if (proxy0.m_clientObject == m_me) {
   return false;
  }
  ///don't do CCD when the collision filters are not matching
  if (!super.needsCollision(proxy0)) {
   return false;
  }
  btCollisionObject otherObj = (btCollisionObject) proxy0.m_clientObject;
  //call needsResponse, see http://code.google.com/p/bullet/issues/detail?id=179
  return m_dispatcher.needsResponse(m_me, otherObj);
 }
};
