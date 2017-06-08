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
package Bullet.Dynamics.character;

import Bullet.Collision.ClosestConvexResultCallback;
import Bullet.Collision.LocalConvexResult;
import Bullet.Collision.btCollisionObject;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btKinematicClosestNotMeConvexResultCallback extends ClosestConvexResultCallback
 implements Serializable {

 public btKinematicClosestNotMeConvexResultCallback(btCollisionObject me, final btVector3 up,
  float minSlopeDot) {
  super(new btVector3(), new btVector3());
  m_me = me;
  m_up.set(up);
  m_minSlopeDot = minSlopeDot;
 }

 @Override
 public float addSingleResult(LocalConvexResult convexResult, boolean normalInWorldSpace) {
  if (convexResult.m_hitCollisionObject == m_me) {
   return (1.0f);
  }
  if (!convexResult.m_hitCollisionObject.hasContactResponse()) {
   return (1.0f);
  }
  final btVector3 hitNormalWorld = new btVector3();
  if (normalInWorldSpace) {
   hitNormalWorld.set(convexResult.m_hitNormalLocal);
  } else {
   ///need to transform normal into worldspace
   convexResult.m_hitCollisionObject.getWorldTransform().transform3x3(hitNormalWorld.set(
    convexResult.m_hitNormalLocal));
  }
  float dotUp = m_up.dot(hitNormalWorld);
  if (dotUp < m_minSlopeDot) {
   return (1.0f);
  }
  return super.addSingleResult(convexResult, normalInWorldSpace);
 }
 protected btCollisionObject m_me;
 protected final btVector3 m_up = new btVector3();
 protected float m_minSlopeDot;
}
