/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.Collision;

import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class ClosestConvexResultCallback extends ConvexResultCallback implements Serializable {

 public final btVector3 m_convexFromWorld = new btVector3();//used to calculate hitPointWorld from hitFraction
 public final btVector3 m_convexToWorld = new btVector3();
 public final btVector3 m_hitNormalWorld = new btVector3();
 public final btVector3 m_hitPointWorld = new btVector3();
 public btCollisionObject m_hitCollisionObject;

 public ClosestConvexResultCallback(final btVector3 convexFromWorld, final btVector3 convexToWorld) {
  m_convexFromWorld.set(convexFromWorld);
  m_convexToWorld.set(convexToWorld);
  m_hitCollisionObject = null;
 }

 @Override
 public float addSingleResult(LocalConvexResult convexResult, boolean normalInWorldSpace) {
//caller already does the filter on the m_closestHitFraction
  assert (convexResult.m_hitFraction <= m_closestHitFraction);
  m_closestHitFraction = convexResult.m_hitFraction;
  m_hitCollisionObject = convexResult.m_hitCollisionObject;
  if (normalInWorldSpace) {
   m_hitNormalWorld.set(convexResult.m_hitNormalLocal);
  } else {
   ///need to transform normal into worldspace
   m_hitNormalWorld.set(m_hitCollisionObject.getWorldTransformPtr().transform3x3(new btVector3(
    convexResult.m_hitNormalLocal)));
  }
  m_hitPointWorld.set(convexResult.m_hitPointLocal);
  return convexResult.m_hitFraction;
 }
}
