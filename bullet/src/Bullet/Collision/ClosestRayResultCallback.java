/*
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org
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
package Bullet.Collision;

import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class ClosestRayResultCallback extends RayResultCallback implements
 Serializable {

 public final btVector3 m_rayFromWorld = new btVector3();//used to calculate hitPointWorld from hitFraction
 public final btVector3 m_rayToWorld = new btVector3();
 public final btVector3 m_hitNormalWorld = new btVector3();
 public final btVector3 m_hitPointWorld = new btVector3();

 public ClosestRayResultCallback(final btVector3 rayFromWorld,
  final btVector3 rayToWorld) {
  m_rayFromWorld.set(rayFromWorld);
  m_rayToWorld.set(rayToWorld);
 }

 @Override
 public float addSingleResult(LocalRayResult rayResult,
  boolean normalInWorldSpace
 ) {
  //caller already does the filter on the m_closestHitFraction
  assert (rayResult.m_hitFraction <= m_closestHitFraction);
  m_closestHitFraction = rayResult.m_hitFraction;
  m_collisionObject = rayResult.m_collisionObject;
  if (normalInWorldSpace) {
   m_hitNormalWorld.set(rayResult.m_hitNormalLocal);
  } else {
   ///need to transform normal into worldspace
   m_hitNormalWorld.set(m_collisionObject.getWorldTransformPtr().transform3x3(
    new btVector3(
     rayResult.m_hitNormalLocal)));
  }
  m_hitPointWorld.setInterpolate3(m_rayFromWorld, m_rayToWorld,
   rayResult.m_hitFraction);
  return rayResult.m_hitFraction;
 }

}
