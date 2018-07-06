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

import Bullet.Collision.Shape.btConcaveShape;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;
//ConvexCast.CastResult

/**
 *
 * @author Gregery Barton
 */
public class BridgeTriangleRaycastCallback extends btTriangleRaycastCallback
 implements Serializable {

 final RayResultCallback m_resultCallback;
 final btCollisionObject m_collisionObject;
 final btTransform m_colObjWorldTransform = new btTransform();

 BridgeTriangleRaycastCallback(final btVector3 from, final btVector3 to,
  RayResultCallback resultCallback, btCollisionObject collisionObject,
  btConcaveShape triangleMesh,
  final btTransform colObjWorldTransform) {
  //@BP Mod
  super(from, to, resultCallback.m_flags);
  m_resultCallback = resultCallback;
  m_collisionObject = collisionObject;
  m_colObjWorldTransform.set(colObjWorldTransform);
 }

 @Override
 float reportHit(final btVector3 hitNormalLocal,
  float hitFraction, int partId, int triangleIndex
 ) {
  LocalShapeInfo shapeInfo = new LocalShapeInfo();
  shapeInfo.m_shapePart = partId;
  shapeInfo.m_triangleIndex = triangleIndex;
  final btVector3 hitNormalWorld = m_colObjWorldTransform
   .transform3x3(new btVector3(hitNormalLocal));
  LocalRayResult rayResult = new LocalRayResult(m_collisionObject,
   shapeInfo,
   hitNormalWorld,
   hitFraction
  );
  boolean normalInWorldSpace = true;
  return m_resultCallback.addSingleResult(rayResult, normalInWorldSpace);
 }

};
