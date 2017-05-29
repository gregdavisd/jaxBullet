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

import Bullet.Collision.Shape.btConvexShape;
import Bullet.Collision.Shape.btTriangleMeshShape;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;
//ConvexCast.CastResult

/**
 *
 * @author Gregery Barton
 */
public class BridgeTriangleConvexcastCallbackA extends btTriangleConvexcastCallback implements
 Serializable {

 final ConvexResultCallback m_resultCallback;
 final btCollisionObject m_collisionObject;
 final btTriangleMeshShape m_triangleMesh;

 BridgeTriangleConvexcastCallbackA(btConvexShape castShape, final btTransform from,
  final btTransform to,
  ConvexResultCallback resultCallback, btCollisionObject collisionObject,
  btTriangleMeshShape triangleMesh, final btTransform triangleToWorld) {
  super(castShape, from, to, triangleToWorld, triangleMesh.getMargin());
  m_resultCallback = resultCallback;
  m_collisionObject = collisionObject;
  m_triangleMesh = triangleMesh;
 }

 @Override
 float reportHit(final btVector3 hitNormalLocal, final btVector3 hitPointLocal,
  float hitFraction, int partId, int triangleIndex
 ) {
  LocalShapeInfo shapeInfo = new LocalShapeInfo();
  shapeInfo.m_shapePart = partId;
  shapeInfo.m_triangleIndex = triangleIndex;
  if (hitFraction <= m_resultCallback.m_closestHitFraction) {
   LocalConvexResult convexResult = new LocalConvexResult(m_collisionObject,
    shapeInfo,
    hitNormalLocal,
    hitPointLocal,
    hitFraction
   );
   boolean normalInWorldSpace = true;
   return m_resultCallback.addSingleResult(convexResult, normalInWorldSpace);
  }
  return hitFraction;
 }
}
