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
package Bullet.Collision;

import Bullet.Collision.Shape.btConvexShape;
import Bullet.Collision.Shape.btTriangleShape;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public abstract class btTriangleConvexcastCallback implements btTriangleCallback  , Serializable {

 final btConvexShape m_convexShape;
 final btTransform m_convexShapeFrom;
 final btTransform m_convexShapeTo;
 final btTransform m_triangleToWorld;
 float m_hitFraction;
 float m_triangleCollisionMargin;
 float m_allowedPenetration;

 btTriangleConvexcastCallback(btConvexShape convexShape, final btTransform convexShapeFrom,
  final btTransform convexShapeTo, final btTransform triangleToWorld, float triangleCollisionMargin) {
  m_convexShape = convexShape;
  m_convexShapeFrom = new btTransform(convexShapeFrom);
  m_convexShapeTo = new btTransform(convexShapeTo);
  m_triangleToWorld = new btTransform(triangleToWorld);
  m_hitFraction = 1.0f;
  m_triangleCollisionMargin = triangleCollisionMargin;
  m_allowedPenetration = 0.f;
 }

 /**
  *
  * @param triangle
  * @param partId
  * @param triangleIndex
  */
 @Override
 public void processTriangle(btVector3[] triangle, int partId, int triangleIndex) {
  btTriangleShape triangleShape = new btTriangleShape(triangle[0], triangle[1], triangle[2]);
  triangleShape.setMargin(m_triangleCollisionMargin);
  btVoronoiSimplexSolver simplexSolver = new btVoronoiSimplexSolver();
  btGjkEpaPenetrationDepthSolver gjkEpaPenetrationSolver = new btGjkEpaPenetrationDepthSolver();
  //btGjkConvexCast	convexCaster(m_convexShape,&triangleShape,&simplexSolver);
  btContinuousConvexCollision convexCaster = new btContinuousConvexCollision(m_convexShape,
   triangleShape, simplexSolver, gjkEpaPenetrationSolver);
  btConvexCast.CastResult castResult = new btConvexCast.CastResult();
  castResult.m_fraction = (1.f);
  castResult.m_allowedPenetration = m_allowedPenetration;
  if (convexCaster.calcTimeOfImpact(m_convexShapeFrom, m_convexShapeTo, m_triangleToWorld,
   m_triangleToWorld, castResult)) {
   //add hit
   if (castResult.m_normal.lengthSquared() > (0.0001f)) {
    if (castResult.m_fraction < m_hitFraction) {
     castResult.m_normal.normalize();
     reportHit(castResult.m_normal,
      castResult.m_hitPoint,
      castResult.m_fraction,
      partId,
      triangleIndex);
    }
   }
  }
 }

 abstract float reportHit(final btVector3 hitNormalLocal, final btVector3 hitPointLocal,
  float hitFraction,
  int partId, int triangleIndex);
}
