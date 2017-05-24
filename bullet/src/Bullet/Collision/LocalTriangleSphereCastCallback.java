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

import Bullet.Collision.Shape.btSphereShape;
import Bullet.Collision.Shape.btTriangleShape;
import static Bullet.LinearMath.btQuickprof.BT_PROFILE;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class LocalTriangleSphereCastCallback implements btTriangleCallback  , Serializable  {

 public final btTransform m_ccdSphereFromTrans = new btTransform();
 public final btTransform m_ccdSphereToTrans = new btTransform();
 public final btTransform m_meshTransform = new btTransform();
 public float m_ccdSphereRadius;
 public float m_hitFraction;

 public LocalTriangleSphereCastCallback(final btTransform from, final btTransform to, float ccdSphereRadius,
  float hitFraction) {
  m_ccdSphereFromTrans.set(from);
  m_ccdSphereToTrans.set(to);
  m_ccdSphereRadius = ccdSphereRadius;
  m_hitFraction = hitFraction;
 }

 @Override
 public void processTriangle(btVector3[] triangle, int partId, int triangleIndex) {
  BT_PROFILE("processTriangle");
  //do a swept sphere for now
  final btTransform ident = new btTransform();
  ident.setIdentity();
  btConvexCast.CastResult castResult = new btConvexCast.CastResult();
  castResult.m_fraction = m_hitFraction;
  btSphereShape pointShape = new btSphereShape(m_ccdSphereRadius);
  btTriangleShape triShape = new btTriangleShape(triangle[0], triangle[1], triangle[2]);
  btVoronoiSimplexSolver simplexSolver = new btVoronoiSimplexSolver();
  btSubsimplexConvexCast convexCaster = new btSubsimplexConvexCast(pointShape, triShape,
   simplexSolver);
  if (convexCaster.calcTimeOfImpact(m_ccdSphereFromTrans, m_ccdSphereToTrans,
   ident, ident, castResult)) {
   if (m_hitFraction > castResult.m_fraction) {
    m_hitFraction = castResult.m_fraction;
   }
  }
 }
}
