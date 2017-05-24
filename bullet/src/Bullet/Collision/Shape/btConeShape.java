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

package Bullet.Collision.Shape;

import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.CONE_SHAPE_PROXYTYPE;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btConeShape extends btConvexInternalShape  implements Serializable {

 float m_sinAngle;
 float m_radius;
 float m_height;
 final int[] m_coneIndices = new int[3];

public  btVector3 coneLocalSupport(final btVector3 v) {
  float halfHeight = m_height * (0.5f);
  if (v.getElement(m_coneIndices[1]) > v.length() * m_sinAngle) {
   final btVector3 tmp = new btVector3();
   tmp.setElement(m_coneIndices[0], 0);
   tmp.setElement(m_coneIndices[1], halfHeight);
   tmp.setElement(m_coneIndices[2], 0);
   return tmp;
  } else {
   float s = btSqrt(v.getElement(m_coneIndices[0]) * v.getElement(m_coneIndices[0]) + v.getElement(
    m_coneIndices[2]) * v.getElement(m_coneIndices[2]));
   if (s > SIMD_EPSILON) {
    float d = m_radius / s;
    final btVector3 tmp = new btVector3();
    tmp.setElement(m_coneIndices[0], v.getElement(m_coneIndices[0]) * d);
    tmp.setElement(m_coneIndices[1], -halfHeight);
    tmp.setElement(m_coneIndices[2], v.getElement(m_coneIndices[2]) * d);
    return tmp;
   } else {
    final btVector3 tmp = new btVector3();
    tmp.setElement(m_coneIndices[0], 0);
    tmp.setElement(m_coneIndices[1], -halfHeight);
    tmp.setElement(m_coneIndices[2], 0);
    return tmp;
   }
  }
 }

 public btConeShape(float radius, float height) {
  super();
  m_radius = radius;
  m_height = height;
  m_shapeType = CONE_SHAPE_PROXYTYPE;
  setConeUpIndex(1);
  //btVector3 halfExtents;
  m_sinAngle = (m_radius / btSqrt(m_radius * m_radius + m_height * m_height));
 }

 @Override
 public btVector3 localGetSupportingVertex(final btVector3 vec) {
  final btVector3 supVertex = coneLocalSupport(vec);
  if (getMargin() != (0.f)) {
   final btVector3 vecnorm = new btVector3(vec);
   if (vecnorm.lengthSquared() < (SIMD_EPSILON * SIMD_EPSILON)) {
    vecnorm.set((-1.f), (-1.f), (-1.f));
   }
   vecnorm.normalize();
   supVertex.add(vecnorm.scale(getMargin()));
  }
  return supVertex;
 }

 @Override
 public btVector3 localGetSupportingVertexWithoutMargin(final btVector3 vec) {
  return coneLocalSupport(vec);
 }

 @Override
 public void batchedUnitVectorGetSupportingVertexWithoutMargin(btVector3[] vectors,
  btVector3[] supportVerticesOut, int numVectors) {
  for (int i = 0; i < numVectors; i++) {
   final btVector3 vec = vectors[i];
   if (supportVerticesOut[i] == null) {
    supportVerticesOut[i] = new btVector3();
   }
   supportVerticesOut[i].set( coneLocalSupport(vec));
  }
 }

 public float getRadius() {
  return m_radius;
 }

 public float getHeight() {
  return m_height;
 }

 public void setRadius(float radius) {
  m_radius = radius;
 }

 public void setHeight(float height) {
  m_height = height;
 }

 @Override
 public void calculateLocalInertia(float mass, final btVector3 inertia) {
  final btTransform identity = new btTransform();
  identity.setIdentity();
  final btVector3 aabbMin = new btVector3();
  final btVector3 aabbMax = new btVector3();
  getAabb(identity, aabbMin, aabbMax);
  final btVector3 halfExtents = new btVector3(aabbMax).sub(aabbMin).scale(0.5f);
  float margin = getMargin();
  float lx = (2.f) * (halfExtents.x() + margin);
  float ly = (2.f) * (halfExtents.y() + margin);
  float lz = (2.f) * (halfExtents.z() + margin);
  float x2 = lx * lx;
  float y2 = ly * ly;
  float z2 = lz * lz;
  float scaledmass = mass * (0.08333333f);
  inertia.set(y2 + z2, x2 + z2, x2 + y2).scale(scaledmass);
 }

 @Override
 public String getName() {
  return "Cone";
 }

 ///choose upAxis index
 public final void setConeUpIndex(int upIndex) {
  switch (upIndex) {
   case 0:
    m_coneIndices[0] = 1;
    m_coneIndices[1] = 0;
    m_coneIndices[2] = 2;
    break;
   case 1:
    m_coneIndices[0] = 0;
    m_coneIndices[1] = 1;
    m_coneIndices[2] = 2;
    break;
   case 2:
    m_coneIndices[0] = 0;
    m_coneIndices[1] = 2;
    m_coneIndices[2] = 1;
    break;
   default:
    assert(false);
  }
  m_implicitShapeDimensions.setElement(m_coneIndices[0], m_radius);
  m_implicitShapeDimensions.setElement(m_coneIndices[1], m_height);
  m_implicitShapeDimensions.setElement(m_coneIndices[2], m_radius);
 }

 public int getConeUpIndex() {
  return m_coneIndices[1];
 }

 @Override
 public btVector3 getAnisotropicRollingFrictionDirection() {
  return new btVector3(0, 1, 0);
 }

 @Override
 public void setLocalScaling(final btVector3 scaling) {
  int axis = m_coneIndices[1];
  int r1 = m_coneIndices[0];
  int r2 = m_coneIndices[2];
  m_height *= scaling.getElement(axis) / m_localScaling.getElement(axis);
  m_radius *= (scaling.getElement(r1) / m_localScaling.getElement(r1) + scaling.getElement(r2) /
   m_localScaling.getElement(r2)) / 2;
  m_sinAngle = (m_radius / btSqrt(m_radius * m_radius + m_height * m_height));
  super.setLocalScaling(scaling);
 }
}
