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

import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE;
import Bullet.LinearMath.btMatrix3x3;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.btMul;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btCapsuleShape extends btConvexInternalShape implements Serializable {

 int m_upAxis;

 ///only used for btCapsuleShapeZ and btCapsuleShapeX subclasses.
 public btCapsuleShape() {
  super();
  m_shapeType = CAPSULE_SHAPE_PROXYTYPE;
 }

 public btCapsuleShape(float radius, float height) {
  super();
  m_collisionMargin = radius;
  m_shapeType = CAPSULE_SHAPE_PROXYTYPE;
  m_upAxis = 1;
  m_implicitShapeDimensions.set(radius, 0.5f * height, radius);
 }

 ///CollisionShape Interface
 @Override
 public void calculateLocalInertia(float mass, final btVector3 inertia) {
  //as an approximation, take the inertia of the box that bounds the spheres
  final btTransform ident = new btTransform();
  ident.setIdentity();
  float radius = getRadius();
  final btVector3 halfExtents = new btVector3(radius, radius, radius);
  halfExtents.setElement(getUpAxis(), halfExtents.getElement(getUpAxis()) + getHalfHeight());
  float lx = (2.f) * halfExtents.getElement(0);
  float ly = (2.f) * halfExtents.getElement(1);
  float lz = (2.f) * halfExtents.getElement(2);
  float x2 = lx * lx;
  float y2 = ly * ly;
  float z2 = lz * lz;
  float scaledmass = mass * (.08333333f);
  inertia.x = scaledmass * (y2 + z2);
  inertia.y = scaledmass * (x2 + z2);
  inertia.z = scaledmass * (x2 + y2);
 }

 /// btConvexShape Interface
 @Override
 public btVector3 localGetSupportingVertexWithoutMargin(final btVector3 vec0) {
  final btVector3 supVec = new btVector3();
  float maxDot = -BT_LARGE_FLOAT;
  final btVector3 vec = new btVector3(vec0);
  float lenSqr = vec.lengthSquared();
  if (lenSqr < (0.0001f)) {
   vec.set(1f, 0f, 0f);
  } else {
   float rlen = (1.f) / btSqrt(lenSqr);
   vec.scale(rlen);
  }
  final btVector3 vtx = new btVector3();
  float newDot;
  {
   final btVector3 pos = new btVector3();
   pos.setElement(getUpAxis(), getHalfHeight());
   vtx.set(pos);
   newDot = vec.dot(vtx);
   if (newDot > maxDot) {
    maxDot = newDot;
    supVec.set(vtx);
   }
  }
  {
   final btVector3 pos = new btVector3();
   pos.setElement(getUpAxis(), -getHalfHeight());
   vtx.set(pos);
   newDot = vec.dot(vtx);
   if (newDot > maxDot) {
    //maxDot = newDot;
    supVec.set(vtx);
   }
  }
  return supVec;
 }

 @Override
 public void batchedUnitVectorGetSupportingVertexWithoutMargin(btVector3[] vectors,
  btVector3[] supportVerticesOut, int numVectors) {
  for (int j = 0; j < numVectors; j++) {
   float maxDot = ((-BT_LARGE_FLOAT));
   final btVector3 vec = vectors[j];
   final btVector3 vtx;
   float newDot;
   {
    final btVector3 pos = new btVector3();
    pos.setElement(getUpAxis(), getHalfHeight());
    vtx = pos;
    newDot = vec.dot(vtx);
    if (newDot > maxDot) {
     maxDot = newDot;
     if (supportVerticesOut[j] == null) {
      supportVerticesOut[j] = new btVector3();
     }
     supportVerticesOut[j].set(vtx);
    }
   }
   {
    final btVector3 pos = new btVector3();
    pos.setElement(getUpAxis(), -getHalfHeight());
    vtx.set(pos);
    newDot = vec.dot(vtx);
    if (newDot > maxDot) {
     //maxDot = newDot;			
     if (supportVerticesOut[j] == null) {
      supportVerticesOut[j] = new btVector3();
     }
     supportVerticesOut[j].set(vtx);
    }
   }
  }
 }

 @Override
 public void setMargin(float collisionMargin) {
  //don't override the margin for capsules, their entire radius == margin
 }

 @Override
 public void getAabb(final btTransform t, final btVector3 aabbMin, final btVector3 aabbMax) {
  final btVector3 halfExtents = new btVector3(getRadius(), getRadius(), getRadius());
  halfExtents.setElement(m_upAxis, getRadius() + getHalfHeight());
  final btMatrix3x3 abs_b = t.getBasis().abs();
  final btVector3 center = t.getOrigin();
  final btVector3 extent = halfExtents.dot3(abs_b.getRow(0), abs_b.getRow(1), abs_b.getRow(2));
  aabbMin.set(center).sub(extent);
  aabbMax.set(center).add(extent);
 }

 @Override
 public String getName() {
  return "CapsuleShape";
 }

 public int getUpAxis() {
  return m_upAxis;
 }

 public float getRadius() {
  int radiusAxis = (m_upAxis + 2) % 3;
  return m_implicitShapeDimensions.getElement(radiusAxis);
 }

 public float getHalfHeight() {
  return m_implicitShapeDimensions.getElement(m_upAxis);
 }

 @Override
 public void setLocalScaling(final btVector3 scaling) {
  final btVector3 unScaledImplicitShapeDimensions = new btVector3(m_implicitShapeDimensions).div(
   m_localScaling);
  super.setLocalScaling(scaling);
  m_implicitShapeDimensions.set(btMul(unScaledImplicitShapeDimensions, scaling));
  //update m_collisionMargin, since entire radius==margin
  int radiusAxis = (m_upAxis + 2) % 3;
  m_collisionMargin = m_implicitShapeDimensions.getElement(radiusAxis);
 }

 @Override
 public btVector3 getAnisotropicRollingFrictionDirection() {
  final btVector3 aniDir = new btVector3();
  aniDir.setElement(getUpAxis(), 1f);
  return aniDir;
 }

 @Override
 public int hashCode() {
  int hash = 5;
  hash += super.hashCode();
  hash = 59 * hash + this.m_upAxis;
  return hash;
 }

 @Override
 public boolean equals(Object obj) {
  if (this == obj) {
   return true;
  }
  if (obj == null) {
   return false;
  }
  if (getClass() != obj.getClass()) {
   return false;
  }
  final btCapsuleShape other = (btCapsuleShape) obj;
  if (this.m_upAxis != other.m_upAxis) {
   return false;
  }
  return super.equals(obj);
 }
}
