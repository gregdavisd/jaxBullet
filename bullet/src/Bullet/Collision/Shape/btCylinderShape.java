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

import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE;
import static Bullet.LinearMath.btAabbUtil2.btTransformAabb;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import static Bullet.common.IFDEF.USE_BOX_INERTIA_APPROXIMATION;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btCylinderShape extends btConvexInternalShape implements Serializable {

 int m_upAxis;

 public btVector3 getHalfExtentsWithMargin() {
  final btVector3 halfExtents = getHalfExtentsWithoutMargin();
  final btVector3 margin = new btVector3(getMargin(), getMargin(), getMargin());
  halfExtents.add(margin);
  return halfExtents;
 }

 public btVector3 getHalfExtentsWithoutMargin() {
  return new btVector3(m_implicitShapeDimensions);//changed in Bullet 2.63: assume the scaling and margin are included
 }

 public btCylinderShape(final btVector3 halfExtents) {
  super();
  m_upAxis = 1;
  setSafeMargin(halfExtents);
  final btVector3 margin = new btVector3(m_collisionMargin, m_collisionMargin, m_collisionMargin);
  m_implicitShapeDimensions.set(new btVector3(halfExtents).mul(m_localScaling).sub(margin));
  m_shapeType = CYLINDER_SHAPE_PROXYTYPE;
 }

 @Override
 public void getAabb(final btTransform t, final btVector3 aabbMin, final btVector3 aabbMax) {
  btTransformAabb(getHalfExtentsWithoutMargin(), getMargin(), t, aabbMin, aabbMax);
 }

 @Override
 public void calculateLocalInertia(float mass, final btVector3 inertia) {
//Until Bullet 2.77 a box approximation was used, so uncomment this if you need backwards compatibility
//#define USE_BOX_INERTIA_APPROXIMATION 1
  if (!USE_BOX_INERTIA_APPROXIMATION) {

   /*
	cylinder is defined as following:
	*
	* - principle axis aligned along y by default, radius in x, z-value not used
	* - for btCylinderShapeX: principle axis aligned along x, radius in y direction, z-value not used
	* - for btCylinderShapeZ: principle axis aligned along z, radius in x direction, y-value not used
	*
    */
   float radius2;	// square of cylinder radius
   float height2;	// square of cylinder height
   final btVector3 halfExtents = getHalfExtentsWithMargin();	// get cylinder dimension
   float div12 = mass / 12.f;
   float div4 = mass / 4.f;
   float div2 = mass / 2.f;
   int idxRadius, idxHeight;
   switch (m_upAxis) // get indices of radius and height of cylinder
   {
    case 0: // cylinder is aligned along x
     idxRadius = 1;
     idxHeight = 0;
     break;
    case 2: // cylinder is aligned along z
     idxRadius = 0;
     idxHeight = 2;
     break;
    default: // cylinder is aligned along y
     idxRadius = 0;
     idxHeight = 1;
   }
   // calculate squares
   radius2 = halfExtents.getElement(idxRadius) * halfExtents.getElement(idxRadius);
   height2 = (4.f) * halfExtents.getElement(idxHeight) * halfExtents.getElement(idxHeight);
   // calculate tensor terms
   float t1 = div12 * height2 + div4 * radius2;
   float t2 = div2 * radius2;
   switch (m_upAxis) // set diagonal elements of inertia tensor
   {
    case 0: // cylinder is aligned along x
     inertia.set(t2, t1, t1);
     break;
    case 2: // cylinder is aligned along z
     inertia.set(t1, t1, t2);
     break;
    default: // cylinder is aligned along y
     inertia.set(t1, t2, t1);
   }
  } else //USE_BOX_INERTIA_APPROXIMATION
  {
   //approximation of box shape
   final btVector3 halfExtents = getHalfExtentsWithMargin();
   float lx = (2.f) * (halfExtents.x());
   float ly = (2.f) * (halfExtents.y());
   float lz = (2.f) * (halfExtents.z());
   inertia.set(mass / ((12.0f)) * (ly * ly + lz * lz),
    mass / ((12.0f)) * (lx * lx + lz * lz),
    mass / ((12.0f)) * (lx * lx + ly * ly));
  }
 }

 @Override
 public btVector3 localGetSupportingVertexWithoutMargin(final btVector3 vec) {
  return CylinderLocalSupportY(getHalfExtentsWithoutMargin(), vec);
 }

 @Override
 public void batchedUnitVectorGetSupportingVertexWithoutMargin(btVector3[] vectors,
  btVector3[] supportVerticesOut, int numVectors) {
  for (int i = 0; i < numVectors; i++) {
   if (supportVerticesOut[i] == null) {
    supportVerticesOut[i] = new btVector3();
   }
   supportVerticesOut[i].set(CylinderLocalSupportY(getHalfExtentsWithoutMargin(), vectors[i]));
  }
 }

 @Override
 public void setMargin(float collisionMargin) {
  //correct the m_implicitShapeDimensions for the margin
  final btVector3 oldMargin = new btVector3(getMargin(), getMargin(), getMargin());
  final btVector3 implicitShapeDimensionsWithMargin = new btVector3(m_implicitShapeDimensions).add(
   oldMargin);
  super.setMargin(collisionMargin);
  final btVector3 newMargin = new btVector3(getMargin(), getMargin(), getMargin());
  m_implicitShapeDimensions.set(implicitShapeDimensionsWithMargin).sub(newMargin);
 }

 @Override
 public btVector3 localGetSupportingVertex(final btVector3 vec) {
  final btVector3 supVertex = localGetSupportingVertexWithoutMargin(vec);
  if (getMargin() != (0.f)) {
   final btVector3 vecnorm = new btVector3(vec);
   if (vecnorm.lengthSquared() < (SIMD_EPSILON * SIMD_EPSILON)) {
    vecnorm.set((-1.f), (-1.f), (-1.f));
   }
   vecnorm.normalize();
   supVertex.add((vecnorm).scale(getMargin()));
  }
  return supVertex;
 }

 //use box inertia
 //	  void	calculateLocalInertia(float mass,btVector3& inertia)  ;
 public int getUpAxis() {
  return m_upAxis;
 }

 @Override
 public btVector3 getAnisotropicRollingFrictionDirection() {
  final btVector3 aniDir = new btVector3();
  aniDir.setElement(getUpAxis(), 1f);
  return aniDir;
 }

 public float getRadius() {
  return getHalfExtentsWithMargin().getX();
 }

 @Override
 public void setLocalScaling(final btVector3 scaling) {
  final btVector3 oldMargin = new btVector3(getMargin(), getMargin(), getMargin());
  final btVector3 implicitShapeDimensionsWithMargin = new btVector3(m_implicitShapeDimensions).add(
   oldMargin);
  final btVector3 unScaledImplicitShapeDimensionsWithMargin = new btVector3(
   implicitShapeDimensionsWithMargin).div(m_localScaling);
  super.setLocalScaling(scaling);
  m_implicitShapeDimensions.set(unScaledImplicitShapeDimensionsWithMargin).mul(m_localScaling).sub(
   oldMargin);
 }

 //debugging
 @Override
 public String getName() {
  return "CylinderY";
 }

 static btVector3 CylinderLocalSupportX(final btVector3 halfExtents, final btVector3 v) {
  int cylinderUpAxis = 0;
  int XX = 1;
  int YY = 0;
  int ZZ = 2;
  //mapping depends on how cylinder local orientation is
  // extents of the cylinder is: X,Y is for radius, and Z for height
  float radius = halfExtents.getElement(XX);
  float halfHeight = halfExtents.getElement(cylinderUpAxis);
  final btVector3 tmp = new btVector3();
  float d;
  float s = btSqrt(v.getElement(XX) * v.getElement(XX) + v.getElement(ZZ) * v.getElement(ZZ));
  if (s != (0.0f)) {
   d = radius / s;
   tmp.setElement(XX, v.getElement(XX) * d);
   tmp.setElement(YY, v.getElement(YY) < 0.0 ? -halfHeight : halfHeight);
   tmp.setElement(ZZ, v.getElement(ZZ) * d);
   return tmp;
  } else {
   tmp.setElement(XX, radius);
   tmp.setElement(YY, v.getElement(YY) < 0.0 ? -halfHeight : halfHeight);
   tmp.setElement(ZZ, (0.0f));
   return tmp;
  }
 }

 static btVector3 CylinderLocalSupportY(final btVector3 halfExtents, final btVector3 v) {
  int cylinderUpAxis = 1;
  int XX = 0;
  int YY = 1;
  int ZZ = 2;
  float radius = halfExtents.getElement(XX);
  float halfHeight = halfExtents.getElement(cylinderUpAxis);
  final btVector3 tmp = new btVector3();
  float d;
  float s = btSqrt(v.getElement(XX) * v.getElement(XX) + v.getElement(ZZ) * v.getElement(ZZ));
  if (s != (0.0f)) {
   d = radius / s;
   tmp.setElement(XX, v.getElement(XX) * d);
   tmp.setElement(YY, v.getElement(YY) < 0.0 ? -halfHeight : halfHeight);
   tmp.setElement(ZZ, v.getElement(ZZ) * d);
   return tmp;
  } else {
   tmp.setElement(XX, radius);
   tmp.setElement(YY, v.getElement(YY) < 0.0 ? -halfHeight : halfHeight);
   tmp.setElement(ZZ, 0.0f);
   return tmp;
  }
 }

 public btVector3 CylinderLocalSupportZ(final btVector3 halfExtents, final btVector3 v) {
  final int cylinderUpAxis = 2;
  final int XX = 0;
  final int YY = 2;
  final int ZZ = 1;
  //mapping depends on how cylinder local orientation is
  // extents of the cylinder is: X,Y is for radius, and Z for height
  float radius = halfExtents.getElement(XX);
  float halfHeight = halfExtents.getElement(cylinderUpAxis);
  final btVector3 tmp = new btVector3();
  float d;
  float s = btSqrt(v.getElement(XX) * v.getElement(XX) + v.getElement(ZZ) * v.getElement(ZZ));
  if (s != (0.0f)) {
   d = radius / s;
   tmp.setElement(XX, v.getElement(XX) * d);
   tmp.setElement(YY, v.getElement(YY) < 0.0f ? -halfHeight : halfHeight);
   tmp.setElement(ZZ, v.getElement(ZZ) * d);
   return tmp;
  } else {
   tmp.setElement(XX, radius);
   tmp.setElement(YY, v.getElement(YY) < 0.0 ? -halfHeight : halfHeight);
   tmp.setElement(ZZ, 0.0f);
   return tmp;
  }
 }

 @Override
 public int hashCode() {
  int hash = 7;
  hash += super.hashCode();
  hash = 29 * hash + this.m_upAxis;
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
  final btCylinderShape other = (btCylinderShape) obj;
  if (this.m_upAxis != other.m_upAxis) {
   return false;
  }
  return super.equals(obj);
 }
}
