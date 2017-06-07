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

import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.CONE_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.CONVEX_HULL_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE;
import Bullet.LinearMath.btMatrix3x3;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.btFsels;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public abstract class btConvexShape extends btCollisionShape implements Serializable {

 public static final int MAX_PREFERRED_PENETRATION_DIRECTIONS = 10;

 static btVector3 convexHullSupport(final btVector3 localDirOrg, btVector3[] points, int numPoints,
  final btVector3 localScaling) {
  final btVector3 vec = new btVector3(localDirOrg).mul(localScaling);
  float[] maxDot = new float[1];
  int ptIndex = vec.maxDot(points, numPoints, maxDot);
  assert (ptIndex >= 0);
  final btVector3 supVec = new btVector3(points[ptIndex]).mul(localScaling);
  return supVec;
 }

 public btConvexShape() {
 }

 public abstract btVector3 localGetSupportingVertex(final btVector3 vec);

 public abstract btVector3 localGetSupportingVertexWithoutMargin(final btVector3 vec);

 public btVector3 localGetSupportVertexWithoutMarginNonVirtual(final btVector3 localDir) {
  switch (m_shapeType) {
   case SPHERE_SHAPE_PROXYTYPE: {
    return new btVector3();
   }
   case BOX_SHAPE_PROXYTYPE: {
    btBoxShape convexShape = (btBoxShape) this;
    final btVector3 halfExtents = convexShape.getImplicitShapeDimensions();
    return new btVector3(btFsels(localDir.x(), halfExtents.x(), -halfExtents.x()),
     btFsels(localDir.y(), halfExtents.y(), -halfExtents.y()),
     btFsels(localDir.z(), halfExtents.z(), -halfExtents.z()));
   }
   case TRIANGLE_SHAPE_PROXYTYPE: {
    btTriangleShape triangleShape = (btTriangleShape) this;
    final btVector3 dir = new btVector3(localDir.getX(), localDir.getY(), localDir.getZ());
    btVector3[] vertices = triangleShape.m_vertices1;
    final btVector3 dots = dir.dot3(vertices[0], vertices[1], vertices[2]);
    final btVector3 sup = vertices[dots.maxAxis()];
    return new btVector3(sup.getX(), sup.getY(), sup.getZ());
   }
   case CYLINDER_SHAPE_PROXYTYPE: {
    btCylinderShape cylShape = (btCylinderShape) this;
    //mapping of halfextents/dimension onto radius/height depends on how cylinder local orientation is (upAxis)
    final btVector3 halfExtents = cylShape.getImplicitShapeDimensions();
    final btVector3 v = new btVector3(localDir.getX(), localDir.getY(), localDir.getZ());
    int cylinderUpAxis = cylShape.getUpAxis();
    int XX = 1, YY = 0, ZZ = 2;
    switch (cylinderUpAxis) {
     case 0: {
      XX = 1;
      YY = 0;
      ZZ = 2;
     }
     break;
     case 1: {
      XX = 0;
      YY = 1;
      ZZ = 2;
     }
     break;
     case 2: {
      XX = 0;
      YY = 2;
      ZZ = 1;
     }
     break;
     default:
      assert (false);
      break;
    }
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
     return new btVector3(tmp.getX(), tmp.getY(), tmp.getZ());
    } else {
     tmp.setElement(XX, radius);
     tmp.setElement(YY, v.getElement(YY) < 0.0 ? -halfHeight : halfHeight);
     tmp.setElement(ZZ, 0.0f);
     return new btVector3(tmp.getX(), tmp.getY(), tmp.getZ());
    }
   }
   case CAPSULE_SHAPE_PROXYTYPE: {
    btCapsuleShape capsuleShape = (btCapsuleShape) this;
    float halfHeight = capsuleShape.getHalfHeight();
    int capsuleUpAxis = capsuleShape.getUpAxis();
    final btVector3 supVec = new btVector3();
    float maxDot = ((-BT_LARGE_FLOAT));
    final btVector3 vec = new btVector3(localDir.getX(), localDir.getY(), localDir.getZ());
    float lenSqr = vec.lengthSquared();
    if (lenSqr < SIMD_EPSILON * SIMD_EPSILON) {
     vec.set(1, 0, 0);
    } else {
     float rlen = (1.f) / btSqrt(lenSqr);
     vec.scale(rlen);
    }
    final btVector3 vtx = new btVector3();
    float newDot;
    {
     final btVector3 pos = new btVector3();
     pos.setElement(capsuleUpAxis, halfHeight);
     vtx.set(pos);
     newDot = vec.dot(vtx);
     if (newDot > maxDot) {
      maxDot = newDot;
      supVec.set(vtx);
     }
    }
    {
     final btVector3 pos = new btVector3();
     pos.setElement(capsuleUpAxis, -halfHeight);
     vtx.set(pos);
     newDot = vec.dot(vtx);
     if (newDot > maxDot) {
      //maxDot = newDot;
      supVec.set(vtx);
     }
    }
    return new btVector3(supVec);
   }
   case CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE: {
    btConvexPointCloudShape convexPointCloudShape = (btConvexPointCloudShape) this;
    btVector3[] points = convexPointCloudShape.getUnscaledPoints();
    int numPoints = convexPointCloudShape.getNumPoints();
    return convexHullSupport(localDir, points, numPoints, convexPointCloudShape.getLocalScalingNV());
   }
   case CONVEX_HULL_SHAPE_PROXYTYPE: {
    btConvexHullShape convexHullShape = (btConvexHullShape) this;
    btVector3[] points = convexHullShape.getUnscaledPoints();
    int numPoints = convexHullShape.getNumPoints();
    return convexHullSupport(localDir, points, numPoints, convexHullShape.getLocalScalingNV());
   }
   default:
    return this.localGetSupportingVertexWithoutMargin(localDir);
  }
  // should never reach here
  //assert(false);
  //return new btVector3((0.0f), (0.0f), (0.0f));
 }

 public final btVector3 localGetSupportVertexNonVirtual(final btVector3 localDir) {
  final btVector3 localDirNorm = new btVector3(localDir);
  if (localDirNorm.lengthSquared() < (SIMD_EPSILON * SIMD_EPSILON)) {
   localDirNorm.set((-1.f), (-1.f), (-1.f));
  }
  localDirNorm.normalize();
  return localGetSupportVertexWithoutMarginNonVirtual(localDirNorm)
   .add(localDirNorm
    .scale(getMarginNonVirtual()));
 }

 public final float getMarginNonVirtual() {
  switch (m_shapeType) {
   case SPHERE_SHAPE_PROXYTYPE: {
    btSphereShape sphereShape = (btSphereShape) this;
    return sphereShape.getRadius();
   }
   case BOX_SHAPE_PROXYTYPE: {
    btBoxShape convexShape = (btBoxShape) this;
    return convexShape.getMarginNV();
   }
   case TRIANGLE_SHAPE_PROXYTYPE: {
    btTriangleShape triangleShape = (btTriangleShape) this;
    return triangleShape.getMarginNV();
   }
   case CYLINDER_SHAPE_PROXYTYPE: {
    btCylinderShape cylShape = (btCylinderShape) this;
    return cylShape.getMarginNV();
   }
   case CONE_SHAPE_PROXYTYPE: {
    btConeShape conShape = (btConeShape) this;
    return conShape.getMarginNV();
   }
   case CAPSULE_SHAPE_PROXYTYPE: {
    btCapsuleShape capsuleShape = (btCapsuleShape) this;
    return capsuleShape.getMarginNV();
   }
   case CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE:
   /* fall through */
   case CONVEX_HULL_SHAPE_PROXYTYPE: {
    btPolyhedralConvexShape convexHullShape = (btPolyhedralConvexShape) this;
    return convexHullShape.getMarginNV();
   }
   default:
    return this.getMargin();
  }
  // should never reach here
  //assert(false);
  //return (0.0f);
 }

 public final void getAabbNonVirtual(final btTransform t, final btVector3 aabbMin,
  final btVector3 aabbMax) {
  switch (m_shapeType) {
   case SPHERE_SHAPE_PROXYTYPE: {
    btSphereShape sphereShape = (btSphereShape) this;
    float radius = sphereShape.getImplicitShapeDimensions().getX();// * convexShape.getLocalScaling().getX();
    float margin = radius + sphereShape.getMarginNonVirtual();
    final btVector3 center = t.getOrigin();
    final btVector3 extent = new btVector3(margin, margin, margin);
    aabbMin.set(center).sub(extent);
    aabbMax.set(center).add(extent);
   }
   break;
   case CYLINDER_SHAPE_PROXYTYPE:
   /* fall through */
   case BOX_SHAPE_PROXYTYPE: {
    btBoxShape convexShape = (btBoxShape) this;
    float margin = convexShape.getMarginNonVirtual();
    final btVector3 halfExtents = convexShape.getImplicitShapeDimensions();
    halfExtents.add(new btVector3(margin, margin, margin));
    final btMatrix3x3 abs_b = t.getBasis().abs();
    final btVector3 center = t.getOrigin();
    final btVector3 extent = halfExtents.dot3(abs_b.getRow(0), abs_b.getRow(1), abs_b.getRow(2));
    aabbMin.set(center).sub(extent);
    aabbMax.set(center).add(extent);
    break;
   }
   case TRIANGLE_SHAPE_PROXYTYPE: {
    btTriangleShape triangleShape = (btTriangleShape) this;
    float margin = triangleShape.getMarginNonVirtual();
    for (int i = 0; i < 3; i++) {
     final btVector3 vec = new btVector3((0.f), (0.f), (0.f));
     vec.setElement(i, (1.f));
     final btVector3 sv = localGetSupportVertexWithoutMarginNonVirtual(t.transposeTransform3x3(
      new btVector3(
       vec)));
     final btVector3 tmp = t.transform(new btVector3(sv));
     aabbMax.setElement(i, tmp.getElement(i) + margin);
     vec.setElement(i, (-1.f));
     tmp.set(t.transform(localGetSupportVertexWithoutMarginNonVirtual(t.transposeTransform3x3(
      new btVector3(
       vec)))));
     aabbMin.setElement(i, tmp.getElement(i) - margin);
    }
   }
   break;
   case CAPSULE_SHAPE_PROXYTYPE: {
    btCapsuleShape capsuleShape = (btCapsuleShape) this;
    final btVector3 halfExtents = new btVector3(capsuleShape.getRadius(), capsuleShape.getRadius(),
     capsuleShape.getRadius());
    int m_upAxis = capsuleShape.getUpAxis();
    halfExtents.setElement(m_upAxis, capsuleShape.getRadius() + capsuleShape.getHalfHeight());
    final btMatrix3x3 abs_b = t.getBasis().abs();
    final btVector3 center = t.getOrigin();
    final btVector3 extent = halfExtents.dot3(abs_b.getRow(0), abs_b.getRow(1), abs_b.getRow(2));
    aabbMin.set(center).sub(extent);
    aabbMax.set(center).add(extent);
   }
   break;
   case CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE:
   case CONVEX_HULL_SHAPE_PROXYTYPE: {
    btPolyhedralConvexAabbCachingShape convexHullShape = (btPolyhedralConvexAabbCachingShape) this;
    float margin = convexHullShape.getMarginNonVirtual();
    convexHullShape.getNonvirtualAabb(t, aabbMin, aabbMax, margin);
   }
   break;
   default:
    this.getAabb(t, aabbMin, aabbMax);
    break;
  }
  // should never reach here
  assert (false);
 }

 public void project(final btTransform trans, final btVector3 dir, float[] min, float[] max,
  final btVector3 witnesPtMin, final btVector3 witnesPtMax) {
  final btVector3 localAxis = trans.transposeTransform3x3(new btVector3(dir));
  final btVector3 vtx1 = trans.transform(localGetSupportingVertex(localAxis));
  final btVector3 vtx2 = trans
   .transform(localGetSupportingVertex(new btVector3(localAxis).negate()));
  min[0] = vtx1.dot(dir);
  max[0] = vtx2.dot(dir);
  witnesPtMax.set(vtx2);
  witnesPtMin.set(vtx1);
  if (min[0] > max[0]) {
   float tmp = min[0];
   min[0] = max[0];
   max[0] = tmp;
   witnesPtMax.set(vtx1);
   witnesPtMin.set(vtx2);
  }
 }

 //notice that the vectors should be unit length
 public abstract void batchedUnitVectorGetSupportingVertexWithoutMargin(btVector3[] vectors,
  btVector3[] supportVerticesOut, int numVectors);

 ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
 @Override
 public abstract void getAabb(final btTransform t, final btVector3 aabbMin, final btVector3 aabbMax);

 public abstract void getAabbSlow(final btTransform t, final btVector3 aabbMin,
  final btVector3 aabbMax);

 @Override
 public abstract void setLocalScaling(final btVector3 scaling);

 @Override
 public abstract btVector3 getLocalScaling();

 @Override
 public abstract void setMargin(float margin);

 @Override
 public abstract float getMargin();

 public abstract int getNumPreferredPenetrationDirections();

 public abstract void getPreferredPenetrationDirection(int index, final btVector3 penetrationVector);
}
