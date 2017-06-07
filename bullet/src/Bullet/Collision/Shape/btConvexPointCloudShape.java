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

import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import java.util.Arrays;

/**
 *
 * @author Gregery Barton
 */
public class btConvexPointCloudShape extends btPolyhedralConvexAabbCachingShape implements
 Serializable {

 btVector3[] m_unscaledPoints;
 int m_numPoints;

 btConvexPointCloudShape() {
  m_localScaling.set(1.f, 1.f, 1.f);
  m_shapeType = CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE;
  m_unscaledPoints = null;
  m_numPoints = 0;
 }

 btConvexPointCloudShape(btVector3[] points, int numPoints, final btVector3 localScaling) {
  this(points, numPoints, localScaling, true);
 }

 btConvexPointCloudShape(btVector3[] points, int numPoints, final btVector3 localScaling,
  boolean computeAabb) {
  m_localScaling.set(localScaling);
  m_shapeType = CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE;
  m_unscaledPoints = points;
  m_numPoints = numPoints;
  if (computeAabb) {
   recalcLocalAabb();
  }
 }

 void setPoints(btVector3[] points, int numPoints) {
  setPoints(points, numPoints, true, new btVector3(1.f, 1.f, 1.f));
 }

 void setPoints(btVector3[] points, int numPoints, boolean computeAabb) {
  setPoints(points, numPoints, computeAabb, new btVector3(1.f, 1.f, 1.f));
 }

 //	void setPoints (btVector3[] points, int numPoints, boolean computeAabb = true,  btVector3& localScaling=btVector3(1.f,1.f,1.f))
 void setPoints(btVector3[] points, int numPoints, boolean computeAabb, final btVector3 localScaling) {
  m_unscaledPoints = Arrays.copyOfRange(points, 0, numPoints);
  m_numPoints = numPoints;
  m_localScaling.set(localScaling);
  if (computeAabb) {
   recalcLocalAabb();
  }
 }

 btVector3[] getUnscaledPoints() {
  return m_unscaledPoints;
 }

 int getNumPoints() {
  return m_numPoints;
 }

 btVector3 getScaledPoint(int index) {
  return new btVector3(m_unscaledPoints[index]).mul(m_localScaling);
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

 @Override
 public btVector3 localGetSupportingVertexWithoutMargin(final btVector3 vec0) {
  final btVector3 supVec = new btVector3();
  float[] maxDot = new float[]{(-BT_LARGE_FLOAT)};
  final btVector3 vec = new btVector3(vec0);
  float lenSqr = vec.lengthSquared();
  if (lenSqr < (0.0001f)) {
   vec.set(1, 0, 0);
  } else {
   float rlen = (1.f) / btSqrt(lenSqr);
   vec.scale(rlen);
  }
  if (m_numPoints > 0) {
   // Here we take advantage of dot(a*b, c) = dot( a, b*c) to do less work. Note this transformation is true mathematically, not numerically.
   //    btVector3 scaled = vec * m_localScaling;
   int index = vec.maxDot(m_unscaledPoints, m_numPoints, maxDot);   //FIXME: may violate encapsulation of m_unscaledPoints
   return getScaledPoint(index);
  }
  return supVec;
 }

 @Override
 public void batchedUnitVectorGetSupportingVertexWithoutMargin(btVector3[] vectors,
  btVector3[] supportVerticesOut, int numVectors) {
  float[] maxDot = new float[1];
  for (int j = 0; j < numVectors; j++) {
   final btVector3 vec = new btVector3(vectors[j]).mul(m_localScaling);  // dot( a*c, b) = dot(a, b*c)
   int index = vec.maxDot(m_unscaledPoints, m_numPoints, maxDot);
   if (supportVerticesOut[j] == null) {
    supportVerticesOut[j] = new btVector3();
   }
   //supportVerticesOut[j].w = (-BT_LARGE_FLOAT);
   if (0 <= index) {
    //WARNING: don't swap next lines, the w component would get overwritten!
    //(supportVerticesOut[j].set(getScaledPoint(index))).w = maxDot[0];
    supportVerticesOut[j].set(getScaledPoint(index));
   }
  }
 }

 //debugging
 @Override
 public String getName() {
  return "ConvexPointCloud";
 }

//currently just for debugging (drawing), perhaps future support for algebraic continuous collision detection
//Please note that you can debug-draw btConvexHullShape with the Raytracer Demo
 @Override
 public int getNumVertices() {
  return m_numPoints;
 }

 @Override
 public int getNumEdges() {
  return 0;
 }

 @Override
 public void getEdge(int i, final btVector3 pa, final btVector3 pb) {
  assert (false);
 }

 @Override
 public void getVertex(int i, final btVector3 vtx) {
  vtx.set(m_unscaledPoints[i]).mul(m_localScaling);
 }

 @Override
 public int getNumPlanes() {
  return 0;
 }

 @Override
 public void getPlane(final btVector3 planeNormal, final btVector3 planeSupport, int i) {
  assert (false);
 }

 @Override
 public boolean isInside(final btVector3 pt, float tolerance) {
  assert (false);
  return false;
 }

 ///in case we receive negative scaling
 @Override
 public void setLocalScaling(final btVector3 scaling) {
  m_localScaling.set(scaling);
  recalcLocalAabb();
 }

 @Override
 public int hashCode() {
  int hash = 3;
  hash += super.hashCode();
  hash = 47 * hash + Arrays.deepHashCode(this.m_unscaledPoints);
  hash = 47 * hash + this.m_numPoints;
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
  final btConvexPointCloudShape other = (btConvexPointCloudShape) obj;
  if (this.m_numPoints != other.m_numPoints) {
   return false;
  }
  if (!Arrays.deepEquals(this.m_unscaledPoints, other.m_unscaledPoints)) {
   return false;
  }
  return super.equals(obj);
 }
}
