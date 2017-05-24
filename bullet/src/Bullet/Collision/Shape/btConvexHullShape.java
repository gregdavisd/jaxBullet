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

import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.CONVEX_HULL_SHAPE_PROXYTYPE;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import static Bullet.LinearMath.btScalar.FLT_MAX;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.btMul;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btConvexHullShape extends btPolyhedralConvexAabbCachingShape implements Serializable {

 /* have roll own growing array because can't get a backed array from ArrayList
  */
 final btVector3[] m_unscaledPoints;
 int point_count;

 ///this constructor optionally takes in a pointer to points. Each point is assumed to be 3 consecutive float (x,y,z), the striding defines the number of floats between each point, in memory.
 ///It is easier to not pass any points in the constructor, and just add one point at a time, using addPoint.
 ///btConvexHullShape make an internal copy of the points.
 btConvexHullShape(float[] points, int numPoints, int stride) {
  super();
  m_shapeType = CONVEX_HULL_SHAPE_PROXYTYPE;
  m_unscaledPoints = new btVector3[numPoints];
  point_count = numPoints;
  int use_stride = stride != 0 ? stride : 3;
  for (int i = 0; i < numPoints; i++) {
   int offset = i * use_stride;
   m_unscaledPoints[i] = new btVector3(points[offset + 0], points[offset + 1], points[offset + 2]);
  }
  recalcLocalAabb();
 }

 btConvexHullShape(float[] points, int numPoints) {
  this(points, numPoints, 3);
 }

 btConvexHullShape(float[] points) {
  this(points, points.length / 3, 3);
 }

 btConvexHullShape() {
  this(null, 0, 0);
 }

 void addPoint(final btVector3 point, boolean recalculateLocalAabb) {
  if (point_count == m_unscaledPoints.length) {
   btVector3[] new_array = new btVector3[Math.max(point_count * 2, 16)];
   System.arraycopy(m_unscaledPoints, 0, new_array, 0, point_count);
  }
  m_unscaledPoints[point_count] = new btVector3(point);
  ++point_count;
  if (recalculateLocalAabb) {
   recalcLocalAabb();
  }
 }

 void addPoint(final btVector3 point) {
  addPoint(point, true);
 }

 btVector3[] getUnscaledPoints() {
  return m_unscaledPoints;
 }

 ///getPoints is obsolete, please use getUnscaledPoints
 btVector3[] getPoints() {
  return getUnscaledPoints();
 }

 void optimizeConvexHull() {
  // convex hull computer not implemented
  assert (false);
 }

 btVector3 getScaledPoint(int i) {
  return btMul(m_unscaledPoints[i], m_localScaling);
 }

 int getNumPoints() {
  return point_count;
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
 public btVector3 localGetSupportingVertexWithoutMargin(final btVector3 vec) {
  final btVector3 supVec = new btVector3();
  float[] maxDot = new float[]{(-BT_LARGE_FLOAT)};
  // Here we take advantage of dot(a, b*c) = dot(a*b, c).  Note: This is true mathematically, but not numerically. 
  if (0 < point_count) {
   final btVector3 scaled = btMul(vec, m_localScaling);
   int index = scaled.maxDot(getUnscaledPoints(), point_count, maxDot); // FIXME: may violate encapsulation of m_unscaledPoints
   return btMul(m_unscaledPoints[index], m_localScaling);
  }
  return supVec;
 }

 @Override
 public void batchedUnitVectorGetSupportingVertexWithoutMargin(btVector3[] vectors,
  btVector3[] supportVerticesOut, int numVectors) {
  //use 'w' component of supportVerticesOut?
  {
   for (int i = 0; i < numVectors; i++) {
    if (supportVerticesOut[i] == null) {
     supportVerticesOut[i] = new btVector3();
    }
    //supportVerticesOut[i].w = (-BT_LARGE_FLOAT);
   }
  }
  float[] newDot = new float[1];
  btVector3[] unscaled_points = getUnscaledPoints();
  for (int j = 0; j < numVectors; j++) {
   final btVector3 vec = btMul(vectors[j], m_localScaling);        // dot(a*b,c) = dot(a,b*c)
   if (0 < point_count) {
    int i = vec.maxDot(unscaled_points, point_count, newDot);
    //(supportVerticesOut[j].set(getScaledPoint(i))).w = newDot[0];
    supportVerticesOut[j].set(getScaledPoint(i));
   } else {
    //supportVerticesOut[j].w = -BT_LARGE_FLOAT;
   }
  }
 }

 @Override
 public void project(final btTransform trans, final btVector3 dir, float[] minProj, float[] maxProj,
  final btVector3 witnesPtMin, final btVector3 witnesPtMax) {
  minProj[0] = FLT_MAX;
  maxProj[0] = -FLT_MAX;
  int numVerts = point_count;
  for (int i = 0; i < numVerts; i++) {
   final btVector3 vtx = btMul(m_unscaledPoints[i], m_localScaling);
   final btVector3 pt = trans.transform(new btVector3(vtx));
   float dp = pt.dot(dir);
   if (dp < minProj[0]) {
    minProj[0] = dp;
    witnesPtMin.set(pt);
   }
   if (dp > maxProj[0]) {
    maxProj[0] = dp;
    witnesPtMax.set(pt);
   }
  }
  if (minProj[0] > maxProj[0]) {
   {
    float swapper = minProj[0];
    minProj[0] = maxProj[0];
    maxProj[0] = swapper;
   }
   {
    final btVector3 swapper = new btVector3(witnesPtMin);
    witnesPtMin.set(witnesPtMax);
    witnesPtMax.set(swapper);
   }
  }
 }

 //debugging
 @Override
 public String getName() {
  return "Convex";
 }

 @Override
 public int getNumVertices() {
  return point_count;
 }

 @Override
 public int getNumEdges() {
  return point_count;
 }

 @Override
 public void getEdge(int i, final btVector3 pa, final btVector3 pb) {
  int index0 = i % point_count;
  int index1 = (i + 1) % point_count;
  pa.set(getScaledPoint(index0));
  pb.set(getScaledPoint(index1));
 }

 @Override
 public void getVertex(int i, final btVector3 vtx) {
  vtx.set(getScaledPoint(i));
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
 public boolean isInside(final btVector3 pt, float tolerance) //not yet
 {
  assert (false);
  return false;
 }

 ///in case we receive negative scaling
 @Override
 public void setLocalScaling(final btVector3 scaling) {
  m_localScaling.set(scaling);
  recalcLocalAabb();
 }
};
