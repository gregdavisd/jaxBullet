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

import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.init;
import java.io.Serializable;
import java.util.Arrays;

/**
 *
 * @author Gregery Barton
 */
public class btTriangleShape extends btPolyhedralConvexShape implements Serializable {

 public final btVector3[] m_vertices1;

 @Override
 public int getNumVertices() {
  return 3;
 }

 public btVector3 getVertexPtr(int index) {
  return m_vertices1[index];
 }

 public btVector3[] getVertexPtr() {
  return m_vertices1;
 }

 @Override
 public void getVertex(int index, final btVector3 vert) {
  vert.set(m_vertices1[index]);
 }

 @Override
 public int getNumEdges() {
  return 3;
 }

 @Override
 public void getEdge(int i, final btVector3 pa, final btVector3 pb) {
  getVertex(i, pa);
  getVertex((i + 1) % 3, pb);
 }

 @Override
 public void getAabb(final btTransform t, final btVector3 aabbMin, final btVector3 aabbMax) {
//		assert(0);
  getAabbSlow(t, aabbMin, aabbMax);
 }

 @Override
 public btVector3 localGetSupportingVertexWithoutMargin(final btVector3 dir) {
  final btVector3 dots = dir.dot3(m_vertices1[0], m_vertices1[1], m_vertices1[2]);
  return new btVector3(m_vertices1[dots.maxAxis()]);
 }

 @Override
 public void batchedUnitVectorGetSupportingVertexWithoutMargin(btVector3[] vectors,
  btVector3[] supportVerticesOut, int numVectors) {
  for (int i = 0; i < numVectors; i++) {
   final btVector3 dir = vectors[i];
   final btVector3 dots = dir.dot3(m_vertices1[0], m_vertices1[1], m_vertices1[2]);
   if (supportVerticesOut[i] == null) {
    supportVerticesOut[i] = new btVector3();
   }
   supportVerticesOut[i].set(m_vertices1[dots.maxAxis()]);
  }
 }

 btTriangleShape() {
  super();
  m_shapeType = TRIANGLE_SHAPE_PROXYTYPE;
  m_vertices1 = new btVector3[3];
  init(m_vertices1);
 }

 public btTriangleShape(final btVector3 p0, final btVector3 p1, final btVector3 p2) {
  super();
  m_shapeType = TRIANGLE_SHAPE_PROXYTYPE;
  m_vertices1 = new btVector3[]{new btVector3(p0), new btVector3(p1), new btVector3(p2)};
 }

 @Override
 public void getPlane(final btVector3 planeNormal, final btVector3 planeSupport, int i) {
  getPlaneEquation(i, planeNormal, planeSupport);
 }

 @Override
 public int getNumPlanes() {
  return 1;
 }

 void calcNormal(final btVector3 normal) {
  normal
   .set(m_vertices1[1])
   .sub(m_vertices1[0])
   .cross(new btVector3(m_vertices1[2])
    .sub(m_vertices1[0]))
   .normalize();
 }

 void getPlaneEquation(int i, final btVector3 planeNormal, final btVector3 planeSupport) {
  calcNormal(planeNormal);
  planeSupport.set(m_vertices1[0]);
 }

 @Override
 public void calculateLocalInertia(float mass, final btVector3 inertia) {
  assert (false);
  inertia.set(0, 0, 0);
 }

 @Override
 public boolean isInside(final btVector3 pt, float tolerance) {
  final btVector3 normal = new btVector3();
  calcNormal(normal);
  //distance to plane
  float dist = pt.dot(normal);
  float planeconst = m_vertices1[0].dot(normal);
  dist -= planeconst;
  if (dist >= -tolerance && dist <= tolerance) {
   //inside check on edge-planes
   int i;
   for (i = 0; i < 3; i++) {
    final btVector3 pa = new btVector3();
    final btVector3 pb = new btVector3();
    getEdge(i, pa, pb);
    final btVector3 edge = new btVector3(pb).sub(pa);
    final btVector3 edgeNormal = new btVector3(edge).cross(normal);
    edgeNormal.normalize();
    float dist_ = pt.dot(edgeNormal);
    float edgeConst = pa.dot(edgeNormal);
    dist_ -= edgeConst;
    if (dist_ < -tolerance) {
     return false;
    }
   }
   return true;
  }
  return false;
 }
 //debugging

 @Override
 public String getName() {
  return "Triangle";
 }

 @Override
 public int getNumPreferredPenetrationDirections() {
  return 2;
 }

 @Override
 public void getPreferredPenetrationDirection(int index, final btVector3 penetrationVector) {
  calcNormal(penetrationVector);
  if (index != 0) {
   penetrationVector.negate();
  }
 }

 @Override
 public int hashCode() {
  int hash = 7;
  hash += super.hashCode();
  hash = 13 * hash + Arrays.deepHashCode(this.m_vertices1);
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
  final btTriangleShape other = (btTriangleShape) obj;
  if (!Arrays.deepEquals(this.m_vertices1, other.m_vertices1)) {
   return false;
  }
  return super.equals(obj);
 }
}
