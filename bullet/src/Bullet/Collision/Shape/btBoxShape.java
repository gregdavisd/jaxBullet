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
import static Bullet.LinearMath.btAabbUtil2.btTransformAabb;
import static Bullet.LinearMath.btScalar.btFsels;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import Bullet.LinearMath.btVector4;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btBoxShape extends btPolyhedralConvexShape implements Serializable {

 public btBoxShape(final btVector3 boxHalfExtents) {
  super();
  m_shapeType = BOX_SHAPE_PROXYTYPE;
  setSafeMargin(boxHalfExtents);
  final btVector3 margin = new btVector3(m_collisionMargin, m_collisionMargin, m_collisionMargin);
  m_implicitShapeDimensions.set(boxHalfExtents).mul(m_localScaling).sub(margin);
 }

 public btVector3 getHalfExtentsWithMargin() {
  final btVector3 halfExtents = getHalfExtentsWithoutMargin();
  final btVector3 margin = new btVector3(getMargin(), getMargin(), getMargin());
  halfExtents.add(margin);
  return halfExtents;
 }

 public btVector3 getHalfExtentsWithoutMargin() {
  return new btVector3(m_implicitShapeDimensions);//scaling is included, margin is not
 }

 @Override
 public btVector3 localGetSupportingVertex(final btVector3 vec) {
  final btVector3 halfExtents = getHalfExtentsWithoutMargin();
  final btVector3 margin = new btVector3(getMargin(), getMargin(), getMargin());
  halfExtents.add(margin);
  return new btVector3(btFsels(vec.x(), halfExtents.x(), -halfExtents.x()),
   btFsels(vec.y(), halfExtents.y(), -halfExtents.y()),
   btFsels(vec.z(), halfExtents.z(), -halfExtents.z()));
 }

 @Override
 public btVector3 localGetSupportingVertexWithoutMargin(final btVector3 vec) {
  final btVector3 halfExtents = getHalfExtentsWithoutMargin();
  return new btVector3(btFsels(vec.x(), halfExtents.x(), -halfExtents.x()),
   btFsels(vec.y(), halfExtents.y(), -halfExtents.y()),
   btFsels(vec.z(), halfExtents.z(), -halfExtents.z()));
 }

 @Override
 public void batchedUnitVectorGetSupportingVertexWithoutMargin(btVector3[] vectors,
  btVector3[] supportVerticesOut, int numVectors) {
  final btVector3 halfExtents = getHalfExtentsWithoutMargin();
  for (int i = 0; i < numVectors; i++) {
   final btVector3 vec = vectors[i];
   if (supportVerticesOut[i] == null) {
    supportVerticesOut[i] = new btVector3();
   }
   supportVerticesOut[i].set(btFsels(vec.x(), halfExtents.x(), -halfExtents.x()),
    btFsels(vec.y(), halfExtents.y(), -halfExtents.y()),
    btFsels(vec.z(), halfExtents.z(), -halfExtents.z()));
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

 @Override
 public void getAabb(final btTransform t, final btVector3 aabbMin, final btVector3 aabbMax) {
  btTransformAabb(getHalfExtentsWithoutMargin(), getMargin(), t, aabbMin, aabbMax);
 }

 @Override
 public void calculateLocalInertia(float mass, final btVector3 inertia) {
  //float margin = float(0.);
  final btVector3 halfExtents = getHalfExtentsWithMargin();
  float lx = (2.f) * (halfExtents.x());
  float ly = (2.f) * (halfExtents.y());
  float lz = (2.f) * (halfExtents.z());
  inertia.set(mass / ((12.0f)) * (ly * ly + lz * lz),
   mass / ((12.0f)) * (lx * lx + lz * lz),
   mass / ((12.0f)) * (lx * lx + ly * ly));
 }

 @Override
 public void getPlane(final btVector3 planeNormal, final btVector3 planeSupport, int i) {
  //this plane might not be aligned...
  btVector4 plane = new btVector4();
  getPlaneEquation(plane, i);
  planeNormal.set(plane.getX(), plane.getY(), plane.getZ());
  planeSupport.set(localGetSupportingVertex(new btVector3(planeNormal).negate()));
 }

 @Override
 public int getNumPlanes() {
  return 6;
 }

 @Override
 public int getNumVertices() {
  return 8;
 }

 @Override
 public int getNumEdges() {
  return 12;
 }

 @Override
 public void getVertex(int i, final btVector3 vtx) {
  final btVector3 halfExtents = getHalfExtentsWithMargin();
  vtx.set(
   halfExtents.x() * (1 - (i & 1)) - halfExtents.x() * (i & 1),
   halfExtents.y() * (1 - ((i & 2) >> 1)) - halfExtents.y() * ((i & 2) >> 1),
   halfExtents.z() * (1 - ((i & 4) >> 2)) - halfExtents.z() * ((i & 4) >> 2));
 }

 void getPlaneEquation(btVector4 plane, int i) {
  final btVector3 halfExtents = getHalfExtentsWithoutMargin();
  switch (i) {
   case 0:
    plane.set((1.f), (0.f), (0.f), -halfExtents.x());
    break;
   case 1:
    plane.set((-1.f), (0.f), (0.f), -halfExtents.x());
    break;
   case 2:
    plane.set((0.f), (1.f), (0.f), -halfExtents.y());
    break;
   case 3:
    plane.set((0.f), (-1.f), (0.f), -halfExtents.y());
    break;
   case 4:
    plane.set((0.f), (0.f), (1.f), -halfExtents.z());
    break;
   case 5:
    plane.set((0.f), (0.f), (-1.f), -halfExtents.z());
    break;
   default:
    assert (false);
  }
 }

 @Override
 public void getEdge(int i, final btVector3 pa, final btVector3 pb) //  void getEdge(int i,Edge& edge)  
 {
  int edgeVert0 = 0;
  int edgeVert1 = 0;
  switch (i) {
   case 0:
    edgeVert0 = 0;
    edgeVert1 = 1;
    break;
   case 1:
    edgeVert0 = 0;
    edgeVert1 = 2;
    break;
   case 2:
    edgeVert0 = 1;
    edgeVert1 = 3;
    break;
   case 3:
    edgeVert0 = 2;
    edgeVert1 = 3;
    break;
   case 4:
    edgeVert0 = 0;
    edgeVert1 = 4;
    break;
   case 5:
    edgeVert0 = 1;
    edgeVert1 = 5;
    break;
   case 6:
    edgeVert0 = 2;
    edgeVert1 = 6;
    break;
   case 7:
    edgeVert0 = 3;
    edgeVert1 = 7;
    break;
   case 8:
    edgeVert0 = 4;
    edgeVert1 = 5;
    break;
   case 9:
    edgeVert0 = 4;
    edgeVert1 = 6;
    break;
   case 10:
    edgeVert0 = 5;
    edgeVert1 = 7;
    break;
   case 11:
    edgeVert0 = 6;
    edgeVert1 = 7;
    break;
   default:
    assert (false);
  }
  getVertex(edgeVert0, pa);
  getVertex(edgeVert1, pb);
 }

 @Override
 public boolean isInside(final btVector3 pt, float tolerance) {
  final btVector3 halfExtents = getHalfExtentsWithoutMargin();
  //float minDist = 2*tolerance;
  boolean result = (pt.x() <= (halfExtents.x() + tolerance)) &&
   (pt.x() >= (-halfExtents.x() - tolerance)) &&
   (pt.y() <= (halfExtents.y() + tolerance)) &&
   (pt.y() >= (-halfExtents.y() - tolerance)) &&
   (pt.z() <= (halfExtents.z() + tolerance)) &&
   (pt.z() >= (-halfExtents.z() - tolerance));
  return result;
 }

 //debugging
 @Override
 public String getName() {
  return "Box";
 }

 @Override
 public int getNumPreferredPenetrationDirections() {
  return 6;
 }

 @Override
 public void getPreferredPenetrationDirection(int index, final btVector3 penetrationVector) {
  switch (index) {
   case 0:
    penetrationVector.set((1.f), (0.f), (0.f));
    break;
   case 1:
    penetrationVector.set((-1.f), (0.f), (0.f));
    break;
   case 2:
    penetrationVector.set((0.f), (1.f), (0.f));
    break;
   case 3:
    penetrationVector.set((0.f), (-1.f), (0.f));
    break;
   case 4:
    penetrationVector.set((0.f), (0.f), (1.f));
    break;
   case 5:
    penetrationVector.set((0.f), (0.f), (-1.f));
    break;
   default:
    assert (false);
  }
 }
};
