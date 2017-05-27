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

import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE;
import Bullet.Collision.CenterCallback;
import Bullet.Collision.InertiaCallback;
import Bullet.Collision.LocalSupportVertexCallback;
import Bullet.LinearMath.btMatrix3x3;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 * The btConvexTriangleMeshShape is a convex hull of a triangle mesh, but the performance is not as
 * good as btConvexHullShape. A small benefit of this class is that it uses the
 * btStridingMeshInterface, so you can avoid the duplication of the triangle mesh data.
 * Nevertheless, most users should use the much better performing btConvexHullShape instead.
 *
 * @author Gregery Barton
 */
public class btConvexTriangleMeshShape extends btPolyhedralConvexAabbCachingShape  implements Serializable {

 final btStridingMeshInterface m_stridingMesh;

 btConvexTriangleMeshShape(btStridingMeshInterface meshInterface) {
  this(meshInterface, true);
 }

 btConvexTriangleMeshShape(btStridingMeshInterface meshInterface, boolean calcAabb) {
  super();
  m_stridingMesh = meshInterface;
  m_shapeType = CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE;
  if (calcAabb) {
   recalcLocalAabb();
  }
 }

 public btStridingMeshInterface getMeshInterface() {
  return m_stridingMesh;
 }

 @Override
 public btVector3 localGetSupportingVertex(final btVector3 vec0) {
  final btVector3 vec = vec0;
  float lenSqr = vec.lengthSquared();
  if (lenSqr < 0.0001f) {
   vec.set(1, 0, 0);
  } else {
   vec.normalize();
  }
  LocalSupportVertexCallback supportCallback = new LocalSupportVertexCallback(vec);
  final btVector3 aabbMax = new btVector3((BT_LARGE_FLOAT), (BT_LARGE_FLOAT), (BT_LARGE_FLOAT));
  m_stridingMesh.InternalProcessAllTriangles(supportCallback, new btVector3(aabbMax).negate(),
   aabbMax);
  return supportCallback.getSupportVertexLocal();
 }

 @Override
 public btVector3 localGetSupportingVertexWithoutMargin(final btVector3 vec0) {
  final btVector3 supVec = new btVector3();
  final btVector3 vec = vec0;
  float lenSqr = vec.lengthSquared();
  if (lenSqr < 0.0001f) {
   vec.set(1, 0, 0);
  } else {
   float rlen = 1.f / btSqrt(lenSqr);
   vec.scale(rlen);
  }
  LocalSupportVertexCallback supportCallback = new LocalSupportVertexCallback(vec);
  final btVector3 aabbMax = new btVector3((BT_LARGE_FLOAT), (BT_LARGE_FLOAT), (BT_LARGE_FLOAT));
  m_stridingMesh.InternalProcessAllTriangles(supportCallback, new btVector3(aabbMax).negate(),
   aabbMax);
  supVec.set(supportCallback.getSupportVertexLocal());
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
  ///@todo: could do the batch inside the callback!
  for (int j = 0; j < numVectors; j++) {
   final btVector3 vec = vectors[j];
   LocalSupportVertexCallback supportCallback = new LocalSupportVertexCallback(vec);
   final btVector3 aabbMax = new btVector3((BT_LARGE_FLOAT), (BT_LARGE_FLOAT), (BT_LARGE_FLOAT));
   m_stridingMesh.InternalProcessAllTriangles(supportCallback, new btVector3(aabbMax).negate(),
    aabbMax);
   supportVerticesOut[j].set(supportCallback.getSupportVertexLocal());
  }
 }

 //debugging
 @Override
 public String getName() {
  return "ConvexTrimesh";
 }

//currently just for debugging (drawing), perhaps future support for algebraic continuous collision detection
//Please note that you can debug-draw btConvexTriangleMeshShape with the Raytracer Demo
 @Override
 public int getNumVertices() {
  //cache this?
  return 0;
 }

 @Override
 public int getNumEdges() {
  return 0;
 }

 @Override
 public void getEdge(int i, final btVector3 pa, final btVector3 pb) {
  assert(false);
 }

 @Override
 public void getVertex(int i, final btVector3 vtx) {
  assert(false);
 }

 @Override
 public int getNumPlanes() {
  return 0;
 }

 @Override
 public void getPlane(final btVector3 planeNormal, final btVector3 planeSupport, int i) {
  assert(false);
 }

 @Override
 public boolean isInside(final btVector3 pt, float tolerance) {
  assert(false);
  return false;
 }

 @Override
 public void setLocalScaling(final btVector3 scaling) {
  m_stridingMesh.setScaling(scaling);
  recalcLocalAabb();
 }

 @Override
 public btVector3 getLocalScaling() {
  return m_stridingMesh.getScaling();
 }

 ///computes the exact moment of inertia and the transform from the coordinate system defined by the principal axes of the moment of inertia
 ///and the center of mass to the current coordinate system. A mass of 1 is assumed, for other masses just multiply the computed "inertia"
 ///by the mass. The resulting transform "principal" has to be applied inversely to the mesh in order for the local coordinate system of the
 ///shape to be centered at the center of mass and to coincide with the principal axes. This also necessitates a correction of the world transform
 ///of the collision object by the principal transform. This method also computes the volume of the convex mesh.
 public void calculatePrincipalAxisTransform(final btTransform principal, final btVector3 inertia,
  float[] volume) {
  CenterCallback centerCallback = new CenterCallback();
  final btVector3 aabbMax = new btVector3((BT_LARGE_FLOAT), (BT_LARGE_FLOAT), (BT_LARGE_FLOAT));
  m_stridingMesh.InternalProcessAllTriangles(centerCallback, new btVector3(aabbMax).negate(),
   aabbMax);
  final btVector3 center = centerCallback.getCenter();
  principal.setOrigin(center);
  volume[0] = centerCallback.getVolume();
  InertiaCallback inertiaCallback = new InertiaCallback(center);
  m_stridingMesh.InternalProcessAllTriangles(inertiaCallback, new btVector3(aabbMax).negate(),
   aabbMax);
  final btMatrix3x3 i = inertiaCallback.getInertia();
  i.diagonalize(principal.getBasis(), (0.00001f), 20);
  inertia.set(i.m00, i.m11, i.m22);
  inertia.scale(1.0f / volume[0]);
 }
};
