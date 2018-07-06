/*
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it freely,
 * subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.Collision.Shape;

import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.TRIANGLE_MESH_SHAPE_PROXYTYPE;
import Bullet.Collision.MyNodeOverlapCallbackAllTriangles;
import Bullet.Collision.MyNodeOverlapCallbackConvexcast;
import Bullet.Collision.MyNodeOverlapCallbackRaycast;
import Bullet.Collision.btOptimizedBvh;
import Bullet.Collision.btTriangleCallback;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import Bullet.LinearMath.btVector3;
import Bullet.stubs.btTriangleInfoMap;
import java.io.Serializable;
import java.util.Objects;

/**
 * The btBvhTriangleMeshShape is a static-triangle mesh shape, it can only be
 * used for fixed/non-moving objects. If you required moving concave triangle
 * meshes, it is recommended to perform convex decomposition using HACD, see
 * Bullet/Demos/ConvexDecompositionDemo. Alternatively, you can use
 * btGimpactMeshShape for moving concave triangle meshes. btBvhTriangleMeshShape
 * has several optimizations, such as bounding volume hierarchy and cache
 * friendly traversal for PlayStation 3 Cell SPU. It is recommended to enable
 * useQuantizedAabbCompression for better memory usage. It takes a triangle mesh
 * as input, for example a btTriangleMesh or btTriangleIndexVertexArray. The
 * btBvhTriangleMeshShape class allows for triangle mesh deformations by a refit
 * or partialRefit method. Instead of building the bounding volume hierarchy
 * acceleration structure, it is also possible to serialize (save) and
 * deserialize (load) the structure from disk. See
 * Demos\ConcaveDemo\ConcavePhysicsDemo.cpp for an example.
 *
 * @author Gregery Barton
 */
public class btBvhTriangleMeshShape extends btTriangleMeshShape implements
 Serializable {

 private static final long serialVersionUID = -6792520148457019195L;
 btOptimizedBvh m_bvh;
 btTriangleInfoMap m_triangleInfoMap;
 boolean m_useQuantizedAabbCompression;
 boolean m_ownsBvh;

 public btBvhTriangleMeshShape(btStridingMeshInterface meshInterface,
  boolean useQuantizedAabbCompression,
  boolean buildBvh) {
  super(meshInterface);
  m_bvh = null;
  m_triangleInfoMap = null;
  m_useQuantizedAabbCompression = useQuantizedAabbCompression;
  //m_useQuantizedAabbCompression = false;
  m_ownsBvh = false;
  m_shapeType = TRIANGLE_MESH_SHAPE_PROXYTYPE;
  //construct bvh from meshInterface
  if (buildBvh) {
   buildOptimizedBvh();
  }
 }

 public btBvhTriangleMeshShape(btStridingMeshInterface meshInterface,
  boolean useQuantizedAabbCompression) {
  this(meshInterface, useQuantizedAabbCompression, true);
 }

 ///optionally pass in a larger bvh aabb, used for quantization. This allows for deformations within this aabb
 public btBvhTriangleMeshShape(btStridingMeshInterface meshInterface,
  boolean useQuantizedAabbCompression,
  final btVector3 bvhAabbMin, final btVector3 bvhAabbMax, boolean buildBvh) {
  super(meshInterface);
  m_triangleInfoMap = null;
  m_useQuantizedAabbCompression = useQuantizedAabbCompression;
  m_ownsBvh = false;
  m_shapeType = TRIANGLE_MESH_SHAPE_PROXYTYPE;
  //construct bvh from meshInterface
  if (buildBvh) {
   m_bvh = new btOptimizedBvh();
   m_bvh.build(meshInterface, m_useQuantizedAabbCompression, bvhAabbMin,
    bvhAabbMax);
   m_ownsBvh = true;
  } else {
   m_bvh = null;
  }
 }

 public btBvhTriangleMeshShape(btStridingMeshInterface meshInterface,
  boolean useQuantizedAabbCompression,
  final btVector3 bvhAabbMin, final btVector3 bvhAabbMax) {
  this(meshInterface, useQuantizedAabbCompression, bvhAabbMin, bvhAabbMax, true);
 }

 boolean getOwnsBvh() {
  return m_ownsBvh;
 }

 public void performRaycast(btTriangleCallback callback,
  final btVector3 raySource,
  final btVector3 rayTarget) {
  MyNodeOverlapCallbackRaycast myNodeCallback = new MyNodeOverlapCallbackRaycast(
   callback,
   m_meshInterface);
  m_bvh.reportRayOverlappingNodex(myNodeCallback, raySource, rayTarget);
 }

 public void performConvexcast(btTriangleCallback callback,
  final btVector3 raySource,
  final btVector3 rayTarget, final btVector3 aabbMin, final btVector3 aabbMax) {
  MyNodeOverlapCallbackConvexcast myNodeCallback = new MyNodeOverlapCallbackConvexcast(
   callback,
   m_meshInterface);
  m_bvh.reportBoxCastOverlappingNodex(myNodeCallback, raySource, rayTarget,
   aabbMin, aabbMax);
 }

 @Override
 public void processAllTriangles(btTriangleCallback callback,
  final btVector3 aabbMin,
  final btVector3 aabbMax) {
  MyNodeOverlapCallbackAllTriangles myNodeCallback = new MyNodeOverlapCallbackAllTriangles(
   callback,
   m_meshInterface);
  m_bvh.reportAabbOverlappingNodex(myNodeCallback, aabbMin, aabbMax);
 }

 void refitTree(final btVector3 aabbMin, final btVector3 aabbMax) {
  m_bvh.refit(m_meshInterface, aabbMin, aabbMax);
  recalcLocalAabb();
 }

 ///for a fast incremental refit of parts of the tree. Note: the entire AABB of the tree will become more conservative, it never shrinks
 void partialRefitTree(final btVector3 aabbMin, final btVector3 aabbMax) {
  m_bvh.refitPartial(m_meshInterface, aabbMin, aabbMax);
  m_localAabbMin.setMin(aabbMin);
  m_localAabbMax.setMax(aabbMax);
 }

 //debugging
 @Override
 public String getName() {
  return "BVHTRIANGLEMESH";
 }

 @Override
 public void setLocalScaling(final btVector3 scaling) {
  if ((new btVector3(getLocalScaling()).sub(scaling)).lengthSquared()
   > SIMD_EPSILON) {
   super.setLocalScaling(scaling);
   buildOptimizedBvh();
  }
 }

 btOptimizedBvh getOptimizedBvh() {
  return m_bvh;
 }

 void setOptimizedBvh(btOptimizedBvh bvh) {
  setOptimizedBvh(bvh, new btVector3(1f, 1f, 1f));
 }

 void setOptimizedBvh(btOptimizedBvh bvh, final btVector3 localScaling) {
  assert (m_bvh != null);
  assert (!m_ownsBvh);
  m_bvh = bvh;
  m_ownsBvh = false;
  // update the scaling without rebuilding the bvh
  if ((new btVector3(getLocalScaling()).sub(localScaling)).lengthSquared()
   > SIMD_EPSILON) {
   super.setLocalScaling(localScaling);
  }
 }

 final void buildOptimizedBvh() {
  ///m_localAabbMin/m_localAabbMax is already re-calculated in btTriangleMeshShape. We could just scale aabb, but this needs some more work
  m_bvh = new btOptimizedBvh();
  //rebuild the bvh...
  m_bvh.build(m_meshInterface, m_useQuantizedAabbCompression, m_localAabbMin,
   m_localAabbMax);
  m_ownsBvh = true;
 }

 boolean usesQuantizedAabbCompression() {
  return m_useQuantizedAabbCompression;
 }

 void setTriangleInfoMap(btTriangleInfoMap triangleInfoMap) {
  m_triangleInfoMap = triangleInfoMap;
 }

 btTriangleInfoMap getTriangleInfoMap() {
  return m_triangleInfoMap;
 }

 @Override
 public int hashCode() {
  int hash = 7;
  hash += super.hashCode();
  hash = 41 * hash + Objects.hashCode(this.m_triangleInfoMap);
  hash = 41 * hash + (this.m_useQuantizedAabbCompression ? 1 : 0);
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
  final btBvhTriangleMeshShape other = (btBvhTriangleMeshShape) obj;
  if (this.m_useQuantizedAabbCompression != other.m_useQuantizedAabbCompression) {
   return false;
  }
  if (!Objects.equals(this.m_triangleInfoMap, other.m_triangleInfoMap)) {
   return false;
  }
  return super.equals(obj);
 }

};
