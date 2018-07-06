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

import Bullet.Collision.btTriangleCallback;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import java.util.Objects;

/**
 *
 * @author Gregery Barton
 */
abstract public class btStridingMeshInterface implements Serializable {

 private static final long serialVersionUID = 1L;
 final btVector3 m_scaling = new btVector3();
 final btTriangleCallback vertex_scaling;
 btTriangleCallback callback;

 public btStridingMeshInterface() {
  m_scaling.set(1, 1, 1);
  vertex_scaling = new btTriangleCallback() {
   private static final long serialVersionUID = 1L;

   @Override
   public boolean processTriangle(btVector3[] triangle, int partId,
    int triangleIndex) {
    for (int i = 0; i < 3; i++) {
     triangle[i].mul(m_scaling);
    }
    callback.processTriangle(triangle, partId, triangleIndex);
    return true;
   }

  };
 }

 public void InternalProcessAllTriangles(btTriangleCallback callback,
  final btVector3 aabbMin,
  final btVector3 aabbMax) {
  this.callback = callback;
  int graphicssubparts = getNumSubParts();
  ///if the number of parts is big, the performance might drop due to the innerloop switch on indextype
  for (int part = 0; part < graphicssubparts; part++) {
   btStridingMeshLock mesh = getLockedReadOnlyVertexIndexBase(part);
   mesh.process_all_triangles(vertex_scaling, part);
   //unLockReadOnlyVertexBase(part);
  }
 }

 public void InternalProcessSubPart(btTriangleCallback callback,
  btStridingMeshLock mesh,
  final btVector3 aabbMin, final btVector3 aabbMax, int part,
  int baseTriangleIndex) {
  this.callback = callback;
  mesh.process_all_triangles(vertex_scaling, part);
  //unLockReadOnlyVertexBase(part);
 }

 ///brute force method to calculate aabb
 public void calculateAabbBruteForce(final btVector3 aabbMin,
  final btVector3 aabbMax) {
  //first calculate the total aabb for all triangles
  aabbMin.set((-BT_LARGE_FLOAT), (-BT_LARGE_FLOAT), (-BT_LARGE_FLOAT));
  aabbMax.set((BT_LARGE_FLOAT), (BT_LARGE_FLOAT), (BT_LARGE_FLOAT));
  btTriangleCallback aabbCallback = new btTriangleCallbackImpl(aabbMin,
   aabbMax);
  InternalProcessAllTriangles(aabbCallback, aabbMin, aabbMax);
 }

 /// get read and write access to a subpart of a triangle mesh
 /// this subpart has a continuous array of vertices and indices
 /// in this way the mesh can be handled as chunks of memory with striding
 /// very similar to OpenGL vertexarray support
 /// make a call to unLockVertexBase when the read and write access is finished	
 public abstract btStridingMeshLock getLockedReadOnlyVertexIndexBase(int subpart);

 public btStridingMeshLock getLockedReadOnlyVertexIndexBase() {
  return getLockedReadOnlyVertexIndexBase(0);
 }

 /// unLockVertexBase finishes the access to a subpart of the triangle mesh
 /// make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished
 public final void unLockVertexBase(int subpart) {
 }

 public final void unLockReadOnlyVertexBase(int subpart) {
 }

 /// getNumSubParts returns the number of seperate subparts
 /// each subpart has a continuous array of vertices and indices
 public abstract int getNumSubParts();

 public abstract void preallocateVertices(int numverts);

 public abstract void preallocateIndices(int numindices);

 public abstract boolean hasPremadeAabb();

 public abstract void setPremadeAabb(final btVector3 aabbMin,
  final btVector3 aabbMax);

 public abstract void getPremadeAabb(final btVector3 aabbMin,
  final btVector3 aabbMax);

 public btVector3 getScaling() {
  return new btVector3(m_scaling);
 }

 public void setScaling(final btVector3 scaling) {
  m_scaling.set(scaling);
 }

 private static class btTriangleCallbackImpl implements btTriangleCallback {

  private static final long serialVersionUID = 1L;
  private final btVector3 aabbMin;
  private final btVector3 aabbMax;

  public btTriangleCallbackImpl(final btVector3 aabbMin, final btVector3 aabbMax) {
   this.aabbMin = aabbMin;
   this.aabbMax = aabbMax;
  }

  @Override
  public boolean processTriangle(btVector3[] triangle, int partId,
   int triangleIndex) {
   aabbMin.setMin(triangle[0]);
   aabbMax.setMax(triangle[0]);
   aabbMin.setMin(triangle[1]);
   aabbMax.setMax(triangle[1]);
   aabbMin.setMin(triangle[2]);
   aabbMax.setMax(triangle[2]);
   return true;
  }

 }

 @Override
 public int hashCode() {
  int hash = 7;
  hash = 59 * hash + Objects.hashCode(this.m_scaling);
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
  final btStridingMeshInterface other = (btStridingMeshInterface) obj;
  if (!Objects.equals(this.m_scaling, other.m_scaling)) {
   return false;
  }
  return true;
 }

};
