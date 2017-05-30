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

import static Bullet.Collision.Shape.btStridingMeshLock.TRIANGLES;
import Bullet.LinearMath.btVector3;
import java.util.ArrayList;
import org.apache.commons.collections.primitives.ArrayFloatList;
import org.apache.commons.collections.primitives.ArrayIntList;

/**
 * The btTriangleIndexVertexArray allows to access multiple triangle meshes, by indexing into
 * existing triangle/index arrays. Additional meshes can be added using addIndexedMesh No duplcate
 * is made of the vertex/index data, it only indexes into external vertex/index arrays. So keep
 * those arrays around during the lifetime of this btTriangleIndexVertexArray.
 *
 * @author Gregery Barton
 */
public class btTriangleIndexVertexArray extends btStridingMeshInterface {

 private static final long serialVersionUID = 1L;
 protected final ArrayList<btIndexedMesh> m_indexedMeshes = new ArrayList<>(0);
 protected int m_hasAabb; // using int instead of bool to maintain alignment
 protected final btVector3 m_aabbMin = new btVector3();
 protected final btVector3 m_aabbMax = new btVector3();

 public btTriangleIndexVertexArray() {
 }
 //just to be backwards compatible

 public btTriangleIndexVertexArray(int numTriangles, ArrayIntList triangleIndexBase,
  int triangleIndexStride,
  int numVertices, ArrayFloatList vertexBase, int vertexStride) {
  btIndexedMesh mesh = new btIndexedMesh();
  mesh.m_numTriangles = numTriangles;
  mesh.m_triangleIndexBase = triangleIndexBase;
  mesh.m_triangleIndexStride = triangleIndexStride;
  mesh.m_numVertices = numVertices;
  mesh.m_vertexBase = vertexBase;
  mesh.m_vertexStride = vertexStride;
  addIndexedMesh(mesh);
 }

 public final void addIndexedMesh(btIndexedMesh mesh) {
  m_indexedMeshes.add(mesh);
 }

 @Override
 public btStridingMeshLock getLockedReadOnlyVertexIndexBase(int subpart) {
  btIndexedMesh mesh = m_indexedMeshes.get(subpart);
  btStridingMeshLock lock = new btStridingMeshLock(
   mesh.m_vertexBase.toBackedArray(),
   mesh.m_vertexStride,
   mesh.m_triangleIndexBase.toBackedArray(),
   TRIANGLES,
   0,
   1,
   mesh.m_numTriangles * 3);
  return lock;
 }

 @Override
 public void unLockVertexBase(int subpart) {
 }

 @Override
 public int getNumSubParts() {
  return m_indexedMeshes.size();
 }

 @Override
 public void preallocateVertices(int numverts) {
 }

 @Override
 public void preallocateIndices(int numindices) {
 }

 @Override
 public boolean hasPremadeAabb() {
  return m_hasAabb != 0;
 }

 @Override
 public void setPremadeAabb(final btVector3 aabbMin, final btVector3 aabbMax) {
  aabbMin.set(m_aabbMin);
  aabbMax.set(m_aabbMax);
 }

 @Override
 public void getPremadeAabb(final btVector3 aabbMin, final btVector3 aabbMax) {
  m_aabbMin.set(aabbMin);
  m_aabbMax.set(aabbMax);
  m_hasAabb = 1; // this is intentionally an int see notes in header }
 }
}
