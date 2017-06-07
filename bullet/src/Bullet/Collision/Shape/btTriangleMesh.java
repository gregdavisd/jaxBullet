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

import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import org.apache.commons.collections.primitives.ArrayFloatList;
import org.apache.commons.collections.primitives.ArrayIntList;

/**
 * The btTriangleMesh class is a convenience class derived from btTriangleIndexVertexArray, that
 * provides storage for a concave triangle mesh. It can be used as data for the
 * btBvhTriangleMeshShape. It allows either 32bit or 16bit indices, and 4 (x-y-z-w) or 3 (x-y-z)
 * component vertices. If you want to share triangle/index data between graphics mesh and collision
 * mesh (btBvhTriangleMeshShape), you can directly use btTriangleIndexVertexArray or derive your own
 * class from btStridingMeshInterface. Performance of btTriangleMesh and btTriangleIndexVertexArray
 * used in a btBvhTriangleMeshShape is the same.
 *
 * @author Gregery Barton
 */
public class btTriangleMesh extends btTriangleIndexVertexArray implements Serializable {

 private static final long serialVersionUID = 1L;
 private final ArrayFloatList m_3componentVertices = new ArrayFloatList(0);
 private final ArrayIntList m_32bitIndices = new ArrayIntList(0);
 public float m_weldingThreshold;

 public btTriangleMesh() {
  this(true);
 }

 public btTriangleMesh(boolean use32bitIndices) {
  this(use32bitIndices, true);
 }

 public btTriangleMesh(boolean use32bitIndices, boolean use4componentVertices) {
  m_weldingThreshold = (0.0f);
  btIndexedMesh meshIndex = new btIndexedMesh();
  meshIndex.m_numTriangles = 0;
  meshIndex.m_numVertices = 0;
  meshIndex.m_triangleIndexBase = m_32bitIndices;
  meshIndex.m_triangleIndexStride = 3;
  meshIndex.m_vertexBase = m_3componentVertices;
  meshIndex.m_vertexStride = 3;
  m_indexedMeshes.add(meshIndex);
 }

 public boolean getUse32bitIndices() {
  return true;
 }

 public boolean getUse4componentVertices() {
  return false;
 }
 ///By default addTriangle won't search for duplicate vertices, because the search is very slow for large triangle meshes.
 ///In general it is better to directly use btTriangleIndexVertexArray instead.

 public void addTriangle(final btVector3 vertex0, final btVector3 vertex1, final btVector3 vertex2) {
  addTriangle(vertex0, vertex1, vertex2, false);
 }

 public void addTriangle(final btVector3 vertex0, final btVector3 vertex1, final btVector3 vertex2,
  boolean removeDuplicateVertices) {
  m_indexedMeshes.get(0).m_numTriangles++;
  addIndex(findOrAddVertex(vertex0, removeDuplicateVertices));
  addIndex(findOrAddVertex(vertex1, removeDuplicateVertices));
  addIndex(findOrAddVertex(vertex2, removeDuplicateVertices));
 }

 ///Add a triangle using its indices. Make sure the indices are pointing within the vertices array, so add the vertices first (and to be sure, avoid removal of duplicate vertices)	
 public void addTriangleIndices(int index1, int index2, int index3) {
  m_indexedMeshes.get(0).m_numTriangles++;
  addIndex(index1);
  addIndex(index2);
  addIndex(index3);
 }

 public int getNumTriangles() {
  return m_32bitIndices.size() / 3;
 }

 public void preallocateVertices(int numverts) {
  m_3componentVertices.ensureCapacity(numverts * 3);
 }

 public void preallocateIndices(int numindices) {
  m_32bitIndices.ensureCapacity(numindices);
 }

 ///findOrAddVertex is an internal method, use addTriangle instead
 private int findOrAddVertex(final btVector3 vertex, boolean removeDuplicateVertices) {
  //return index of new/existing vertex
  ///@todo: could use acceleration structure for this
  if (removeDuplicateVertices) {
   final btVector3 vtx = new btVector3();
   for (int i = 0; i < m_3componentVertices.size(); i += 3) {
    vtx.set(m_3componentVertices.get(i), m_3componentVertices.get(i + 1), m_3componentVertices.get(
     i +
     2));
    if (vtx.sub(vertex).lengthSquared() <= m_weldingThreshold) {
     return i / 3;
    }
   }
  }
  m_3componentVertices.add(vertex.getX());
  m_3componentVertices.add(vertex.getY());
  m_3componentVertices.add(vertex.getZ());
  m_indexedMeshes.get(0).m_numVertices++;
  m_indexedMeshes.get(0).m_vertexBase = m_3componentVertices;
  return (m_3componentVertices.size() / 3) - 1;
 }

 ///addIndex is an internal method, use addTriangle instead
 private void addIndex(int index) {
  m_32bitIndices.add(index);
  m_indexedMeshes.get(0).m_triangleIndexBase = m_32bitIndices;
 }
}
