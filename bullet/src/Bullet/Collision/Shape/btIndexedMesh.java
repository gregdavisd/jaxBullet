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

import java.io.Serializable;
import java.util.Objects;
import org.apache.commons.collections.primitives.ArrayFloatList;
import org.apache.commons.collections.primitives.ArrayIntList;

/**
 *
 * @author Gregery Barton
 */
public class btIndexedMesh implements Serializable {

 public int m_numTriangles;
 public ArrayIntList m_triangleIndexBase;
 // Size in byte of the indices for one triangle (3*sizeof(index_type) if the indices are tightly packed)
 public int m_triangleIndexStride;
 public int m_numVertices;
 public ArrayFloatList m_vertexBase;
 // Size of a vertex, in bytes
 public int m_vertexStride;
 // The index type is set when adding an indexed mesh to the
 // btTriangleIndexVertexArray, do not set it manually
//   PHY_ScalarType m_indexType;
 // The vertex type has a default type similar to Bullet's precision mode (float or double)
 // but can be set manually if you for example run Bullet with double precision but have
 // mesh data in single precision..
//   PHY_ScalarType m_vertexType;

 @Override
 public int hashCode() {
  int hash = 5;
  hash += super.hashCode();
  hash = 53 * hash + this.m_numTriangles;
  hash = 53 * hash + Objects.hashCode(this.m_triangleIndexBase);
  hash = 53 * hash + this.m_triangleIndexStride;
  hash = 53 * hash + this.m_numVertices;
  hash = 53 * hash + Objects.hashCode(this.m_vertexBase);
  hash = 53 * hash + this.m_vertexStride;
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
  final btIndexedMesh other = (btIndexedMesh) obj;
  if (this.m_numTriangles != other.m_numTriangles) {
   return false;
  }
  if (this.m_triangleIndexStride != other.m_triangleIndexStride) {
   return false;
  }
  if (this.m_numVertices != other.m_numVertices) {
   return false;
  }
  if (this.m_vertexStride != other.m_vertexStride) {
   return false;
  }
  if (!Objects.equals(this.m_triangleIndexBase, other.m_triangleIndexBase)) {
   return false;
  }
  if (!Objects.equals(this.m_vertexBase, other.m_vertexBase)) {
   return false;
  }
  return true;
 }

}
