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

import org.apache.commons.collections.primitives.ArrayFloatList;
import org.apache.commons.collections.primitives.ArrayIntList;

/**
 *
 * @author Gregery Barton
 */
public class btIndexedMesh {

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
}
