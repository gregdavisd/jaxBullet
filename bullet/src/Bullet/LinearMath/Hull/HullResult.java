/*
Stan Melax Convex Hull Computation
Copyright (c) 2008 Stan Melax http://www.melax.com/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.LinearMath.Hull;

import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class HullResult implements Serializable {

 public HullResult() {
  mPolygons = true;
  mNumOutputVertices = 0;
  mNumFaces = 0;
  mNumIndices = 0;
 }
 public boolean mPolygons;                  // true if indices represents polygons, false indices are triangles
 public int mNumOutputVertices;         // number of vertices in the output hull
 public btVector3[] m_OutputVertices;            // array of vertices
 public int mNumFaces;                  // the number of faces produced
 public int mNumIndices;                // the total number of indices
 public int[] m_Indices;                   // pointer to indices.
// If triangles, then indices are array indexes into the vertex list.
// If polygons, indices are in the form (number of points in face) (p1, p2, p3, ..) etc..
}
