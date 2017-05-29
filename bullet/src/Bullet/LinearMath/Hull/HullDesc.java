
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

import static Bullet.LinearMath.Hull.HullFlag.QF_DEFAULT;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class HullDesc implements Serializable {

 public HullDesc() {
  mFlags = QF_DEFAULT;
  mVcount = 0;
  mVertices = null;
  mVertexStride = 1;
  mNormalEpsilon = 0.001f;
  mMaxVertices = 4096; // maximum number of points to be considered for a convex hull.
  mMaxFaces = 4096;
 }

 ;

public HullDesc(int flag,
  int vcount,
  btVector3[] vertices) {
  mFlags = flag;
  mVcount = vcount;
  mVertices = vertices;
  mVertexStride = 1;
  mNormalEpsilon = (0.001f);
  mMaxVertices = 4096;
 }

 public boolean HasHullFlag(int flag) {
  return (mFlags & flag) != 0;
 }

 public void SetHullFlag(int flag) {
  mFlags |= flag;
 }

 public void ClearHullFlag(int flag) {
  mFlags &= ~flag;
 }
 public int mFlags;           // flags to use when generating the convex hull.
 public int mVcount;          // number of vertices in the input point cloud
 public btVector3[] mVertices;        // the array of vertices.
 public int mVertexStride;    // the stride of each vertex, in bytes.
 public float mNormalEpsilon;   // the epsilon for removing duplicates.  This is a normalized value, if normalized bit is on.
 public int mMaxVertices;     // maximum number of vertices to be considered for the hull!
 public int mMaxFaces;
}
