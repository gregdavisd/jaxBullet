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
import org.apache.commons.collections.primitives.ArrayIntList;

/**
 *
 * @author Gregery Barton
 */
public class PHullResult  implements Serializable {

 public PHullResult() {
  mVcount = 0;
  mIndexCount = 0;
  mFaceCount = 0;
  mVertices = null;
 }
 public int mVcount;
 public int mIndexCount;
 public int mFaceCount;
 public btVector3[] mVertices;
 public final ArrayIntList m_Indices = new ArrayIntList();
}
