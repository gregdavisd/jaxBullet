/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
package Bullet.Collision;

import static Bullet.LinearMath.btAabbUtil2.TestTriangleAgainstAabb2;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class FilteredCallback implements btTriangleCallback,   Serializable  {

  public final btTriangleCallback m_callback;
  public final btVector3 m_aabbMin = new btVector3();
  public final btVector3 m_aabbMax = new btVector3();

  public FilteredCallback(btTriangleCallback callback, final btVector3 aabbMin, final btVector3 aabbMax) {
  m_callback = callback;
  m_aabbMin.set(aabbMin);
  m_aabbMax.set(aabbMax);
 }

 /**
  *
  * @param triangle
  * @param partId
  * @param triangleIndex
  * @return
  */
 @Override
 public boolean processTriangle(btVector3[] triangle, int partId, int triangleIndex) {
  if (TestTriangleAgainstAabb2(triangle, m_aabbMin, m_aabbMax)) {
   //check aabb in triangle-space, before doing this
   m_callback.processTriangle(triangle, partId, triangleIndex);
  }
  return true;
 }
}
