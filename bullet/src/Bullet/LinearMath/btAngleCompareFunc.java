/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2011 Advanced Micro Devices, Inc.  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.LinearMath;

import java.util.Comparator;
import javax.vecmath.Tuple3f;

/**
 *
 * @author Gregery Barton
 */
public class btAngleCompareFunc implements Comparator<GrahamVector3> {

 public final btVector3 m_anchor = new btVector3();

 public btAngleCompareFunc(final Tuple3f anchor) {
  m_anchor.set(anchor);
 }

 @Override
 public int compare(GrahamVector3 a, GrahamVector3 b) {
  if (a.m_angle != b.m_angle) {
   return a.m_angle < b.m_angle ? -1 : 1;
  } else {
   float al = (new btVector3(a).sub(m_anchor)).lengthSquared();
   float bl = (new btVector3(b).sub(m_anchor)).lengthSquared();
   if (al != bl) {
    return al < bl ? -1 : 1;
   } else {
    return a.m_orgIndex - b.m_orgIndex;
   }
  }
 }
}
