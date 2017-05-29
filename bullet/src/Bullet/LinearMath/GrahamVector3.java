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

import static Bullet.LinearMath.btScalar.FLT_EPSILON;
import static Bullet.LinearMath.btScalar.btAtan2Fast;
import static Bullet.LinearMath.btVector3.btPlaneSpace1;
import java.util.ArrayList;
import java.util.Collections;
import javax.vecmath.Tuple3f;

/**
 *
 * @author Gregery Barton
 */
public final class GrahamVector3 extends Tuple3f<GrahamVector3> {

 public GrahamVector3(final btVector3 org, int orgIndex) {
  super(org);
  m_orgIndex = orgIndex;
 }

 public GrahamVector3(final GrahamVector3 gv) {
  set(gv);
  m_angle = gv.m_angle;
  m_orgIndex = gv.m_orgIndex;
 }
 public float m_angle;
 public int m_orgIndex;

 public static void GrahamScanConvexHull2D(ArrayList<GrahamVector3> originalPoints,
  ArrayList<GrahamVector3> hull, final btVector3 normalAxis) {
  final btVector3 axis0 = new btVector3();
  final btVector3 axis1 = new btVector3();
  btPlaneSpace1(normalAxis, axis0, axis1);
  if (originalPoints.size() <= 1) {
   for (int i = 0; i < originalPoints.size(); i++) {
    hull.add(new GrahamVector3(originalPoints.get(0)));
   }
   return;
  }
  //step1 : find anchor point with smallest projection on axis0 and move it to first location
  for (int i = 0; i < originalPoints.size(); i++) {
//		const btVector3& left = originalPoints[i];
//		const btVector3& right = originalPoints[0];
   float projL = originalPoints.get(i).dot(axis0);
   float projR = originalPoints.get(0).dot(axis0);
   if (projL < projR) {
    Collections.swap(originalPoints, 0, i);
   }
  }
  //also precompute angles
  originalPoints.get(0).m_angle = -1e30f;
  for (int i = 1; i < originalPoints.size(); i++) {
   Tuple3f ar = new GrahamVector3(originalPoints.get(i)).sub(originalPoints.get(0));
   float ar1 = axis1.dot(ar);
   float ar0 = axis0.dot(ar);
   if (ar1 * ar1 + ar0 * ar0 < FLT_EPSILON) {
    originalPoints.get(i).m_angle = 0.0f;
   } else {
    originalPoints.get(i).m_angle = btAtan2Fast(ar1, ar0);
   }
  }
  //step 2: sort all points, based on 'angle' with this anchor
  btAngleCompareFunc comp = new btAngleCompareFunc(originalPoints.get(0));
  originalPoints.subList(1, originalPoints.size() - 1).sort(comp);
  int i;
  for (i = 0; i < 2; i++) {
   hull.add(new GrahamVector3(originalPoints.get(i)));
  }
  //step 3: keep all 'convex' points and discard concave points (using back tracking)
  for (; i != originalPoints.size(); i++) {
   boolean isConvex = false;
   while (!isConvex && hull.size() > 1) {
    Tuple3f a = hull.get(hull.size() - 2);
    Tuple3f b = hull.get(hull.size() - 1);
    isConvex = new btVector3(a).sub(b).cross(new btVector3(a).sub(originalPoints.get(i))).dot(
     normalAxis) > 0;
    if (!isConvex) {
     hull.remove(hull.size() - 1);
    } else {
     hull.add(new GrahamVector3(originalPoints.get(i)));
    }
   }
   if (hull.size() == 1) {
    hull.add(new GrahamVector3(originalPoints.get(i)));
   }
  }
 }
}
