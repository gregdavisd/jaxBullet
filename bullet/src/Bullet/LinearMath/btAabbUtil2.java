/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

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

import static Bullet.Extras.btMinMax.btMax;
import static Bullet.Extras.btMinMax.btMin;
import static Bullet.LinearMath.btScalar.btSelect;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btAabbUtil2 implements Serializable {

 /**
  *
  * @param rayFrom
  * @param rayInvDirection
  * @param raySign
  * @param bounds
  * @param tmin
  * @param lambda_min
  * @param lambda_max
  * @return
  */
 public static boolean btRayAabb2(final btVector3 rayFrom, final btVector3 rayInvDirection,
  int[] raySign,
  btVector3[] bounds,
  float[] tmin,
  float lambda_min,
  float lambda_max) {
  float tmax, tymin, tymax, tzmin, tzmax;
  tmin[0] = (bounds[raySign[0]].getX() - rayFrom.getX()) * rayInvDirection.getX();
  tmax = (bounds[1 - raySign[0]].getX() - rayFrom.getX()) * rayInvDirection.getX();
  tymin = (bounds[raySign[1]].getY() - rayFrom.getY()) * rayInvDirection.getY();
  tymax = (bounds[1 - raySign[1]].getY() - rayFrom.getY()) * rayInvDirection.getY();
  if ((tmin[0] > tymax) || (tymin > tmax)) {
   return false;
  }
  if (tymin > tmin[0]) {
   tmin[0] = tymin;
  }
  if (tymax < tmax) {
   tmax = tymax;
  }
  tzmin = (bounds[raySign[2]].getZ() - rayFrom.getZ()) * rayInvDirection.getZ();
  tzmax = (bounds[1 - raySign[2]].getZ() - rayFrom.getZ()) * rayInvDirection.getZ();
  if ((tmin[0] > tzmax) || (tzmin > tmax)) {
   return false;
  }
  if (tzmin > tmin[0]) {
   tmin[0] = tzmin;
  }
  if (tzmax < tmax) {
   tmax = tzmax;
  }
  return ((tmin[0] < lambda_max) && (tmax > lambda_min));
 }

 /**
  *
  * @param halfExtents
  * @param margin
  * @param t
  * @param aabbMinOut
  * @param aabbMaxOut
  */
 public static void btTransformAabb(final btVector3 halfExtents, float margin, final btTransform t,
  final btVector3 aabbMinOut, final btVector3 aabbMaxOut) {
  final btVector3 halfExtentsWithMargin = new btVector3(halfExtents).add(new btVector3(margin,
   margin,
   margin));
  final btMatrix3x3 abs_b = t.getBasis().abs();
  final btVector3 center = t.getOrigin();
//  final btVector3 extent = halfExtentsWithMargin.dot3(abs_b.getRow(0), abs_b.getRow(1), abs_b
//   .getRow(2));
  final btVector3 extent = abs_b.transform(halfExtentsWithMargin);
  aabbMinOut.set(center).sub(extent);
  aabbMaxOut.set(center).add(extent);
 }

 /**
  *
  * @param localAabbMin
  * @param localAabbMax
  * @param margin
  * @param trans
  * @param aabbMinOut
  * @param aabbMaxOut
  */
 public static void btTransformAabb(final btVector3 localAabbMin, final btVector3 localAabbMax,
  float margin, final btTransform trans, final btVector3 aabbMinOut, final btVector3 aabbMaxOut) {
  assert (localAabbMin.getX() <= localAabbMax.getX());
  assert (localAabbMin.getY() <= localAabbMax.getY());
  assert (localAabbMin.getZ() <= localAabbMax.getZ());
  final btVector3 localHalfExtents = new btVector3(localAabbMax).sub(localAabbMin).scale(0.5f);
  localHalfExtents.add(new btVector3(margin, margin, margin));
  final btVector3 localCenter = new btVector3(localAabbMax).add(localAabbMin).scale(0.5f);
  final btMatrix3x3 abs_b = (btMatrix3x3) trans.getBasis().abs();
  final btVector3 center = trans.transform(new btVector3(localCenter));
  final btVector3 extent = localHalfExtents.dot3(abs_b.getRow(0), abs_b.getRow(1), abs_b.getRow(2));
  aabbMinOut.set(center).sub(extent);
  aabbMaxOut.set(center).add(extent);
 }

 /// conservative test for overlap between triangle and aabb
 /**
  *
  * @param vertices
  * @param aabbMin
  * @param aabbMax
  * @return
  */
 public static boolean TestTriangleAgainstAabb2(btVector3[] vertices, final btVector3 aabbMin,
  final btVector3 aabbMax) {
  final btVector3 p1 = vertices[0];
  final btVector3 p2 = vertices[1];
  final btVector3 p3 = vertices[2];
  if (btMin(btMin(p1.x, p2.x), p3.x) > aabbMax.x) {
   return false;
  }
  if (btMax(btMax(p1.x, p2.x), p3.x) < aabbMin.x) {
   return false;
  }
  if (btMin(btMin(p1.z, p2.z), p3.z) > aabbMax.z) {
   return false;
  }
  if (btMax(btMax(p1.z, p2.z), p3.z) < aabbMin.z) {
   return false;
  }
  if (btMin(btMin(p1.y, p2.y), p3.y) > aabbMax.y) {
   return false;
  }
  return btMax(btMax(p1.y, p2.y), p3.y) >= aabbMin.y;
 }
/// conservative test for overlap between two aabbs

 /**
  *
  * @param aabbMin1
  * @param aabbMax1
  * @param aabbMin2
  * @param aabbMax2
  * @return
  */
 public static boolean TestAabbAgainstAabb2(final btVector3 aabbMin1, final btVector3 aabbMax1,
  final btVector3 aabbMin2, final btVector3 aabbMax2) {
  boolean overlap = true;
  overlap = (aabbMin1.getX() > aabbMax2.getX() || aabbMax1.getX() < aabbMin2.getX()) ? false :
   overlap;
  overlap = (aabbMin1.getZ() > aabbMax2.getZ() || aabbMax1.getZ() < aabbMin2.getZ()) ? false :
   overlap;
  overlap = (aabbMin1.getY() > aabbMax2.getY() || aabbMax1.getY() < aabbMin2.getY()) ? false :
   overlap;
  return overlap;
 }
 //This block replaces the block below and uses no branches, and replaces the 8 bit return with a 32 bit return for improved performance (~3x on XBox 360)

 /**
  *
  * @param aabbMin1
  * @param aabbMax1
  * @param aabbMin2
  * @param aabbMax2
  * @return
  */
 public static int testQuantizedAabbAgainstQuantizedAabb(short[] aabbMin1, short[] aabbMax1,
  short[] aabbMin2, short[] aabbMax2) {
  return (btSelect(((aabbMin1[0] <= aabbMax2[0]) & (aabbMax1[0] >= aabbMin2[0]) &
   (aabbMin1[2] <= aabbMax2[2]) & (aabbMax1[2] >= aabbMin2[2]) &
   (aabbMin1[1] <= aabbMax2[1]) & (aabbMax1[1] >= aabbMin2[1])) ? 1 : 0,
   1, 0));
 }
}
