/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

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

import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
abstract public class btTriangleRaycastCallback implements btTriangleCallback, Serializable {

 //input
 final btVector3 m_from = new btVector3();
 final btVector3 m_to = new btVector3();
 //@BP Mod - allow backface filtering and unflipped normals
 /**
  *
  */
 public static final int kF_None = 0;
 /**
  *
  */
 public static final int kF_FilterBackfaces = 1;
 /**
  *
  */
 public static final int kF_KeepUnflippedNormal = 1 << 1;   // Prevents returned face normal getting flipped when a ray hits a back-facing triangle
 ///SubSimplexConvexCastRaytest is the default, even if kF_None is set.
 /**
  *
  */
 public static final int kF_UseSubSimplexConvexCastRaytest = 1 << 2;   // Uses an approximate but faster ray versus convex intersection algorithm
 /**
  *
  */
 public static final int kF_UseGjkConvexCastRaytest = 1 << 3;
 /**
  *
  */
 public static final int kF_Terminator = 0xFFFFFFFF;
 int m_flags;
 float m_hitFraction;

 btTriangleRaycastCallback(final btVector3 from, final btVector3 to, int flags) {
  m_from.set(from);
  m_to.set(to);
  //@BP Mod
  m_flags = flags;
  m_hitFraction = 1.f;
 }

 btTriangleRaycastCallback(final btVector3 from, final btVector3 to) {
  this(from, to, 0);
 }

 /**
  *
  * @param triangle
  * @param partId
  * @param triangleIndex
  */
 @Override
 public boolean processTriangle(btVector3[] triangle, int partId, int triangleIndex) {
  final btVector3 vert0 = triangle[0];
  final btVector3 vert1 = triangle[1];
  final btVector3 vert2 = triangle[2];
  final btVector3 v10 = new btVector3(vert1).sub(vert0);
  final btVector3 v20 = new btVector3(vert2).sub(vert0);
  final btVector3 triangleNormal = new btVector3(v10).cross(v20);
  float dist = vert0.dot(triangleNormal);
  float dist_a = triangleNormal.dot(m_from);
  dist_a -= dist;
  float dist_b = triangleNormal.dot(m_to);
  dist_b -= dist;
  if (dist_a * dist_b >= 0) {
   return true; // same sign
  }
  if (((m_flags & kF_FilterBackfaces) != 0) && (dist_a <= 0)) {
   // Backface, skip check
   return true;
  }
  float proj_length = dist_a - dist_b;
  float distance = (dist_a) / (proj_length);
  // Now we have the intersection point on the plane, we'll see if it's inside the triangle
  // Add an epsilon as a tolerance for the raycast,
  // in case the ray hits exacly on the edge of the triangle.
  // It must be scaled for the triangle size.
  if (distance < m_hitFraction) {
   float edge_tolerance = triangleNormal.lengthSquared();
   edge_tolerance *= -0.0001f;
   final btVector3 point = new btVector3();
   point.setInterpolate3(m_from, m_to, distance);
   {
    final btVector3 v0p = new btVector3(vert0).sub(point);
    final btVector3 v1p = new btVector3(vert1).sub(point);
    final btVector3 cp0 = new btVector3(v0p).cross(v1p);
    if ((float) (cp0.dot(triangleNormal)) >= edge_tolerance) {
     final btVector3 v2p = new btVector3(vert2).sub(point);
     final btVector3 cp1 = new btVector3(v1p).cross(v2p);
     if ((float) (cp1.dot(triangleNormal)) >= edge_tolerance) {
      final btVector3 cp2 = new btVector3(v2p).cross(v0p);
      if ((float) (cp2.dot(triangleNormal)) >= edge_tolerance) {
       //@BP Mod
       // Triangle normal isn't normalized
       triangleNormal.normalize();
       //@BP Mod - Allow for unflipped normal when raycasting against backfaces
       if (((m_flags & kF_KeepUnflippedNormal) == 0) && (dist_a <= 0.0f)) {
        m_hitFraction = reportHit(new btVector3(triangleNormal).negate(), distance, partId,
         triangleIndex);
       } else {
        m_hitFraction = reportHit(triangleNormal, distance, partId, triangleIndex);
       }
      }
     }
    }
   }
  }
  return true;
 }

 abstract float reportHit(final btVector3 hitNormalLocal, float hitFraction, int partId,
  int triangleIndex);
}
