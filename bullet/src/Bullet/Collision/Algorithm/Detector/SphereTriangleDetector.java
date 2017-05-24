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

package Bullet.Collision.Algorithm.Detector;

import Bullet.Collision.Shape.btSphereShape;
import Bullet.Collision.Shape.btTriangleShape;
import Bullet.Collision.btIDebugDraw;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * sphere-triangle to match the btDiscreteCollisionDetectorInterface
 *
 * @author Gregery Barton
 */
public class SphereTriangleDetector extends btDiscreteCollisionDetectorInterface implements
 Serializable {

 static float SegmentSqrDistance(final btVector3 from, final btVector3 to, final btVector3 p,
  final btVector3 nearest) {
  final btVector3 diff = new btVector3(p).sub(from);
  final btVector3 v = new btVector3(to).sub(from);
  float t = v.dot(diff);
  if (t > 0) {
   float dotVV = v.dot(v);
   if (t < dotVV) {
    t /= dotVV;
    diff.scaleAdd(-t, v, diff);
   } else {
    t = 1;
    diff.sub(v);
   }
  } else {
   t = 0;
  }
  nearest.scaleAdd(t, v, from);
  return diff.dot(diff);
 }
 public final btSphereShape m_sphere;
 public final btTriangleShape m_triangle;
 public float m_contactBreakingThreshold;

 public SphereTriangleDetector(btSphereShape sphere, btTriangleShape triangle,
  float contactBreakingThreshold) {
  m_sphere = sphere;
  m_triangle = triangle;
  m_contactBreakingThreshold = contactBreakingThreshold;
 }

 @Override
 public void getClosestPoints(ClosestPointInput input, Result output, btIDebugDraw debugDraw,
  boolean swapResults) {
  final btTransform transformA = input.m_transformA;
  final btTransform transformB = input.m_transformB;
  final btVector3 point = new btVector3();
  final btVector3 normal = new btVector3();
  float timeOfImpact = (1.f);
  float depth[] = new float[1];
//	output.m_distance = float(BT_LARGE_FLOAT);
//move sphere into triangle space
  final btTransform sphereInTr = transformB.inverseTimes(transformA);
  if (collide(sphereInTr.getOrigin(), point, normal, depth, timeOfImpact, m_contactBreakingThreshold)) {
   if (swapResults) {
    final btVector3 normalOnB = transformB.transform3x3(new btVector3(normal));
    final btVector3 normalOnA = new btVector3(normalOnB).negate();
    final btVector3 pointOnA = new btVector3().scaleAdd(depth[0], normalOnB, transformB.transform(
     new btVector3(point)));
    output.addContactPoint(normalOnA, pointOnA, depth[0]);
   } else {
    output.addContactPoint(transformB.transform3x3(new btVector3(normal)), transformB.transform(
     new btVector3(point)), depth[0]);
   }
  }
 }

 boolean collide(final btVector3 sphereCenter, final btVector3 point, final btVector3 resultNormal,
  float[] depth, float timeOfImpact, float contactBreakingThreshold) {
  final btVector3 vertex0 = m_triangle.getVertexPtr(0);
  final btVector3 vertex1 = m_triangle.getVertexPtr(1);
  final btVector3 vertex2 = m_triangle.getVertexPtr(2);
  float radius = m_sphere.getRadius();
  float radiusWithThreshold = radius + contactBreakingThreshold;
  final btVector3 normal = new btVector3(vertex1).sub(vertex0).cross(new btVector3(vertex2).sub(
   vertex0));
  float l2 = normal.lengthSquared();
  boolean hasContact = false;
  final btVector3 contactPoint = new btVector3();
  if (l2 >= SIMD_EPSILON * SIMD_EPSILON) {
   normal.scale(1.0f / btSqrt(l2));
   final btVector3 p1ToCentre = new btVector3(sphereCenter).sub(vertex0);
   float distanceFromPlane = p1ToCentre.dot(normal);
   if (distanceFromPlane < (0.f)) {
    //triangle facing the other way
    distanceFromPlane *= -1f;
    normal.negate();
   }
   boolean isInsideContactPlane = distanceFromPlane < radiusWithThreshold;
   // Check for contact / intersection
   if (isInsideContactPlane) {
    if (facecontains(sphereCenter, m_triangle.getVertexPtr(), normal)) {
     // Inside the contact wedge - touches a point on the shell plane
     hasContact = true;
     contactPoint.scaleAdd(-distanceFromPlane, normal, sphereCenter);
    } else {
     // Could be inside one of the contact capsules
     float contactCapsuleRadiusSqr = radiusWithThreshold * radiusWithThreshold;
     final btVector3 nearestOnEdge = new btVector3();
     for (int i = 0; i < m_triangle.getNumEdges(); i++) {
      final btVector3 pa = new btVector3();
      final btVector3 pb = new btVector3();
      m_triangle.getEdge(i, pa, pb);
      float distanceSqr = SegmentSqrDistance(pa, pb, sphereCenter, nearestOnEdge);
      if (distanceSqr < contactCapsuleRadiusSqr) {
       // Yep, we're inside a capsule
       hasContact = true;
       contactPoint.set(nearestOnEdge);
      }
     }
    }
   }
  }
  if (hasContact) {
   final btVector3 contactToCentre = new btVector3(sphereCenter).sub(contactPoint);
   float distanceSqr = contactToCentre.lengthSquared();
   if (distanceSqr < radiusWithThreshold * radiusWithThreshold) {
    if (distanceSqr > SIMD_EPSILON) {
     float distance = btSqrt(distanceSqr);
     resultNormal.set(contactToCentre);
     resultNormal.normalize();
     point.set(contactPoint);
     depth[0] = -(radius - distance);
    } else {
     resultNormal.set(normal);
     point.set(contactPoint);
     depth[0] = -radius;
    }
    return true;
   }
  }
  return false;
 }

 boolean facecontains(final btVector3 p, btVector3[] vertices, final btVector3 normal) {
  final btVector3 lp = new btVector3(p);
  final btVector3 lnormal = new btVector3(normal);
  return pointInTriangle(vertices, lnormal, lp);
 }

 boolean pointInTriangle(btVector3 vertices[], final btVector3 normal, final btVector3 p) {
  final btVector3 p1 = vertices[0];
  final btVector3 p2 = vertices[1];
  final btVector3 p3 = vertices[2];
  final btVector3 edge1 = new btVector3(p2).sub(p1);
  final btVector3 edge2 = new btVector3(p3).sub(p2);
  final btVector3 edge3 = new btVector3(p1).sub(p3);
  final btVector3 p1_to_p = new btVector3(p).sub(p1);
  final btVector3 p2_to_p = new btVector3(p).sub(p2);
  final btVector3 p3_to_p = new btVector3(p).sub(p3);
  final btVector3 edge1_normal = new btVector3(edge1).cross(normal);
  final btVector3 edge2_normal = new btVector3(edge2).cross(normal);
  final btVector3 edge3_normal = new btVector3(edge3).cross(normal);
  float r1, r2, r3;
  r1 = edge1_normal.dot(p1_to_p);
  r2 = edge2_normal.dot(p2_to_p);
  r3 = edge3_normal.dot(p3_to_p);
  return (r1 > 0 && r2 > 0 && r3 > 0) ||
   (r1 <= 0 && r2 <= 0 && r3 <= 0);
 }
};
