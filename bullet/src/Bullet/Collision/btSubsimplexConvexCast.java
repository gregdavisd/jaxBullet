/*
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it freely,
 * subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.Collision;

import Bullet.Collision.Shape.btConvexShape;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import static javax.vecmath.VecMath.DEBUG_BLOCKS;
import static javax.vecmath.VecMath.is_good_matrix;

/**
 * btSubsimplexConvexCast implements Gino van den Bergens' paper "Ray Casting
 * against bteral Convex Objects with Application to Continuous Collision
 * Detection" GJK based Ray Cast, optimized version Objects should not start in
 * overlap, otherwise results are not defined.
 *
 * @author Gregery Barton
 */
public class btSubsimplexConvexCast extends btConvexCast implements Serializable {

 static final int MAX_ITERATIONS = 32;
 final btSimplexSolverInterface m_simplexSolver;
 final btConvexShape m_convexA;
 final btConvexShape m_convexB;

 btSubsimplexConvexCast(btConvexShape convexA, btConvexShape convexB,
  btSimplexSolverInterface simplexSolver) {
  m_simplexSolver = simplexSolver;
  m_convexA = convexA;
  m_convexB = convexB;
 }

 //  ~btSubsimplexConvexCast();
 ///SimsimplexConvexCast calculateTimeOfImpact calculates the time of impact+normal for the linear cast (sweep) between two moving objects.
 ///Precondition is that objects should not penetration/overlap at the start from the interval. Overlap can be tested using btGjkPairDetector.
 /**
  *
  * @param fromA
  * @param toA
  * @param fromB
  * @param toB
  * @param result
  * @return
  */
 @Override
 boolean calcTimeOfImpact(
  final btTransform fromA, final btTransform toA, final btTransform fromB,
  final btTransform toB,
  CastResult result) {
  if (DEBUG_BLOCKS) {
   assert (is_good_matrix(fromA));
   assert (is_good_matrix(toA));
   assert (is_good_matrix(fromB));
   assert (is_good_matrix(toB));
  }
  m_simplexSolver.reset();
  final btVector3 linVelA = toA.getOrigin().sub(fromA.getOrigin());
  final btVector3 linVelB = toB.getOrigin().sub(fromB.getOrigin());
  float lambda = (0.f);
  final btTransform interpolatedTransA = new btTransform(fromA);
  final btTransform interpolatedTransB = new btTransform(fromB);
  ///take relative motion
  final btVector3 r = new btVector3(linVelA).sub(linVelB);
  final btVector3 v;
  final btVector3 supVertexA = fromA.transform(m_convexA
   .localGetSupportingVertex(fromA
    .transposeTransform3x3(
     new btVector3(r).negate())));
  final btVector3 supVertexB = fromB.transform(m_convexB
   .localGetSupportingVertex(fromB
    .transposeTransform3x3(
     new btVector3(r))));
  v = new btVector3(supVertexA).sub(supVertexB);
  int maxIter = MAX_ITERATIONS;
  final btVector3 n = new btVector3();
  final btVector3 c;
  float dist2 = v.lengthSquared();
  float epsilon = (0.0001f);
  final btVector3 w = new btVector3();
  float VdotR;
  while ((dist2 > epsilon) && (maxIter-- > 0)) {
   supVertexA.set(interpolatedTransA.transform(m_convexA
    .localGetSupportingVertex(interpolatedTransA
     .transposeTransform3x3(new btVector3(v).negate()))));
   supVertexB.set(interpolatedTransB.transform(m_convexB
    .localGetSupportingVertex(interpolatedTransB
     .transposeTransform3x3(new btVector3(v)))));
   w.set(supVertexA).sub(supVertexB);
   float VdotW = v.dot(w);
   if (lambda > (1.0f)) {
    return false;
   }
   if (VdotW > (0.f)) {
    VdotR = v.dot(r);
    if (VdotR >= -(SIMD_EPSILON * SIMD_EPSILON)) {
     return false;
    } else {
     lambda -= VdotW / VdotR;
     assert (lambda >= 0);
     //interpolate to next lambda
     //	x = s + lambda * r;
     interpolatedTransA.setOrigin(new btVector3().mix(fromA.getOrigin(), toA
      .getOrigin(), lambda));
     interpolatedTransB.setOrigin(new btVector3().mix(fromB.getOrigin(), toB
      .getOrigin(), lambda));
     //m_simplexSolver.reset();
     //check next line
     //w.set(supVertexA).sub(supVertexB);
     n.set(v);
    }
   }
   ///Just like regular GJK only add the vertex if it isn't already (close) to current vertex, it would lead to divisions by zero and NaN etc.
   if (!m_simplexSolver.inSimplex(w)) {
    m_simplexSolver.addVertex(w, supVertexA, supVertexB);
   }
   if (m_simplexSolver.closest(v)) {
    dist2 = v.lengthSquared();
    //todo: check this normal for validity
    //n=v;
    //printf("V=%f , %f, %f\n",v[0],v[1],v[2]);
    //printf("DIST2=%f\n",dist2);
    //printf("numverts = %i\n",m_simplexSolver.numVertices());
   } else {
    dist2 = (0.f);
   }
  }
  //int numiter = MAX_ITERATIONS - maxIter;
//	printf("number of iterations: %d", numiter);
  //don't report a time of impact when moving 'away' from the hitnormal
  result.m_fraction = lambda;
  if (n.lengthSquared() >= (SIMD_EPSILON * SIMD_EPSILON)) {
   result.m_normal.set(n).normalize();
  } else {
   result.m_normal.setZero();
  }
  //don't report time of impact for motion away from the contact normal (or causes minor penetration)
  if (result.m_normal.dot(r) >= -result.m_allowedPenetration) {
   return false;
  }
  final btVector3 hitA = new btVector3();
  final btVector3 hitB = new btVector3();
  m_simplexSolver.compute_points(hitA, hitB);
  result.m_hitPoint.set(hitB);
  return true;
 }

};
