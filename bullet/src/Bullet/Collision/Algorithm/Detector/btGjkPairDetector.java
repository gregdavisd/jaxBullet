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
package Bullet.Collision.Algorithm.Detector;

import Bullet.Collision.Shape.btConvexShape;
import Bullet.Collision.btConvexPenetrationDepthSolver;
import Bullet.Collision.btIDebugDraw;
import Bullet.Collision.btSimplexSolverInterface;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btGjkPairDetector extends btDiscreteCollisionDetectorInterface
 implements Serializable {

 static final float REL_ERROR2 = 1.0e-6f;
 static float gGjkEpaPenetrationTolerance = 0.001f;
 static int gNumDeepPenetrationChecks = 0;
 static int gNumGjkChecks = 0;
 final btVector3 m_cachedSeparatingAxis = new btVector3();
 btConvexPenetrationDepthSolver m_penetrationDepthSolver;
 final btSimplexSolverInterface m_simplexSolver;
 btConvexShape m_minkowskiA;
 btConvexShape m_minkowskiB;
 int m_shapeTypeA;
 int m_shapeTypeB;
 float m_marginA;
 float m_marginB;
 boolean m_ignoreMargin;
 float m_cachedSeparatingDistance;
 //some debugging to fix degeneracy problems
 int m_lastUsedMethod;
 int m_curIter;
 int m_degenerateSimplex;
 int m_catchDegeneracies;
 int m_fixContactNormalDirection;

 public btGjkPairDetector(btConvexShape objectA, btConvexShape objectB,
  btSimplexSolverInterface simplexSolver,
  btConvexPenetrationDepthSolver penetrationDepthSolver) {
  m_cachedSeparatingAxis.set(0f, 1f, 0f);
  m_penetrationDepthSolver = penetrationDepthSolver;
  m_simplexSolver = simplexSolver;
  m_minkowskiA = objectA;
  m_minkowskiB = objectB;
  m_shapeTypeA = objectA.getShapeType();
  m_shapeTypeB = objectB.getShapeType();
  m_marginA = objectA.getMargin();
  m_marginB = objectB.getMargin();
  m_ignoreMargin = false;
  m_lastUsedMethod = -1;
  m_catchDegeneracies = 1;
  m_fixContactNormalDirection = 1;
 }

 public btGjkPairDetector(btConvexShape objectA, btConvexShape objectB,
  int shapeTypeA,
  int shapeTypeB,
  float marginA, float marginB, btSimplexSolverInterface simplexSolver,
  btConvexPenetrationDepthSolver penetrationDepthSolver) {
  m_cachedSeparatingAxis.set(0f, 1f, 0f);
  m_penetrationDepthSolver = penetrationDepthSolver;
  m_simplexSolver = simplexSolver;
  m_minkowskiA = objectA;
  m_minkowskiB = objectB;
  m_shapeTypeA = shapeTypeA;
  m_shapeTypeB = shapeTypeB;
  m_marginA = marginA;
  m_marginB = marginB;
  m_ignoreMargin = false;
  m_lastUsedMethod = -1;
  m_catchDegeneracies = 1;
  m_fixContactNormalDirection = 1;
 }

 @Override
 public void getClosestPoints(ClosestPointInput input, Result output,
  btIDebugDraw debugDraw,
  boolean swapResults) {
  getClosestPointsNonVirtual(input, output, debugDraw);
 }

 void getClosestPointsNonVirtual(ClosestPointInput input, Result output,
  btIDebugDraw debugDraw) {
  m_cachedSeparatingDistance = 0.f;
  float distance = 0f;
  final btVector3 normalInB = new btVector3();
  final btVector3 pointOnA = new btVector3();
  final btVector3 pointOnB = new btVector3();
  final btTransform localTransA = new btTransform(input.m_transformA);
  final btTransform localTransB = new btTransform(input.m_transformB);
  final btVector3 positionOffset = localTransA.getOrigin().add(localTransB
   .getOrigin()).scale(0.5f);
  localTransA.setOrigin(localTransA.getOrigin().sub(positionOffset));
  localTransB.setOrigin(localTransB.getOrigin().sub(positionOffset));
  boolean check2d = m_minkowskiA.isConvex2d() && m_minkowskiB.isConvex2d();
  float marginA = m_marginA;
  float marginB = m_marginB;
  gNumGjkChecks++;
  //for CCD we don't use margins
  if (m_ignoreMargin) {
   marginA = 0f;
   marginB = 0f;
  }
  m_curIter = 0;
  int gGjkMaxIter = 1000;//this is to catch invalid input, perhaps check for #NaN?
  m_cachedSeparatingAxis.set(0f, 1f, 0f);
  boolean isValid = false;
  boolean checkSimplex = false;
  boolean checkPenetration = true;
  m_degenerateSimplex = 0;
  m_lastUsedMethod = -1;
  {
   float squaredDistance = BT_LARGE_FLOAT;
   float delta;
   float margin = marginA + marginB;
   m_simplexSolver.reset();
   for (;;) //while (true)
   {
    final btVector3 seperatingAxisInA = input.m_transformA
     .transposeTransform3x3(new btVector3(
      m_cachedSeparatingAxis).negate());
    final btVector3 seperatingAxisInB = input.m_transformB
     .transposeTransform3x3(new btVector3(
      m_cachedSeparatingAxis));
    final btVector3 pInA = m_minkowskiA
     .localGetSupportVertexWithoutMarginNonVirtual(
      seperatingAxisInA);
    final btVector3 qInB = m_minkowskiB
     .localGetSupportVertexWithoutMarginNonVirtual(
      seperatingAxisInB);
    final btVector3 pWorld = localTransA.transform(new btVector3(pInA));
    final btVector3 qWorld = localTransB.transform(new btVector3(qInB));
    if (check2d) {
     pWorld.z = 0.f;
     qWorld.z = 0.f;
    }
    final btVector3 w = new btVector3(pWorld).sub(qWorld);
    delta = m_cachedSeparatingAxis.dot(w);
    // potential exit, they don't overlap
    if ((delta > 0.0f) && (delta * delta > squaredDistance
     * input.m_maximumDistanceSquared)) {
     m_degenerateSimplex = 10;
     checkSimplex = true;
     //checkPenetration = false;
     break;
    }
    //exit 0: the new point is already in the simplex, or we didn't come any closer
    if (m_simplexSolver.inSimplex(w)) {
     m_degenerateSimplex = 1;
     checkSimplex = true;
     break;
    }
    // are we getting any closer ?
    float f0 = squaredDistance - delta;
    float f1 = squaredDistance * REL_ERROR2;
    if (f0 <= f1) {
     if (f0 <= 0.f) {
      m_degenerateSimplex = 2;
     } else {
      m_degenerateSimplex = 11;
     }
     checkSimplex = true;
     break;
    }
    //add current vertex to simplex
    m_simplexSolver.addVertex(w, pWorld, qWorld);
    final btVector3 newCachedSeparatingAxis = new btVector3();
    //calculate the closest point to the origin (update vector v)
    if (!m_simplexSolver.closest(newCachedSeparatingAxis)) {
     m_degenerateSimplex = 3;
     checkSimplex = true;
     break;
    }
    if (newCachedSeparatingAxis.lengthSquared() < REL_ERROR2) {
     m_cachedSeparatingAxis.set(newCachedSeparatingAxis);
     m_degenerateSimplex = 6;
     checkSimplex = true;
     break;
    }
    float previousSquaredDistance = squaredDistance;
    squaredDistance = newCachedSeparatingAxis.lengthSquared();
    //redundant m_simplexSolver.compute_points(pointOnA, pointOnB);
    //are we getting any closer ?
    if (previousSquaredDistance - squaredDistance <= SIMD_EPSILON
     * previousSquaredDistance) {
//				m_simplexSolver.backup_closest(m_cachedSeparatingAxis);
     checkSimplex = true;
     m_degenerateSimplex = 12;
     break;
    }
    m_cachedSeparatingAxis.set(newCachedSeparatingAxis);
    //degeneracy, this is typically due to invalid/uninitialized worldtransforms for a btCollisionObject   
    if (m_curIter++ > gGjkMaxIter) {
     System.out.printf("btGjkPairDetector maxIter exceeded:%d\n", m_curIter);
     System.out.printf(
      "sepAxis=(%f,%f,%f), squaredDistance = %f, shapeTypeA=%d,shapeTypeB=%d\n",
      m_cachedSeparatingAxis.getX(),
      m_cachedSeparatingAxis.getY(),
      m_cachedSeparatingAxis.getZ(),
      squaredDistance,
      m_minkowskiA.getShapeType(),
      m_minkowskiB.getShapeType());
     break;
    }
    boolean check = (!m_simplexSolver.fullSimplex());
    //boolean check = (!m_simplexSolver.fullSimplex() && squaredDistance > SIMD_EPSILON * m_simplexSolver.maxVertex());
    if (!check) {
     //do we need this backup_closest here ?
//				m_simplexSolver.backup_closest(m_cachedSeparatingAxis);
     m_degenerateSimplex = 13;
     break;
    }
   }
   if (checkSimplex) {
    m_simplexSolver.compute_points(pointOnA, pointOnB);
    normalInB.set(m_cachedSeparatingAxis);
    float lenSqr = m_cachedSeparatingAxis.lengthSquared();
    //valid normal
    if (lenSqr < REL_ERROR2) {
     m_degenerateSimplex = 5;
    }
    if (lenSqr > SIMD_EPSILON * SIMD_EPSILON) {
     float rlen = 1f / btSqrt(lenSqr);
     normalInB.scale(rlen); //normalize
     float s = btSqrt(squaredDistance);
     assert (s > (0.0));
     pointOnA.sub(new btVector3(m_cachedSeparatingAxis).scale((marginA / s)));
     pointOnB.add(new btVector3(m_cachedSeparatingAxis).scale((marginB / s)));
     distance = ((1.f) / rlen) - margin;
     isValid = true;
     m_lastUsedMethod = 1;
    } else {
     m_lastUsedMethod = 2;
    }
   }
   boolean catchDegeneratePenetrationCase
    = (m_catchDegeneracies != 0 && m_penetrationDepthSolver != null
    && m_degenerateSimplex != 0 && ((distance + margin)
    < gGjkEpaPenetrationTolerance));
   //if (checkPenetration && !isValid)
   if (checkPenetration && (!isValid || catchDegeneratePenetrationCase)) {
    //penetration case
    //if there is no way to handle penetrations, bail out
    if (m_penetrationDepthSolver != null) {
     // Penetration depth case.
     final btVector3 tmpPointOnA = new btVector3();
     final btVector3 tmpPointOnB = new btVector3();
     gNumDeepPenetrationChecks++;
     m_cachedSeparatingAxis.setZero();
     boolean isValid2 = m_penetrationDepthSolver.calcPenDepth(
      m_simplexSolver,
      m_minkowskiA, m_minkowskiB,
      localTransA, localTransB,
      m_cachedSeparatingAxis, tmpPointOnA, tmpPointOnB,
      debugDraw
     );
     if (isValid2) {
      final btVector3 tmpNormalInB = new btVector3(tmpPointOnB).sub(tmpPointOnA);
      float lenSqr = tmpNormalInB.lengthSquared();
      if (lenSqr <= (SIMD_EPSILON * SIMD_EPSILON)) {
       tmpNormalInB.set(m_cachedSeparatingAxis);
       lenSqr = m_cachedSeparatingAxis.lengthSquared();
      }
      if (lenSqr > (SIMD_EPSILON * SIMD_EPSILON)) {
       tmpNormalInB.scale(1.0f / btSqrt(lenSqr));
       float distance2 = -(new btVector3(tmpPointOnA).sub(tmpPointOnB)).length();
       m_lastUsedMethod = 3;
       //only replace valid penetrations when the result is deeper (check)
       if (!isValid || (distance2 < distance)) {
        distance = distance2;
        pointOnA.set(tmpPointOnA);
        pointOnB.set(tmpPointOnB);
        normalInB.set(tmpNormalInB);
        isValid = true;
       } else {
        m_lastUsedMethod = 8;
       }
      } else {
       m_lastUsedMethod = 9;
      }
     } else /*
      * this is another degenerate case, where the initial GJK calculation
      * reports a degenerate case EPA reports no penetration, and the second GJK
      * (using the supporting vector without margin) reports a valid positive
      * distance. Use the results of the second GJK instead of failing. thanks
      * to Jacob.Langford for the reproduction case
      * http://code.google.com/p/bullet/issues/detail?id=250
      */ if (m_cachedSeparatingAxis.lengthSquared() > 0f) {
      float distance2 = (new btVector3(tmpPointOnA).sub(tmpPointOnB)).length()
       - margin;
      //only replace valid distances when the distance is less
      if (!isValid || (distance2 < distance)) {
       distance = distance2;
       pointOnA.set(tmpPointOnA);
       pointOnB.set(tmpPointOnB);
       pointOnA.sub(new btVector3(m_cachedSeparatingAxis).scale(marginA));
       pointOnB.add(new btVector3(m_cachedSeparatingAxis).scale(marginB));
       normalInB.set(m_cachedSeparatingAxis);
       normalInB.normalize();
       isValid = true;
       m_lastUsedMethod = 6;
      } else {
       m_lastUsedMethod = 5;
      }
     }
    }
   }
  }
  if (isValid && ((distance < 0) || (distance * distance
   < input.m_maximumDistanceSquared))) {
   m_cachedSeparatingAxis.set(normalInB);
   m_cachedSeparatingDistance = distance;
   {
    ///todo: need to track down this EPA penetration solver degeneracy
    ///the penetration solver reports penetration but the contact normal
    ///connecting the contact points is pointing in the opposite direction
    ///until then, detect the issue and revert the normal
    float d1;
    {
     final btVector3 seperatingAxisInA = input.m_transformA
      .transposeTransform3x3(new btVector3(
       normalInB));
     final btVector3 seperatingAxisInB = input.m_transformB
      .transposeTransform3x3(new btVector3(
       normalInB).negate());
     final btVector3 pInA = m_minkowskiA
      .localGetSupportVertexWithoutMarginNonVirtual(
       seperatingAxisInA);
     final btVector3 qInB = m_minkowskiB
      .localGetSupportVertexWithoutMarginNonVirtual(
       seperatingAxisInB);
     final btVector3 pWorld = localTransA.transform(new btVector3(pInA));
     final btVector3 qWorld = localTransB.transform(new btVector3(qInB));
     final btVector3 w = new btVector3(pWorld).sub(qWorld);
     d1 = new btVector3(normalInB).negate().dot(w);
    }
    float d0;
    {
     final btVector3 seperatingAxisInA = input.m_transformA
      .transposeTransform3x3(new btVector3(
       normalInB)
       .negate());
     final btVector3 seperatingAxisInB = input.m_transformB
      .transposeTransform3x3(new btVector3(
       normalInB));
     final btVector3 pInA = m_minkowskiA
      .localGetSupportVertexWithoutMarginNonVirtual(
       seperatingAxisInA);
     final btVector3 qInB = m_minkowskiB
      .localGetSupportVertexWithoutMarginNonVirtual(
       seperatingAxisInB);
     final btVector3 pWorld = localTransA.transform(new btVector3(pInA));
     final btVector3 qWorld = localTransB.transform(new btVector3(qInB));
     final btVector3 w = new btVector3(pWorld).sub(qWorld);
     d0 = normalInB.dot(w);
    }
    if (d1 > d0) {
     m_lastUsedMethod = 10;
     normalInB.negate();
    }
   }
   output.addContactPoint(
    normalInB,
    new btVector3(pointOnB).add(positionOffset),
    distance);
  }
 }

 public void setMinkowskiA(btConvexShape minkA) {
  m_minkowskiA = minkA;
 }

 public void setMinkowskiB(btConvexShape minkB) {
  m_minkowskiB = minkB;
 }

 public void setCachedSeperatingAxis(final btVector3 seperatingAxis) {
  m_cachedSeparatingAxis.set(seperatingAxis);
 }

 public btVector3 getCachedSeparatingAxis() {
  return new btVector3(m_cachedSeparatingAxis);
 }

 public float getCachedSeparatingDistance() {
  return m_cachedSeparatingDistance;
 }

 void setPenetrationDepthSolver(
  btConvexPenetrationDepthSolver penetrationDepthSolver) {
  m_penetrationDepthSolver = penetrationDepthSolver;
 }

 ///don't use setIgnoreMargin, it's for Bullet's internal use
 void setIgnoreMargin(boolean ignoreMargin) {
  m_ignoreMargin = ignoreMargin;
 }

};
