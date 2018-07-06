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

import Bullet.Collision.Algorithm.Detector.btGjkPairDetector;
import Bullet.Collision.Shape.btConvexShape;
import Bullet.Collision.Shape.btStaticPlaneShape;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btTransformUtil;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btContinuousConvexCollision extends btConvexCast implements
 Serializable {

 static final int MAX_ITERATIONS = 64;
 final btSimplexSolverInterface m_simplexSolver;
 final btConvexPenetrationDepthSolver m_penetrationDepthSolver;
 final btConvexShape m_convexA;
 //second object is either a convex or a plane (code sharing)
 final btConvexShape m_convexB1;
 final btStaticPlaneShape m_planeShape;

 btContinuousConvexCollision(
  btConvexShape convexA,
  btConvexShape convexB,
  btSimplexSolverInterface simplexSolver,
  btConvexPenetrationDepthSolver penetrationDepthSolver) {
  super();
  m_simplexSolver = simplexSolver;
  m_penetrationDepthSolver = penetrationDepthSolver;
  m_convexA = convexA;
  m_convexB1 = convexB;
  m_planeShape = null;
 }

 btContinuousConvexCollision(btConvexShape convexA, btStaticPlaneShape plane) {
  m_simplexSolver = null;
  m_penetrationDepthSolver = null;
  m_convexA = convexA;
  m_convexB1 = null;
  m_planeShape = plane;
 }

 void computeClosestPoints(final btTransform transA, final btTransform transB,
  btPointCollector pointCollector) {
  if (m_convexB1 != null) {
   m_simplexSolver.reset();
   btGjkPairDetector gjk = new btGjkPairDetector(m_convexA, m_convexB1,
    m_convexA.getShapeType(),
    m_convexB1.getShapeType(), m_convexA.getMargin(), m_convexB1.getMargin(),
    m_simplexSolver,
    m_penetrationDepthSolver);
   btGjkPairDetector.ClosestPointInput input = new btGjkPairDetector.ClosestPointInput();
   input.m_transformA.set(transA);
   input.m_transformB.set(transB);
   gjk.getClosestPoints(input, pointCollector, null);
  } else {
   //convex versus plane
   btConvexShape convexShape = m_convexA;
   btStaticPlaneShape planeShape = m_planeShape;
   final btVector3 planeNormal = planeShape.getPlaneNormal();
   final float planeConstant = planeShape.getPlaneConstant();
   final btTransform convexWorldTransform = new btTransform(transA);
   final btTransform convexInPlaneTrans = new btTransform(transB).invert().mul(
    convexWorldTransform);
   final btTransform planeInConvex = new btTransform(convexWorldTransform)
    .invert().mul(transB);
   final btVector3 vtx = convexShape.localGetSupportingVertex(planeInConvex
    .transform3x3(
     new btVector3(planeNormal).negate()));
   final btVector3 vtxInPlane = convexInPlaneTrans.transform(new btVector3(vtx));
   float distance = (planeNormal.dot(vtxInPlane) - planeConstant);
   final btVector3 vtxInPlaneProjected = new btVector3()
    .scaleAdd(-distance, planeNormal, vtxInPlane);
   final btVector3 vtxInPlaneWorld = transB.transform(new btVector3(
    vtxInPlaneProjected));
   final btVector3 normalOnSurfaceB = transB.transform3x3(new btVector3(
    planeNormal));
   pointCollector.addContactPoint(
    normalOnSurfaceB,
    vtxInPlaneWorld,
    distance);
  }
 }

 @Override
 boolean calcTimeOfImpact(
  final btTransform fromA, final btTransform toA, final btTransform fromB,
  final btTransform toB,
  CastResult result) {
  /// compute linear and angular velocity for this interval, to interpolate
  final btVector3 linVelA = new btVector3();
  final btVector3 angVelA = new btVector3();
  final btVector3 linVelB = new btVector3();
  final btVector3 angVelB = new btVector3();
  btTransformUtil.calculateVelocity(fromA, toA, 1f, linVelA, angVelA);
  btTransformUtil.calculateVelocity(fromB, toB, 1f, linVelB, angVelB);
  float boundingRadiusA = m_convexA.getAngularMotionDisc();
  float boundingRadiusB = m_convexB1 != null ? m_convexB1.getAngularMotionDisc() : 0.f;
  float maxAngularProjectedVelocity = angVelA.length() * boundingRadiusA
   + angVelB.length() * boundingRadiusB;
  final btVector3 relLinVel = new btVector3(linVelB).sub(linVelA);
  float relLinVelocLength = relLinVel.length();
  if ((relLinVelocLength + maxAngularProjectedVelocity) == 0.f) {
   return false;
  }
  float lambda = 0f;
  //final btVector3 v = new btVector3();
  int maxIter = MAX_ITERATIONS;
  final btVector3 n = new btVector3();
  boolean hasResult = false;
  final btVector3 c = new btVector3();
  float lastLambda = lambda;
  //float epsilon = float(0.001);
  int numIter = 0;
  //first solution, using GJK
  float radius = 0.001f;
//	result.drawCoordSystem(sphereTr);
  btPointCollector pointCollector1 = new btPointCollector();
  {
   computeClosestPoints(fromA, fromB, pointCollector1);
   hasResult = pointCollector1.m_hasResult;
   c.set(pointCollector1.m_pointInWorld);
  }
  if (hasResult) {
   float dist;
   dist = pointCollector1.m_distance + result.m_allowedPenetration;
   n.set(pointCollector1.m_normalOnBInWorld);
   float projectedLinearVelocity = relLinVel.dot(n);
   if ((projectedLinearVelocity + maxAngularProjectedVelocity) <= SIMD_EPSILON) {
    return false;
   }
   //not close enough
   while (dist > radius) {
    if (result.m_debugDrawer != null) {
     result.m_debugDrawer.drawSphere(c, 0.2f, new btVector3(1, 1, 1));
    }
    projectedLinearVelocity = relLinVel.dot(n);
    //don't report time of impact for motion away from the contact normal (or causes minor penetration)
    if ((projectedLinearVelocity + maxAngularProjectedVelocity) <= SIMD_EPSILON) {
     return false;
    }
    float dLambda = dist / (projectedLinearVelocity
     + maxAngularProjectedVelocity);
    lambda += dLambda;
    if (lambda > (1.f)) {
     return false;
    }
    if (lambda < (0.f)) {
     return false;
    }
    //todo: next check with relative epsilon
    if (lambda <= lastLambda) {
     return false;
    }
    lastLambda = lambda;
    //interpolate to next lambda
    final btTransform interpolatedTransA = new btTransform();
    final btTransform interpolatedTransB = new btTransform();
    btTransformUtil.integrateTransform(fromA, linVelA, angVelA, lambda,
     interpolatedTransA);
    btTransformUtil.integrateTransform(fromB, linVelB, angVelB, lambda,
     interpolatedTransB);
    //btTransform relativeTrans = interpolatedTransB.inverseTimes(interpolatedTransA);
    if (result.m_debugDrawer != null) {
     result.m_debugDrawer.drawSphere(interpolatedTransA.getOrigin(), 0.2f,
      new btVector3(1, 0, 0));
    }
    result.DebugDraw(lambda);
    btPointCollector pointCollector = new btPointCollector();
    computeClosestPoints(interpolatedTransA, interpolatedTransB, pointCollector);
    if (pointCollector.m_hasResult) {
     dist = pointCollector.m_distance + result.m_allowedPenetration;
     c.set(pointCollector.m_pointInWorld);
     n.set(pointCollector.m_normalOnBInWorld);
    } else {
     result.reportFailure(-1, numIter);
     return false;
    }
    numIter++;
    if (numIter > maxIter) {
     result.reportFailure(-2, numIter);
     return false;
    }
   }
   result.m_fraction = lambda;
   result.m_normal.set(n);
   result.m_hitPoint.set(c);
   return true;
  }
  return false;
 }

}
