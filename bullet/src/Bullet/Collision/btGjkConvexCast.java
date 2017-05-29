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

import Bullet.Collision.Algorithm.Detector.btGjkPairDetector;
import Bullet.Collision.Shape.btConvexShape;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btGjkConvexCast extends btConvexCast implements Serializable {

 static final int MAX_ITERATIONS = 32;
 btSimplexSolverInterface m_simplexSolver;
 final btConvexShape m_convexA;
 final btConvexShape m_convexB;

 public btGjkConvexCast(btConvexShape convexA, btConvexShape convexB,
  btSimplexSolverInterface simplexSolver) {
  m_simplexSolver = simplexSolver;
  m_convexA = convexA;
  m_convexB = convexB;
 }

 /// cast a convex against another convex object
 @Override
 public boolean calcTimeOfImpact(
  final btTransform fromA, final btTransform toA, final btTransform fromB, final btTransform toB,
  CastResult result) {
  m_simplexSolver.reset();
  /// compute linear velocity for this interval, to interpolate
  //assume no rotation/angular velocity, assert here?
  final btVector3 linVelA;
  final btVector3 linVelB;
  linVelA = toA.getOrigin().sub(fromA.getOrigin());
  linVelB = toB.getOrigin().sub(fromB.getOrigin());
  float radius = (0.001f);
  float lambda = (0.f);
  final btVector3 v = new btVector3(1f, 0f, 0f);
  int maxIter = MAX_ITERATIONS;
  final btVector3 n = new btVector3();
  boolean hasResult;
  final btVector3 c = new btVector3();
  final btVector3 r = new btVector3(linVelA).sub(linVelB);
  float lastLambda = lambda;
  //float epsilon = float(0.001);
  int numIter = 0;
  //first solution, using GJK
  final btTransform identityTrans = new btTransform();
  identityTrans.setIdentity();
  btPointCollector pointCollector = new btPointCollector();
  btGjkPairDetector gjk = new btGjkPairDetector(m_convexA, m_convexB, m_simplexSolver, null);//m_penetrationDepthSolver);		
  btGjkPairDetector.ClosestPointInput input = new btGjkPairDetector.ClosestPointInput();
  //we don't use margins during CCD
  //	gjk.setIgnoreMargin(true);
  input.m_transformA.set(fromA);
  input.m_transformB.set(fromB);
  gjk.getClosestPoints(input, pointCollector, null);
  hasResult = pointCollector.m_hasResult;
  c.set(pointCollector.m_pointInWorld);
  if (hasResult) {
   float dist;
   dist = pointCollector.m_distance;
   n.set(pointCollector.m_normalOnBInWorld);
   //not close enough
   while (dist > radius) {
    numIter++;
    if (numIter > maxIter) {
     return false; //todo: report a failure
    }
    float dLambda;
    float projectedLinearVelocity = r.dot(n);
    dLambda = dist / (projectedLinearVelocity);
    lambda -= dLambda;
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
    result.DebugDraw(lambda);
    input.m_transformA.setOrigin(new btVector3().mix(fromA.getOrigin(), toA.getOrigin(), lambda));
    input.m_transformB.setOrigin(new btVector3().mix(fromB.getOrigin(), toB.getOrigin(), lambda));
    gjk.getClosestPoints(input, pointCollector, null);
    if (pointCollector.m_hasResult) {
     if (pointCollector.m_distance < (0.f)) {
      result.m_fraction = lastLambda;
      n.set(pointCollector.m_normalOnBInWorld);
      result.m_normal.set(n);
      result.m_hitPoint.set(pointCollector.m_pointInWorld);
      return true;
     }
     c.set(pointCollector.m_pointInWorld);
     n.set(pointCollector.m_normalOnBInWorld);
     dist = pointCollector.m_distance;
    } else {
     //??
     return false;
    }
   }
   //is n normalized?
   //don't report time of impact for motion away from the contact normal (or causes minor penetration)
   if (n.dot(r) >= -result.m_allowedPenetration) {
    return false;
   }
   result.m_fraction = lambda;
   result.m_normal.set(n);
   result.m_hitPoint.set(c);
   return true;
  }
  return false;
 }
}
