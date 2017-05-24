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
import static Bullet.Collision.Shape.btConvexShape.MAX_PREFERRED_PENETRATION_DIRECTIONS;
import static Bullet.Collision.btShapeHull.NUM_UNITSPHERE_POINTS;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.init;
import java.io.Serializable;
import java.util.Arrays;

/**
 *
 * @author Gregery Barton
 */
class btMinkowskiPenetrationDepthSolver implements btConvexPenetrationDepthSolver  , Serializable {

 @Override
 public boolean calcPenDepth(btSimplexSolverInterface simplexSolver,
  btConvexShape convexA, btConvexShape convexB, final btTransform transA, final btTransform transB,
  final btVector3 v, final btVector3 pa, final btVector3 pb,
  btIDebugDraw debugDraw
 ) {
  boolean check2d = convexA.isConvex2d() && convexB.isConvex2d();
  //just take fixed number of orientation, and sample the penetration depth in that direction
  float minProj = (BT_LARGE_FLOAT);
  final btVector3 minNorm = new btVector3();
  final btVector3 minA = new btVector3();
  final btVector3 minB = new btVector3();
//  btVector3 seperatingAxisInA = new btVector3();
//  btVector3 seperatingAxisInB = new btVector3();
  final btVector3 pInA = new btVector3();
  final btVector3 qInB = new btVector3();
  final btVector3 pWorld = new btVector3();
  final btVector3 qWorld = new btVector3();
  final btVector3 w = new btVector3();
  btVector3[] supportVerticesABatch = new btVector3[NUM_UNITSPHERE_POINTS +
   MAX_PREFERRED_PENETRATION_DIRECTIONS * 2];
  btVector3[] supportVerticesBBatch = new btVector3[NUM_UNITSPHERE_POINTS +
   MAX_PREFERRED_PENETRATION_DIRECTIONS * 2];
  btVector3[] seperatingAxisInABatch = new btVector3[NUM_UNITSPHERE_POINTS +
   MAX_PREFERRED_PENETRATION_DIRECTIONS * 2];
  btVector3[] seperatingAxisInBBatch = new btVector3[NUM_UNITSPHERE_POINTS +
   MAX_PREFERRED_PENETRATION_DIRECTIONS * 2];
  int numSampleDirections = NUM_UNITSPHERE_POINTS;
  {
   for (int i = 0; i < numSampleDirections; i++) {
    final btVector3 norm = getPenetrationDirections()[i];
    seperatingAxisInABatch[i] = transA.transposeTransform3x3(new btVector3(norm).negate());
    seperatingAxisInBBatch[i] = transB.transposeTransform3x3(new btVector3(norm));
   }
  }
  {
   int numPDA = convexA.getNumPreferredPenetrationDirections();
   if (numPDA != 0) {
    for (int i = 0; i < numPDA; i++) {
     final btVector3 norm = new btVector3();
     convexA.getPreferredPenetrationDirection(i, norm);
     transA.transform3x3(norm);
     getPenetrationDirections()[numSampleDirections].set(norm);
     seperatingAxisInABatch[numSampleDirections] = transA.transposeTransform3x3(new btVector3(norm)
      .negate());
     seperatingAxisInBBatch[numSampleDirections] = transB.transposeTransform3x3(norm);
     numSampleDirections++;
    }
   }
  }
  {
   int numPDB = convexB.getNumPreferredPenetrationDirections();
   if (numPDB != 0) {
    for (int i = 0; i < numPDB; i++) {
     final btVector3 norm = new btVector3();
     convexB.getPreferredPenetrationDirection(i, norm);
     transB.transform3x3(norm);
     getPenetrationDirections()[numSampleDirections].set(norm);
     seperatingAxisInABatch[numSampleDirections] = transA.transform3x3(new btVector3(norm).negate());
     seperatingAxisInBBatch[numSampleDirections] = transB.transform3x3(new btVector3(norm));
     numSampleDirections++;
    }
   }
  }
  convexA.batchedUnitVectorGetSupportingVertexWithoutMargin(seperatingAxisInABatch,
   supportVerticesABatch, numSampleDirections);
  convexB.batchedUnitVectorGetSupportingVertexWithoutMargin(seperatingAxisInBBatch,
   supportVerticesBBatch, numSampleDirections);
  for (int i = 0; i < numSampleDirections; i++) {
   final btVector3 norm = new btVector3(getPenetrationDirections()[i]);
   if (check2d) {
    norm.z = 0.f;
   }
   if (norm.lengthSquared() > 0.01f) {
//    seperatingAxisInA = seperatingAxisInABatch[i];
//    seperatingAxisInB = seperatingAxisInBBatch[i];
    pInA.set(supportVerticesABatch[i]);
    qInB.set(supportVerticesBBatch[i]);
    transA.transform(pInA, pWorld);
    transB.transform(qInB, qWorld);
    if (check2d) {
     pWorld.z = 0.f;
     qWorld.z = 0.f;
    }
    w.set(qWorld).sub(pWorld);
    float delta = norm.dot(w);
    //find smallest delta
    if (delta < minProj) {
     minProj = delta;
     minNorm.set(norm);
     minA.set(pWorld);
     minB.set(qWorld);
    }
   }
  }
  //add the margins
  minA.add(new btVector3(minNorm).scale(convexA.getMarginNonVirtual()));
  minB.sub(new btVector3(minNorm).scale(convexB.getMarginNonVirtual()));
  //no penetration
  if (minProj < (0.f)) {
   return false;
  }
  float extraSeparation = 0.5f;///scale dependent
  minProj += extraSeparation + (convexA.getMarginNonVirtual() + convexB.getMarginNonVirtual());
  btGjkPairDetector gjkdet = new btGjkPairDetector(convexA, convexB, simplexSolver, null);
  float offsetDist = minProj;
  final btVector3 offset = new btVector3(minNorm).scale(offsetDist);
  btGjkPairDetector.ClosestPointInput input = new btGjkPairDetector.ClosestPointInput();
  final btVector3 newOrg = transA.getOrigin().add(offset);
  final btTransform displacedTrans = new btTransform(transA);
  displacedTrans.setOrigin(newOrg);
  input.m_transformA.set(displacedTrans);
  input.m_transformB.set(transB);
  input.m_maximumDistanceSquared = (BT_LARGE_FLOAT);//minProj;
  btIntermediateResult res = new btIntermediateResult();
  gjkdet.setCachedSeperatingAxis(new btVector3(minNorm).negate());
  gjkdet.getClosestPoints(input, res, debugDraw);
  float correctedMinNorm = minProj - res.m_depth;
  //the penetration depth is over-estimated, relax it
  float penetration_relaxation = (1.f);
  minNorm.scale(penetration_relaxation);
  if (res.m_hasResult) {
   pa.scaleAdd(-correctedMinNorm, minNorm, res.m_pointInWorld);
   pb.set(res.m_pointInWorld);
   v.set(minNorm);
  }
  return res.m_hasResult;
 }
 static btVector3[] sPenetrationDirections = new btVector3[]{
  new btVector3((0.000000f), (-0.000000f), (-1.000000f)),
  new btVector3((0.723608f), (-0.525725f), (-0.447219f)),
  new btVector3((-0.276388f), (-0.850649f), (-0.447219f)),
  new btVector3((-0.894426f), (-0.000000f), (-0.447216f)),
  new btVector3((-0.276388f), (0.850649f), (-0.447220f)),
  new btVector3((0.723608f), (0.525725f), (-0.447219f)),
  new btVector3((0.276388f), (-0.850649f), (0.447220f)),
  new btVector3((-0.723608f), (-0.525725f), (0.447219f)),
  new btVector3((-0.723608f), (0.525725f), (0.447219f)),
  new btVector3((0.276388f), (0.850649f), (0.447219f)),
  new btVector3((0.894426f), (0.000000f), (0.447216f)),
  new btVector3((-0.000000f), (0.000000f), (1.000000f)),
  new btVector3((0.425323f), (-0.309011f), (-0.850654f)),
  new btVector3((-0.162456f), (-0.499995f), (-0.850654f)),
  new btVector3((0.262869f), (-0.809012f), (-0.525738f)),
  new btVector3((0.425323f), (0.309011f), (-0.850654f)),
  new btVector3((0.850648f), (-0.000000f), (-0.525736f)),
  new btVector3((-0.525730f), (-0.000000f), (-0.850652f)),
  new btVector3((-0.688190f), (-0.499997f), (-0.525736f)),
  new btVector3((-0.162456f), (0.499995f), (-0.850654f)),
  new btVector3((-0.688190f), (0.499997f), (-0.525736f)),
  new btVector3((0.262869f), (0.809012f), (-0.525738f)),
  new btVector3((0.951058f), (0.309013f), (0.000000f)),
  new btVector3((0.951058f), (-0.309013f), (0.000000f)),
  new btVector3((0.587786f), (-0.809017f), (0.000000f)),
  new btVector3((0.000000f), (-1.000000f), (0.000000f)),
  new btVector3((-0.587786f), (-0.809017f), (0.000000f)),
  new btVector3((-0.951058f), (-0.309013f), (-0.000000f)),
  new btVector3((-0.951058f), (0.309013f), (-0.000000f)),
  new btVector3((-0.587786f), (0.809017f), (-0.000000f)),
  new btVector3((-0.000000f), (1.000000f), (-0.000000f)),
  new btVector3((0.587786f), (0.809017f), (-0.000000f)),
  new btVector3((0.688190f), (-0.499997f), (0.525736f)),
  new btVector3((-0.262869f), (-0.809012f), (0.525738f)),
  new btVector3((-0.850648f), (0.000000f), (0.525736f)),
  new btVector3((-0.262869f), (0.809012f), (0.525738f)),
  new btVector3((0.688190f), (0.499997f), (0.525736f)),
  new btVector3((0.525730f), (0.000000f), (0.850652f)),
  new btVector3((0.162456f), (-0.499995f), (0.850654f)),
  new btVector3((-0.425323f), (-0.309011f), (0.850654f)),
  new btVector3((-0.425323f), (0.309011f), (0.850654f)),
  new btVector3((0.162456f), (0.499995f), (0.850654f))
 };
 static {
  sPenetrationDirections=Arrays.copyOf(sPenetrationDirections, NUM_UNITSPHERE_POINTS+MAX_PREFERRED_PENETRATION_DIRECTIONS*2);
  init(sPenetrationDirections);
 }

 btVector3[] getPenetrationDirections() {
  return sPenetrationDirections;
 }
}
