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
package Bullet.Collision.Algorithm;

import Bullet.Collision.Algorithm.Detector.btDiscreteCollisionDetectorInterface;
import Bullet.Collision.Algorithm.Detector.btGjkPairDetector;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE;
import Bullet.Collision.Broadphase.btDispatcherInfo;
import Bullet.Collision.Shape.btCapsuleShape;
import Bullet.Collision.Shape.btConvexShape;
import Bullet.Collision.Shape.btPolyhedralConvexShape;
import Bullet.Collision.Shape.btSphereShape;
import Bullet.Collision.Shape.btTriangleShape;
import Bullet.Collision.btCollisionObject;
import Bullet.Collision.btCollisionObjectWrapper;
import Bullet.Collision.btConvexCast;
import Bullet.Collision.btConvexPenetrationDepthSolver;
import Bullet.Collision.btGjkConvexCast;
import Bullet.Collision.btManifoldResult;
import Bullet.Collision.btPersistentManifold;
import static Bullet.Collision.btPersistentManifold.gContactBreakingThreshold;
import Bullet.Collision.btPerturbedContactResult;
import Bullet.Collision.btPolyhedralContactClipping;
import Bullet.Collision.btVoronoiSimplexSolver;
import Bullet.LinearMath.btMatrix3x3;
import Bullet.LinearMath.btQuaternion;
import static Bullet.LinearMath.btScalar.SIMD_2_PI;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.SIMD_PI;
import static Bullet.LinearMath.btScalar.btRecipSqrt;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.btDot;
import static Bullet.LinearMath.btVector3.btPlaneSpace1;
import java.io.Serializable;
import java.util.ArrayList;
import javax.vecmath.AxisAngle4f;

/**
 * Specialized capsule-capsule collision algorithm has been added for Bullet 2.75 release to
 * increase ragdoll performance If you experience problems with capsule-capsule collision, try to
 * define BT_DISABLE_CAPSULE_CAPSULE_COLLIDER and report it in the Bullet forums with reproduction
 * case
 *
 * @author Gregery Barton
 */
public class btConvexConvexAlgorithm extends btActivatingCollisionAlgorithm implements Serializable {

 static float capsuleCapsuleDistance(
  final btVector3 normalOnB, final btVector3 pointOnB,
  float capsuleLengthA,
  float capsuleRadiusA,
  float capsuleLengthB,
  float capsuleRadiusB,
  int capsuleAxisA,
  int capsuleAxisB, final btTransform transformA, final btTransform transformB,
  float distanceThreshold) {
  final btVector3 directionA = transformA.getBasisColumn(capsuleAxisA);
  final btVector3 translationA = transformA.getOrigin();
  final btVector3 directionB = transformB.getBasisColumn(capsuleAxisB);
  final btVector3 translationB = transformB.getOrigin();
  // translation between centers
  final btVector3 translation = new btVector3(translationB).sub(translationA);
  // compute the closest points of the capsule line segments
  final btVector3 ptsVector = new btVector3(); // the vector between the closest points
  final btVector3 offsetA = new btVector3();
  final btVector3 offsetB = new btVector3(); // offsets from segment centers to their closest points
  float[] tA = new float[1];
  float[] tB = new float[1];              // parameters on line segment
  segmentsClosestPoints(ptsVector, offsetA, offsetB, tA, tB, translation,
   directionA, capsuleLengthA, directionB, capsuleLengthB);
  float distance = ptsVector.length() - capsuleRadiusA - capsuleRadiusB;
  if (distance > distanceThreshold) {
   return distance;
  }
  float lenSqr = ptsVector.lengthSquared();
  if (lenSqr <= (SIMD_EPSILON * SIMD_EPSILON)) {
   //degenerate case where 2 capsules are likely at the same location: take a vector tangential to 'directionA'
   final btVector3 q = new btVector3();
   btPlaneSpace1(directionA, normalOnB, q);
  } else {
   // compute the contact normal
   normalOnB.set(ptsVector).scale(-btRecipSqrt(lenSqr));
  }
  pointOnB.set(transformB.getOrigin().add(new btVector3().scaleAdd(capsuleRadiusB, normalOnB,
   offsetB)));
  return distance;
 }

 static void segmentsClosestPoints(
  final btVector3 ptsVector, final btVector3 offsetA, final btVector3 offsetB,
  float[] tA, float[] tB, final btVector3 translation, final btVector3 dirA, float hlenA,
  final btVector3 dirB, float hlenB) {
  // compute the parameters of the closest points on each line segment
  float dirA_dot_dirB = btDot(dirA, dirB);
  float dirA_dot_trans = btDot(dirA, translation);
  float dirB_dot_trans = btDot(dirB, translation);
  float denom = 1.0f - dirA_dot_dirB * dirA_dot_dirB;
  if (denom == 0.0f) {
   tA[0] = 0.0f;
  } else {
   tA[0] = (dirA_dot_trans - dirB_dot_trans * dirA_dot_dirB) / denom;
   if (tA[0] < -hlenA) {
    tA[0] = -hlenA;
   } else if (tA[0] > hlenA) {
    tA[0] = hlenA;
   }
  }
  tB[0] = tA[0] * dirA_dot_dirB - dirB_dot_trans;
  if (tB[0] < -hlenB) {
   tB[0] = -hlenB;
   tA[0] = tB[0] * dirA_dot_dirB + dirA_dot_trans;
   if (tA[0] < -hlenA) {
    tA[0] = -hlenA;
   } else if (tA[0] > hlenA) {
    tA[0] = hlenA;
   }
  } else if (tB[0] > hlenB) {
   tB[0] = hlenB;
   tA[0] = tB[0] * dirA_dot_dirB + dirA_dot_trans;
   if (tA[0] < -hlenA) {
    tA[0] = -hlenA;
   } else if (tA[0] > hlenA) {
    tA[0] = hlenA;
   }
  }
  // compute the closest points relative to segment centers.
  offsetA.set(dirA).scale(tA[0]);
  offsetB.set(dirB).scale(tB[0]);
  ptsVector.set(translation).sub(offsetA).add(offsetB);
 }
 final btConvexPenetrationDepthSolver m_pdSolver;
 final ArrayList<btVector3> worldVertsB1 = new ArrayList<>(0);
 final ArrayList<btVector3> worldVertsB2 = new ArrayList<>(0);
 boolean m_ownManifold;
 btPersistentManifold m_manifoldPtr;
 boolean m_lowLevelOfDetail;
 int m_numPerturbationIterations;
 int m_minimumPointsPerturbationThreshold;
 boolean disableCcd = false;

 ///cache separating vector to speedup collision detection
 btConvexConvexAlgorithm(btPersistentManifold mf, btCollisionAlgorithmConstructionInfo ci,
  btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap,
  btConvexPenetrationDepthSolver pdSolver, int numPerturbationIterations,
  int minimumPointsPerturbationThreshold) {
  super(ci, body0Wrap, body1Wrap);
  m_pdSolver = pdSolver;
  m_ownManifold = false;
  m_manifoldPtr = mf;
  m_lowLevelOfDetail = false;
  m_numPerturbationIterations = numPerturbationIterations;
  m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;
 }

 @Override
 public void destroy() {
  if (m_ownManifold) {
   if (m_manifoldPtr != null) {
    m_dispatcher.releaseManifold(m_manifoldPtr);
   }
  }
 }

//
// Convex-Convex collision algorithm
//
 @Override
 public void processCollision(btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap,
  btDispatcherInfo dispatchInfo, btManifoldResult resultOut) {
  if (m_manifoldPtr == null) {
   //swapped?
   m_manifoldPtr = m_dispatcher.getNewManifold(body0Wrap.getCollisionObject(), body1Wrap
    .getCollisionObject());
   m_ownManifold = true;
  }
  resultOut.setPersistentManifold(m_manifoldPtr);
  //comment-out next line to test multi-contact generation
  //resultOut.getPersistentManifold().clearManifold();
  btConvexShape min0 = (btConvexShape) (body0Wrap.getCollisionShape());
  btConvexShape min1 = (btConvexShape) (body1Wrap.getCollisionShape());
  final btVector3 normalOnB = new btVector3();
  final btVector3 pointOnBWorld = new btVector3();
  if ((min0.getShapeType() == CAPSULE_SHAPE_PROXYTYPE) && (min1.getShapeType() ==
   CAPSULE_SHAPE_PROXYTYPE)) {
   //m_manifoldPtr.clearManifold();
   btCapsuleShape capsuleA = (btCapsuleShape) min0;
   btCapsuleShape capsuleB = (btCapsuleShape) min1;
   float threshold = m_manifoldPtr.getContactBreakingThreshold();
   float dist = capsuleCapsuleDistance(normalOnB, pointOnBWorld, capsuleA.getHalfHeight(), capsuleA
    .getRadius(),
    capsuleB.getHalfHeight(), capsuleB.getRadius(), capsuleA.getUpAxis(), capsuleB.getUpAxis(),
    body0Wrap.getWorldTransform(), body1Wrap.getWorldTransform(), threshold);
   if (dist < threshold) {
    assert (normalOnB.lengthSquared() >= (SIMD_EPSILON * SIMD_EPSILON));
    resultOut.addContactPoint(normalOnB, pointOnBWorld, dist);
   }
   resultOut.refreshContactPoints();
   return;
  }
  if ((min0.getShapeType() == CAPSULE_SHAPE_PROXYTYPE) && (min1.getShapeType() ==
   SPHERE_SHAPE_PROXYTYPE)) {
   //m_manifoldPtr.clearManifold();
   btCapsuleShape capsuleA = (btCapsuleShape) min0;
   btSphereShape capsuleB = (btSphereShape) min1;
   float threshold = m_manifoldPtr.getContactBreakingThreshold();
   float dist = capsuleCapsuleDistance(normalOnB, pointOnBWorld, capsuleA.getHalfHeight(), capsuleA
    .getRadius(),
    0.f, capsuleB.getRadius(), capsuleA.getUpAxis(), 1,
    body0Wrap.getWorldTransform(), body1Wrap.getWorldTransform(), threshold);
   if (dist < threshold) {
    assert (normalOnB.lengthSquared() >= (SIMD_EPSILON * SIMD_EPSILON));
    resultOut.addContactPoint(normalOnB, pointOnBWorld, dist);
   }
   resultOut.refreshContactPoints();
   return;
  }
  if ((min0.getShapeType() == SPHERE_SHAPE_PROXYTYPE) && (min1.getShapeType() ==
   CAPSULE_SHAPE_PROXYTYPE)) {
   //m_manifoldPtr.clearManifold();
   btSphereShape capsuleA = (btSphereShape) min0;
   btCapsuleShape capsuleB = (btCapsuleShape) min1;
   float threshold = m_manifoldPtr.getContactBreakingThreshold();
   float dist = capsuleCapsuleDistance(normalOnB, pointOnBWorld, 0.f, capsuleA.getRadius(),
    capsuleB.getHalfHeight(), capsuleB.getRadius(), 1, capsuleB.getUpAxis(),
    body0Wrap.getWorldTransform(), body1Wrap.getWorldTransform(), threshold);
   if (dist < threshold) {
    assert (normalOnB.lengthSquared() >= (SIMD_EPSILON * SIMD_EPSILON));
    resultOut.addContactPoint(normalOnB, pointOnBWorld, dist);
   }
   resultOut.refreshContactPoints();
   return;
  }
  {
   btGjkPairDetector.ClosestPointInput input = new btGjkPairDetector.ClosestPointInput();
   btVoronoiSimplexSolver simplexSolver = new btVoronoiSimplexSolver();
   btGjkPairDetector gjkPairDetector = new btGjkPairDetector(min0, min1, simplexSolver, m_pdSolver);
   //TODO: if (dispatchInfo.m_useContinuous)
   gjkPairDetector.setMinkowskiA(min0);
   gjkPairDetector.setMinkowskiB(min1);
   {
    //if (dispatchInfo.m_convexMaxDistanceUseCPT)
    //{
    //	input.m_maximumDistanceSquared = min0.getMargin() + min1.getMargin() + m_manifoldPtr.getContactProcessingThreshold();
    //} else
    //{
    input.m_maximumDistanceSquared = min0.getMargin() + min1.getMargin() + m_manifoldPtr
     .getContactBreakingThreshold() + resultOut.m_closestPointDistanceThreshold;
//		}
    input.m_maximumDistanceSquared *= input.m_maximumDistanceSquared;
   }
   input.m_transformA.set(body0Wrap.getWorldTransform());
   input.m_transformB.set(body1Wrap.getWorldTransform());
   if (min0.isPolyhedral() && min1.isPolyhedral()) {
    btDummyResult dummy = new btDummyResult();
///btBoxShape is an exception: its vertices are created WITH margin so don't subtract it
    float min0Margin = min0.getShapeType() == BOX_SHAPE_PROXYTYPE ? 0.f : min0.getMargin();
    float min1Margin = min1.getShapeType() == BOX_SHAPE_PROXYTYPE ? 0.f : min1.getMargin();
    btWithoutMarginResult withoutMargin = new btWithoutMarginResult(resultOut, min0Margin,
     min1Margin);
    btPolyhedralConvexShape polyhedronA = (btPolyhedralConvexShape) min0;
    btPolyhedralConvexShape polyhedronB = (btPolyhedralConvexShape) min1;
    if (polyhedronA.getConvexPolyhedron() != null && polyhedronB.getConvexPolyhedron() != null) {
     float threshold = m_manifoldPtr.getContactBreakingThreshold();
     float minDist = -1e30f;
     final btVector3 sepNormalWorldSpace = new btVector3();
     boolean foundSepAxis = true;
     if (dispatchInfo.m_enableSatConvex) {
      foundSepAxis = btPolyhedralContactClipping.findSeparatingAxis(
       polyhedronA.getConvexPolyhedron(), polyhedronB.getConvexPolyhedron(),
       body0Wrap.getWorldTransform(),
       body1Wrap.getWorldTransform(),
       sepNormalWorldSpace, resultOut);
     } else {
      gjkPairDetector.getClosestPoints(input, withoutMargin, dispatchInfo.m_debugDraw);
      //gjkPairDetector.getClosestPoints(input,dummy,dispatchInfo.m_debugDraw);
      //float l2 = gjkPairDetector.getCachedSeparatingAxis().lengthSquared();
      //if (l2>SIMD_EPSILON)
      {
       sepNormalWorldSpace.set(withoutMargin.m_reportedNormalOnWorld);//gjkPairDetector.getCachedSeparatingAxis()*(1.f/l2);
       //minDist = -1e30f;//gjkPairDetector.getCachedSeparatingDistance();
       minDist = withoutMargin.m_reportedDistance;//gjkPairDetector.getCachedSeparatingDistance()+min0.getMargin()+min1.getMargin();
       foundSepAxis = withoutMargin.m_foundResult && minDist < 0;//-(min0.getMargin()+min1.getMargin());
      }
     }
     if (foundSepAxis) {
//				printf("sepNormalWorldSpace=%f,%f,%f\n",sepNormalWorldSpace.getX(),sepNormalWorldSpace.getY(),sepNormalWorldSpace.getZ());
      worldVertsB1.clear();
      btPolyhedralContactClipping.clipHullAgainstHull(sepNormalWorldSpace, polyhedronA
       .getConvexPolyhedron(), polyhedronB.getConvexPolyhedron(),
       body0Wrap.getWorldTransform(),
       body1Wrap.getWorldTransform(), minDist - threshold, threshold, worldVertsB1, worldVertsB2,
       resultOut);
     }
     if (m_ownManifold) {
      resultOut.refreshContactPoints();
     }
     return;
    } else //we can also deal with convex versus triangle (without connectivity data)
     if (polyhedronA.getConvexPolyhedron() != null && polyhedronB.getShapeType() ==
      TRIANGLE_SHAPE_PROXYTYPE) {
      ArrayList<btVector3> vertices = new ArrayList<>(0);
      btTriangleShape tri = (btTriangleShape) polyhedronB;
      vertices.add(body1Wrap.getWorldTransform().transform(new btVector3(tri.m_vertices1[0])));
      vertices.add(body1Wrap.getWorldTransform().transform(new btVector3(tri.m_vertices1[1])));
      vertices.add(body1Wrap.getWorldTransform().transform(new btVector3(tri.m_vertices1[2])));
      //tri.initializePolyhedralFeatures();
      float threshold = m_manifoldPtr.getContactBreakingThreshold();
      final btVector3 sepNormalWorldSpace = new btVector3();
      float minDist = -1e30f;
      float maxDist = threshold;
      boolean foundSepAxis = false;
      {
       gjkPairDetector.getClosestPoints(input, dummy, dispatchInfo.m_debugDraw);
       float l2 = gjkPairDetector.getCachedSeparatingAxis().lengthSquared();
       if (l2 > SIMD_EPSILON) {
        sepNormalWorldSpace.set(gjkPairDetector.getCachedSeparatingAxis().scale(1.f / l2));
        //minDist = gjkPairDetector.getCachedSeparatingDistance();
        //maxDist = threshold;
        minDist = gjkPairDetector.getCachedSeparatingDistance() - min0.getMargin() - min1
         .getMargin();
        foundSepAxis = true;
       }
      }
      if (foundSepAxis) {
       worldVertsB2.clear();
       btPolyhedralContactClipping.clipFaceAgainstHull(sepNormalWorldSpace, polyhedronA
        .getConvexPolyhedron(),
        body0Wrap.getWorldTransform(), vertices, worldVertsB2, minDist - threshold, maxDist,
        resultOut);
      }
      if (m_ownManifold) {
       resultOut.refreshContactPoints();
      }
      return;
     }
   }
   gjkPairDetector.getClosestPoints(input, resultOut, dispatchInfo.m_debugDraw);
   //now perform 'm_numPerturbationIterations' collision queries with the perturbated collision objects
   //perform perturbation when more then 'm_minimumPointsPerturbationThreshold' points
   if (m_numPerturbationIterations != 0 && resultOut.getPersistentManifold().getNumContacts() <
    m_minimumPointsPerturbationThreshold) {
    int i;
    final btVector3 v0 = new btVector3();
    final btVector3 v1 = new btVector3();
    final btVector3 sepNormalWorldSpace = new btVector3();
    float l2 = gjkPairDetector.getCachedSeparatingAxis().lengthSquared();
    if (l2 > SIMD_EPSILON) {
     sepNormalWorldSpace.set(gjkPairDetector.getCachedSeparatingAxis().scale(1.f / l2));
     btPlaneSpace1(sepNormalWorldSpace, v0, v1);
     boolean perturbeA = true;
     float angleLimit = 0.125f * SIMD_PI;
     float perturbeAngle;
     float radiusA = min0.getAngularMotionDisc();
     float radiusB = min1.getAngularMotionDisc();
     if (radiusA < radiusB) {
      perturbeAngle = gContactBreakingThreshold / radiusA;
      perturbeA = true;
     } else {
      perturbeAngle = gContactBreakingThreshold / radiusB;
      perturbeA = false;
     }
     if (perturbeAngle > angleLimit) {
      perturbeAngle = angleLimit;
     }
     final btTransform unPerturbedTransform = new btTransform();
     if (perturbeA) {
      unPerturbedTransform.set(input.m_transformA);
     } else {
      unPerturbedTransform.set(input.m_transformB);
     }
     for (i = 0; i < m_numPerturbationIterations; i++) {
      if (v0.lengthSquared() > SIMD_EPSILON) {
       final btQuaternion perturbeRot = new btQuaternion().set(new AxisAngle4f(v0, perturbeAngle));
       float iterationAngle = i * (SIMD_2_PI / (m_numPerturbationIterations));
       final btQuaternion rotq = new btQuaternion().set(new AxisAngle4f(sepNormalWorldSpace,
        iterationAngle));
       if (perturbeA) {
        input.m_transformA.setBasis(new btMatrix3x3().set(new btQuaternion(rotq).conjugate().mul(
         perturbeRot).mul(rotq)).mul(body0Wrap.getWorldTransform().getBasis()));
        input.m_transformB.set(body1Wrap.getWorldTransform());
       } else {
        input.m_transformA.set(body0Wrap.getWorldTransform());
        input.m_transformB.setBasis(new btMatrix3x3().set(new btQuaternion(rotq).conjugate().mul(
         perturbeRot).mul(rotq)).mul(body1Wrap.getWorldTransform().getBasis()));
       }
       btPerturbedContactResult perturbedResultOut = new btPerturbedContactResult(resultOut,
        input.m_transformA, input.m_transformB, unPerturbedTransform, perturbeA,
        dispatchInfo.m_debugDraw);
       gjkPairDetector.getClosestPoints(input, perturbedResultOut, dispatchInfo.m_debugDraw);
      }
     }
    }
   }
  }
  if (m_ownManifold) {
   resultOut.refreshContactPoints();
  }
 }

 @Override
 public float calculateTimeOfImpact(btCollisionObject col0, btCollisionObject col1,
  btDispatcherInfo dispatchInfo, btManifoldResult resultOut) {
  ///Rather then checking ALL pairs, only calculate TOI when motion exceeds threshold
  ///Linear motion for one of objects needs to exceed m_ccdSquareMotionThreshold
  ///col0.m_worldTransform,
  float resultFraction = (1.f);
  float squareMot0 = (col0.getInterpolationWorldTransform().getOrigin().sub(col0
   .getWorldTransformPtr()
   .getOrigin())).lengthSquared();
  float squareMot1 = (col1.getInterpolationWorldTransform().getOrigin().sub(col1
   .getWorldTransformPtr()
   .getOrigin())).lengthSquared();
  if (squareMot0 < col0.getCcdSquareMotionThreshold() &&
   squareMot1 < col1.getCcdSquareMotionThreshold()) {
   return resultFraction;
  }
  if (disableCcd) {
   return (1.f);
  }
  //An adhoc way of testing the Continuous Collision Detection algorithms
  //One object is approximated as a sphere, to simplify things
  //Starting in penetration should report no time of impact
  //For proper CCD, better accuracy and handling of 'allowed' penetration should be added
  //also the mainloop of the physics should have a kind of toi queue (something like Brian Mirtich's application of Timewarp for Rigidbodies)
  /// Convex0 against sphere for Convex1
  {
   btConvexShape convex0 = (btConvexShape) (col0.getCollisionShape());
   btSphereShape sphere1 = new btSphereShape(col1.getCcdSweptSphereRadius()); //todo: allow non-zero sphere sizes, for better approximation
   btConvexCast.CastResult result = new btConvexCast.CastResult();
   btVoronoiSimplexSolver voronoiSimplex = new btVoronoiSimplexSolver();
   //SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
   ///Simplification, one object is simplified as a sphere
   btGjkConvexCast ccd1 = new btGjkConvexCast(convex0, sphere1, voronoiSimplex);
   //ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
   if (ccd1.calcTimeOfImpact(col0.getWorldTransformPtr(), col0.getInterpolationWorldTransform(),
    col1.getWorldTransformPtr(), col1.getInterpolationWorldTransform(), result)) {
    //store result.m_fraction in both bodies
    if (col0.getHitFraction() > result.m_fraction) {
     col0.setHitFraction(result.m_fraction);
    }
    if (col1.getHitFraction() > result.m_fraction) {
     col1.setHitFraction(result.m_fraction);
    }
    if (resultFraction > result.m_fraction) {
     resultFraction = result.m_fraction;
    }
   }
  }
  /// Sphere (for convex0) against Convex1
  {
   btConvexShape convex1 = (btConvexShape) (col1.getCollisionShape());
   btSphereShape sphere0 = new btSphereShape(col0.getCcdSweptSphereRadius()); //todo: allow non-zero sphere sizes, for better approximation
   btConvexCast.CastResult result = new btConvexCast.CastResult();
   btVoronoiSimplexSolver voronoiSimplex = new btVoronoiSimplexSolver();
   //SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
   ///Simplification, one object is simplified as a sphere
   btGjkConvexCast ccd1 = new btGjkConvexCast(sphere0, convex1, voronoiSimplex);
   //ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
   if (ccd1.calcTimeOfImpact(col0.getWorldTransformPtr(), col0.getInterpolationWorldTransform(),
    col1.getWorldTransformPtr(), col1.getInterpolationWorldTransform(), result)) {
    //store result.m_fraction in both bodies
    if (col0.getHitFraction() > result.m_fraction) {
     col0.setHitFraction(result.m_fraction);
    }
    if (col1.getHitFraction() > result.m_fraction) {
     col1.setHitFraction(result.m_fraction);
    }
    if (resultFraction > result.m_fraction) {
     resultFraction = result.m_fraction;
    }
   }
  }
  return resultFraction;
 }

 @Override
 public void getAllContactManifolds(ArrayList<btPersistentManifold> manifoldArray) {
  ///should we use m_ownManifold to avoid adding duplicates?
  if (m_manifoldPtr != null && m_ownManifold) {
   manifoldArray.add(m_manifoldPtr);
  }
 }

 void setLowLevelOfDetail(boolean useLowLevel) {
  m_lowLevelOfDetail = useLowLevel;
 }

 btPersistentManifold getManifold() {
  return m_manifoldPtr;
 }

 static class btDummyResult extends btDiscreteCollisionDetectorInterface.Result {

  @Override
  public void setShapeIdentifiersA(int partId0, int index0) {
  }

  @Override
  public void setShapeIdentifiersB(int partId1, int index1) {
  }

  @Override
  public void addContactPoint(final btVector3 normalOnBInWorld, final btVector3 pointInWorld,
   float depth) {
  }
 }

 static class btWithoutMarginResult extends btDiscreteCollisionDetectorInterface.Result {

  btDiscreteCollisionDetectorInterface.Result m_originalResult;
  final btVector3 m_reportedNormalOnWorld = new btVector3();
  float m_marginOnA;
  float m_marginOnB;
  float m_reportedDistance;
  boolean m_foundResult;

  btWithoutMarginResult(btDiscreteCollisionDetectorInterface.Result result, float marginOnA,
   float marginOnB) {
   m_originalResult = result;
   m_marginOnA = marginOnA;
   m_marginOnB = marginOnB;
   m_foundResult = false;
  }

  @Override
  public void setShapeIdentifiersA(int partId0, int index0) {
  }

  @Override
  public void setShapeIdentifiersB(int partId1, int index1) {
  }

  @Override
  public void addContactPoint(final btVector3 normalOnBInWorld, final btVector3 pointInWorldOrg,
   float depthOrg) {
   m_reportedDistance = depthOrg;
   m_reportedNormalOnWorld.set(normalOnBInWorld);
   final btVector3 adjustedPointB = new btVector3().scaleAdd(-m_marginOnB, normalOnBInWorld,
    pointInWorldOrg);
   m_reportedDistance = depthOrg + (m_marginOnA + m_marginOnB);
   if (m_reportedDistance < 0.f) {
    m_foundResult = true;
   }
   m_originalResult.addContactPoint(normalOnBInWorld, adjustedPointB, m_reportedDistance);
  }
 }

 public static class CreateFunc extends btCollisionAlgorithmCreateFunc {

  final btConvexPenetrationDepthSolver m_pdSolver;
  int m_numPerturbationIterations;
  int m_minimumPointsPerturbationThreshold;

  public CreateFunc(btConvexPenetrationDepthSolver pdSolver) {
   m_numPerturbationIterations = 0;
   m_minimumPointsPerturbationThreshold = 3;
   m_pdSolver = pdSolver;
  }

  @Override
  public btCollisionAlgorithm CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo ci,
   btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap) {
   return new btConvexConvexAlgorithm(ci.m_manifold, ci, body0Wrap, body1Wrap, m_pdSolver,
    m_numPerturbationIterations, m_minimumPointsPerturbationThreshold);
  }
 }
};
