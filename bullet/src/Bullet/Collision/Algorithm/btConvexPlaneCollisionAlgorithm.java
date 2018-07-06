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
package Bullet.Collision.Algorithm;

import Bullet.Collision.Broadphase.btDispatcherInfo;
import Bullet.Collision.Shape.btConvexShape;
import Bullet.Collision.Shape.btStaticPlaneShape;
import Bullet.Collision.btCollisionObject;
import Bullet.Collision.btCollisionObjectWrapper;
import Bullet.Collision.btManifoldResult;
import Bullet.Collision.btPersistentManifold;
import static Bullet.Collision.btPersistentManifold.gContactBreakingThreshold;
import Bullet.LinearMath.btMatrix3x3;
import Bullet.LinearMath.btQuaternion;
import static Bullet.LinearMath.btScalar.SIMD_2_PI;
import static Bullet.LinearMath.btScalar.SIMD_PI;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.btPlaneSpace1;
import java.io.Serializable;
import java.util.ArrayList;
import javax.vecmath.AxisAngle4f;

/**
 * btSphereBoxCollisionAlgorithm provides sphere-box collision detection. Other
 * features are frame-coherency (persistent data) and collision response.
 *
 * @author Gregery Barton
 */
public class btConvexPlaneCollisionAlgorithm extends btCollisionAlgorithm
 implements Serializable {

 boolean m_ownManifold;
 btPersistentManifold m_manifoldPtr;
 boolean m_isSwapped;
 int m_numPerturbationIterations;
 int m_minimumPointsPerturbationThreshold;

 btConvexPlaneCollisionAlgorithm(btPersistentManifold mf,
  btCollisionAlgorithmConstructionInfo ci,
  btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap,
  boolean isSwapped,
  int numPerturbationIterations, int minimumPointsPerturbationThreshold) {
  super(ci);
  m_ownManifold = false;
  m_manifoldPtr = mf;
  m_isSwapped = isSwapped;
  m_numPerturbationIterations = numPerturbationIterations;
  m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;
  btCollisionObjectWrapper convexObjWrap = m_isSwapped ? body1Wrap : body0Wrap;
  btCollisionObjectWrapper planeObjWrap = m_isSwapped ? body0Wrap : body1Wrap;
  if (m_manifoldPtr == null && m_dispatcher.needsCollision(convexObjWrap
   .getCollisionObject(),
   planeObjWrap.getCollisionObject())) {
   m_manifoldPtr = m_dispatcher.getNewManifold(convexObjWrap
    .getCollisionObject(), planeObjWrap
     .getCollisionObject());
   m_ownManifold = true;
  }
 }

 @Override
 public void destroy() {
  if (m_ownManifold) {
   if (m_manifoldPtr != null) {
    m_dispatcher.releaseManifold(m_manifoldPtr);
   }
  }
 }

 @Override
 public void processCollision(btCollisionObjectWrapper body0Wrap,
  btCollisionObjectWrapper body1Wrap,
  btDispatcherInfo dispatchInfo, btManifoldResult resultOut) {
  if (m_manifoldPtr == null) {
   return;
  }
  btCollisionObjectWrapper convexObjWrap = m_isSwapped ? body1Wrap : body0Wrap;
  btCollisionObjectWrapper planeObjWrap = m_isSwapped ? body0Wrap : body1Wrap;
  btConvexShape convexShape = (btConvexShape) convexObjWrap.getCollisionShape();
  btStaticPlaneShape planeShape = (btStaticPlaneShape) planeObjWrap
   .getCollisionShape();
  boolean hasCollision = false;
  final btVector3 planeNormal = planeShape.getPlaneNormal();
  float planeConstant = planeShape.getPlaneConstant();
  final btTransform planeInConvex = convexObjWrap.getWorldTransform().invert()
   .mul(planeObjWrap
    .getWorldTransform());
  final btTransform convexInPlaneTrans = planeObjWrap.getWorldTransform()
   .invert().mul(convexObjWrap
    .getWorldTransform());
  final btVector3 vtx = convexShape.localGetSupportingVertex(planeInConvex
   .transform3x3(
    new btVector3(
     planeNormal).negate()));
  final btVector3 vtxInPlane = convexInPlaneTrans.transform(new btVector3(vtx));
  float distance = (planeNormal.dot(vtxInPlane) - planeConstant);
  final btVector3 vtxInPlaneProjected = new btVector3().scaleAdd(-distance,
   planeNormal, vtxInPlane);
  final btVector3 vtxInPlaneWorld = planeObjWrap.getWorldTransform().transform(
   new btVector3(
    vtxInPlaneProjected));
  hasCollision = distance < m_manifoldPtr.getContactBreakingThreshold();
  resultOut.setPersistentManifold(m_manifoldPtr);
  if (hasCollision) {
   /// report a contact. internally this will be kept persistent, and contact reduction is done
   final btVector3 normalOnSurfaceB = planeObjWrap.getWorldTransform()
    .transform3x3(new btVector3(
     planeNormal));
   final btVector3 pOnB = new btVector3(vtxInPlaneWorld);
   resultOut.addContactPoint(normalOnSurfaceB, pOnB, distance);
  }
  //the perturbation algorithm doesn't work well with implicit surfaces such as spheres, cylinder and cones:
  //they keep on rolling forever because of the additional off-center contact points
  //so only enable the feature for polyhedral shapes (btBoxShape, btConvexHullShape etc)
  if (convexShape.isPolyhedral() && resultOut.getPersistentManifold()
   .getNumContacts() < m_minimumPointsPerturbationThreshold) {
   final btVector3 v0 = new btVector3();
   final btVector3 v1 = new btVector3();
   btPlaneSpace1(planeNormal, v0, v1);
   //now perform 'm_numPerturbationIterations' collision queries with the perturbated collision objects
   float angleLimit = 0.125f * SIMD_PI;
   float perturbeAngle;
   float radius = convexShape.getAngularMotionDisc();
   perturbeAngle = gContactBreakingThreshold / radius;
   if (perturbeAngle > angleLimit) {
    perturbeAngle = angleLimit;
   }
   final btQuaternion perturbeRot = new btQuaternion().set(new AxisAngle4f(v0,
    perturbeAngle));
   for (int i = 0; i < m_numPerturbationIterations; i++) {
    float iterationAngle = i * (SIMD_2_PI / m_numPerturbationIterations);
    final btQuaternion rotq = new btQuaternion()
     .set(planeNormal, iterationAngle);
    collideSingleContact(new btQuaternion(rotq).conjugate().mul(perturbeRot)
     .mul(rotq), body0Wrap,
     body1Wrap, dispatchInfo, resultOut);
   }
  }
  if (m_ownManifold) {
   if (m_manifoldPtr.getNumContacts() > 0) {
    resultOut.refreshContactPoints();
   }
  }
 }

 void collideSingleContact(final btQuaternion perturbeRot,
  btCollisionObjectWrapper body0Wrap,
  btCollisionObjectWrapper body1Wrap, btDispatcherInfo dispatchInfo,
  btManifoldResult resultOut) {
  btCollisionObjectWrapper convexObjWrap = m_isSwapped ? body1Wrap : body0Wrap;
  btCollisionObjectWrapper planeObjWrap = m_isSwapped ? body0Wrap : body1Wrap;
  btConvexShape convexShape = (btConvexShape) convexObjWrap.getCollisionShape();
  btStaticPlaneShape planeShape = (btStaticPlaneShape) planeObjWrap
   .getCollisionShape();
  boolean hasCollision = false;
  final btVector3 planeNormal = planeShape.getPlaneNormal();
  float planeConstant = planeShape.getPlaneConstant();
  final btTransform convexWorldTransform = convexObjWrap.getWorldTransform();
  final btTransform convexInPlaneTrans;
  convexInPlaneTrans = planeObjWrap.getWorldTransform().invert().mul(
   convexWorldTransform);
  //now perturbe the convex-world transform
  convexWorldTransform.setBasis(convexWorldTransform.getBasis().mul(
   new btMatrix3x3().set(
    perturbeRot)));
  final btTransform planeInConvex;
  planeInConvex = new btTransform(convexWorldTransform).invert().mul(
   planeObjWrap
    .getWorldTransform());
  final btVector3 vtx = convexShape.localGetSupportingVertex(planeInConvex
   .transform3x3(
    new btVector3(
     planeNormal).negate()));
  final btVector3 vtxInPlane = convexInPlaneTrans.transform(new btVector3(vtx));
  float distance = (planeNormal.dot(vtxInPlane) - planeConstant);
  final btVector3 vtxInPlaneProjected = new btVector3().scaleAdd(-distance,
   planeNormal, vtxInPlane);
  final btVector3 vtxInPlaneWorld = planeObjWrap.getWorldTransform().transform(
   new btVector3(
    vtxInPlaneProjected));
  hasCollision = distance < m_manifoldPtr.getContactBreakingThreshold();
  resultOut.setPersistentManifold(m_manifoldPtr);
  if (hasCollision) {
   /// report a contact. internally this will be kept persistent, and contact reduction is done
   final btVector3 normalOnSurfaceB = planeObjWrap.getWorldTransform()
    .transform3x3(new btVector3(
     planeNormal));
   final btVector3 pOnB = vtxInPlaneWorld;
   resultOut.addContactPoint(normalOnSurfaceB, pOnB, distance);
  }
 }

 @Override
 public float calculateTimeOfImpact(btCollisionObject body0,
  btCollisionObject body1,
  btDispatcherInfo dispatchInfo, btManifoldResult resultOut) {
  //not yet
  return (1.f);
 }

 @Override
 public void getAllContactManifolds(
  ArrayList<btPersistentManifold> manifoldArray) {
  if (m_manifoldPtr != null && m_ownManifold) {
   manifoldArray.add(m_manifoldPtr);
  }
 }

 public static class CreateFunc extends btCollisionAlgorithmCreateFunc {

  int m_numPerturbationIterations;
  int m_minimumPointsPerturbationThreshold;

  public CreateFunc() {
   m_numPerturbationIterations = 1;
   m_minimumPointsPerturbationThreshold = 0;
  }

  @Override
  public btCollisionAlgorithm CreateCollisionAlgorithm(
   btCollisionAlgorithmConstructionInfo ci,
   btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap) {
   if (!m_swapped) {
    return new btConvexPlaneCollisionAlgorithm(null, ci, body0Wrap, body1Wrap,
     false,
     m_numPerturbationIterations, m_minimumPointsPerturbationThreshold);
   } else {
    return new btConvexPlaneCollisionAlgorithm(null, ci, body0Wrap, body1Wrap,
     true,
     m_numPerturbationIterations, m_minimumPointsPerturbationThreshold);
   }
  }

 };
};
