/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

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

import Bullet.Collision.Algorithm.btCollisionAlgorithm;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.COMPOUND_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.CONE_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.MULTI_SPHERE_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.STATIC_PLANE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.TRIANGLE_MESH_SHAPE_PROXYTYPE;
import Bullet.Collision.Broadphase.btBroadphaseInterface;
import Bullet.Collision.Broadphase.btBroadphaseProxy;
import Bullet.Collision.Broadphase.btDbvt;
import Bullet.Collision.Broadphase.btDbvtAabbMm;
import Bullet.Collision.Broadphase.btDispatcher;
import Bullet.Collision.Broadphase.btDispatcherInfo;
import Bullet.Collision.Broadphase.btOverlappingPairCache;
import Bullet.Collision.Broadphase.btSingleContactCallback;
import Bullet.Collision.Broadphase.btSingleRayCallback;
import Bullet.Collision.Broadphase.btSingleSweepCallback;
import static Bullet.Collision.CollisionFlags.CF_DISABLE_VISUALIZE_OBJECT;
import static Bullet.Collision.CollisionObjectTypes.CO_RIGID_BODY;
import Bullet.Collision.Shape.btBoxShape;
import Bullet.Collision.Shape.btBvhTriangleMeshShape;
import Bullet.Collision.Shape.btCapsuleShape;
import Bullet.Collision.Shape.btCollisionShape;
import Bullet.Collision.Shape.btCompoundShape;
import Bullet.Collision.Shape.btConcaveShape;
import Bullet.Collision.Shape.btConeShape;
import Bullet.Collision.Shape.btConvexShape;
import Bullet.Collision.Shape.btConvexTriangleMeshShape;
import Bullet.Collision.Shape.btCylinderShape;
import Bullet.Collision.Shape.btMultiSphereShape;
import Bullet.Collision.Shape.btPolyhedralConvexShape;
import Bullet.Collision.Shape.btSphereShape;
import Bullet.Collision.Shape.btStaticPlaneShape;
import static Bullet.Collision.btCollisionObject.DISABLE_SIMULATION;
import static Bullet.Collision.btIDebugDraw.DBG_DrawContactPoints;
import static Bullet.Collision.btPersistentManifold.gContactBreakingThreshold;
import static Bullet.Collision.ebtDispatcherQueryType.BT_CLOSEST_POINT_ALGORITHMS;
import static Bullet.LinearMath.btQuickprof.BT_PROFILE;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btTransformUtil;
import Bullet.LinearMath.btVector3;
import static Bullet.common.btAlignedObjectArray.findLinearSearch;
import java.io.Serializable;
import java.util.ArrayList;
import static java.util.Collections.swap;

/**
 * CollisionWorld is interface and container for the collision detection
 *
 * @author Gregery Barton
 */
public class btCollisionWorld implements Serializable {

 static boolean reportMe = true;
 private static final long serialVersionUID = 1L;

 /// rayTestSingle performs a raycast call and calls the resultCallback. It is used internally by rayTest.
 /// In a future implementation, we consider moving the ray test as a   method in btCollisionShape.
 /// This allows more customization.
 public static void rayTestSingle(final btTransform rayFromTrans, final btTransform rayToTrans,
  btCollisionObject collisionObject,
  btCollisionShape collisionShape, final btTransform colObjWorldTransform,
  RayResultCallback resultCallback) {
  btCollisionObjectWrapper colObWrap = new btCollisionObjectWrapper(null, collisionShape,
   collisionObject, colObjWorldTransform, -1, -1);
  btCollisionWorld.rayTestSingleInternal(rayFromTrans, rayToTrans, colObWrap, resultCallback);
 }

 public static void rayTestSingleInternal(final btTransform rayFromTrans,
  final btTransform rayToTrans,
  btCollisionObjectWrapper collisionObjectWrap,
  RayResultCallback resultCallback) {
  btSphereShape pointShape = new btSphereShape((0.0f));
  pointShape.setMargin(0.f);
  btConvexShape castShape = pointShape;
  btCollisionShape collisionShape = collisionObjectWrap.getCollisionShape();
  final btTransform colObjWorldTransform = collisionObjectWrap.getWorldTransform();
  if (collisionShape.isConvex()) {
   //		BT_PROFILE("rayTestConvex");
   btConvexCast.CastResult castResult = new btConvexCast.CastResult();
   castResult.m_fraction = resultCallback.m_closestHitFraction;
   btConvexShape convexShape = (btConvexShape) collisionShape;
   btVoronoiSimplexSolver simplexSolver = new btVoronoiSimplexSolver();
   btConvexCast convexCasterPtr;
   //use kF_UseSubSimplexConvexCastRaytest by default
   if ((resultCallback.m_flags & btTriangleRaycastCallback.kF_UseGjkConvexCastRaytest) != 0) {
    convexCasterPtr = new btGjkConvexCast(castShape, convexShape, simplexSolver);
   } else {
    convexCasterPtr = new btSubsimplexConvexCast(castShape, convexShape, simplexSolver);
   }
   btConvexCast convexCaster = convexCasterPtr;
   if (convexCaster.calcTimeOfImpact(rayFromTrans, rayToTrans, colObjWorldTransform,
    colObjWorldTransform, castResult)) {
    //add hit
    if (castResult.m_normal.lengthSquared() > 0.0001f) {
     if (castResult.m_fraction < resultCallback.m_closestHitFraction) {
      LocalRayResult localRayResult = new LocalRayResult(
       collisionObjectWrap.getCollisionObject(),
       null,
       castResult.m_normal,
       castResult.m_fraction
      );
      boolean normalInWorldSpace = true;
      resultCallback.addSingleResult(localRayResult, normalInWorldSpace);
     }
    }
   }
  } else if (collisionShape.isConcave()) {
   final btTransform worldTocollisionObject = new btTransform(colObjWorldTransform).invert();
   final btVector3 rayFromLocal = worldTocollisionObject.transform(rayFromTrans.getOrigin());
   final btVector3 rayToLocal = worldTocollisionObject.transform(rayToTrans.getOrigin());
   BT_PROFILE("rayTestConcave");
   if (collisionShape.getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE) {
    ///optimized version for btBvhTriangleMeshShape
    btBvhTriangleMeshShape triangleMesh = (btBvhTriangleMeshShape) collisionShape;
    BridgeTriangleRaycastCallback rcb = new BridgeTriangleRaycastCallback(rayFromLocal, rayToLocal,
     resultCallback,
     collisionObjectWrap.getCollisionObject(), triangleMesh, colObjWorldTransform
    );
    rcb.m_hitFraction = resultCallback.m_closestHitFraction;
    triangleMesh.performRaycast(rcb, rayFromLocal, rayToLocal);
   } else {
    //generic (slower) case
    btConcaveShape concaveShape = (btConcaveShape) collisionShape;
    BridgeTriangleRaycastCallback rcb = new BridgeTriangleRaycastCallback(rayFromLocal, rayToLocal,
     resultCallback,
     collisionObjectWrap.getCollisionObject(), concaveShape, colObjWorldTransform
    );
    rcb.m_hitFraction = resultCallback.m_closestHitFraction;
    final btVector3 rayAabbMinLocal = rayFromLocal;
    rayAabbMinLocal.setMin(rayToLocal);
    final btVector3 rayAabbMaxLocal = rayFromLocal;
    rayAabbMaxLocal.setMax(rayToLocal);
    concaveShape.processAllTriangles(rcb, rayAabbMinLocal, rayAabbMaxLocal);
   }
  } else {
   BT_PROFILE("rayTestCompound");
   if (collisionShape.isCompound()) {
    btCompoundShape compoundShape = (btCompoundShape) (collisionShape);
    RayTester rayCB = new RayTester(
     collisionObjectWrap.getCollisionObject(),
     compoundShape,
     colObjWorldTransform,
     rayFromTrans,
     rayToTrans,
     resultCallback
    );
    btDbvt dbvt = compoundShape.getDynamicAabbTree();
    if (dbvt != null) {
     final btVector3 localRayFrom = colObjWorldTransform.inverseTimes(rayFromTrans).getOrigin();
     final btVector3 localRayTo = colObjWorldTransform.inverseTimes(rayToTrans).getOrigin();
     btDbvt.rayTest(dbvt.m_root, localRayFrom, localRayTo, rayCB);
    } else {
     for (int i = 0, n = compoundShape.getNumChildShapes(); i < n; ++i) {
      rayCB.processLeaf(i);
     }
    }
   }
  }
 }

 public static void objectQuerySingleInternal(btConvexShape castShape,
  final btTransform convexFromTrans, final btTransform convexToTrans,
  btCollisionObjectWrapper colObjWrap,
  ConvexResultCallback resultCallback, float allowedPenetration) {
  btCollisionShape collisionShape = colObjWrap.getCollisionShape();
  final btTransform colObjWorldTransform = colObjWrap.getWorldTransform();
  if (collisionShape.isConvex()) {
   BT_PROFILE("convexSweepConvex");
   btConvexCast.CastResult castResult = new btConvexCast.CastResult();
   castResult.m_allowedPenetration = allowedPenetration;
   castResult.m_fraction = resultCallback.m_closestHitFraction;//float(1.);//??
   btConvexShape convexShape = (btConvexShape) collisionShape;
   btVoronoiSimplexSolver simplexSolver = new btVoronoiSimplexSolver();
   btGjkEpaPenetrationDepthSolver gjkEpaPenetrationSolver = new btGjkEpaPenetrationDepthSolver();
   btContinuousConvexCollision convexCaster1 = new btContinuousConvexCollision(castShape,
    convexShape, simplexSolver, gjkEpaPenetrationSolver);
   btConvexCast castPtr = convexCaster1;
   if (castPtr.calcTimeOfImpact(convexFromTrans, convexToTrans, colObjWorldTransform,
    colObjWorldTransform, castResult)) {
    //add hit
    if (castResult.m_normal.lengthSquared() > 0.0001f) {
     if (castResult.m_fraction < resultCallback.m_closestHitFraction) {
      castResult.m_normal.normalize();
      LocalConvexResult localConvexResult = new LocalConvexResult(
       colObjWrap.getCollisionObject(),
       null,
       castResult.m_normal,
       castResult.m_hitPoint,
       castResult.m_fraction
      );
      boolean normalInWorldSpace = true;
      resultCallback.addSingleResult(localConvexResult, normalInWorldSpace);
     }
    }
   }
  } else if (collisionShape.isConcave()) {
   switch (collisionShape.getShapeType()) {
    case TRIANGLE_MESH_SHAPE_PROXYTYPE: {
     BT_PROFILE("convexSweepbtBvhTriangleMesh");
     btBvhTriangleMeshShape triangleMesh = (btBvhTriangleMeshShape) collisionShape;
     final btTransform worldTocollisionObject = new btTransform(colObjWorldTransform).invert();
     final btVector3 convexFromLocal = worldTocollisionObject.transform(convexFromTrans.getOrigin());
     final btVector3 convexToLocal = worldTocollisionObject.transform(convexToTrans.getOrigin());
     // rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation
     final btTransform rotationXform = new btTransform(worldTocollisionObject.getBasis().mul(
      convexToTrans
      .getBasis()));
     BridgeTriangleConvexcastCallbackA tccb = new BridgeTriangleConvexcastCallbackA(castShape,
      convexFromTrans, convexToTrans, resultCallback, colObjWrap.getCollisionObject(), triangleMesh,
      colObjWorldTransform);
     tccb.m_hitFraction = resultCallback.m_closestHitFraction;
     tccb.m_allowedPenetration = allowedPenetration;
     final btVector3 boxMinLocal = new btVector3();
     final btVector3 boxMaxLocal = new btVector3();
     castShape.getAabb(rotationXform, boxMinLocal, boxMaxLocal);
     triangleMesh.performConvexcast(tccb, convexFromLocal, convexToLocal, boxMinLocal, boxMaxLocal);
     break;
    }
    case STATIC_PLANE_PROXYTYPE:
     btConvexCast.CastResult castResult = new btConvexCast.CastResult();
     castResult.m_allowedPenetration = allowedPenetration;
     castResult.m_fraction = resultCallback.m_closestHitFraction;
     btStaticPlaneShape planeShape = (btStaticPlaneShape) collisionShape;
     btContinuousConvexCollision convexCaster1 = new btContinuousConvexCollision(castShape,
      planeShape);
     btConvexCast castPtr = convexCaster1;
     if (castPtr.calcTimeOfImpact(convexFromTrans, convexToTrans, colObjWorldTransform,
      colObjWorldTransform, castResult)) {
      //add hit
      if (castResult.m_normal.lengthSquared() > (0.0001f)) {
       if (castResult.m_fraction < resultCallback.m_closestHitFraction) {
        castResult.m_normal.normalize();
        LocalConvexResult localConvexResult = new LocalConvexResult(
         colObjWrap.getCollisionObject(),
         null,
         castResult.m_normal,
         castResult.m_hitPoint,
         castResult.m_fraction
        );
        boolean normalInWorldSpace = true;
        resultCallback.addSingleResult(localConvexResult, normalInWorldSpace);
       }
      }
     }
     break;
    default: {
     BT_PROFILE("convexSweepConcave");
     btConcaveShape concaveShape = (btConcaveShape) collisionShape;
     final btTransform worldTocollisionObject = new btTransform(colObjWorldTransform).invert();
     final btVector3 convexFromLocal = worldTocollisionObject.transform(convexFromTrans.getOrigin());
     final btVector3 convexToLocal = worldTocollisionObject.transform(convexToTrans.getOrigin());
     // rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation
     final btTransform rotationXform = new btTransform(worldTocollisionObject.getBasis().mul(
      convexToTrans
      .getBasis()));
     BridgeTriangleConvexcastCallbackB tccb = new BridgeTriangleConvexcastCallbackB(castShape,
      convexFromTrans, convexToTrans, resultCallback, colObjWrap.getCollisionObject(), concaveShape,
      colObjWorldTransform);
     tccb.m_hitFraction = resultCallback.m_closestHitFraction;
     tccb.m_allowedPenetration = allowedPenetration;
     final btVector3 boxMinLocal = new btVector3();
     final btVector3 boxMaxLocal = new btVector3();
     castShape.getAabb(rotationXform, boxMinLocal, boxMaxLocal);
     final btVector3 rayAabbMinLocal = new btVector3(convexFromLocal);
     rayAabbMinLocal.setMin(convexToLocal);
     final btVector3 rayAabbMaxLocal = new btVector3(convexFromLocal);
     rayAabbMaxLocal.setMax(convexToLocal);
     rayAabbMinLocal.add(boxMinLocal);
     rayAabbMaxLocal.add(boxMaxLocal);
     concaveShape.processAllTriangles(tccb, rayAabbMinLocal, rayAabbMaxLocal);
     break;
    }
   }
  } else if (collisionShape.isCompound()) {
   BT_PROFILE("convexSweepCompound");
   btCompoundShape compoundShape = (btCompoundShape) (collisionShape);
   final btVector3 fromLocalAabbMin = new btVector3();
   final btVector3 fromLocalAabbMax = new btVector3();
   final btVector3 toLocalAabbMin = new btVector3();
   final btVector3 toLocalAabbMax = new btVector3();
   final btTransform colObjWorldTransform_inverted = new btTransform(colObjWorldTransform).invert();
   castShape.getAabb(colObjWorldTransform_inverted.mul(convexFromTrans), fromLocalAabbMin,
    fromLocalAabbMax);
   castShape.getAabb(colObjWorldTransform_inverted.mul(convexToTrans), toLocalAabbMin,
    toLocalAabbMax);
   fromLocalAabbMin.setMin(toLocalAabbMin);
   fromLocalAabbMax.setMax(toLocalAabbMax);
   btCompoundLeafSweepCallback callback = new btCompoundLeafSweepCallback(colObjWrap, castShape,
    convexFromTrans, convexToTrans,
    allowedPenetration, compoundShape, colObjWorldTransform, resultCallback);
   btDbvt tree = compoundShape.getDynamicAabbTree();
   if (tree != null) {
    btDbvtAabbMm bounds = btDbvtAabbMm.fromMM(fromLocalAabbMin, fromLocalAabbMax);
    tree.collideTV(tree.m_root, bounds, callback);
   } else {
    int i;
    for (i = 0; i < compoundShape.getNumChildShapes(); i++) {
     btCollisionShape childCollisionShape = compoundShape.getChildShape(i);
     final btTransform childTrans = compoundShape.getChildTransform(i);
     callback.processChild(i, childTrans, childCollisionShape);
    }
   }
  }
 }
 protected final ArrayList<btCollisionObject> m_collisionObjects = new ArrayList<>(0);
 final protected btDispatcher m_dispatcher1;
 final protected btDispatcherInfo m_dispatchInfo = new btDispatcherInfo();
 protected btBroadphaseInterface m_broadphasePairCache;
 protected btIDebugDraw m_debugDrawer;
 ///m_forceUpdateAllAabbs can be set to false as an optimization to only update active object AABBs
 ///it is true by default, because it is error-prone (setting the position of static objects wouldn't update their AABB)
 boolean m_forceUpdateAllAabbs;

 /**
  *
  * @param dispatcher
  * @param broadphasePairCache
  * @param collisionConfiguration
  */
 public btCollisionWorld(btDispatcher dispatcher, btBroadphaseInterface broadphasePairCache,
  btCollisionConfiguration collisionConfiguration) {
  m_dispatcher1 = dispatcher;
  m_broadphasePairCache = broadphasePairCache;
  m_debugDrawer = null;
  m_forceUpdateAllAabbs = true;
 }

 /**
  * C++ destructor need to call manually in java
  */
 public void destroy() {
  //clean up remaining objects
  int i;
  for (i = 0; i < m_collisionObjects.size(); i++) {
   btCollisionObject collisionObject = m_collisionObjects.get(i);
   btBroadphaseProxy bp = collisionObject.getBroadphaseHandle();
   if (bp != null) {
    //
    // only clear the cached algorithms
    //
    getBroadphase().getOverlappingPairCache().cleanProxyFromPairs(bp, m_dispatcher1);
    getBroadphase().destroyProxy(bp, m_dispatcher1);
    collisionObject.setBroadphaseHandle(null);
   }
  }
 }

 public void setBroadphase(btBroadphaseInterface pairCache) {
  m_broadphasePairCache = pairCache;
 }

 public btBroadphaseInterface getBroadphase() {
  return m_broadphasePairCache;
 }

 public btOverlappingPairCache getPairCache() {
  return m_broadphasePairCache.getOverlappingPairCache();
 }

 public btDispatcher getDispatcher() {
  return m_dispatcher1;
 }

 public void updateSingleAabb(btCollisionObject colObj) {
  final btVector3 minAabb = new btVector3();
  final btVector3 maxAabb = new btVector3();
  colObj.getCollisionShape().getAabb(colObj.getWorldTransformPtr(), minAabb, maxAabb);
  //need to increase the aabb for contact thresholds
  final btVector3 contactThreshold = new btVector3(gContactBreakingThreshold,
   gContactBreakingThreshold,
   gContactBreakingThreshold);
  minAabb.sub(contactThreshold);
  maxAabb.add(contactThreshold);
  if (getDispatchInfo().m_useContinuous && colObj.getInternalType() == CO_RIGID_BODY && !colObj
   .isStaticOrKinematicObject()) {
   final btVector3 minAabb2 = new btVector3();
   final btVector3 maxAabb2 = new btVector3();
   colObj.getCollisionShape().getAabb(colObj.getInterpolationWorldTransform(), minAabb2, maxAabb2);
   minAabb2.sub(contactThreshold);
   maxAabb2.sub(contactThreshold);
   minAabb.setMin(minAabb2);
   maxAabb.setMax(maxAabb2);
  }
  btBroadphaseInterface bp = (btBroadphaseInterface) m_broadphasePairCache;
  //moving objects should be moderately sized, probably something wrong if not
  if (colObj.isStaticObject() || ((new btVector3(maxAabb).sub(minAabb).lengthSquared() < 1e12f))) {
   bp.setAabb(colObj.getBroadphaseHandle(), minAabb, maxAabb, m_dispatcher1);
  } else {
   //something went wrong, investigate
   //this assert is unwanted in 3D modelers (danger of loosing work)
   colObj.setActivationState(DISABLE_SIMULATION);
   if (reportMe && m_debugDrawer != null) {
    reportMe = false;
    m_debugDrawer.reportErrorWarning("Overflow in AABB, object removed from simulation");
    m_debugDrawer.reportErrorWarning(
     "If you can reproduce this, please email bugs@continuousphysics.com\n");
    m_debugDrawer.reportErrorWarning(
     "Please include above information, your Platform, version of OS.\n");
    m_debugDrawer.reportErrorWarning("Thanks.\n");
   }
  }
 }

 public void updateAabbs() {
  BT_PROFILE("updateAabbs");
  for (int i = 0; i < m_collisionObjects.size(); i++) {
   btCollisionObject colObj = m_collisionObjects.get(i);
   assert (colObj.getWorldArrayIndex() == i);
   //only update aabb of active objects
   if (m_forceUpdateAllAabbs || colObj.isActive()) {
    updateSingleAabb(colObj);
   }
  }
 }

 ///the computeOverlappingPairs is usually already called by performDiscreteCollisionDetection (or stepSimulation)
 ///it can be useful to use if you perform ray tests without collision detection/simulation
 public void computeOverlappingPairs() {
  BT_PROFILE("calculateOverlappingPairs");
  m_broadphasePairCache.calculateOverlappingPairs(m_dispatcher1);
 }

 public void setDebugDrawer(btIDebugDraw debugDrawer) {
  m_debugDrawer = debugDrawer;
 }

 public btIDebugDraw getDebugDrawer() {
  return m_debugDrawer;
 }

 public boolean debugDrawWorld() {
  if (getDebugDrawer() != null) {
   boolean all_sleeping = true;
   DefaultColors defaultColors = getDebugDrawer().getDefaultColors();
   if ((getDebugDrawer().getDebugMode() & DBG_DrawContactPoints) != 0) {
    if (getDispatcher() != null) {
     int numManifolds = getDispatcher().getNumManifolds();
     for (int i = 0; i < numManifolds; i++) {
      btPersistentManifold contactManifold = getDispatcher().getManifoldByIndexInternal(i);
      //btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold.getBody0());
      //btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold.getBody1());
      int numContacts = contactManifold.getNumContacts();
      for (int j = 0; j < numContacts; j++) {
       btManifoldPoint cp = contactManifold.getContactPoint(j);
       getDebugDrawer().drawContactPoint(cp.m_positionWorldOnB, cp.m_normalWorldOnB, cp
        .getDistance(), cp.getLifeTime(), defaultColors.m_contactPoint);
      }
     }
    }
   }
   if ((getDebugDrawer().getDebugMode() & (btIDebugDraw.DBG_DrawWireframe |
    btIDebugDraw.DBG_DrawAabb)) != 0) {
    int i;
    for (i = 0; i < m_collisionObjects.size(); i++) {
     btCollisionObject colObj = m_collisionObjects.get(i);
     if ((colObj.getCollisionFlags() & CF_DISABLE_VISUALIZE_OBJECT) == 0) {
      if ((getDebugDrawer() != null) && ((getDebugDrawer().getDebugMode() &
       btIDebugDraw.DBG_DrawWireframe) != 0)) {
       final btVector3 color = new btVector3();
       switch (colObj.getActivationState()) {
        case btCollisionObject.ACTIVE_TAG:
         color.set(defaultColors.m_activeObject);
         all_sleeping = false;
         break;
        case btCollisionObject.ISLAND_SLEEPING:
         color.set(defaultColors.m_deactivatedObject);
         break;
        case btCollisionObject.WANTS_DEACTIVATION:
         color.set(defaultColors.m_wantsDeactivationObject);
         all_sleeping = false;
         break;
        case btCollisionObject.DISABLE_DEACTIVATION:
         color.set(defaultColors.m_disabledDeactivationObject);
         all_sleeping = false;
         break;
        case btCollisionObject.DISABLE_SIMULATION:
         color.set(defaultColors.m_disabledSimulationObject);
         break;
        default: {
         color.set(new btVector3(.3f, 0.3f, 0.3f));
        }
       }
       colObj.getCustomDebugColor(color);
       debugDrawObject(colObj.getWorldTransformPtr(), colObj.getCollisionShape(), color);
      }
      if ((m_debugDrawer != null) && ((m_debugDrawer.getDebugMode() & btIDebugDraw.DBG_DrawAabb) !=
       0)) {
       final btVector3 minAabb = new btVector3();
       final btVector3 maxAabb = new btVector3();
       final btVector3 colorvec = defaultColors.m_aabb;
       colObj.getCollisionShape().getAabb(colObj.getWorldTransformPtr(), minAabb, maxAabb);
       final btVector3 contactThreshold = new btVector3(gContactBreakingThreshold,
        gContactBreakingThreshold, gContactBreakingThreshold);
       minAabb.sub(contactThreshold);
       maxAabb.add(contactThreshold);
       final btVector3 minAabb2 = new btVector3();
       final btVector3 maxAabb2 = new btVector3();
       if (getDispatchInfo().m_useContinuous && colObj.getInternalType() == CO_RIGID_BODY && !colObj
        .isStaticOrKinematicObject()) {
        colObj.getCollisionShape().getAabb(colObj.getInterpolationWorldTransform(), minAabb2,
         maxAabb2);
        minAabb2.sub(contactThreshold);
        maxAabb2.add(contactThreshold);
        minAabb.setMin(minAabb2);
        maxAabb.setMax(maxAabb2);
       }
       m_debugDrawer.drawAabb(minAabb, maxAabb, colorvec);
      }
     }
    }
   }
   return !all_sleeping;
  } else {
   return false;
  }
 }

 public void debugDrawObject(final btTransform worldTransform, btCollisionShape shape,
  final btVector3 color) {
  // Draw a small simplex at the center of the object
  if (getDebugDrawer() != null && (getDebugDrawer().getDebugMode() & btIDebugDraw.DBG_DrawFrames) !=
   0) {
   getDebugDrawer().drawTransform(worldTransform, 1);
  }
  if (shape.getShapeType() == COMPOUND_SHAPE_PROXYTYPE) {
   btCompoundShape compoundShape = (btCompoundShape) (shape);
   for (int i = compoundShape.getNumChildShapes() - 1; i >= 0; i--) {
    final btTransform childTrans = compoundShape.getChildTransform(i);
    btCollisionShape colShape = compoundShape.getChildShape(i);
    debugDrawObject(new btTransform(worldTransform).mul(childTrans), colShape, color);
   }
  } else {
   switch (shape.getShapeType()) {
    case BOX_SHAPE_PROXYTYPE: {
     btBoxShape boxShape = (btBoxShape) (shape);
     final btVector3 halfExtents = boxShape.getHalfExtentsWithMargin();
     getDebugDrawer().drawBox(new btVector3(halfExtents).negate(), halfExtents, worldTransform,
      color);
     break;
    }
    case SPHERE_SHAPE_PROXYTYPE: {
     btSphereShape sphereShape = (btSphereShape) (shape);
     float radius = sphereShape.getMargin();//radius doesn't include the margin, so draw with margin
     getDebugDrawer().drawSphere(radius, worldTransform, color);
     break;
    }
    case MULTI_SPHERE_SHAPE_PROXYTYPE: {
     btMultiSphereShape multiSphereShape = (btMultiSphereShape) (shape);
     final btTransform childTransform = new btTransform();
     childTransform.setIdentity();
     for (int i = multiSphereShape.getSphereCount() - 1; i >= 0; i--) {
      childTransform.setOrigin(multiSphereShape.getSpherePosition(i));
      getDebugDrawer().drawSphere(multiSphereShape.getSphereRadius(i), new btTransform(
       worldTransform).mul(childTransform), color);
     }
     break;
    }
    case CAPSULE_SHAPE_PROXYTYPE: {
     btCapsuleShape capsuleShape = (btCapsuleShape) (shape);
     float radius = capsuleShape.getRadius();
     float halfHeight = capsuleShape.getHalfHeight();
     int upAxis = capsuleShape.getUpAxis();
     getDebugDrawer().drawCapsule(radius, halfHeight, upAxis, worldTransform, color);
     break;
    }
    case CONE_SHAPE_PROXYTYPE: {
     btConeShape coneShape = (btConeShape) (shape);
     float radius = coneShape.getRadius();//+coneShape.getMargin();
     float height = coneShape.getHeight();//+coneShape.getMargin();
     int upAxis = coneShape.getConeUpIndex();
     getDebugDrawer().drawCone(radius, height, upAxis, worldTransform, color);
     break;
    }
    case CYLINDER_SHAPE_PROXYTYPE: {
     btCylinderShape cylinder = (btCylinderShape) (shape);
     int upAxis = cylinder.getUpAxis();
     float radius = cylinder.getRadius();
     float halfHeight = cylinder.getHalfExtentsWithMargin().getElement(upAxis);
     getDebugDrawer().drawCylinder(radius, halfHeight, upAxis, worldTransform, color);
     break;
    }
    case STATIC_PLANE_PROXYTYPE: {
     btStaticPlaneShape staticPlaneShape = (btStaticPlaneShape) (shape);
     float planeConst = staticPlaneShape.getPlaneConstant();
     final btVector3 planeNormal = staticPlaneShape.getPlaneNormal();
     getDebugDrawer().drawPlane(planeNormal, planeConst, worldTransform, color);
     break;
    }
    default: {
     /// for polyhedral shapes
     if (shape.isPolyhedral()) {
      btPolyhedralConvexShape polyshape = (btPolyhedralConvexShape) shape;
      int i;
      if (polyshape.getConvexPolyhedron() != null) {
       /* dead code */
       assert (false);
       /*
       btConvexPolyhedron poly = polyshape.getConvexPolyhedron();
       for (i = 0; i < poly.m_faces.size(); i++) {
        btFace face = poly.m_faces.get(i);
        final btVector3 centroid = new btVector3(0, 0, 0);
        int numVerts = face.m_indices.length;
        if (numVerts > 0) {
         int lastV = face.m_indices[numVerts - 1];
         for (int v = 0; v < face.m_indices.length; v++) {
          int curVert = face.m_indices[v];
          centroid.add(poly.m_vertices.get(curVert));
          getDebugDrawer().drawLine(
           worldTransform.transform(new btVector3(poly.m_vertices.get(lastV))),
           worldTransform.transform(new btVector3(poly.m_vertices.get(curVert))),
           color);
          lastV = curVert;
         }
        }
        centroid.scale(1.f / numVerts);
        if ((getDebugDrawer().getDebugMode() & btIDebugDraw.DBG_DrawNormals) != 0) {
         final btVector3 normalColor = new btVector3(1, 1, 0);
         final btVector3 faceNormal = new btVector3(face.m_plane[0], face.m_plane[1],
          face.m_plane[2]);
         getDebugDrawer().drawLine(
          worldTransform.transform(new btVector3(centroid)),
          worldTransform.transform(new btVector3(centroid).add(faceNormal)),
          normalColor);
        }
       }
        */
      } else {
       final btVector3 a = new btVector3();
       final btVector3 b = new btVector3();
       for (i = 0; i < polyshape.getNumEdges(); i++) {
        polyshape.getEdge(i, a, b);
        final btVector3 wa = worldTransform.transform(a);
        final btVector3 wb = worldTransform.transform(b);
        getDebugDrawer().drawLine(wa, wb, color);
       }
      }
     }
     if (shape.isConcave()) {
      btConcaveShape concaveMesh = (btConcaveShape) shape;
      ///@todo pass camera, for some culling? no . we are not a graphics lib
      final btVector3 aabbMax = new btVector3((BT_LARGE_FLOAT), (BT_LARGE_FLOAT), (BT_LARGE_FLOAT));
      final btVector3 aabbMin = new btVector3((-BT_LARGE_FLOAT), (-BT_LARGE_FLOAT),
       (-BT_LARGE_FLOAT));
      DebugDrawcallback drawCallback =
       new DebugDrawcallback(getDebugDrawer(), worldTransform, color);
      concaveMesh.processAllTriangles(drawCallback, aabbMin, aabbMax);
     }
     if (shape.getShapeType() == CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE) {
      btConvexTriangleMeshShape convexMesh = (btConvexTriangleMeshShape) shape;
      //todo: pass camera for some culling			
      final btVector3 aabbMax = new btVector3((BT_LARGE_FLOAT), (BT_LARGE_FLOAT), (BT_LARGE_FLOAT));
      final btVector3 aabbMin = new btVector3((-BT_LARGE_FLOAT), (-BT_LARGE_FLOAT),
       (-BT_LARGE_FLOAT));
      DebugDrawcallback drawCallback =
       new DebugDrawcallback(getDebugDrawer(), worldTransform, color);
      convexMesh.getMeshInterface().InternalProcessAllTriangles(drawCallback, aabbMin, aabbMax);
     }
    }
   }
  }
 }

 public int getNumCollisionObjects() {
  return m_collisionObjects.size();
 }

 /// rayTest performs a raycast on all objects in the btCollisionWorld, and calls the resultCallback
 /// This allows for several queries: first hit, all hits, any hit, dependent on the value returned by the callback.
 public void rayTest(final btVector3 rayFromWorld, final btVector3 rayToWorld,
  RayResultCallback resultCallback) {
  //BT_PROFILE("rayTest");
  /// use the broadphase to accelerate the search for objects, based on their aabb
  /// and for each object with ray-aabb overlap, perform an exact ray test
  btSingleRayCallback rayCB =
   new btSingleRayCallback(rayFromWorld, rayToWorld, this, resultCallback);
  m_broadphasePairCache.rayTest(rayFromWorld, rayToWorld, rayCB);
 }

 /// convexTest performs a swept convex cast on all objects in the btCollisionWorld, and calls the resultCallback
 /// This allows for several queries: first hit, all hits, any hit, dependent on the value return by the callback.
 public void convexSweepTest(btConvexShape castShape, final btTransform convexFromWorld,
  final btTransform convexToWorld, ConvexResultCallback resultCallback, float allowedCcdPenetration) {
  BT_PROFILE("convexSweepTest");
  /// use the broadphase to accelerate the search for objects, based on their aabb
  /// and for each object with ray-aabb overlap, perform an exact ray test
  /// unfortunately the implementation for rayTest and convexSweepTest duplicated, albeit practically identical
  final btTransform convexFromTrans;
  final btTransform convexToTrans;
  convexFromTrans = convexFromWorld;
  convexToTrans = convexToWorld;
  final btVector3 castShapeAabbMin = new btVector3();
  final btVector3 castShapeAabbMax = new btVector3();
  /*
   * Compute AABB that encompasses angular movement
   */
  {
   final btVector3 linVel = new btVector3();
   final btVector3 angVel = new btVector3();
   btTransformUtil.calculateVelocity(convexFromTrans, convexToTrans, 1.0f, linVel, angVel);
   final btVector3 zeroLinVel = new btVector3();
   final btTransform R = new btTransform();
   R.setIdentity();
   R.set3x3(convexFromTrans.getRotation());
   castShape.calculateTemporalAabb(R, zeroLinVel, angVel, 1.0f, castShapeAabbMin, castShapeAabbMax);
  }
  btSingleSweepCallback convexCB = new btSingleSweepCallback(castShape, convexFromWorld,
   convexToWorld, this, resultCallback, allowedCcdPenetration);
  m_broadphasePairCache.rayTest(convexFromTrans.getOrigin(), convexToTrans.getOrigin(), convexCB,
   castShapeAabbMin, castShapeAabbMax);
 }

 public void convexSweepTest(btConvexShape castShape, final btTransform convexFromWorld,
  final btTransform convexToWorld, ConvexResultCallback resultCallback) {
  convexSweepTest(castShape, convexFromWorld, convexToWorld, resultCallback, 0f);
 }

 ///contactTest performs a discrete collision test between colObj against all objects in the btCollisionWorld, and calls the resultCallback.
 ///it reports one or more contact points for every overlapping object (including the one with deepest penetration)
 public void contactTest(btCollisionObject colObj, ContactResultCallback resultCallback) {
  final btVector3 aabbMin = new btVector3();
  final btVector3 aabbMax = new btVector3();
  colObj.getCollisionShape().getAabb(colObj.getWorldTransformPtr(), aabbMin, aabbMax);
  btSingleContactCallback contactCB = new btSingleContactCallback(colObj, this, resultCallback);
  m_broadphasePairCache.aabbTest(aabbMin, aabbMax, contactCB);
 }

 ///contactTest performs a discrete collision test between two collision objects and calls the resultCallback if overlap if detected.
 ///it reports one or more contact points (including the one with deepest penetration)
 public void contactPairTest(btCollisionObject colObjA, btCollisionObject colObjB,
  ContactResultCallback resultCallback) {
  btCollisionObjectWrapper obA = new btCollisionObjectWrapper(null, colObjA.getCollisionShape(),
   colObjA, colObjA.getWorldTransformPtr(), -1, -1);
  btCollisionObjectWrapper obB = new btCollisionObjectWrapper(null, colObjB.getCollisionShape(),
   colObjB, colObjB.getWorldTransformPtr(), -1, -1);
  btCollisionAlgorithm algorithm = getDispatcher().findAlgorithm(obA, obB, null,
   BT_CLOSEST_POINT_ALGORITHMS);
  if (algorithm != null) {
   btBridgedManifoldResult contactPointResult =
    new btBridgedManifoldResult(obA, obB, resultCallback);
   contactPointResult.m_closestPointDistanceThreshold = resultCallback.m_closestDistanceThreshold;
   //discrete collision detection query
   algorithm.processCollision(obA, obB, getDispatchInfo(), contactPointResult);
  }
 }

/// objectQuerySingle performs a collision detection query and calls the resultCallback. It is used internally by rayTest.
 public void objectQuerySingle(btConvexShape castShape, final btTransform convexFromTrans,
  final btTransform convexToTrans,
  btCollisionObject collisionObject,
  btCollisionShape collisionShape, final btTransform colObjWorldTransform,
  ConvexResultCallback resultCallback, float allowedPenetration
 ) {
  btCollisionObjectWrapper tmpOb = new btCollisionObjectWrapper(null, collisionShape,
   collisionObject, colObjWorldTransform, -1, -1);
  btCollisionWorld.objectQuerySingleInternal(castShape, convexFromTrans, convexToTrans, tmpOb,
   resultCallback, allowedPenetration);
 }

 public void addCollisionObject(btCollisionObject collisionObject) {
  addCollisionObject(collisionObject, btBroadphaseProxy.DefaultFilter, btBroadphaseProxy.AllFilter);
 }

 public void addCollisionObject(btCollisionObject collisionObject, int collisionFilterGroup) {
  addCollisionObject(collisionObject, collisionFilterGroup, btBroadphaseProxy.AllFilter);
 }

 public void addCollisionObject(btCollisionObject collisionObject, int collisionFilterGroup,
  int collisionFilterMask) {
  assert (collisionObject != null);
  //check that the object isn't already added
  assert (findLinearSearch(m_collisionObjects, collisionObject) == m_collisionObjects.size());
  assert (collisionObject.getWorldArrayIndex() == -1);  // do not add the same object to more than one collision world
  collisionObject.setWorldArrayIndex(m_collisionObjects.size());
  m_collisionObjects.add(collisionObject);
  //calculate new AABB
  final btTransform trans = collisionObject.getWorldTransformPtr();
  final btVector3 minAabb = new btVector3();
  final btVector3 maxAabb = new btVector3();
  collisionObject.getCollisionShape().getAabb(trans, minAabb, maxAabb);
  int type = collisionObject.getCollisionShape().getShapeType();
  collisionObject.setBroadphaseHandle(getBroadphase().createProxy(
   minAabb,
   maxAabb,
   type,
   collisionObject,
   collisionFilterGroup,
   collisionFilterMask,
   m_dispatcher1));
 }

 public ArrayList<btCollisionObject> getCollisionObjectArray() {
  return m_collisionObjects;
 }

 public void removeCollisionObject(btCollisionObject collisionObject) {
  //boolean removeFromBroadphase = false;
  {
   btBroadphaseProxy bp = collisionObject.getBroadphaseHandle();
   if (bp != null) {
    //
    // only clear the cached algorithms
    //
    getBroadphase().getOverlappingPairCache().cleanProxyFromPairs(bp, m_dispatcher1);
    getBroadphase().destroyProxy(bp, m_dispatcher1);
    collisionObject.setBroadphaseHandle(null);
   }
  }
  int iObj = collisionObject.getWorldArrayIndex();
//    assert(iObj >= 0 && iObj < m_collisionObjects.size()); // trying to remove an object that was never added or already removed previously?
  if (iObj >= 0 && iObj < m_collisionObjects.size()) {
   assert (collisionObject == m_collisionObjects.get(iObj));
   //m_collisionObjects.swap(iObj, m_collisionObjects.size()-1);
   swap(m_collisionObjects, iObj, m_collisionObjects.size() - 1);
   m_collisionObjects.remove(m_collisionObjects.size() - 1);
   if (iObj < m_collisionObjects.size()) {
    m_collisionObjects.get(iObj).setWorldArrayIndex(iObj);
   }
  } else {
   // slow linear search
   //swapremove
   m_collisionObjects.remove(collisionObject);
  }
  collisionObject.setWorldArrayIndex(-1);
 }

 public void performDiscreteCollisionDetection() {
  BT_PROFILE("performDiscreteCollisionDetection");
  btDispatcherInfo dispatchInfo = getDispatchInfo();
  updateAabbs();
  computeOverlappingPairs();
  btDispatcher dispatcher = getDispatcher();
  {
   BT_PROFILE("dispatchAllCollisionPairs");
   if (dispatcher != null) {
    dispatcher.dispatchAllCollisionPairs(m_broadphasePairCache.getOverlappingPairCache(),
     dispatchInfo, m_dispatcher1);
   }
  }
 }

 public btDispatcherInfo getDispatchInfo() {
  return m_dispatchInfo;
 }

 public boolean getForceUpdateAllAabbs() {
  return m_forceUpdateAllAabbs;
 }

 public void setForceUpdateAllAabbs(boolean forceUpdateAllAabbs) {
  m_forceUpdateAllAabbs = forceUpdateAllAabbs;
 }
}
