/*
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org
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
package Bullet.Dynamics;

import Bullet.Collision.Broadphase.btBroadphaseInterface;
import Bullet.Collision.Broadphase.btBroadphaseProxy;
import Bullet.Collision.Broadphase.btDispatcher;
import Bullet.Collision.Broadphase.btDispatcherInfo;
import Bullet.Collision.Shape.btSphereShape;
import Bullet.Collision.btCollisionConfiguration;
import Bullet.Collision.btCollisionObject;
import static Bullet.Collision.btCollisionObject.ACTIVE_TAG;
import static Bullet.Collision.btCollisionObject.DISABLE_DEACTIVATION;
import static Bullet.Collision.btCollisionObject.ISLAND_SLEEPING;
import static Bullet.Collision.btCollisionObject.WANTS_DEACTIVATION;
import Bullet.Collision.btCollisionWorld;
import Bullet.Collision.btIDebugDraw;
import Bullet.Collision.btManifoldPoint;
import Bullet.Collision.btManifoldResult;
import Bullet.Collision.btPersistentManifold;
import Bullet.Collision.btSimulationIslandManager;
import Bullet.Dynamics.CollisionObjects.btRigidBody;
import Bullet.Dynamics.Constraint.btConeTwistConstraint;
import Bullet.Dynamics.Constraint.btGeneric6DofConstraint;
import Bullet.Dynamics.Constraint.btGeneric6DofSpring2Constraint;
import Bullet.Dynamics.Constraint.btHingeConstraint;
import Bullet.Dynamics.Constraint.btPoint2PointConstraint;
import Bullet.Dynamics.Constraint.btSliderConstraint;
import Bullet.Dynamics.Constraint.btTypedConstraint;
import static Bullet.Dynamics.Constraint.btTypedConstraint.CONETWIST_CONSTRAINT_TYPE;
import static Bullet.Dynamics.Constraint.btTypedConstraint.D6_CONSTRAINT_TYPE;
import static Bullet.Dynamics.Constraint.btTypedConstraint.D6_SPRING_2_CONSTRAINT_TYPE;
import static Bullet.Dynamics.Constraint.btTypedConstraint.D6_SPRING_CONSTRAINT_TYPE;
import static Bullet.Dynamics.Constraint.btTypedConstraint.HINGE_CONSTRAINT_TYPE;
import static Bullet.Dynamics.Constraint.btTypedConstraint.POINT2POINT_CONSTRAINT_TYPE;
import static Bullet.Dynamics.Constraint.btTypedConstraint.SLIDER_CONSTRAINT_TYPE;
import Bullet.Dynamics.ConstraintSolver.btConstraintSolver;
import Bullet.Dynamics.ConstraintSolver.btSequentialImpulseConstraintSolver;
import static Bullet.Dynamics.btDynamicsWorldType.BT_DISCRETE_DYNAMICS_WORLD;
import static Bullet.Dynamics.btRigidBodyFlags.BT_DISABLE_WORLD_GRAVITY;
import static Bullet.LinearMath.btQuickprof.BT_PROFILE;
import static Bullet.LinearMath.btScalar.SIMD_2_PI;
import static Bullet.LinearMath.btScalar.SIMD_PI;
import static Bullet.LinearMath.btScalar.btCos;
import static Bullet.LinearMath.btScalar.btFuzzyZero;
import static Bullet.LinearMath.btScalar.btSin;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btTransformUtil;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;
import static javax.vecmath.VecMath.is_good_matrix;

/**
 * btDiscreteDynamicsWorld provides discrete rigid body simulation those classes
 * replace the obsolete CcdPhysicsEnvironment/CcdPhysicsController
 *
 * @author Gregery Barton
 */
public class btDiscreteDynamicsWorld extends btDynamicsWorld implements
 Serializable {

 ///internal debugging variable. this value shouldn't be too high
 static int gNumClampedCcdMotions = 0;

 static int btGetConstraintIslandId(btTypedConstraint lhs) {
  int islandId;
  btCollisionObject rcolObj0 = lhs.getRigidBodyA();
  btCollisionObject rcolObj1 = lhs.getRigidBodyB();
  islandId = rcolObj0.getIslandTag() >= 0 ? rcolObj0.getIslandTag() : rcolObj1
   .getIslandTag();
  return islandId;
 }

 final ArrayList<btTypedConstraint> m_sortedConstraints = new ArrayList<>(0);
 final InplaceSolverIslandCallback m_solverIslandCallback;
 btConstraintSolver m_constraintSolver;
 final btSimulationIslandManager m_islandManager;
 final ArrayList<btTypedConstraint> m_constraints = new ArrayList<>(0);
 final ArrayList<btRigidBody> m_nonStaticRigidBodies = new ArrayList<>(0);
 final btVector3 m_gravity = new btVector3();
 //for variable timesteps
 float m_localTime;
 float m_fixedTimeStep;
 //for variable timesteps
 boolean m_ownsIslandManager;
 boolean m_ownsConstraintSolver;
 boolean m_synchronizeAllMotionStates;
 boolean m_applySpeculativeContactRestitution;
 final ArrayList<btActionInterface> m_actions = new ArrayList<>(0);
 int m_profileTimings;
 boolean m_latencyMotionStateInterpolation;
 final ArrayList<btPersistentManifold> m_predictiveManifolds = new ArrayList<>(0);
 final Object m_predictiveManifoldsMutex = new Object();  // used to synchronize threads creating predictive contacts
 ///this btDiscreteDynamicsWorld constructor gets created objects from the user, and will not delete those

 public btDiscreteDynamicsWorld(btDispatcher dispatcher,
  btBroadphaseInterface pairCache,
  btConstraintSolver constraintSolver,
  btCollisionConfiguration collisionConfiguration) {
  super(dispatcher, pairCache, collisionConfiguration);
  m_gravity.set(0f, -10f, 0f);
  m_localTime = 0f;
  m_fixedTimeStep = 0f;
  m_synchronizeAllMotionStates = false;
  m_applySpeculativeContactRestitution = false;
  m_profileTimings = 0;
  m_latencyMotionStateInterpolation = true;
  if (constraintSolver == null) {
   m_constraintSolver = new btSequentialImpulseConstraintSolver();
   m_ownsConstraintSolver = true;
  } else {
   m_constraintSolver = constraintSolver;
   m_ownsConstraintSolver = false;
  }
  {
   m_islandManager = new btSimulationIslandManager();
  }
  m_ownsIslandManager = true;
  {
   m_solverIslandCallback = new InplaceSolverIslandCallback(m_constraintSolver,
    null, dispatcher);
  }
 }

 void predictUnconstraintMotion(float timeStep) {
  BT_PROFILE("predictUnconstraintMotion");
  for (int i = 0; i < m_nonStaticRigidBodies.size(); i++) {
   btRigidBody body = m_nonStaticRigidBodies.get(i);
   if (!body.isStaticOrKinematicObject()) {
    //don't integrate/update velocities here, it happens in the constraint solver
    body.applyDamping(timeStep);
    body.predictIntegratedTransform(timeStep, body
     .getInterpolationWorldTransform());
   }
  }
 }

 void integrateTransformsInternal(List<btRigidBody> bodies, int numBodies,
  float timeStep) // can be called in parallel
 {
  final btTransform predictedTrans = new btTransform();
  for (int i = 0; i < numBodies; i++) {
   btRigidBody body = bodies.get(i);
   body.setHitFraction(1.f);
   if (body.isActive() && (!body.isStaticOrKinematicObject())) {
    body.predictIntegratedTransform(timeStep, predictedTrans);
    assert (is_good_matrix(predictedTrans));
    float squareMotion = (predictedTrans.getOrigin().sub(body
     .getWorldTransformPtr().getOrigin()))
     .lengthSquared();
    if (getDispatchInfo().m_useContinuous && body.getCcdSquareMotionThreshold()
     != 0.0f && body
      .getCcdSquareMotionThreshold() < squareMotion) {
     BT_PROFILE("CCD motion clamping");
     if (body.getCollisionShape().isConvex()) {
      gNumClampedCcdMotions++;
      btClosestNotMeConvexResultCallback sweepResults = new btClosestNotMeConvexResultCallback(
       body,
       body.getWorldTransformPtr().getOrigin(), predictedTrans.getOrigin(),
       getBroadphase()
        .getOverlappingPairCache(), getDispatcher());
      assert (is_good_matrix(body.getWorldTransformPtr()));
      btSphereShape tmpSphere = new btSphereShape(body.getCcdSweptSphereRadius());//btConvexShape* convexShape = static_cast<btConvexShape*>(body.getCollisionShape());
      sweepResults.m_allowedPenetration = getDispatchInfo().m_allowedCcdPenetration;
      sweepResults.m_collisionFilterGroup = body.getBroadphaseProxy().m_collisionFilterGroup;
      sweepResults.m_collisionFilterMask = body.getBroadphaseProxy().m_collisionFilterMask;
      final btTransform modifiedPredictedTrans = new btTransform(predictedTrans);
      assert (is_good_matrix(predictedTrans));
      modifiedPredictedTrans.setBasis(body.getWorldTransformPtr().getBasis());
      assert (is_good_matrix(modifiedPredictedTrans));
      convexSweepTest(tmpSphere, body.getWorldTransformPtr(),
       modifiedPredictedTrans, sweepResults);
      assert (is_good_matrix(modifiedPredictedTrans));
      assert (is_good_matrix(body.getWorldTransformPtr()));
      if (sweepResults.hasHit() && (sweepResults.m_closestHitFraction < 1.f)) {
       body.setHitFraction(sweepResults.m_closestHitFraction);
       body.predictIntegratedTransform(timeStep * body.getHitFraction(),
        predictedTrans);
       assert (is_good_matrix(predictedTrans));
       body.setHitFraction(0.f);
       body.proceedToTransform(predictedTrans);
       assert (is_good_matrix(predictedTrans));
       //don't apply the collision response right now, it will happen next frame
       //if you really need to, you can uncomment next 3 lines. Note that is uses zero restitution.
       //float appliedImpulse = 0.f;
       //float depth = 0.f;
       //appliedImpulse = resolveSingleCollision(body,(btCollisionObject*)sweepResults.m_hitCollisionObject,sweepResults.m_hitPointWorld,sweepResults.m_hitNormalWorld,getSolverInfo(), depth);
       continue;
      }
     }
    }
    body.proceedToTransform(predictedTrans);
   }
  }
 }

 void integrateTransforms(float timeStep) {
  BT_PROFILE("integrateTransforms");
  if (m_nonStaticRigidBodies.size() > 0) {
   integrateTransformsInternal(m_nonStaticRigidBodies, m_nonStaticRigidBodies
    .size(), timeStep);
  }
  ///this should probably be switched on by default, but it is not well tested yet
  if (m_applySpeculativeContactRestitution) {
   BT_PROFILE("apply speculative contact restitution");
   for (int i = 0; i < m_predictiveManifolds.size(); i++) {
    btPersistentManifold manifold = m_predictiveManifolds.get(i);
    btRigidBody body0 = btRigidBody.upcast((btCollisionObject) manifold
     .getBody0());
    btRigidBody body1 = btRigidBody.upcast((btCollisionObject) manifold
     .getBody1());
    if (body0 == null || body1 == null) {
     continue;
    }
    for (int p = 0; p < manifold.getNumContacts(); p++) {
     btManifoldPoint pt = manifold.getContactPoint(p);
     float combinedRestitution = btManifoldResult.calculateCombinedRestitution(
      body0, body1);
     if (combinedRestitution > 0 && pt.m_appliedImpulse != 0.f) //if (pt.getDistance()>0 && combinedRestitution>0 && pt.m_appliedImpulse != 0.f)
     {
      final btVector3 imp = new btVector3(pt.m_normalWorldOnB).scale(
       -pt.m_appliedImpulse * combinedRestitution);
      final btVector3 rel_pos0 = pt.getPositionWorldOnA().sub(body0
       .getWorldTransformPtr()
       .getOrigin());
      final btVector3 rel_pos1 = pt.getPositionWorldOnB().sub(body1
       .getWorldTransformPtr()
       .getOrigin());
      body0.applyImpulse(imp, rel_pos0);
      body1.applyImpulse(new btVector3(imp).negate(), rel_pos1);
     }
    }
   }
  }
 }

 void calculateSimulationIslands() {
  BT_PROFILE("calculateSimulationIslands");
  getSimulationIslandManager().updateActivationState(getCollisionWorld(),
   getCollisionWorld()
    .getDispatcher());
  {
   //merge islands based on speculative contact manifolds too
   for (int i = 0; i < this.m_predictiveManifolds.size(); i++) {
    btPersistentManifold manifold = m_predictiveManifolds.get(i);
    btCollisionObject colObj0 = manifold.getBody0();
    btCollisionObject colObj1 = manifold.getBody1();
    if (!(colObj0).isStaticOrKinematicObject() && !(colObj1)
     .isStaticOrKinematicObject()) {
     getSimulationIslandManager().getUnionFind().unite((colObj0).getIslandTag(),
      (colObj1)
       .getIslandTag());
    }
   }
  }
  {
   int i;
   int numConstraints = m_constraints.size();
   for (i = 0; i < numConstraints; i++) {
    btTypedConstraint constraint = m_constraints.get(i);
    if (constraint.isEnabled()) {
     btRigidBody colObj0 = constraint.getRigidBodyA();
     btRigidBody colObj1 = constraint.getRigidBodyB();
     if (!colObj0.isStaticOrKinematicObject() && !(colObj1)
      .isStaticOrKinematicObject()) {
      getSimulationIslandManager().getUnionFind()
       .unite((colObj0).getIslandTag(), (colObj1)
        .getIslandTag());
     }
    }
   }
  }
  //Store the island id in each body
  getSimulationIslandManager().storeIslandActivationState(getCollisionWorld());
 }

 void solveConstraints(btContactSolverInfo solverInfo) {
  BT_PROFILE("solveConstraints");
  m_sortedConstraints.clear();
  m_sortedConstraints.addAll(m_constraints);
  m_sortedConstraints.sort(new btSortConstraintOnIslandPredicate());
  ArrayList<btTypedConstraint> constraintsPtr = getNumConstraints() != 0 ? m_sortedConstraints
   : null;
  m_solverIslandCallback.setup(solverInfo, constraintsPtr, m_sortedConstraints
   .size(),
   getDebugDrawer());
  m_constraintSolver.prepareSolve(getCollisionWorld().getNumCollisionObjects(),
   getCollisionWorld()
    .getDispatcher().getNumManifolds());
  /// solve all the constraints for this island
  m_islandManager.buildAndProcessIslands(getCollisionWorld().getDispatcher(),
   getCollisionWorld(),
   m_solverIslandCallback);
  m_solverIslandCallback.processConstraints();
  m_constraintSolver.allSolved(solverInfo, m_debugDrawer);
 }

 void updateActivationState(float timeStep) {
  BT_PROFILE("updateActivationState");
  for (int i = 0; i < m_nonStaticRigidBodies.size(); i++) {
   btRigidBody body = m_nonStaticRigidBodies.get(i);
   body.updateDeactivation(timeStep);
   if (body.wantsSleeping()) {
    if (body.isStaticOrKinematicObject()) {
     body.setActivationState(ISLAND_SLEEPING);
    } else {
     if (body.getActivationState() == ACTIVE_TAG) {
      body.setActivationState(WANTS_DEACTIVATION);
     }
     if (body.getActivationState() == ISLAND_SLEEPING) {
      body.setAngularVelocity(new btVector3());
      body.setLinearVelocity(new btVector3());
     }
    }
   } else if (body.getActivationState() != DISABLE_DEACTIVATION) {
    body.setActivationState(ACTIVE_TAG);
   }
  }
 }

 void updateActions(float timeStep) {
  BT_PROFILE("updateActions");
  for (int i = 0; i < m_actions.size(); i++) {
   m_actions.get(i).updateAction(this, timeStep);
  }
 }

 void startProfiling(float timeStep) {
 }

 void internalSingleStepSimulation(float timeStep) {
  BT_PROFILE("internalSingleStepSimulation");
  if (null != m_internalPreTickCallback) {
   (m_internalPreTickCallback).callback(this, timeStep);
  }
  ///apply gravity, predict motion
  predictUnconstraintMotion(timeStep);
  btDispatcherInfo dispatchInfo = getDispatchInfo();
  dispatchInfo.m_timeStep = timeStep;
  dispatchInfo.m_stepCount = 0;
  dispatchInfo.m_debugDraw = getDebugDrawer();
  createPredictiveContacts(timeStep);
  ///perform collision detection
  performDiscreteCollisionDetection();
  calculateSimulationIslands();
  getSolverInfo().m_timeStep = timeStep;
  ///solve contact and other joint constraints
  solveConstraints(getSolverInfo());
  ///CallbackTriggers();
  ///integrate transforms
  integrateTransforms(timeStep);
  ///update vehicle simulation
  updateActions(timeStep);
  updateActivationState(timeStep);
  if (null != m_internalTickCallback) {
   (m_internalTickCallback).callback(this, timeStep);
  }
 }

 void releasePredictiveContacts() {
  BT_PROFILE("release predictive contact manifolds");
  for (int i = 0; i < m_predictiveManifolds.size(); i++) {
   btPersistentManifold manifold = m_predictiveManifolds.get(i);
   m_dispatcher1.releaseManifold(manifold);
  }
  m_predictiveManifolds.clear();
 }

 void createPredictiveContactsInternal(List<btRigidBody> bodies, int numBodies,
  float timeStep) // can be called in parallel
 {
  final btTransform predictedTrans = new btTransform();
  for (int i = 0; i < numBodies; i++) {
   btRigidBody body = bodies.get(i);
   body.setHitFraction(1.f);
   if (body.isActive() && (!body.isStaticOrKinematicObject())) {
    body.predictIntegratedTransform(timeStep, predictedTrans);
    float squareMotion = (predictedTrans.getOrigin().sub(body
     .getWorldTransformPtr().getOrigin()))
     .lengthSquared();
    if (getDispatchInfo().m_useContinuous && body.getCcdSquareMotionThreshold()
     != 0f && body
      .getCcdSquareMotionThreshold() < squareMotion) {
     BT_PROFILE("predictive convexSweepTest");
     if (body.getCollisionShape().isConvex()) {
      gNumClampedCcdMotions++;
      btClosestNotMeConvexResultCallback sweepResults = new btClosestNotMeConvexResultCallback(
       body,
       body.getWorldTransformPtr().getOrigin(), predictedTrans.getOrigin(),
       getBroadphase()
        .getOverlappingPairCache(), getDispatcher());
      btSphereShape tmpSphere = new btSphereShape(body.getCcdSweptSphereRadius());
      sweepResults.m_allowedPenetration = getDispatchInfo().m_allowedCcdPenetration;
      sweepResults.m_collisionFilterGroup = body.getBroadphaseProxy().m_collisionFilterGroup;
      sweepResults.m_collisionFilterMask = body.getBroadphaseProxy().m_collisionFilterMask;
      final btTransform modifiedPredictedTrans = predictedTrans;
      modifiedPredictedTrans.setBasis(body.getWorldTransformPtr().getBasis());
      convexSweepTest(tmpSphere, body.getWorldTransformPtr(),
       modifiedPredictedTrans, sweepResults);
      if (sweepResults.hasHit() && (sweepResults.m_closestHitFraction < 1.f)) {
       final btVector3 distVec = (predictedTrans.getOrigin().sub(body
        .getWorldTransformPtr()
        .getOrigin())).scale(sweepResults.m_closestHitFraction);
       float distance = distVec.dot(new btVector3(sweepResults.m_hitNormalWorld)
        .negate());
       btPersistentManifold manifold = m_dispatcher1.getNewManifold(body,
        sweepResults.m_hitCollisionObject);
       //synchronized (m_predictiveManifoldsMutex) {
       m_predictiveManifolds.add(manifold);
       // }
       final btVector3 worldPointB = body.getWorldTransformPtr().getOrigin()
        .add(distVec);
       final btVector3 localPointB = sweepResults.m_hitCollisionObject
        .getWorldTransform().invert().transform(new btVector3(worldPointB));
       btManifoldPoint newPoint = new btManifoldPoint(new btVector3(),
        localPointB,
        sweepResults.m_hitNormalWorld, distance);
       boolean isPredictive = true;
       int index = manifold.addManifoldPoint(newPoint, isPredictive);
       btManifoldPoint pt = manifold.getContactPoint(index);
       pt.m_combinedRestitution = 0;
       pt.m_combinedFriction = btManifoldResult.calculateCombinedFriction(body,
        sweepResults.m_hitCollisionObject);
       pt.m_positionWorldOnA.set(body.getWorldTransformPtr().getOrigin());
       pt.m_positionWorldOnB.set(worldPointB);
      }
     }
    }
   }
  }
 }

 void createPredictiveContacts(float timeStep) {
  BT_PROFILE("createPredictiveContacts");
  releasePredictiveContacts();
  if (m_nonStaticRigidBodies.size() > 0) {
   createPredictiveContactsInternal(m_nonStaticRigidBodies,
    m_nonStaticRigidBodies.size(), timeStep);
  }
 }

 void saveKinematicState(float timeStep) {
///would like to iterate over m_nonStaticRigidBodies, but unfortunately old API allows
///to switch status _after_ adding kinematic objects to the world
///fix it for Bullet 3.x release
  for (int i = 0; i < m_collisionObjects.size(); i++) {
   btCollisionObject colObj = m_collisionObjects.get(i);
   btRigidBody body = btRigidBody.upcast(colObj);
   if (body != null && body.getActivationState() != ISLAND_SLEEPING) {
    if (body.isKinematicObject()) {
     //to calculate velocities next frame
     body.saveKinematicState(timeStep);
    }
   }
  }
 }

 ///if maxSubSteps > 0, it will interpolate motion between fixedTimeStep's
 @Override
 public int stepSimulation(float timeStep, int maxSubSteps, float fixedTimeStep) {
  int do_maxSubSteps = maxSubSteps;
  float do_fixedTimeStep = fixedTimeStep;
  startProfiling(timeStep);
  BT_PROFILE("stepSimulation");
  int numSimulationSubSteps = 0;
  if (do_maxSubSteps > 0) {
   //fixed timestep with interpolation
   assert (timeStep >= 0);
   m_fixedTimeStep = do_fixedTimeStep;
   m_localTime += timeStep;
   assert (m_localTime >= 0);
   if (m_localTime >= do_fixedTimeStep) {
    numSimulationSubSteps = (int) (m_localTime / do_fixedTimeStep);
    m_localTime -= numSimulationSubSteps * do_fixedTimeStep;
    //assert(m_localTime>=0);
   }
  } else {
   //variable timestep
   assert (timeStep >= 0);
   do_fixedTimeStep = timeStep;
   m_localTime = m_latencyMotionStateInterpolation ? 0 : timeStep;
   m_fixedTimeStep = 0;
   if (btFuzzyZero(timeStep)) {
    numSimulationSubSteps = 0;
    do_maxSubSteps = 0;
   } else {
    numSimulationSubSteps = 1;
    do_maxSubSteps = 1;
   }
  }
  //process some debugging flags
  if (getDebugDrawer() != null) {
   btIDebugDraw debugDrawer = getDebugDrawer();
   btRigidBody.gDisableDeactivation
    = (debugDrawer.getDebugMode() & btIDebugDraw.DBG_NoDeactivation) != 0;
  }
  if (numSimulationSubSteps > 0) {
   //clamp the number of substeps, to prevent simulation grinding spiralling down to a halt
   int clampedSimulationSteps = (numSimulationSubSteps > do_maxSubSteps) ? do_maxSubSteps
    : numSimulationSubSteps;
   saveKinematicState(do_fixedTimeStep * clampedSimulationSteps);
   applyGravity();
   for (int i = 0; i < clampedSimulationSteps; i++) {
    internalSingleStepSimulation(do_fixedTimeStep);
    synchronizeMotionStates();
   }
  } else {
   synchronizeMotionStates();
  }
  clearForces();
  return numSimulationSubSteps;
 }

 @Override
 public void synchronizeMotionStates() {
  BT_PROFILE("synchronizeMotionStates");
  if (m_synchronizeAllMotionStates) {
   //iterate  over all collision objects
   for (int i = 0; i < m_collisionObjects.size(); i++) {
    btCollisionObject colObj = m_collisionObjects.get(i);
    btRigidBody body = btRigidBody.upcast(colObj);
    if (body != null) {
     synchronizeSingleMotionState(body);
    }
   }
  } else {
   //iterate over all active rigid bodies
   for (int i = 0; i < m_nonStaticRigidBodies.size(); i++) {
    btRigidBody body = m_nonStaticRigidBodies.get(i);
    if (body != null && body.isActive()) {
     synchronizeSingleMotionState(body);
    }
   }
  }
 }

 ///this can be useful to synchronize a single rigid body . graphics object
 void synchronizeSingleMotionState(btRigidBody body) {
  assert (body != null);
  if (body.getMotionState() != null && !body.isStaticOrKinematicObject()) {
   //we need to call the update at least once, even for sleeping objects
   //otherwise the 'graphics' transform never updates properly
   ///@todo: add 'dirty' flag
   //if (body.getActivationState() != ISLAND_SLEEPING)
   {
    final btTransform interpolatedTransform = new btTransform();
    btTransformUtil.integrateTransform(body.getInterpolationWorldTransform(),
     body.getInterpolationLinearVelocity(), body
     .getInterpolationAngularVelocity(),
     (m_latencyMotionStateInterpolation && m_fixedTimeStep != 0f) ? m_localTime
      - m_fixedTimeStep
      : m_localTime * body.getHitFraction(),
     interpolatedTransform);
    body.getMotionState().setWorldTransform(interpolatedTransform);
   }
  }
 }

 @Override
 public void addConstraint(btTypedConstraint constraint,
  boolean disableCollisionsBetweenLinkedBodies) {
  m_constraints.add(constraint);
  //Make sure the two bodies of a type constraint are different (possibly add this to the btTypedConstraint constructor?)
  assert (constraint.getRigidBodyA() != constraint.getRigidBodyB());
  if (disableCollisionsBetweenLinkedBodies) {
   constraint.getRigidBodyA().addConstraintRef(constraint);
   constraint.getRigidBodyB().addConstraintRef(constraint);
  }
 }

 @Override
 public void removeConstraint(btTypedConstraint constraint) {
  m_constraints.remove(constraint);
  constraint.getRigidBodyA().removeConstraintRef(constraint);
  constraint.getRigidBodyB().removeConstraintRef(constraint);
 }

 @Override
 public void addAction(btActionInterface action) {
  m_actions.add(action);
 }

 @Override
 public void removeAction(btActionInterface action) {
  m_actions.remove(action);
 }

 public btSimulationIslandManager getSimulationIslandManager() {
  return m_islandManager;
 }

 public btCollisionWorld getCollisionWorld() {
  return this;
 }

 @Override
 public void setGravity(final btVector3 gravity) {
  m_gravity.set(gravity);
  for (int i = 0; i < m_nonStaticRigidBodies.size(); i++) {
   btRigidBody body = m_nonStaticRigidBodies.get(i);
   if (body.isActive() && 0 == (body.getFlags() & BT_DISABLE_WORLD_GRAVITY)) {
    body.setGravity(gravity);
   }
  }
 }

 @Override
 public btVector3 getGravity() {
  return new btVector3(m_gravity);
 }

 @Override
 public void addCollisionObject(btCollisionObject collisionObject) {
  addCollisionObject(collisionObject, btBroadphaseProxy.STATIC_FILTER);
 }

 @Override
 public void addCollisionObject(btCollisionObject collisionObject,
  int collisionFilterGroup) {
  addCollisionObject(collisionObject, collisionFilterGroup,
   btBroadphaseProxy.ALL_FILTER ^ btBroadphaseProxy.STATIC_FILTER);
 }

 @Override
 public void addCollisionObject(btCollisionObject collisionObject,
  int collisionFilterGroup,
  int collisionFilterMask) {
  super.addCollisionObject(collisionObject, collisionFilterGroup,
   collisionFilterMask);
 }

 @Override
 public void addRigidBody(btRigidBody body) {
  if (!body.isStaticOrKinematicObject() && 0 == (body.getFlags()
   & BT_DISABLE_WORLD_GRAVITY)) {
   body.setGravity(m_gravity);
  }
  if (body.getCollisionShape() != null) {
   if (!body.isStaticObject()) {
    m_nonStaticRigidBodies.add(body);
   } else {
    body.setActivationState(ISLAND_SLEEPING);
   }
   boolean isDynamic = !(body.isStaticObject() || body.isKinematicObject());
   int collisionFilterGroup = isDynamic ? (btBroadphaseProxy.DEFAULT_FILTER)
    : (btBroadphaseProxy.STATIC_FILTER);
   int collisionFilterMask = isDynamic ? (btBroadphaseProxy.ALL_FILTER)
    : (btBroadphaseProxy.ALL_FILTER ^ btBroadphaseProxy.STATIC_FILTER);
   addCollisionObject(body, collisionFilterGroup, collisionFilterMask);
  }
 }

 @Override
 public void addRigidBody(btRigidBody body, int group, int mask) {
  if (!body.isStaticOrKinematicObject() && 0 == (body.getFlags()
   & BT_DISABLE_WORLD_GRAVITY)) {
   body.setGravity(m_gravity);
  }
  if (body.getCollisionShape() != null) {
   if (!body.isStaticObject()) {
    m_nonStaticRigidBodies.add(body);
   } else {
    body.setActivationState(ISLAND_SLEEPING);
   }
   addCollisionObject(body, group, mask);
  }
 }

 @Override
 public void removeRigidBody(btRigidBody body) {
  m_nonStaticRigidBodies.remove(body);
  super.removeCollisionObject(body);
 }

 ///removeCollisionObject will first check if it is a rigid body, if so call removeRigidBody otherwise call btCollisionWorld.removeCollisionObject
 @Override
 public void removeCollisionObject(btCollisionObject collisionObject) {
  btRigidBody body = btRigidBody.upcast(collisionObject);
  if (body != null) {
   removeRigidBody(body);
  } else {
   super.removeCollisionObject(collisionObject);
  }
 }

 void debugDrawConstraint(btTypedConstraint constraint) {
  boolean drawFrames = (getDebugDrawer().getDebugMode()
   & btIDebugDraw.DBG_DrawConstraints) != 0;
  boolean drawLimits = (getDebugDrawer().getDebugMode()
   & btIDebugDraw.DBG_DrawConstraintLimits) != 0;
  float dbgDrawSize = constraint.getDbgDrawSize();
  if (dbgDrawSize <= (0.f)) {
   return;
  }
  switch (constraint.getConstraintType()) {
   case POINT2POINT_CONSTRAINT_TYPE: {
    btPoint2PointConstraint p2pC = (btPoint2PointConstraint) constraint;
    final btTransform tr = new btTransform().setIdentity();
    final btVector3 pivot
     = p2pC.getRigidBodyA().getCenterOfMassTransform().transform(p2pC
      .getPivotInA());
    tr.setOrigin(pivot);
    getDebugDrawer().drawTransform(tr, dbgDrawSize);
    // that ideally should draw the same frame
    pivot.set(p2pC.getPivotInB());
    p2pC.getRigidBodyB().getCenterOfMassTransform().transform(pivot);
    tr.setOrigin(pivot);
    if (drawFrames) {
     getDebugDrawer().drawTransform(tr, dbgDrawSize);
    }
   }
   break;
   case HINGE_CONSTRAINT_TYPE: {
    btHingeConstraint pHinge = (btHingeConstraint) constraint;
    final btTransform tr = pHinge.getRigidBodyA().getCenterOfMassTransform()
     .mul(pHinge.getAFrame());
    if (drawFrames) {
     getDebugDrawer().drawTransform(tr, dbgDrawSize);
    }
    tr.set(pHinge.getRigidBodyB().getCenterOfMassTransform()).mul(pHinge
     .getBFrame());
    if (drawFrames) {
     getDebugDrawer().drawTransform(tr, dbgDrawSize);
    }
    float minAng = pHinge.getLowerLimit();
    float maxAng = pHinge.getUpperLimit();
    if (minAng == maxAng) {
     break;
    }
    boolean drawSect = true;
    if (!pHinge.hasLimit()) {
     minAng = (0.f);
     maxAng = SIMD_2_PI;
     drawSect = false;
    }
    if (drawLimits) {
     final btVector3 center = tr.getOrigin();
     final btVector3 normal = tr.getBasisColumn(2);
     final btVector3 axis = tr.getBasisColumn(0);
     getDebugDrawer().drawArc(center, normal, axis, dbgDrawSize, dbgDrawSize,
      minAng, maxAng,
      new btVector3(), drawSect);
    }
   }
   break;
   case CONETWIST_CONSTRAINT_TYPE: {
    btConeTwistConstraint pCT = (btConeTwistConstraint) constraint;
    final btTransform tr = pCT.getRigidBodyA().getCenterOfMassTransform().mul(
     pCT.getAFrame());
    if (drawFrames) {
     getDebugDrawer().drawTransform(tr, dbgDrawSize);
    }
    tr.set(pCT.getRigidBodyB().getCenterOfMassTransform().mul(pCT.getBFrame()));
    if (drawFrames) {
     getDebugDrawer().drawTransform(tr, dbgDrawSize);
    }
    if (drawLimits) {
     //  float length = float(5);
     float length = dbgDrawSize;
     final int nSegments = 8 * 4;
     float fAngleInRadians = (2.f * 3.1415926f) * (float) (nSegments - 1)
      / (float) (nSegments);
     final btVector3 pPrev = pCT.GetPointForAngle(fAngleInRadians, length);
     tr.transform(pPrev);
     for (int i = 0; i < nSegments; i++) {
      fAngleInRadians = (2.f * 3.1415926f) * (float) i / (float) (nSegments);
      final btVector3 pCur = pCT.GetPointForAngle(fAngleInRadians, length);
      tr.transform(pCur);
      getDebugDrawer().drawLine(pPrev, pCur, new btVector3());
      if (i % (nSegments / 8) == 0) {
       getDebugDrawer().drawLine(tr.getOrigin(), pCur, new btVector3());
      }
      pPrev.set(pCur);
     }
     float tws = pCT.getTwistSpan();
     float twa = pCT.getTwistAngle();
     boolean useFrameB = (pCT.getRigidBodyB().getInvMass() > (0.f));
     if (useFrameB) {
      tr
       .set(pCT.getRigidBodyB().getCenterOfMassTransform().mul(pCT.getBFrame()));
     } else {
      tr
       .set(pCT.getRigidBodyA().getCenterOfMassTransform().mul(pCT.getAFrame()));
     }
     final btVector3 pivot = tr.getOrigin();
     final btVector3 normal = tr.getBasisColumn(0);
     final btVector3 axis1 = tr.getBasisColumn(1);
     getDebugDrawer()
      .drawArc(pivot, normal, axis1, dbgDrawSize, dbgDrawSize, -twa - tws, -twa
       + tws,
       new btVector3(), true);
    }
   }
   break;
   case D6_SPRING_CONSTRAINT_TYPE:
   case D6_CONSTRAINT_TYPE: {
    btGeneric6DofConstraint p6DOF = (btGeneric6DofConstraint) constraint;
    final btTransform tr = p6DOF.getCalculatedTransformA();
    if (drawFrames) {
     getDebugDrawer().drawTransform(tr, dbgDrawSize);
    }
    tr.set(p6DOF.getCalculatedTransformB());
    if (drawFrames) {
     getDebugDrawer().drawTransform(tr, dbgDrawSize);
    }
    if (drawLimits) {
     tr.set(p6DOF.getCalculatedTransformA());
     final btVector3 center = p6DOF.getCalculatedTransformB().getOrigin();
     final btVector3 up = tr.getBasisColumn(2);
     final btVector3 axis = tr.getBasisColumn(0);
     float minTh = p6DOF.getRotationalLimitMotor(1).m_loLimit;
     float maxTh = p6DOF.getRotationalLimitMotor(1).m_hiLimit;
     float minPs = p6DOF.getRotationalLimitMotor(2).m_loLimit;
     float maxPs = p6DOF.getRotationalLimitMotor(2).m_hiLimit;
     getDebugDrawer().drawSpherePatch(center, up, axis, dbgDrawSize * (.9f),
      minTh, maxTh, minPs,
      maxPs, new btVector3());
     axis.set(tr.getBasisColumn(1));
     float ay = p6DOF.getAngle(1);
     float az = p6DOF.getAngle(2);
     float cy = btCos(ay);
     float sy = btSin(ay);
     float cz = btCos(az);
     float sz = btSin(az);
     final btVector3 ref = new btVector3();
     ref.x = cy * cz * axis.x + cy * sz * axis.y - sy * axis.z;
     ref.y = -sz * axis.x + cz * axis.y;
     ref.z = cz * sy * axis.x + sz * sy * axis.y + cy * axis.z;
     tr.set(p6DOF.getCalculatedTransformB());
     final btVector3 normal = tr.getBasisColumn(0).negate();
     float minFi = p6DOF.getRotationalLimitMotor(0).m_loLimit;
     float maxFi = p6DOF.getRotationalLimitMotor(0).m_hiLimit;
     if (minFi > maxFi) {
      getDebugDrawer().drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize,
       -SIMD_PI, SIMD_PI,
       new btVector3(), false);
     } else if (minFi < maxFi) {
      getDebugDrawer().drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize,
       minFi, maxFi,
       new btVector3(), true);
     }
     tr.set(p6DOF.getCalculatedTransformA());
     final btVector3 bbMin = new btVector3(
      p6DOF.getTranslationalLimitMotor().m_lowerLimit);
     final btVector3 bbMax = new btVector3(
      p6DOF.getTranslationalLimitMotor().m_upperLimit);
     getDebugDrawer().drawBox(bbMin, bbMax, tr, new btVector3());
    }
   }
   break;
   ///note: the code for D6_SPRING_2_CONSTRAINT_TYPE is identical to D6_CONSTRAINT_TYPE, the D6_CONSTRAINT_TYPE+D6_SPRING_CONSTRAINT_TYPE will likely become obsolete/deprecated at some stage
   case D6_SPRING_2_CONSTRAINT_TYPE: {
    {
     btGeneric6DofSpring2Constraint p6DOF = (btGeneric6DofSpring2Constraint) constraint;
     final btTransform tr = p6DOF.getCalculatedTransformA();
     if (drawFrames) {
      getDebugDrawer().drawTransform(tr, dbgDrawSize);
     }
     tr.set(p6DOF.getCalculatedTransformB());
     if (drawFrames) {
      getDebugDrawer().drawTransform(tr, dbgDrawSize);
     }
     if (drawLimits) {
      tr.set(p6DOF.getCalculatedTransformA());
      final btVector3 center = p6DOF.getCalculatedTransformB().getOrigin();
      final btVector3 up = tr.getBasisColumn(2);
      final btVector3 axis = tr.getBasisColumn(0);
      float minTh = p6DOF.getRotationalLimitMotor(1).m_loLimit;
      float maxTh = p6DOF.getRotationalLimitMotor(1).m_hiLimit;
      float minPs = p6DOF.getRotationalLimitMotor(2).m_loLimit;
      float maxPs = p6DOF.getRotationalLimitMotor(2).m_hiLimit;
      getDebugDrawer().drawSpherePatch(center, up, axis, dbgDrawSize * (.9f),
       minTh, maxTh, minPs,
       maxPs, new btVector3());
      axis.set(tr.getBasisColumn(1));
      float ay = p6DOF.getAngle(1);
      float az = p6DOF.getAngle(2);
      float cy = btCos(ay);
      float sy = btSin(ay);
      float cz = btCos(az);
      float sz = btSin(az);
      final btVector3 ref = new btVector3();
      ref.x = cy * cz * axis.x + cy * sz * axis.y - sy * axis.z;
      ref.y = -sz * axis.x + cz * axis.y;
      ref.z = cz * sy * axis.x + sz * sy * axis.y + cy * axis.z;
      tr.set(p6DOF.getCalculatedTransformB());
      final btVector3 normal = tr.getBasisColumn(0).negate();
      float minFi = p6DOF.getRotationalLimitMotor(0).m_loLimit;
      float maxFi = p6DOF.getRotationalLimitMotor(0).m_hiLimit;
      if (minFi > maxFi) {
       getDebugDrawer().drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize,
        -SIMD_PI, SIMD_PI,
        new btVector3(), false);
      } else if (minFi < maxFi) {
       getDebugDrawer().drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize,
        minFi, maxFi,
        new btVector3(), true);
      }
      tr.set(p6DOF.getCalculatedTransformA());
      final btVector3 bbMin = p6DOF.getTranslationalLimitMotor().m_lowerLimit;
      final btVector3 bbMax = p6DOF.getTranslationalLimitMotor().m_upperLimit;
      getDebugDrawer().drawBox(bbMin, bbMax, tr, new btVector3());
     }
    }
    break;
   }
   case SLIDER_CONSTRAINT_TYPE: {
    btSliderConstraint pSlider = (btSliderConstraint) constraint;
    final btTransform tr = (pSlider.getCalculatedTransformA());
    if (drawFrames) {
     getDebugDrawer().drawTransform(tr, dbgDrawSize);
    }
    tr.set(pSlider.getCalculatedTransformB());
    if (drawFrames) {
     getDebugDrawer().drawTransform(tr, dbgDrawSize);
    }
    if (drawLimits) {
     tr.set(pSlider.getUseLinearReferenceFrameA() ? pSlider
      .getCalculatedTransformA() : pSlider
       .getCalculatedTransformB());
     final btVector3 li_min = tr.transform(new btVector3(pSlider
      .getLowerLinLimit(), 0.f, 0.f));
     final btVector3 li_max = tr.transform(new btVector3(pSlider
      .getUpperLinLimit(), 0.f, 0.f));
     getDebugDrawer().drawLine(li_min, li_max, new btVector3());
     final btVector3 normal = tr.getBasisColumn(0);
     final btVector3 axis = tr.getBasisColumn(1);
     float a_min = pSlider.getLowerAngLimit();
     float a_max = pSlider.getUpperAngLimit();
     final btVector3 center = pSlider.getCalculatedTransformB().getOrigin();
     getDebugDrawer().drawArc(center, normal, axis, dbgDrawSize, dbgDrawSize,
      a_min, a_max,
      new btVector3(), true);
    }
   }
   break;
   default:
    break;
  }
  return;
 }

 @Override
 public boolean debugDrawWorld() {
  BT_PROFILE("debugDrawWorld");
  boolean activity = super.debugDrawWorld();
  boolean drawConstraints = false;
  if (getDebugDrawer() != null) {
   int mode = getDebugDrawer().getDebugMode();
   if (mode != 0 & (btIDebugDraw.DBG_DrawConstraints
    | btIDebugDraw.DBG_DrawConstraintLimits) != 0) {
    drawConstraints = true;
   }
  }
  if (drawConstraints) {
   for (int i = getNumConstraints() - 1; i >= 0; i--) {
    btTypedConstraint constraint = getConstraint(i);
    debugDrawConstraint(constraint);
   }
  }
  if (getDebugDrawer() != null && (getDebugDrawer().getDebugMode() != 0
   & (btIDebugDraw.DBG_DrawWireframe | btIDebugDraw.DBG_DrawAabb
   | btIDebugDraw.DBG_DrawNormals) != 0)) {
   int i;
   if (getDebugDrawer() != null && getDebugDrawer().getDebugMode() != 0) {
    for (i = 0; i < m_actions.size(); i++) {
     m_actions.get(i).debugDraw(m_debugDrawer);
    }
   }
  }
  if (getDebugDrawer() != null) {
   getDebugDrawer().flushLines();
  }
  return activity;
 }

 @Override
 public void setConstraintSolver(btConstraintSolver solver) {
  m_ownsConstraintSolver = false;
  m_constraintSolver = solver;
  m_solverIslandCallback.m_solver = solver;
 }

 @Override
 public btConstraintSolver getConstraintSolver() {
  return m_constraintSolver;
 }

 @Override
 public int getNumConstraints() {
  return (m_constraints.size());
 }

 @Override
 public btTypedConstraint getConstraint(int index) {
  return m_constraints.get(index);
 }

 @Override
 public int getWorldType() {
  return BT_DISCRETE_DYNAMICS_WORLD;
 }

 ///the forces on each rigidbody is accumulating together with gravity. clear this after each timestep.
 @Override
 void clearForces() {
  ///@todo: iterate over awake simulation islands!
  for (int i = 0; i < m_nonStaticRigidBodies.size(); i++) {
   btRigidBody body = m_nonStaticRigidBodies.get(i);
   //need to check if next line is ok
   //it might break backward compatibility (people applying forces on sleeping objects get never cleared and accumulate on wake-up
   body.clearForces();
  }
 }

 ///apply gravity, call this once per timestep
 void applyGravity() {
  ///@todo: iterate over awake simulation islands!
  for (int i = 0; i < m_nonStaticRigidBodies.size(); i++) {
   btRigidBody body = m_nonStaticRigidBodies.get(i);
   if (body.isActive()) {
    body.applyGravity();
   }
  }
 }

 void setNumTasks(int numTasks) {
 }

 /*
  * ///obsolete, use updateActions instead void updateVehicles(float timeStep) {
  * updateActions(timeStep); }
  *
  * ///obsolete, use addAction instead @Override public void
  * addVehicle(btActionInterface vehicle) { addAction(vehicle); } ///obsolete,
  * use removeAction instead
  *
  * @Override public void removeVehicle(btActionInterface vehicle) {
  * removeAction(vehicle); } ///obsolete, use addAction instead
  *
  * @Override public void addCharacter(btActionInterface character) {
  * addAction(character); } ///obsolete, use removeAction instead
  *
  * @Override public void removeCharacter(btActionInterface character) {
  * removeAction(character); }
  */
 void setSynchronizeAllMotionStates(boolean synchronizeAll) {
  m_synchronizeAllMotionStates = synchronizeAll;
 }

 boolean getSynchronizeAllMotionStates() {
  return m_synchronizeAllMotionStates;
 }

 void setApplySpeculativeContactRestitution(boolean enable) {
  m_applySpeculativeContactRestitution = enable;
 }

 boolean getApplySpeculativeContactRestitution() {
  return m_applySpeculativeContactRestitution;
 }

 ///Interpolate motion state between previous and current transform, instead of current and next transform.
 ///This can relieve discontinuities in the rendering, due to penetrations
 public void setLatencyMotionStateInterpolation(boolean latencyInterpolation) {
  m_latencyMotionStateInterpolation = latencyInterpolation;
 }

 public boolean getLatencyMotionStateInterpolation() {
  return m_latencyMotionStateInterpolation;
 }

};
