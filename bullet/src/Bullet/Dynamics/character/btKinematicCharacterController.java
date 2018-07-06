/*
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://bulletphysics.com
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
package Bullet.Dynamics.character;

import Bullet.Collision.Broadphase.btBroadphasePair;
import Bullet.Collision.Broadphase.btHashedOverlappingPairCache;
import Bullet.Collision.Shape.btConvexShape;
import Bullet.Collision.btCollisionObject;
import Bullet.Collision.btCollisionWorld;
import Bullet.Collision.btIDebugDraw;
import Bullet.Collision.btManifoldPoint;
import Bullet.Collision.btPersistentManifold;
import Bullet.Dynamics.CollisionObjects.btPairCachingGhostObject;
import static Bullet.Extras.btMinMax.btClamped;
import Bullet.LinearMath.btQuaternion;
import static Bullet.LinearMath.btQuaternion.shortestArcQuatNormalize2;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.SIMD_HALF_PI;
import static Bullet.LinearMath.btScalar.btAcos;
import static Bullet.LinearMath.btScalar.btCos;
import static Bullet.LinearMath.btScalar.btFabs;
import static Bullet.LinearMath.btScalar.btPow;
import static Bullet.LinearMath.btScalar.btSin;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;

/**
 *
 * @author Gregery Barton
 */
public class btKinematicCharacterController implements
 btCharacterControllerInterface {

 private static final btVector3[] S_UP_AXIS_DIRECTIONS = {new btVector3(1.0f,
  0.0f, 0.0f),
  new btVector3(0.0f, 1.0f, 0.0f), new btVector3(0.0f, 0.0f, 1.0f)};

 protected static btVector3[] getUpAxisDirections() {
  return S_UP_AXIS_DIRECTIONS;
 }

 public static btVector3
  getNormalizedVector(final btVector3 v) {
  final btVector3 n = new btVector3();
  if (v.length() > SIMD_EPSILON) {
   n.set(v).normalize();
  }
  return n;
 }

 protected float m_halfHeight;
 protected btPairCachingGhostObject m_ghostObject;
 protected btConvexShape m_convexShape;//is also in m_ghostObject, but it needs to be convex, so we store it here to avoid upcast
 protected float m_maxPenetrationDepth;
 protected float m_verticalVelocity;
 protected float m_verticalOffset;
 protected float m_fallSpeed;
 protected float m_jumpSpeed;
 protected float m_SetjumpSpeed;
 protected float m_maxJumpHeight;
 protected float m_maxSlopeRadians; // Slope angle that is set (used for returning the exact value)
 protected float m_maxSlopeCosine;  // Cosine equivalent of m_maxSlopeRadians (calculated once when set, for optimization)
 protected float m_gravity;
 protected float m_turnAngle;
 protected float m_stepHeight;
 protected float m_addedMargin;//@todo: remove this and fix the code
 ///this is the desired walk direction, set by the user
 protected final btVector3 m_walkDirection = new btVector3();
 protected final btVector3 m_normalizedDirection = new btVector3();
 protected final btVector3 m_AngVel = new btVector3();
 protected final btVector3 m_jumpPosition = new btVector3();
 //some internal variables
 protected final btVector3 m_currentPosition = new btVector3();
 protected float m_currentStepOffset;
 protected final btVector3 m_targetPosition = new btVector3();
 protected final btQuaternion m_currentOrientation = new btQuaternion();
 protected final btQuaternion m_targetOrientation = new btQuaternion();
 ///keep track of the contact manifolds
 protected final ArrayList<btPersistentManifold> m_manifoldArray = new ArrayList<>(
  1);
 protected boolean m_touchingContact;
 protected final btVector3 m_touchingNormal = new btVector3();
 protected float m_linearDamping;
 protected float m_angularDamping;
 protected boolean m_wasOnGround;
 protected boolean m_wasJumping;
 protected boolean m_useGhostObjectSweepTest;
 protected boolean m_useWalkDirection;
 protected float m_velocityTimeInterval;
 protected btVector3 m_up = new btVector3();
 protected btVector3 m_jumpAxis = new btVector3();
 protected boolean m_interpolateUp;
 protected boolean full_drop;
 protected boolean bounce_fix;

 public btKinematicCharacterController(btPairCachingGhostObject ghostObject,
  btConvexShape convexShape,
  float stepHeight) {
  this(ghostObject, convexShape, stepHeight, new btVector3(1, 0, 0));
 }

 public btKinematicCharacterController(btPairCachingGhostObject ghostObject,
  btConvexShape convexShape,
  float stepHeight, final btVector3 up) {
  m_ghostObject = ghostObject;
  m_up.set(up);
  m_jumpAxis.set(up);
  setUp(up);
  setStepHeight(stepHeight);
  m_addedMargin = 0.02f;
  //m_walkDirection.setZero();
  //m_AngVel.setZero();
  m_useGhostObjectSweepTest = true;
  m_turnAngle = (0.0f);
  m_convexShape = convexShape;
  m_useWalkDirection = true;	// use walk direction by default, legacy behavior
  m_velocityTimeInterval = 0.0f;
  m_verticalVelocity = 0.0f;
  m_verticalOffset = 0.0f;
  m_gravity = 9.8f * 3.0f; // 3G acceleration.
  m_fallSpeed = 55.0f; // Terminal velocity of a sky diver in m/s.
  m_jumpSpeed = 10.0f; // ?
  m_SetjumpSpeed = m_jumpSpeed;
  m_wasOnGround = false;
  m_wasJumping = false;
  m_interpolateUp = true;
  setMaxSlope((float) Math.toRadians(45.0));
  m_currentStepOffset = 0.0f;
  m_maxPenetrationDepth = 0.2f;
  full_drop = false;
  bounce_fix = false;
  m_linearDamping = (0.0f);
  m_angularDamping = (0.0f);
 }

 /*
  * Returns the reflection direction of a ray going 'direction' hitting a
  * surface with normal 'normal'
  *
  * from: http://www-cs-students.stanford.edu/~adityagp/final/node3.html
  */
 protected btVector3 computeReflectionDirection(final btVector3 direction,
  final btVector3 normal) {
  return new btVector3(direction).sub(new btVector3(normal).scale((2.0f)
   * direction.dot(normal)));
 }

 /*
  * Returns the portion of 'direction' that is parallel to 'normal'
  */
 protected btVector3 parallelComponent(final btVector3 direction,
  final btVector3 normal) {
  float magnitude = direction.dot(normal);
  return new btVector3(normal).scale(magnitude);
 }

 /*
  * Returns the portion of 'direction' that is perpindicular to 'normal'
  */
 protected btVector3 perpindicularComponent(final btVector3 direction,
  final btVector3 normal) {
  return new btVector3(direction).sub(parallelComponent(direction, normal));
 }

 protected boolean recoverFromPenetration(btCollisionWorld collisionWorld) {
  // Here we must refresh the overlapping paircache as the penetrating movement itself or the
  // previous recovery iteration might have used setWorldTransform and pushed us into an object
  // that is not in the previous cache contents from the last timestep, as will happen if we
  // are pushed into a new AABB overlap. Unhandled this means the next convex sweep gets stuck.
  //
  // Do this by calling the broadphase's setAabb with the moved AABB, this will update the broadphase
  // paircache and the ghostobject's internal paircache at the same time.    /BW
  final btVector3 minAabb = new btVector3();
  final btVector3 maxAabb = new btVector3();
  m_convexShape.getAabb(m_ghostObject.getWorldTransform(), minAabb, maxAabb);
  collisionWorld.getBroadphase().setAabb(m_ghostObject.getBroadphaseHandle(),
   minAabb,
   maxAabb,
   collisionWorld.getDispatcher());
  boolean penetration = false;
  collisionWorld.getDispatcher().dispatchAllCollisionPairs(m_ghostObject
   .getOverlappingPairCache(),
   collisionWorld.getDispatchInfo(), collisionWorld.getDispatcher());
  m_currentPosition.set(m_ghostObject.getWorldTransform().getOrigin());
//	float maxPen = float(0.0);
  for (btBroadphasePair collisionPair : m_ghostObject.getOverlappingPairCache()
   .getOverlappingPairArray()) {
   m_manifoldArray.clear();
   btCollisionObject obj0 = (btCollisionObject) (collisionPair.m_pProxy0.m_clientObject);
   btCollisionObject obj1 = (btCollisionObject) (collisionPair.m_pProxy1.m_clientObject);
   if ((obj0 != null && !obj0.hasContactResponse()) || (obj1 != null && !obj1
    .hasContactResponse())) {
    continue;
   }
   if (!needsCollision(obj0, obj1)) {
    continue;
   }
   if (collisionPair.m_algorithm != null) {
    collisionPair.m_algorithm.getAllContactManifolds(m_manifoldArray);
   }
   for (int j = 0; j < m_manifoldArray.size(); j++) {
    btPersistentManifold manifold = m_manifoldArray.get(j);
    float directionSign = manifold.getBody0() == m_ghostObject ? (-1.0f) : (1.0f);
    for (int p = 0; p < manifold.getNumContacts(); p++) {
     btManifoldPoint pt = manifold.getContactPoint(p);
     float dist = pt.getDistance();
     if (dist < -m_maxPenetrationDepth) {
      // TODO: cause problems on slopes, not sure if it is needed
      //if (dist < maxPen)
      //{
      //	maxPen = dist;
      //	m_touchingNormal = pt.m_normalWorldOnB * directionSign;//??
      //}
      m_currentPosition.add(new btVector3(pt.m_normalWorldOnB).scale(
       directionSign * dist * (0.2f)));
      penetration = true;
     } else {
      //printf("touching %f\n", dist);
     }
    }
    //manifold.clearManifold();
   }
  }
  final btTransform newTrans = m_ghostObject.getWorldTransform();
  newTrans.setOrigin(m_currentPosition);
  m_ghostObject.setWorldTransform(newTrans);
//	printf("m_touchingNormal = %f,%f,%f\n",m_touchingNormal[0],m_touchingNormal[1],m_touchingNormal[2]);
  return penetration;
 }

 protected void stepUp(btCollisionWorld world) {
  float stepHeight = 0.0f;
  if (m_verticalVelocity < 0.0) {
   stepHeight = m_stepHeight;
  }
  // phase 1: up
  final btTransform start = new btTransform();
  final btTransform end = new btTransform();
  start.setIdentity();
  end.setIdentity();

  /*
   * FIXME: Handle penetration properly
   */
  start.setOrigin(m_currentPosition);
  m_targetPosition.set(m_currentPosition)
   .add(new btVector3(m_up).scale(stepHeight))
   .add(new btVector3(m_jumpAxis).scale(
    m_verticalOffset > 0.f ? m_verticalOffset : 0.f));
  m_currentPosition.set(m_targetPosition);
  end.setOrigin(m_targetPosition);
  start.set3x3(m_currentOrientation);
  end.set3x3(m_targetOrientation);
  btKinematicClosestNotMeConvexResultCallback callback
   = new btKinematicClosestNotMeConvexResultCallback(m_ghostObject,
    new btVector3(m_up).negate(),
    m_maxSlopeCosine);
  callback.m_collisionFilterGroup = getGhostObject().getBroadphaseHandle().m_collisionFilterGroup;
  callback.m_collisionFilterMask = getGhostObject().getBroadphaseHandle().m_collisionFilterMask;
  if (m_useGhostObjectSweepTest) {
   m_ghostObject.convexSweepTest(m_convexShape, start, end, callback,
    world.getDispatchInfo().m_allowedCcdPenetration);
  } else {
   world.convexSweepTest(m_convexShape, start, end, callback,
    world.getDispatchInfo().m_allowedCcdPenetration);
  }
  if (callback.hasHit() && m_ghostObject.hasContactResponse() && needsCollision(
   m_ghostObject,
   callback.m_hitCollisionObject)) {
   // Only modify the position if the hit was a slope and not a wall or ceiling.
   if (callback.m_hitNormalWorld.dot(m_up) > 0.0) {
    // we moved up only a fraction of the step height
    m_currentStepOffset = stepHeight * callback.m_closestHitFraction;
    if (m_interpolateUp) {
     m_currentPosition.setInterpolate3(m_currentPosition, m_targetPosition,
      callback.m_closestHitFraction);
    } else {
     m_currentPosition.set(m_targetPosition);
    }
   }
   final btTransform xform = m_ghostObject.getWorldTransform();
   xform.setOrigin(m_currentPosition);
   m_ghostObject.setWorldTransform(xform);
   // fix penetration if we hit a ceiling for example
   int numPenetrationLoops = 0;
   m_touchingContact = false;
   while (recoverFromPenetration(world)) {
    numPenetrationLoops++;
    m_touchingContact = true;
    if (numPenetrationLoops > 4) {
     //printf("character could not recover from penetration = %d\n", numPenetrationLoops);
     break;
    }
   }
   m_targetPosition.set(m_ghostObject.getWorldTransform().getOrigin());
   m_currentPosition.set(m_targetPosition);
   if (m_verticalOffset > 0) {
    m_verticalOffset = 0.0f;
    m_verticalVelocity = 0.0f;
    m_currentStepOffset = m_stepHeight;
   }
  } else {
   m_currentStepOffset = stepHeight;
   m_currentPosition.set(m_targetPosition);
  }
 }

 protected void updateTargetPositionBasedOnCollision(final btVector3 hit_normal) {
  updateTargetPositionBasedOnCollision(hit_normal, 0);
 }

 protected void updateTargetPositionBasedOnCollision(final btVector3 hit_normal,
  float tangentMag) {
  updateTargetPositionBasedOnCollision(hit_normal, tangentMag, 1.0f);
 }

 protected void updateTargetPositionBasedOnCollision(final btVector3 hitNormal,
  float tangentMag,
  float normalMag) {
  final btVector3 movementDirection = new btVector3(m_targetPosition).sub(
   m_currentPosition);
  float movementLength = movementDirection.length();
  if (movementLength > SIMD_EPSILON) {
   movementDirection.normalize();
   final btVector3 reflectDir = computeReflectionDirection(movementDirection,
    hitNormal);
   reflectDir.normalize();
   final btVector3 parallelDir = new btVector3();
   final btVector3 perpindicularDir = new btVector3();
   parallelDir.set(parallelComponent(reflectDir, hitNormal));
   perpindicularDir.set(perpindicularComponent(reflectDir, hitNormal));
   m_targetPosition.set(m_currentPosition);
   if (normalMag != 0.0) {
    final btVector3 perpComponent = new btVector3(perpindicularDir)
     .scale(normalMag * movementLength);
//			printf("perpComponent=%f,%f,%f\n",perpComponent[0],perpComponent[1],perpComponent[2]);
    m_targetPosition.add(perpComponent);
   }
  } else {
//		printf("movementLength don't normalize a zero vector\n");
  }
 }

 protected void stepForwardAndStrafe(btCollisionWorld collisionWorld,
  final btVector3 walkMove) {
  // printf("m_normalizedDirection=%f,%f,%f\n",
  // 	m_normalizedDirection[0],m_normalizedDirection[1],m_normalizedDirection[2]);
  // phase 2: forward and strafe
  final btTransform start = new btTransform();
  final btTransform end = new btTransform();
  m_targetPosition.set(m_currentPosition).add(walkMove);
  start.setIdentity();
  end.setIdentity();
  float fraction = 1.0f;
  float distance2;// = m_currentPosition.distanceSquared(m_targetPosition);
//	printf("distance2=%f\n",distance2);
  int maxIter = 10;
  while (fraction > (0.01f) && maxIter-- > 0) {
   start.setOrigin(m_currentPosition);
   end.setOrigin(m_targetPosition);
   final btVector3 sweepDirNegative = new btVector3(m_currentPosition).sub(
    m_targetPosition);
   start.set3x3(m_currentOrientation);
   end.set3x3(m_targetOrientation);
   btKinematicClosestNotMeConvexResultCallback callback
    = new btKinematicClosestNotMeConvexResultCallback(m_ghostObject,
     sweepDirNegative, (0.0f));
   callback.m_collisionFilterGroup = getGhostObject().getBroadphaseHandle().m_collisionFilterGroup;
   callback.m_collisionFilterMask = getGhostObject().getBroadphaseHandle().m_collisionFilterMask;
   float margin = m_convexShape.getMargin();
   m_convexShape.setMargin(margin + m_addedMargin);
   if (!(start.equals(end))) {
    if (m_useGhostObjectSweepTest) {
     m_ghostObject.convexSweepTest(m_convexShape, start, end, callback,
      collisionWorld
       .getDispatchInfo().m_allowedCcdPenetration);
    } else {
     collisionWorld.convexSweepTest(m_convexShape, start, end, callback,
      collisionWorld
       .getDispatchInfo().m_allowedCcdPenetration);
    }
   }
   m_convexShape.setMargin(margin);
   fraction -= callback.m_closestHitFraction;
   if (callback.hasHit() && m_ghostObject.hasContactResponse()
    && needsCollision(m_ghostObject,
     callback.m_hitCollisionObject)) {
    // we moved only a fraction
    //float hitDistance;
    //hitDistance = (callback.m_hitPointWorld - m_currentPosition).length();
//			m_currentPosition.setInterpolate3 (m_currentPosition, m_targetPosition, callback.m_closestHitFraction);
    updateTargetPositionBasedOnCollision(callback.m_hitNormalWorld);
    final btVector3 currentDir = new btVector3(m_targetPosition).sub(
     m_currentPosition);
    distance2 = currentDir.lengthSquared();
    if (distance2 > SIMD_EPSILON) {
     currentDir.normalize();
     /*
      * See Quake2: "If velocity is against original velocity, stop ead to avoid
      * tiny oscilations in sloping corners."
      */
     if (currentDir.dot(m_normalizedDirection) <= (0.0f)) {
      break;
     }
    } else {
//				printf("currentDir: don't normalize a zero vector\n");
     break;
    }
   } else {
    m_currentPosition.set(m_targetPosition);
   }
  }
 }

 protected void stepDown(btCollisionWorld collisionWorld, float dt) {
  final btTransform start = new btTransform();
  final btTransform end = new btTransform();
  final btTransform end_double = new btTransform();
  boolean runonce = false;
  // phase 3: down
  /*
   * float additionalDownStep = (m_wasOnGround && !onGround()) ? m_stepHeight :
   * 0.0; btVector3 step_drop = m_up * (m_currentStepOffset +
   * additionalDownStep); float downVelocity = (additionalDownStep == 0.0 &&
   * m_verticalVelocity<0.0?-m_verticalVelocity:0.0) * dt; btVector3
   * gravity_drop = m_up * downVelocity; m_targetPosition -= (step_drop +
   * gravity_drop);
   */
  final btVector3 orig_position = new btVector3(m_targetPosition);
  float downVelocity = (m_verticalVelocity < 0.f ? -m_verticalVelocity : 0.f)
   * dt;
  if (m_verticalVelocity > 0.0) {
   return;
  }
  if (downVelocity > 0.0 && downVelocity > m_fallSpeed && (m_wasOnGround
   || !m_wasJumping)) {
   downVelocity = m_fallSpeed;
  }
  final btVector3 step_drop = new btVector3(m_up).scale(m_currentStepOffset
   + downVelocity);
  m_targetPosition.sub(step_drop);
  btKinematicClosestNotMeConvexResultCallback callback
   = new btKinematicClosestNotMeConvexResultCallback(m_ghostObject, m_up,
    m_maxSlopeCosine);
  callback.m_collisionFilterGroup = getGhostObject().getBroadphaseHandle().m_collisionFilterGroup;
  callback.m_collisionFilterMask = getGhostObject().getBroadphaseHandle().m_collisionFilterMask;
  btKinematicClosestNotMeConvexResultCallback callback2
   = new btKinematicClosestNotMeConvexResultCallback(m_ghostObject, m_up,
    m_maxSlopeCosine);
  callback2.m_collisionFilterGroup = getGhostObject().getBroadphaseHandle().m_collisionFilterGroup;
  callback2.m_collisionFilterMask = getGhostObject().getBroadphaseHandle().m_collisionFilterMask;
  while (true) {
   start.setIdentity();
   end.setIdentity();
   end_double.setIdentity();
   start.setOrigin(m_currentPosition);
   end.setOrigin(m_targetPosition);
   start.set3x3(m_currentOrientation);
   end.set3x3(m_targetOrientation);
   //set double test for 2x the step drop, to check for a large drop vs small drop
   end_double.setOrigin(new btVector3(m_targetPosition).sub(step_drop));
   if (m_useGhostObjectSweepTest) {
    m_ghostObject.convexSweepTest(m_convexShape, start, end, callback,
     collisionWorld
      .getDispatchInfo().m_allowedCcdPenetration);
    if (!callback.hasHit() && m_ghostObject.hasContactResponse()) {
     //test a double fall height, to see if the character should interpolate it's fall (full) or not (partial)
     m_ghostObject.convexSweepTest(m_convexShape, start, end_double, callback2,
      collisionWorld
       .getDispatchInfo().m_allowedCcdPenetration);
    }
   } else {
    collisionWorld.convexSweepTest(m_convexShape, start, end, callback,
     collisionWorld
      .getDispatchInfo().m_allowedCcdPenetration);
    if (!callback.hasHit() && m_ghostObject.hasContactResponse()) {
     //test a double fall height, to see if the character should interpolate it's fall (large) or not (small)
     collisionWorld.convexSweepTest(m_convexShape, start, end_double, callback2,
      collisionWorld
       .getDispatchInfo().m_allowedCcdPenetration);
    }
   }
   float downVelocity2 = (m_verticalVelocity < 0.f ? -m_verticalVelocity : 0.f)
    * dt;
   boolean has_hit;
   if (bounce_fix == true) {
    has_hit = (callback.hasHit() || callback2.hasHit()) && m_ghostObject
     .hasContactResponse() && needsCollision(m_ghostObject,
      callback.m_hitCollisionObject);
   } else {
    has_hit = callback2.hasHit() && m_ghostObject.hasContactResponse()
     && needsCollision(
      m_ghostObject, callback2.m_hitCollisionObject);
   }
   float stepHeight = 0.0f;
   if (m_verticalVelocity < 0.0) {
    stepHeight = m_stepHeight;
   }
   if (downVelocity2 > 0.0 && downVelocity2 < stepHeight && has_hit && !runonce
    && (m_wasOnGround || !m_wasJumping)) {
    //redo the velocity calculation when falling a small amount, for fast stairs motion
    //for larger falls, use the smoother/slower interpolated movement by not touching the target position
    m_targetPosition.set(orig_position);
    downVelocity = stepHeight;
    step_drop.set(m_up).scale(m_currentStepOffset + downVelocity);
    m_targetPosition.sub(step_drop);
    runonce = true;
    continue; //re-run previous tests
   }
   break;
  }
  if ((m_ghostObject.hasContactResponse() && (callback.hasHit()
   && needsCollision(m_ghostObject,
    callback.m_hitCollisionObject))) || runonce) {
   // we dropped a fraction of the height . hit floor
   float fraction = (m_currentPosition.getY() - callback.m_hitPointWorld.getY())
    / 2;
   //printf("hitpoint: %g - pos %g\n", callback.m_hitPointWorld.getY(), m_currentPosition.getY());
   if (bounce_fix) {
    if (full_drop) {
     m_currentPosition.setInterpolate3(m_currentPosition, m_targetPosition,
      callback.m_closestHitFraction);
    } else //due to errors in the closestHitFraction variable when used with large polygons, calculate the hit fraction manually
    {
     m_currentPosition.setInterpolate3(m_currentPosition, m_targetPosition,
      fraction);
    }
   } else {
    m_currentPosition.setInterpolate3(m_currentPosition, m_targetPosition,
     callback.m_closestHitFraction);
   }
   full_drop = false;
   m_verticalVelocity = 0.0f;
   m_verticalOffset = 0.0f;
   m_wasJumping = false;
  } else {
   // we dropped the full height
   full_drop = true;
   if (bounce_fix) {
    downVelocity = (m_verticalVelocity < 0.f ? -m_verticalVelocity : 0.f) * dt;
    if (downVelocity > m_fallSpeed && (m_wasOnGround || !m_wasJumping)) {
     m_targetPosition.add(step_drop); //undo previous target change
     downVelocity = m_fallSpeed;
     step_drop.set(m_up).scale(m_currentStepOffset + downVelocity);
     m_targetPosition.sub(step_drop);
    }
   }
   //printf("full drop - %g, %g\n", m_currentPosition.getY(), m_targetPosition.getY());
   m_currentPosition.set(m_targetPosition);
  }
 }

 protected boolean needsCollision(btCollisionObject body0,
  btCollisionObject body1) {
  boolean collides = (body0.getBroadphaseHandle().m_collisionFilterGroup & body1
   .getBroadphaseHandle().m_collisionFilterMask) != 0;
  collides = collides && ((body1.getBroadphaseHandle().m_collisionFilterGroup
   & body0
    .getBroadphaseHandle().m_collisionFilterMask) != 0);
  return collides;
 }

 protected void setUpVector(final btVector3 up) {
  if (m_up.equals(up)) {
   return;
  }
  final btVector3 u = new btVector3(m_up);
  if (up.lengthSquared() > 0) {
   m_up.set(up).normalize();
  } else {
   m_up.setZero();
  }
  if (null == m_ghostObject) {
   return;
  }
  final btQuaternion rot = getRotation(m_up, u);
  //set orientation with new up
  final btTransform xform = m_ghostObject.getWorldTransform();
  final btQuaternion orn = new btQuaternion(rot).conjugate().mul(xform
   .getRotation());
  xform.set3x3(orn);
  m_ghostObject.setWorldTransform(xform);
 }

 protected btQuaternion getRotation(final btVector3 v0, final btVector3 v1) {
  if (v0.lengthSquared() == 0.0f || v1.lengthSquared() == 0.0f) {
   final btQuaternion q = new btQuaternion();
   return q;
  }
  return shortestArcQuatNormalize2(v0, v1);
 }

 ///btActionInterface interface
 @Override
 public void updateAction(btCollisionWorld collisionWorld, float deltaTime) {
  preStep(collisionWorld);
  playerStep(collisionWorld, deltaTime);
 }

 ///btActionInterface interface
 @Override
 public void debugDraw(btIDebugDraw debugDrawer) {
 }

 public final void setUp(final btVector3 up) {
  if (up.lengthSquared() > 0 && m_gravity > 0.0f) {
   setGravity(new btVector3(up).normalize().scale(-m_gravity));
   return;
  }
  setUpVector(up);
 }

 public btVector3 getUp() {
  return new btVector3(m_up);
 }
// static helper method

 /// This should probably be called setPositionIncrementPerSimulatorStep.
 /// This is neither a direction nor a velocity, but the amount to
 ///	increment the position each simulation iteration, regardless
 ///	of dt.
 /// This call will reset any velocity set by setVelocityForTimeInterval().
 @Override
 public void setWalkDirection(final btVector3 walkDirection) {
  m_useWalkDirection = true;
  m_walkDirection.set(walkDirection);
  m_normalizedDirection.set(getNormalizedVector(m_walkDirection));
 }

 /// Caller provides a velocity with which the character should move for
 ///	the given time period.  After the time period, velocity is reset
 ///	to zero.
 /// This call will reset any walk direction set by setWalkDirection().
 /// Negative time intervals will result in no motion.
 @Override
 public void setVelocityForTimeInterval(final btVector3 velocity,
  float timeInterval) {
//	printf("setVelocity!\n");
//	printf("  interval: %f\n", timeInterval);
//	printf("  velocity: (%f, %f, %f)\n",
//		 velocity.x(), velocity.y(), velocity.z());
  m_useWalkDirection = false;
  m_walkDirection.set(velocity);
  m_normalizedDirection.set(getNormalizedVector(m_walkDirection));
  m_velocityTimeInterval += timeInterval;
 }

 public void setAngularVelocity(final btVector3 velocity) {
  m_AngVel.set(velocity);
 }

 public btVector3 getAngularVelocity() {
  return new btVector3(m_AngVel);
 }

 public void setLinearVelocity(final btVector3 velocity) {
  m_walkDirection.set(velocity);
  // HACK: if we are moving in the direction of the up, treat it as a jump :(
  if (m_walkDirection.lengthSquared() > 0) {
   final btVector3 w = new btVector3(velocity).normalize();
   float c = w.dot(m_up);
   if (c != 0) {
    //there is a component in walkdirection for vertical velocity
    final btVector3 upComponent = new btVector3(m_up).scale(btSin(SIMD_HALF_PI
     - btAcos(c)) * m_walkDirection.length());
    m_walkDirection.sub(upComponent);
    m_verticalVelocity = (c < 0.0f ? -1 : 1) * upComponent.length();
    if (c > 0.0f) {
     m_wasJumping = true;
     m_jumpPosition.set(m_ghostObject.getWorldTransform().getOrigin());
    }
   }
  } else {
   m_verticalVelocity = 0.0f;
  }
 }

 public btVector3 getLinearVelocity() {
  return new btVector3(m_up).scale(m_verticalVelocity).add(m_walkDirection);
 }

 public void setLinearDamping(float d) {
  m_linearDamping = btClamped(d, 0f, 1f);
 }

 public float getLinearDamping() {
  return m_linearDamping;
 }

 public void setAngularDamping(float d) {
  m_angularDamping = btClamped(d, 0f, 1f);
 }

 public float getAngularDamping() {
  return m_angularDamping;
 }

 @Override
 public void reset(btCollisionWorld collisionWorld) {
  m_verticalVelocity = 0.0f;
  m_verticalOffset = 0.0f;
  m_wasOnGround = false;
  m_wasJumping = false;
  m_walkDirection.setZero();
  m_velocityTimeInterval = 0.0f;
  //clear pair cache
  btHashedOverlappingPairCache cache = m_ghostObject.getOverlappingPairCache();
  Collection<btBroadphasePair> overlapping_pair_array = cache
   .getOverlappingPairArrayPtr();
  while (overlapping_pair_array.size() > 0) {
   Iterator<btBroadphasePair> i = overlapping_pair_array.iterator();
   btBroadphasePair pair = i.next();
   cache.removeOverlappingPair(pair.m_pProxy0, pair.m_pProxy1, collisionWorld
    .getDispatcher());
  }
 }

 @Override
 public void warp(final btVector3 origin) {
  final btTransform xform = new btTransform();
  xform.setIdentity();
  xform.setOrigin(origin);
  m_ghostObject.setWorldTransform(xform);
 }

 @Override
 public void preStep(btCollisionWorld collisionWorld) {
  m_currentPosition.set(m_ghostObject.getWorldTransform().getOrigin());
  m_targetPosition.set(m_currentPosition);
  m_currentOrientation.set(m_ghostObject.getWorldTransform().getRotation());
  m_targetOrientation.set(m_currentOrientation);
//	printf("m_targetPosition=%f,%f,%f\n",m_targetPosition[0],m_targetPosition[1],m_targetPosition[2]);
 }

 @Override
 public void playerStep(btCollisionWorld collisionWorld, float dt) {
//	printf("playerStep(): ");
//	printf("  dt = %f", dt);
  if (m_AngVel.lengthSquared() > 0.0f) {
   m_AngVel.scale(btPow((1f) - m_angularDamping, dt));
  }
  // integrate for angular velocity
  if (m_AngVel.lengthSquared() > 0.0f) {
   final btTransform xform = m_ghostObject.getWorldTransform();
   final btQuaternion rot = new btQuaternion(new btVector3(m_AngVel).normalize(),
    m_AngVel.length() * dt);
   final btQuaternion orn = new btQuaternion(rot).mul(xform.getRotation());
   xform.set3x3(orn);
   m_ghostObject.setWorldTransform(xform);
   m_currentPosition.set(m_ghostObject.getWorldTransform().getOrigin());
   m_targetPosition.set(m_currentPosition);
   m_currentOrientation.set(m_ghostObject.getWorldTransform().getRotation());
   m_targetOrientation.set(m_currentOrientation);
  }
  // quick check...
  if (!m_useWalkDirection && (m_velocityTimeInterval <= 0.0)) {
//		printf("\n");
   return;		// no motion
  }
  m_wasOnGround = onGround();
  //btVector3 lvel = m_walkDirection;
  //float c = 0.0f;
  if (m_walkDirection.lengthSquared() > 0) {
   // apply damping
   m_walkDirection.scale(btPow((1f) - m_linearDamping, dt));
  }
  m_verticalVelocity *= btPow((1f) - m_linearDamping, dt);
  // Update fall velocity.
  m_verticalVelocity -= m_gravity * dt;
  if (m_verticalVelocity > 0.0 && m_verticalVelocity > m_jumpSpeed) {
   m_verticalVelocity = m_jumpSpeed;
  }
  if (m_verticalVelocity < 0.0 && btFabs(m_verticalVelocity) > btFabs(
   m_fallSpeed)) {
   m_verticalVelocity = -btFabs(m_fallSpeed);
  }
  m_verticalOffset = m_verticalVelocity * dt;
  final btTransform xform = m_ghostObject.getWorldTransform();
//	printf("walkDirection(%f,%f,%f)\n",walkDirection[0],walkDirection[1],walkDirection[2]);
//	printf("walkSpeed=%f\n",walkSpeed);
  stepUp(collisionWorld);
  //todo: Experimenting with behavior of controller when it hits a ceiling..
  //bool hitUp = stepUp (collisionWorld);	
  //if (hitUp)
  //{
  //	m_verticalVelocity -= m_gravity * dt;
  //	if (m_verticalVelocity > 0.0 && m_verticalVelocity > m_jumpSpeed)
  //	{
  //		m_verticalVelocity = m_jumpSpeed;
  //	}
  //	if (m_verticalVelocity < 0.0 && btFabs(m_verticalVelocity) > btFabs(m_fallSpeed))
  //	{
  //		m_verticalVelocity = -btFabs(m_fallSpeed);
  //	}
  //	m_verticalOffset = m_verticalVelocity * dt;
  //	xform = m_ghostObject.getWorldTransform();
  //}
  if (m_useWalkDirection) {
   stepForwardAndStrafe(collisionWorld, m_walkDirection);
  } else {
   //printf("  time: %f", m_velocityTimeInterval);
   // still have some time left for moving!
   float dtMoving
    = (dt < m_velocityTimeInterval) ? dt : m_velocityTimeInterval;
   m_velocityTimeInterval -= dt;
   // how far will we move while we are moving?
   final btVector3 move = new btVector3(m_walkDirection).scale(dtMoving);
   //printf("  dtMoving: %f", dtMoving);
   // okay, step
   stepForwardAndStrafe(collisionWorld, move);
  }
  stepDown(collisionWorld, dt);
  //todo: Experimenting with max jump height
  //if (m_wasJumping)
  //{
  //	float ds = m_currentPosition[m_upAxis] - m_jumpPosition[m_upAxis];
  //	if (ds > m_maxJumpHeight)
  //	{
  //		// substract the overshoot
  //		m_currentPosition[m_upAxis] -= ds - m_maxJumpHeight;
  //		// max height was reached, so potential energy is at max 
  //		// and kinematic energy is 0, thus velocity is 0.
  //		if (m_verticalVelocity > 0.0)
  //			m_verticalVelocity = 0.0;
  //	}
  //}
  // printf("\n");
  xform.setOrigin(m_currentPosition);
  m_ghostObject.setWorldTransform(xform);
  int numPenetrationLoops = 0;
  m_touchingContact = false;
  while (recoverFromPenetration(collisionWorld)) {
   numPenetrationLoops++;
   m_touchingContact = true;
   if (numPenetrationLoops > 4) {
    //printf("character could not recover from penetration = %d\n", numPenetrationLoops);
    break;
   }
  }
 }

 public final void setStepHeight(float h) {
  m_stepHeight = h;
 }

 public float getStepHeight() {
  return m_stepHeight;
 }

 public void setFallSpeed(float fallSpeed) {
  m_fallSpeed = fallSpeed;
 }

 public float getFallSpeed() {
  return m_fallSpeed;
 }

 public void setJumpSpeed(float jumpSpeed) {
  m_jumpSpeed = jumpSpeed;
  m_SetjumpSpeed = m_jumpSpeed;
 }

 public float getJumpSpeed() {
  return m_jumpSpeed;
 }

 public void setMaxJumpHeight(float maxJumpHeight) {
  m_maxJumpHeight = maxJumpHeight;
 }

 @Override
 public boolean canJump() {
  return onGround();
 }

 public final void jump() {
  jump(new btVector3());
 }

 @Override
 public void jump(final btVector3 v) {
  m_jumpSpeed = v.lengthSquared() == 0 ? m_SetjumpSpeed : v.length();
  m_verticalVelocity = m_jumpSpeed;
  m_wasJumping = true;
  m_jumpAxis = v.lengthSquared() == 0 ? m_up : new btVector3(v).normalize();
  m_jumpPosition.set(m_ghostObject.getWorldTransform().getOrigin());
 }

 public void applyImpulse(final btVector3 v) {
  jump(v);
 }

 public void setGravity(final btVector3 gravity) {
  if (gravity.lengthSquared() > 0) {
   setUpVector(new btVector3(gravity).negate());
  }
  m_gravity = gravity.length();
 }

 public btVector3 getGravity() {
  return new btVector3(m_up).scale(-m_gravity);
 }

 /// The max slope determines the maximum angle that the controller can walk up.
 /// The slope angle is measured in radians.
 public final void setMaxSlope(float slopeRadians) {
  m_maxSlopeRadians = slopeRadians;
  m_maxSlopeCosine = btCos(slopeRadians);
 }

 public float getMaxSlope() {
  return m_maxSlopeRadians;
 }

 public void setMaxPenetrationDepth(float d) {
  m_maxPenetrationDepth = d;
 }

 public float getMaxPenetrationDepth() {
  return m_maxPenetrationDepth;
 }

 public btPairCachingGhostObject getGhostObject() {
  return m_ghostObject;
 }

 public void setUseGhostSweepTest(boolean useGhostObjectSweepTest) {
  m_useGhostObjectSweepTest = useGhostObjectSweepTest;
 }

 @Override
 public boolean onGround() {
  return (btFabs(m_verticalVelocity) < SIMD_EPSILON)
   && (btFabs(m_verticalOffset) < SIMD_EPSILON);
 }

 @Override
 public void setUpInterpolate(boolean value) {
  m_interpolateUp = value;
 }

}
