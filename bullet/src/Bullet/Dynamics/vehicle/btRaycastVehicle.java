/*
 * Copyright (c) 2005 Erwin Coumans http://bulletphysics.org
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability 
 * of this software for any purpose.  
 * It is provided "as is" without express or implied warranty.
 */
package Bullet.Dynamics.vehicle;

import Bullet.Collision.btCollisionWorld;
import Bullet.Collision.btIDebugDraw;
import Bullet.Dynamics.CollisionObjects.btRigidBody;
import static Bullet.Dynamics.Constraint.btContactConstraint.resolveSingleBilateral;
import Bullet.Dynamics.btActionInterface;
import static Bullet.Dynamics.btActionInterface.getFixedBody;
import Bullet.LinearMath.btMatrix3x3;
import Bullet.LinearMath.btQuaternion;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import java.util.ArrayList;
import org.apache.commons.collections.primitives.ArrayFloatList;

/**
 *
 * @author Gregery Barton
 */
public class btRaycastVehicle implements btActionInterface, Serializable {

 private static final long serialVersionUID = 1L;
 protected final ArrayList<btVector3> m_forwardWS = new ArrayList<>(0);
 protected final ArrayList<btVector3> m_axle = new ArrayList<>(0);
 protected final ArrayFloatList m_forwardImpulse = new ArrayFloatList(0);
 protected final ArrayFloatList m_sideImpulse = new ArrayFloatList(0);
 ///backwards compatibility
 protected int m_userConstraintType;
 protected int m_userConstraintId;
 private final btVehicleRaycaster m_vehicleRaycaster;
 //private float m_pitchControl;
// private float m_steeringValue;
 private float m_currentVehicleSpeedKmHour;
 private final btRigidBody m_chassisBody;
 private int m_indexRightAxis;
 private int m_indexUpAxis;
 private int m_indexForwardAxis;

 private void defaultInit(btVehicleTuning tuning) {
  m_currentVehicleSpeedKmHour = (0.f);
  //m_steeringValue = (0.f);
 }

 //constructor to create a car from an existing rigidbody
 public btRaycastVehicle(btVehicleTuning tuning, btRigidBody chassis, btVehicleRaycaster raycaster) {
  m_vehicleRaycaster = raycaster;
  //m_pitchControl = ((0.f));
  m_chassisBody = chassis;
  m_indexRightAxis = 0;
  m_indexUpAxis = 2;
  m_indexForwardAxis = 1;
  defaultInit(tuning);
 }

 ///btActionInterface interface
 @Override
 public void updateAction(btCollisionWorld collisionWorld, float step) {
  updateVehicle(step);
 }

 ///btActionInterface interface
 @Override
 public void debugDraw(btIDebugDraw debugDrawer) {
  for (int v = 0; v < getNumWheels(); v++) {
   final btVector3 wheelColor = new btVector3(0, 1, 1);
   if (getWheelInfo(v).m_raycastInfo.m_isInContact) {
    wheelColor.set(0, 0, 1);
   } else {
    wheelColor.set(1, 0, 1);
   }
   final btVector3 wheelPosWS = getWheelInfo(v).m_worldTransform.getOrigin();
   final btVector3 axle = new btVector3(
    getWheelInfo(v).m_worldTransform.getElement(0, getRightAxis()),
    getWheelInfo(v).m_worldTransform.getElement(1, getRightAxis()),
    getWheelInfo(v).m_worldTransform.getElement(2, getRightAxis()));
   //debug wheels (cylinders)
   debugDrawer.drawLine(wheelPosWS, new btVector3(wheelPosWS).add(axle), wheelColor);
   debugDrawer.drawLine(wheelPosWS, getWheelInfo(v).m_raycastInfo.m_contactPointWS, wheelColor);
  }
 }

 public btTransform getChassisWorldTransform() {
  return getRigidBody().getCenterOfMassTransform();
 }

 public float rayCast(btWheelInfo wheel) {
  updateWheelTransformsWS(wheel, false);
  float depth = -1;
  float raylen = wheel.getSuspensionRestLength() + wheel.m_wheelsRadius;
  final btVector3 rayvector = new btVector3(wheel.m_raycastInfo.m_wheelDirectionWS).scale(raylen);
  final btVector3 source = wheel.m_raycastInfo.m_hardPointWS;
  wheel.m_raycastInfo.m_contactPointWS.set(source).add(rayvector);
  final btVector3 target = wheel.m_raycastInfo.m_contactPointWS;
  float param;
  btVehicleRaycaster.btVehicleRaycasterResult rayResults =
   new btVehicleRaycaster.btVehicleRaycasterResult();
  assert (m_vehicleRaycaster != null);
  Object object = m_vehicleRaycaster.castRay(source, target, rayResults);
  wheel.m_raycastInfo.m_groundObject = null;
  if (object != null) {
   param = rayResults.m_distFraction;
   depth = raylen * rayResults.m_distFraction;
   wheel.m_raycastInfo.m_contactNormalWS.set(rayResults.m_hitNormalInWorld);
   wheel.m_raycastInfo.m_isInContact = true;
   wheel.m_raycastInfo.m_groundObject = getFixedBody();///@todo for driving on dynamic/movable objects!;
   //wheel.m_raycastInfo.m_groundObject = object;
   float hitDistance = param * raylen;
   wheel.m_raycastInfo.m_suspensionLength = hitDistance - wheel.m_wheelsRadius;
   //clamp on max suspension travel
   float minSuspensionLength = wheel.getSuspensionRestLength() - wheel.m_maxSuspensionTravelCm *
    (0.01f);
   float maxSuspensionLength = wheel.getSuspensionRestLength() + wheel.m_maxSuspensionTravelCm *
    (0.01f);
   if (wheel.m_raycastInfo.m_suspensionLength < minSuspensionLength) {
    wheel.m_raycastInfo.m_suspensionLength = minSuspensionLength;
   }
   if (wheel.m_raycastInfo.m_suspensionLength > maxSuspensionLength) {
    wheel.m_raycastInfo.m_suspensionLength = maxSuspensionLength;
   }
   wheel.m_raycastInfo.m_contactPointWS.set(rayResults.m_hitPointInWorld);
   float denominator = wheel.m_raycastInfo.m_contactNormalWS.dot(
    wheel.m_raycastInfo.m_wheelDirectionWS);
   final btVector3 chassis_velocity_at_contactPoint = new btVector3();
   final btVector3 relpos = new btVector3(wheel.m_raycastInfo.m_contactPointWS).sub(getRigidBody()
    .getCenterOfMassPosition());
   chassis_velocity_at_contactPoint.set(
    new btVector3(getRigidBody().getVelocityInLocalPoint(relpos)));
   float projVel = wheel.m_raycastInfo.m_contactNormalWS.dot(chassis_velocity_at_contactPoint);
   if (denominator >= (-0.1f)) {
    wheel.m_suspensionRelativeVelocity = (0.0f);
    wheel.m_clippedInvContactDotSuspension = (1.0f) / (0.1f);
   } else {
    float inv = (-1.f) / denominator;
    wheel.m_suspensionRelativeVelocity = projVel * inv;
    wheel.m_clippedInvContactDotSuspension = inv;
   }
  } else {
   //put wheel info as in rest position
   wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength();
   wheel.m_suspensionRelativeVelocity = (0.0f);
   wheel.m_raycastInfo.m_contactNormalWS.set(wheel.m_raycastInfo.m_wheelDirectionWS).negate();
   wheel.m_clippedInvContactDotSuspension = (1.0f);
  }
  return depth;
 }

 public void updateVehicle(float step) {
  {
   for (int i = 0; i < getNumWheels(); i++) {
    updateWheelTransform(i, false);
   }
  }
  m_currentVehicleSpeedKmHour = (3.6f) * getRigidBody().getLinearVelocity().length();
  final btTransform chassisTrans = getChassisWorldTransform();
  final btVector3 forwardW = new btVector3(
   chassisTrans.getElement(0, m_indexForwardAxis),
   chassisTrans.getElement(1, m_indexForwardAxis),
   chassisTrans.getElement(2, m_indexForwardAxis));
  if (forwardW.dot(getRigidBody().getLinearVelocity()) < (0.f)) {
   m_currentVehicleSpeedKmHour *= (-1.);
  }
  //
  // simulate suspension
  //
  for (int i = 0; i < m_wheelInfo.size(); i++) {
   //float depth; 
   //depth = 
   rayCast(m_wheelInfo.get(i));
  }
  updateSuspension(step);
  for (int i = 0; i < m_wheelInfo.size(); i++) {
   //apply suspension force
   btWheelInfo wheel = m_wheelInfo.get(i);
   float suspensionForce = wheel.m_wheelsSuspensionForce;
   if (suspensionForce > wheel.m_maxSuspensionForce) {
    suspensionForce = wheel.m_maxSuspensionForce;
   }
   final btVector3 impulse = new btVector3(wheel.m_raycastInfo.m_contactNormalWS).scale(
    suspensionForce).scale(step);
   final btVector3 relpos = new btVector3(wheel.m_raycastInfo.m_contactPointWS).sub(getRigidBody()
    .getCenterOfMassPosition());
   getRigidBody().applyImpulse(impulse, relpos);
  }
  updateFriction(step);
  for (int i = 0; i < m_wheelInfo.size(); i++) {
   btWheelInfo wheel = m_wheelInfo.get(i);
   final btVector3 relpos = new btVector3(wheel.m_raycastInfo.m_hardPointWS).sub(getRigidBody()
    .getCenterOfMassPosition());
   final btVector3 vel = getRigidBody().getVelocityInLocalPoint(relpos);
   if (wheel.m_raycastInfo.m_isInContact) {
    final btTransform chassisWorldTransform = getChassisWorldTransform();
    final btVector3 fwd = new btVector3(
     chassisWorldTransform.getElement(0, m_indexForwardAxis),
     chassisWorldTransform.getElement(1, m_indexForwardAxis),
     chassisWorldTransform.getElement(2, m_indexForwardAxis));
    float proj = fwd.dot(wheel.m_raycastInfo.m_contactNormalWS);
    fwd.sub(new btVector3(wheel.m_raycastInfo.m_contactNormalWS).scale(proj));
    float proj2 = fwd.dot(vel);
    wheel.m_deltaRotation = (proj2 * step) / (wheel.m_wheelsRadius);
    wheel.m_rotation += wheel.m_deltaRotation;
   } else {
    wheel.m_rotation += wheel.m_deltaRotation;
   }
   wheel.m_deltaRotation *= (0.99f);//damping of rotation when not in contact
  }
 }

 public void resetSuspension() {
  int i;
  for (i = 0; i < m_wheelInfo.size(); i++) {
   btWheelInfo wheel = m_wheelInfo.get(i);
   wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength();
   wheel.m_suspensionRelativeVelocity = (0.0f);
   wheel.m_raycastInfo.m_contactNormalWS.set(wheel.m_raycastInfo.m_wheelDirectionWS).negate();
   wheel.m_clippedInvContactDotSuspension = (1.0f);
  }
 }

 public float getSteeringValue(int wheel) {
  return getWheelInfo(wheel).m_steering;
 }

 public void setSteeringValue(float steering, int wheel) {
  assert (wheel >= 0 && wheel < getNumWheels());
  btWheelInfo wheelInfo = getWheelInfo(wheel);
  wheelInfo.m_steering = steering;
 }

 public void applyEngineForce(float force, int wheel) {
  assert (wheel >= 0 && wheel < getNumWheels());
  btWheelInfo wheelInfo = getWheelInfo(wheel);
  wheelInfo.m_engineForce = force;
 }

 public btTransform getWheelTransformWS(int wheelIndex) {
  assert (wheelIndex < getNumWheels());
  btWheelInfo wheel = m_wheelInfo.get(wheelIndex);
  return wheel.m_worldTransform;
 }

 public void updateWheelTransform(int wheelIndex) {
  updateWheelTransform(wheelIndex, true);
 }

 public void updateWheelTransform(int wheelIndex, boolean interpolatedTransform) {
  btWheelInfo wheel = m_wheelInfo.get(wheelIndex);
  updateWheelTransformsWS(wheel, interpolatedTransform);
  final btVector3 up = new btVector3(wheel.m_raycastInfo.m_wheelDirectionWS).negate();
  final btVector3 right = wheel.m_raycastInfo.m_wheelAxleWS;
  final btVector3 fwd = new btVector3(up).cross(right);
  fwd.normalize();
//	up = right.cross(fwd);
//	up.normalize();
  //rotate around steering over de wheelAxleWS
  float steering = wheel.m_steering;
  final btQuaternion steeringOrn = new btQuaternion(up, steering);//wheel.m_steering);
  final btMatrix3x3 steeringMat = new btMatrix3x3(steeringOrn);
  final btQuaternion rotatingOrn = new btQuaternion(right, -wheel.m_rotation);
  final btMatrix3x3 rotatingMat = new btMatrix3x3(rotatingOrn);
  final btMatrix3x3 basis2 = new btMatrix3x3(
   right.x, fwd.x, up.x,
   right.y, fwd.y, up.y,
   right.z, fwd.z, up.z
  );
  wheel.m_worldTransform.setBasis(new btMatrix3x3(steeringMat).mul(rotatingMat).mul(basis2));
  wheel.m_worldTransform.setOrigin(
   new btVector3(wheel.m_raycastInfo.m_hardPointWS).add(new btVector3(
    wheel.m_raycastInfo.m_wheelDirectionWS).scale(wheel.m_raycastInfo.m_suspensionLength))
  );
 }

//	void	setRaycastWheelInfo( int wheelIndex , bool isInContact, const btVector3& hitPoint, const btVector3& hitNormal,float depth);
 public btWheelInfo addWheel(final btVector3 connectionPointCS, final btVector3 wheelDirectionCS0,
  final btVector3 wheelAxleCS, float suspensionRestLength, float wheelRadius, btVehicleTuning tuning,
  boolean isFrontWheel) {
  btWheelInfoConstructionInfo ci = new btWheelInfoConstructionInfo();
  ci.m_chassisConnectionCS.set(connectionPointCS);
  ci.m_wheelDirectionCS.set(wheelDirectionCS0);
  ci.m_wheelAxleCS.set(wheelAxleCS);
  ci.m_suspensionRestLength = suspensionRestLength;
  ci.m_wheelRadius = wheelRadius;
  ci.m_suspensionStiffness = tuning.m_suspensionStiffness;
  ci.m_wheelsDampingCompression = tuning.m_suspensionCompression;
  ci.m_wheelsDampingRelaxation = tuning.m_suspensionDamping;
  ci.m_frictionSlip = tuning.m_frictionSlip;
  ci.m_bIsFrontWheel = isFrontWheel;
  ci.m_maxSuspensionTravelCm = tuning.m_maxSuspensionTravelCm;
  ci.m_maxSuspensionForce = tuning.m_maxSuspensionForce;
  m_wheelInfo.add(new btWheelInfo(ci));
  btWheelInfo wheel = m_wheelInfo.get(getNumWheels() - 1);
  updateWheelTransformsWS(wheel, false);
  updateWheelTransform(getNumWheels() - 1, false);
  return wheel;
 }

 public int getNumWheels() {
  return (m_wheelInfo.size());
 }
 protected final ArrayList<btWheelInfo> m_wheelInfo = new ArrayList<>();

 public btWheelInfo getWheelInfo(int index) {
  assert ((index >= 0) && (index < getNumWheels()));
  return m_wheelInfo.get(index);
 }

 public void updateWheelTransformsWS(btWheelInfo wheel) {
  updateWheelTransformsWS(wheel, true);
 }

 public void updateWheelTransformsWS(btWheelInfo wheel, boolean interpolatedTransform) {
  wheel.m_raycastInfo.m_isInContact = false;
  final btTransform chassisTrans = getChassisWorldTransform();
  if (interpolatedTransform && (getRigidBody().getMotionState() != null)) {
   getRigidBody().getMotionState().getWorldTransform(chassisTrans);
  }
  chassisTrans.transform(wheel.m_raycastInfo.m_hardPointWS.set(wheel.m_chassisConnectionPointCS));
  chassisTrans.transform3x3(wheel.m_raycastInfo.m_wheelDirectionWS.set(wheel.m_wheelDirectionCS));
  chassisTrans.transform3x3(wheel.m_raycastInfo.m_wheelAxleWS.set(wheel.m_wheelAxleCS));
 }

 public void setBrake(float brake, int wheelIndex) {
  assert ((wheelIndex >= 0) && (wheelIndex < getNumWheels()));
  getWheelInfo(wheelIndex).m_brake = brake;
 }

 public void setPitchControl(float pitch) {
  //m_pitchControl = pitch;
 }

 public void updateSuspension(float deltaTime) {
  float chassisMass = (1.f) / m_chassisBody.getInvMass();
  for (int w_it = 0; w_it < getNumWheels(); w_it++) {
   btWheelInfo wheel_info = m_wheelInfo.get(w_it);
   if (wheel_info.m_raycastInfo.m_isInContact) {
    float force;
    //	Spring
    {
     float susp_length = wheel_info.getSuspensionRestLength();
     float current_length = wheel_info.m_raycastInfo.m_suspensionLength;
     float length_diff = (susp_length - current_length);
     force = wheel_info.m_suspensionStiffness *
      length_diff * wheel_info.m_clippedInvContactDotSuspension;
    }
    // Damper
    {
     float projected_rel_vel = wheel_info.m_suspensionRelativeVelocity;
     {
      float susp_damping;
      if (projected_rel_vel < (0.0f)) {
       susp_damping = wheel_info.m_wheelsDampingCompression;
      } else {
       susp_damping = wheel_info.m_wheelsDampingRelaxation;
      }
      force -= susp_damping * projected_rel_vel;
     }
    }
    // RESULT
    wheel_info.m_wheelsSuspensionForce = force * chassisMass;
    if (wheel_info.m_wheelsSuspensionForce < (0.f)) {
     wheel_info.m_wheelsSuspensionForce = (0.f);
    }
   } else {
    wheel_info.m_wheelsSuspensionForce = (0.0f);
   }
  }
 }
 public float sideFrictionStiffness2 = (1.0f);

 public void updateFriction(float timeStep) {
  //calculate the impulse, so that the wheels don't move sidewards
  int numWheel = getNumWheels();
  if (numWheel == 0) {
   return;
  }
  m_forwardWS.clear();
  m_axle.clear();
  m_forwardImpulse.clear();
  m_sideImpulse.clear();
  int numWheelsOnGround = 0;
  //collapse all those loops into one!
  for (int i = 0; i < getNumWheels(); i++) {
   btWheelInfo wheelInfo = m_wheelInfo.get(i);
   btRigidBody groundObject = (btRigidBody) wheelInfo.m_raycastInfo.m_groundObject;
   if (groundObject != null) {
    numWheelsOnGround++;
   }
   m_sideImpulse.add((0.f));
   m_forwardImpulse.add((0.f));
  }
  {
   for (int i = 0; i < getNumWheels(); i++) {
    btWheelInfo wheelInfo = m_wheelInfo.get(i);
    btRigidBody groundObject = (btRigidBody) wheelInfo.m_raycastInfo.m_groundObject;
    if (groundObject != null) {
     final btTransform wheelTrans = getWheelTransformWS(i);
     final btMatrix3x3 wheelBasis0 = wheelTrans.getBasis();
     m_axle.add(new btVector3(
      wheelBasis0.getElement(0, m_indexRightAxis),
      wheelBasis0.getElement(1, m_indexRightAxis),
      wheelBasis0.getElement(2, m_indexRightAxis)));
     final btVector3 surfNormalWS = wheelInfo.m_raycastInfo.m_contactNormalWS;
     float proj = m_axle.get(i).dot(surfNormalWS);
     m_axle.get(i).sub(new btVector3(surfNormalWS).scale(proj));
     m_axle.get(i).normalize();
     m_forwardWS.add(new btVector3(surfNormalWS).cross(m_axle.get(i)));
     m_forwardWS.get(i).normalize();
     float[] sideImpulse = new float[1];
     resolveSingleBilateral(m_chassisBody, wheelInfo.m_raycastInfo.m_contactPointWS,
      groundObject, wheelInfo.m_raycastInfo.m_contactPointWS,
      (0.f), m_axle.get(i), sideImpulse, timeStep);
     m_sideImpulse.set(i, sideImpulse[0] * sideFrictionStiffness2);
    } else {
     m_axle.add(new btVector3());
     m_forwardWS.add(new btVector3());
    }
   }
  }
  float sideFactor = (1.f);
  float fwdFactor = 0.5f;
  boolean sliding = false;
  {
   for (int wheel = 0; wheel < getNumWheels(); wheel++) {
    btWheelInfo wheelInfo = m_wheelInfo.get(wheel);
    btRigidBody groundObject = (btRigidBody) wheelInfo.m_raycastInfo.m_groundObject;
    float rollingFriction = 0.f;
    if (groundObject != null) {
     if (wheelInfo.m_engineForce != 0.f) {
      rollingFriction = wheelInfo.m_engineForce * timeStep;
     } else {
      float defaultRollingFrictionImpulse = 0.f;
      float maxImpulse = wheelInfo.m_brake != 0.0f ? wheelInfo.m_brake :
       defaultRollingFrictionImpulse;
      btWheelContactPoint contactPt = new btWheelContactPoint(m_chassisBody, groundObject,
       wheelInfo.m_raycastInfo.m_contactPointWS, m_forwardWS.get(wheel), maxImpulse);
      rollingFriction = calcRollingFriction(contactPt);
     }
    }
    //switch between active rolling (throttle), braking and non-active rolling friction (no throttle/break)
    m_forwardImpulse.set(wheel, 0f);
    m_wheelInfo.get(wheel).m_skidInfo = (1.f);
    if (groundObject != null) {
     m_wheelInfo.get(wheel).m_skidInfo = (1.f);
     float maximp = wheelInfo.m_wheelsSuspensionForce * timeStep * wheelInfo.m_frictionSlip;
     float maximpSide = maximp;
     float maximpSquared = maximp * maximpSide;
     m_forwardImpulse.set(wheel, rollingFriction);//wheelInfo.m_engineForce* timeStep;
     float x = (m_forwardImpulse.get(wheel)) * fwdFactor;
     float y = (m_sideImpulse.get(wheel)) * sideFactor;
     float impulseSquared = (x * x + y * y);
     if (impulseSquared > maximpSquared) {
      sliding = true;
      float factor = maximp / btSqrt(impulseSquared);
      m_wheelInfo.get(wheel).m_skidInfo *= factor;
     }
    }
   }
  }
  if (sliding) {
   for (int wheel = 0; wheel < getNumWheels(); wheel++) {
    if (m_sideImpulse.get(wheel) != (0.f)) {
     if (m_wheelInfo.get(wheel).m_skidInfo < (1.f)) {
      m_forwardImpulse.set(wheel, m_forwardImpulse.get(wheel) * m_wheelInfo.get(wheel).m_skidInfo);
      m_sideImpulse.set(wheel, m_sideImpulse.get(wheel) * m_wheelInfo.get(wheel).m_skidInfo);
     }
    }
   }
  }
  // apply the impulses
  {
   for (int wheel = 0; wheel < getNumWheels(); wheel++) {
    btWheelInfo wheelInfo = m_wheelInfo.get(wheel);
    final btVector3 rel_pos = new btVector3(wheelInfo.m_raycastInfo.m_contactPointWS).sub(
     m_chassisBody.getCenterOfMassPosition());
    if (m_forwardImpulse.get(wheel) != (0.f)) {
     m_chassisBody.applyImpulse(new btVector3(m_forwardWS.get(wheel)).scale(m_forwardImpulse.get(
      wheel)), rel_pos);
    }
    if (m_sideImpulse.get(wheel) != (0.f)) {
     btRigidBody groundObject = (btRigidBody) m_wheelInfo.get(wheel).m_raycastInfo.m_groundObject;
     final btVector3 rel_pos2 = new btVector3(wheelInfo.m_raycastInfo.m_contactPointWS).sub(
      groundObject.getCenterOfMassPosition());
     final btVector3 sideImp = new btVector3(m_axle.get(wheel)).scale(m_sideImpulse.get(wheel));
     final btVector3 vChassisWorldUp = getRigidBody().getCenterOfMassTransformPtr().getBasisColumn(
      m_indexUpAxis);
     rel_pos.sub(new btVector3(vChassisWorldUp).scale((vChassisWorldUp.dot(rel_pos) * (1.f -
      wheelInfo.m_rollInfluence))));
     m_chassisBody.applyImpulse(sideImp, rel_pos);
     //apply friction impulse on the ground
     groundObject.applyImpulse(new btVector3(sideImp).negate(), rel_pos2);
    }
   }
  }
 }

 public btRigidBody getRigidBody() {
  return m_chassisBody;
 }

 public int getRightAxis() {
  return m_indexRightAxis;
 }

 public int getUpAxis() {
  return m_indexUpAxis;
 }

 public int getForwardAxis() {
  return m_indexForwardAxis;
 }

 ///Worldspace forward vector
 public btVector3 getForwardVector() {
  final btTransform chassisTrans = getChassisWorldTransform();
  final btVector3 forwardW = new btVector3(
   chassisTrans.getElement(0, m_indexForwardAxis),
   chassisTrans.getElement(1, m_indexForwardAxis),
   chassisTrans.getElement(2, m_indexForwardAxis));
  return forwardW;
 }

 ///Velocity of vehicle (positive if velocity vector has same direction as foward vector)
 public float getCurrentSpeedKmHour() {
  return m_currentVehicleSpeedKmHour;
 }

 public void setCoordinateSystem(int rightIndex, int upIndex, int forwardIndex) {
  m_indexRightAxis = rightIndex;
  m_indexUpAxis = upIndex;
  m_indexForwardAxis = forwardIndex;
 }

 ///backwards compatibility
 public int getUserConstraintType() {
  return m_userConstraintType;
 }

 public void setUserConstraintType(int userConstraintType) {
  m_userConstraintType = userConstraintType;
 }
 

public void setUserConstraintId(int uid) {
  m_userConstraintId = uid;
 }

 public int getUserConstraintId() {
  return m_userConstraintId;
 }

 private static float calcRollingFriction(btWheelContactPoint contactPoint) {
  final btVector3 contactPosWorld = contactPoint.m_frictionPositionWorld;
  final btVector3 rel_pos1 = new btVector3(contactPosWorld).sub(contactPoint.m_body0
   .getCenterOfMassPosition());
  final btVector3 rel_pos2 = new btVector3(contactPosWorld).sub(contactPoint.m_body1
   .getCenterOfMassPosition());
  float maxImpulse = contactPoint.m_maxImpulse;
  final btVector3 vel1 = contactPoint.m_body0.getVelocityInLocalPoint(rel_pos1);
  final btVector3 vel2 = contactPoint.m_body1.getVelocityInLocalPoint(rel_pos2);
  final btVector3 vel = new btVector3(vel1).sub(vel2);
  float vrel = contactPoint.m_frictionDirectionWorld.dot(vel);
  // calculate j that moves us to zero relative velocity
  float j1 = -vrel * contactPoint.m_jacDiagABInv;
  j1 = Math.min(j1, maxImpulse);
  j1 = Math.max(j1, -maxImpulse);
  return j1;
 }
}
