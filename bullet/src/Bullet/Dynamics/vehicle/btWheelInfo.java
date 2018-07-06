/*
 * Copyright (c) 2005 Erwin Coumans http://continuousphysics.com/Bullet/
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
 */
package Bullet.Dynamics.vehicle;

import Bullet.Dynamics.CollisionObjects.btRigidBody;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btWheelInfo implements Serializable {

 private static final long serialVersionUID = 1L;

 public static class RaycastInfo {
  //set by raycaster

  public final btVector3 m_contactNormalWS = new btVector3();//contactnormal
  public final btVector3 m_contactPointWS = new btVector3();//raycast hitpoint
  public float m_suspensionLength;
  public final btVector3 m_hardPointWS = new btVector3();//raycast starting point
  public final btVector3 m_wheelDirectionWS = new btVector3(); //direction in worldspace
  public final btVector3 m_wheelAxleWS = new btVector3(); // axle in worldspace
  public boolean m_isInContact;
  public Object m_groundObject; //could be general void* ptr
 }
 public RaycastInfo m_raycastInfo = new RaycastInfo();
 public final btTransform m_worldTransform = new btTransform();
 public final btVector3 m_chassisConnectionPointCS = new btVector3(); //const
 public final btVector3 m_wheelDirectionCS = new btVector3();//const
 public final btVector3 m_wheelAxleCS = new btVector3(); // const or modified by steering
 public float m_suspensionRestLength1;//const
 public float m_maxSuspensionTravelCm;

 public float getSuspensionRestLength() {
  return m_suspensionRestLength1;
 }

 public float m_wheelsRadius;//const
 public float m_suspensionStiffness;//const
 public float m_wheelsDampingCompression;//const
 public float m_wheelsDampingRelaxation;//const
 public float m_frictionSlip;
 public float m_steering;
 public float m_rotation;
 public float m_deltaRotation;
 public float m_rollInfluence;
 public float m_maxSuspensionForce;
 public float m_engineForce;
 public float m_brake;
 public boolean m_bIsFrontWheel;
 public Object m_clientInfo;//can be used to store pointer to sync transforms...

 public btWheelInfo(btWheelInfoConstructionInfo ci) {
  m_suspensionRestLength1 = ci.m_suspensionRestLength;
  m_maxSuspensionTravelCm = ci.m_maxSuspensionTravelCm;
  m_wheelsRadius = ci.m_wheelRadius;
  m_suspensionStiffness = ci.m_suspensionStiffness;
  m_wheelsDampingCompression = ci.m_wheelsDampingCompression;
  m_wheelsDampingRelaxation = ci.m_wheelsDampingRelaxation;
  m_chassisConnectionPointCS.set(ci.m_chassisConnectionCS);
  m_wheelDirectionCS.set(ci.m_wheelDirectionCS);
  m_wheelAxleCS.set(ci.m_wheelAxleCS);
  m_frictionSlip = ci.m_frictionSlip;
  m_steering = (0.f);
  m_engineForce = (0.f);
  m_rotation = (0.f);
  m_deltaRotation = (0.f);
  m_brake = (0.f);
  m_rollInfluence = (0.1f);
  m_bIsFrontWheel = ci.m_bIsFrontWheel;
  m_maxSuspensionForce = ci.m_maxSuspensionForce;
 }

 public void updateWheel(btRigidBody chassis, RaycastInfo raycastInfo) {
  if (m_raycastInfo.m_isInContact) {
   float project = m_raycastInfo.m_contactNormalWS.dot(
    m_raycastInfo.m_wheelDirectionWS);
   final btVector3 chassis_velocity_at_contactPoint = new btVector3();
   final btVector3 relpos = new btVector3(m_raycastInfo.m_contactPointWS).sub(
    chassis
     .getCenterOfMassPosition());
   chassis_velocity_at_contactPoint.set(chassis.getVelocityInLocalPoint(relpos));
   float projVel = m_raycastInfo.m_contactNormalWS.dot(
    chassis_velocity_at_contactPoint);
   if (project >= (-0.1f)) {
    m_suspensionRelativeVelocity = (0.0f);
    m_clippedInvContactDotSuspension = (1.0f) / (0.1f);
   } else {
    float inv = (-1.f) / project;
    m_suspensionRelativeVelocity = projVel * inv;
    m_clippedInvContactDotSuspension = inv;
   }
  } else // Not in contact : position wheel in a nice (rest length) position
  {
   m_raycastInfo.m_suspensionLength = getSuspensionRestLength();
   m_suspensionRelativeVelocity = (0.0f);
   m_raycastInfo.m_contactNormalWS.set(m_raycastInfo.m_wheelDirectionWS)
    .negate();
   m_clippedInvContactDotSuspension = 1.0f;
  }
 }

 public float m_clippedInvContactDotSuspension;
 public float m_suspensionRelativeVelocity;
 //calculated by suspension
 public float m_wheelsSuspensionForce;
 public float m_skidInfo;
}
