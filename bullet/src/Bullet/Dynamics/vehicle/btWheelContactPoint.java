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
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btWheelContactPoint implements Serializable {

 private static final long serialVersionUID = 1L;
 public btRigidBody m_body0;
 public btRigidBody m_body1;
 public final btVector3 m_frictionPositionWorld = new btVector3();
 public final btVector3 m_frictionDirectionWorld = new btVector3();
 public float m_jacDiagABInv;
 public float m_maxImpulse;

 public btWheelContactPoint(btRigidBody body0, btRigidBody body1, final btVector3 frictionPosWorld,
  final btVector3 frictionDirectionWorld, float maxImpulse) {
  m_body0 = (body0);
  m_body1 = (body1);
  m_frictionPositionWorld.set(frictionPosWorld);
  m_frictionDirectionWorld.set(frictionDirectionWorld);
  m_maxImpulse = (maxImpulse);
  float denom0 = body0.computeImpulseDenominator(frictionPosWorld, frictionDirectionWorld);
  float denom1 = body1.computeImpulseDenominator(frictionPosWorld, frictionDirectionWorld);
  float relaxation = 1.f;
  m_jacDiagABInv = relaxation / (denom0 + denom1);
 }
}
