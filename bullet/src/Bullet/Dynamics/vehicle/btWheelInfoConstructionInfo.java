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

import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btWheelInfoConstructionInfo implements Serializable {

 private static final long serialVersionUID = 1L;
 public final btVector3 m_chassisConnectionCS = new btVector3();
 public final btVector3 m_wheelDirectionCS = new btVector3();
 public final btVector3 m_wheelAxleCS = new btVector3();
 public float m_suspensionRestLength;
 public float m_maxSuspensionTravelCm;
 public float m_wheelRadius;
 public float m_suspensionStiffness;
 public float m_wheelsDampingCompression;
 public float m_wheelsDampingRelaxation;
 public float m_frictionSlip;
 public float m_maxSuspensionForce;
 public boolean m_bIsFrontWheel;
}
