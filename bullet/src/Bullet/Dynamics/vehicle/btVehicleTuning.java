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

import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btVehicleTuning implements Serializable {

 private static final long serialVersionUID = 1L;

 public btVehicleTuning() {
  m_suspensionStiffness = ((5.88f));
  m_suspensionCompression = ((0.83f));
  m_suspensionDamping = ((0.88f));
  m_maxSuspensionTravelCm = ((500.f));
  m_frictionSlip = ((10.5f));
  m_maxSuspensionForce = ((6000.f));
 }

 public float m_suspensionStiffness;
 public float m_suspensionCompression;
 public float m_suspensionDamping;
 public float m_maxSuspensionTravelCm;
 public float m_frictionSlip;
 public float m_maxSuspensionForce;
}
