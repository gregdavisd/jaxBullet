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

import Bullet.LinearMath.btVector3;

/**
 *
 * btVehicleRaycaster is provides interface for between vehicle simulation and raycasting
 *
 * @author Gregery Barton
 */
public abstract class btVehicleRaycaster {

 public static class btVehicleRaycasterResult {

  btVehicleRaycasterResult() {
   m_distFraction = (-1.f);
  }
  final btVector3 m_hitPointInWorld = new btVector3();
  final btVector3 m_hitNormalInWorld = new btVector3();
  float m_distFraction;
 }

 public abstract Object castRay(final btVector3 from, final btVector3 to,
  btVehicleRaycasterResult result);
}
