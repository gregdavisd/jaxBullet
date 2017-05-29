/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.Collision.Broadphase;

import Bullet.Collision.btIDebugDraw;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btDispatcherInfo implements Serializable {

 public static final int DISPATCH_DISCRETE = 1;
 public static final int DISPATCH_CONTINUOUS = 2;

 public btDispatcherInfo() {
  m_timeStep = 0.f;
  m_stepCount = 0;
  m_dispatchFunc = DISPATCH_DISCRETE;
  m_timeOfImpact = 1.f;
  m_useContinuous = true;
  m_debugDraw = null;
  m_enableSatConvex = false;
  m_enableSPU = true;
  m_useEpa = true;
  m_allowedCcdPenetration = 0.04f;
  m_useConvexConservativeDistanceUtil = false;
  m_convexConservativeDistanceThreshold = 0.0f;
 }
 public float m_timeStep;
 public int m_stepCount;
 public int m_dispatchFunc;
 public float m_timeOfImpact;
 public boolean m_useContinuous;
 public btIDebugDraw m_debugDraw;
 public boolean m_enableSatConvex;
 public boolean m_enableSPU;
 public boolean m_useEpa;
 public float m_allowedCcdPenetration;
 public boolean m_useConvexConservativeDistanceUtil;
 public float m_convexConservativeDistanceThreshold;
}
