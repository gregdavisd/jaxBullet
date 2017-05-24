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

package Bullet.Dynamics;

import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btContactSolverInfoData   implements Serializable {

 public float m_tau;
 public float m_damping;//global non-contact constraint damping, can be locally overridden by constraints during 'getInfo2'.
 public float m_friction;
 public float m_timeStep;
 public float m_restitution;
 public int m_numIterations;
 public float m_maxErrorReduction;
 public float m_sor;
 public float m_erp;//used as Baumgarte factor
 public float m_erp2;//used in Split Impulse
 public float m_globalCfm;//constraint force mixing
 public boolean m_splitImpulse;
 public float m_splitImpulsePenetrationThreshold;
 public float m_splitImpulseTurnErp;
 public float m_linearSlop;
 public float m_warmstartingFactor;
 public int m_solverMode;
 public int m_restingContactRestitutionThreshold;
 public int m_minimumSolverBatchSize;
 public float m_maxGyroscopicForce;
 public float m_singleAxisRollingFrictionThreshold;
 public float m_leastSquaresResidualThreshold;
  
}
