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

import static Bullet.Dynamics.ConstraintSolver.btSolverMode.SOLVER_SIMD;
import static Bullet.Dynamics.ConstraintSolver.btSolverMode.SOLVER_USE_WARMSTARTING;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btContactSolverInfo extends btContactSolverInfoData implements Serializable {

 public btContactSolverInfo() {
  m_tau = (0.6f);
  m_damping = (1.0f);
  m_friction = (0.3f);
  m_timeStep = (1.f / 60.f);
  m_restitution = (0.f);
  m_maxErrorReduction = (20.f);
  m_numIterations = 10;
  m_erp = (0.2f);
  m_erp2 = (0.2f);
  m_globalCfm = (0.f);
  m_sor = (1.f);
  m_splitImpulse = true;
  m_splitImpulsePenetrationThreshold = -.04f;
  m_splitImpulseTurnErp = 0.1f;
  m_linearSlop = (0.0f);
  m_warmstartingFactor = (0.85f);
  //m_solverMode =  SOLVER_USE_WARMSTARTING |  SOLVER_SIMD | SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION|SOLVER_USE_2_FRICTION_DIRECTIONS|SOLVER_ENABLE_FRICTION_DIRECTION_CACHING;// | SOLVER_RANDMIZE_ORDER;
  m_solverMode = SOLVER_USE_WARMSTARTING | SOLVER_SIMD;// | SOLVER_RANDMIZE_ORDER;
  m_restingContactRestitutionThreshold = 2;//unused as of 2.81
  m_minimumSolverBatchSize = 128; //try to combine islands until the amount of constraints reaches this limit
  m_maxGyroscopicForce = 100.f; ///it is only used for 'explicit' version of gyroscopic force
  m_singleAxisRollingFrictionThreshold = 1e30f;///if the velocity is above this threshold, it will use a single constraint row (axis), otherwise 3 rows.
  m_leastSquaresResidualThreshold = 0.f;
 }

 public Object clone() throws CloneNotSupportedException {
  Object obj = super.clone();
  return obj;
 }
}
