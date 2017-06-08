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
package Bullet.Dynamics.ConstraintSolver;

import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import javax.vecmath.FloatSmartPointer;

/**
 * 1D constraint along a normal axis between bodyA and bodyB. It can be combined to solve contact
 * and friction constraints.
 *
 * @author Gregery Barton
 */
public class btSolverConstraint implements Serializable {

 public static final int BT_SOLVER_CONTACT_1D = 0;
 public static final int BT_SOLVER_FRICTION_1D = 1;
 public final btVector3 m_relpos1CrossNormal = new btVector3();
 public final btVector3 m_contactNormal1 = new btVector3();
 public final btVector3 m_relpos2CrossNormal = new btVector3();
 public final btVector3 m_contactNormal2 = new btVector3(); //usually m_contactNormal2 == -m_contactNormal1, but not always
 public final btVector3 m_angularComponentA = new btVector3();
 public final btVector3 m_angularComponentB = new btVector3();
 public float m_appliedPushImpulse;
 public float m_appliedImpulse;
 public float m_friction;
 public float m_jacDiagABInv;
 public float m_rhs;
 public float m_cfm;
 public float m_lowerLimit;
 public float m_upperLimit;
 public float m_rhsPenetration;
 public Object m_originalContactPoint;
 public int m_numRowsForNonContactConstraint;
 public int m_overrideNumSolverIterations;
 public btSolverConstraint m_solverFriction;
 public btSolverConstraint m_solverFriction2;
 public btSolverConstraint m_solverTorsionalFriction;
 public btSolverConstraint m_solverTorsionalFriction2;
 public btSolverBody m_solverBodyA;
 public btSolverBody m_solverBodyB;


 /* Pointers for use with btConstraintInfo2
 
  */
 public static class RHSPointer extends FloatSmartPointer<btSolverConstraint> {

  public RHSPointer(btSolverConstraint object) {
   super(object);
  }

  @Override
  public float get() {
   return ((btSolverConstraint) object()).m_rhs;
  }

  @Override
  public void set(float value) {
   ((btSolverConstraint) object()).m_rhs = value;
  }
 }

 public static class CFMPointer extends FloatSmartPointer<btSolverConstraint> {

  public CFMPointer(btSolverConstraint object) {
   super(object);
  }

  @Override
  public float get() {
   return ((btSolverConstraint) object()).m_cfm;
  }

  @Override
  public void set(float value) {
   ((btSolverConstraint) object()).m_cfm = value;
  }
 }

 public static class LowerLimitPointer extends FloatSmartPointer<btSolverConstraint> {

  public LowerLimitPointer(btSolverConstraint object) {
   super(object);
  }

  @Override
  public float get() {
   return ((btSolverConstraint) object()).m_lowerLimit;
  }

  @Override
  public void set(float value) {
   ((btSolverConstraint) object()).m_lowerLimit = value;
  }
 }

 public static class UpperLimitPointer extends FloatSmartPointer<btSolverConstraint> {

  public UpperLimitPointer(btSolverConstraint object) {
   super(object);
  }

  @Override
  public float get() {
   return ((btSolverConstraint) object()).m_upperLimit;
  }

  @Override
  public void set(float value) {
   ((btSolverConstraint) object()).m_upperLimit = value;
  }
 }
};
