/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2010 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.Dynamics.Constraint;

import Bullet.Dynamics.ConstraintSolver.btSolverConstraint;
import Bullet.Dynamics.ConstraintSolver.btSolverBody;
import Bullet.Dynamics.btJointFeedback;
import Bullet.Dynamics.btRigidBody;
import static Bullet.LinearMath.btScalar.SIMD_2_PI;
import static Bullet.LinearMath.btScalar.SIMD_INFINITY;
import static Bullet.LinearMath.btScalar.btFabs;
import static Bullet.LinearMath.btScalar.btNormalizeAngle;
import Bullet.LinearMath.btTypedObject;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import java.util.ArrayList;

/**
 *
 * @author Gregery Barton
 */
public abstract class btTypedConstraint extends btTypedObject implements Serializable {

 public static final int POINT2POINT_CONSTRAINT_TYPE = 3;
 public static final int HINGE_CONSTRAINT_TYPE = 4;
 public static final int CONETWIST_CONSTRAINT_TYPE = 5;
 public static final int D6_CONSTRAINT_TYPE = 6;
 public static final int SLIDER_CONSTRAINT_TYPE = 7;
 public static final int CONTACT_CONSTRAINT_TYPE = 8;
 public static final int D6_SPRING_CONSTRAINT_TYPE = 9;
 public static final int GEAR_CONSTRAINT_TYPE = 10;
 public static final int FIXED_CONSTRAINT_TYPE = 11;
 public static final int D6_SPRING_2_CONSTRAINT_TYPE = 12;
 public static final int MAX_CONSTRAINT_TYPE = 13;
 public static final int BT_CONSTRAINT_ERP = 1;
 public static final int BT_CONSTRAINT_STOP_ERP = 2;
 public static final int BT_CONSTRAINT_CFM = 3;
 public static final int BT_CONSTRAINT_STOP_CFM = 4;
 static final float DEFAULT_DEBUGDRAW_SIZE = 0.3f;
 static btRigidBody s_fixed = new btRigidBody(0, null, null);
 private static final long serialVersionUID = 1L;

 public static btRigidBody getFixedBody() {
  s_fixed.setMassProps((0.f), new btVector3((0.f), (0.f), (0.f)));
  return s_fixed;
 }

 public static float btAdjustAngleToLimits(float angleInRadians, float angleLowerLimitInRadians,
  float angleUpperLimitInRadians) {
  if (angleLowerLimitInRadians >= angleUpperLimitInRadians) {
   return angleInRadians;
  } else if (angleInRadians < angleLowerLimitInRadians) {
   float diffLo = btFabs(btNormalizeAngle(angleLowerLimitInRadians - angleInRadians));
   float diffHi = btFabs(btNormalizeAngle(angleUpperLimitInRadians - angleInRadians));
   return (diffLo < diffHi) ? angleInRadians : (angleInRadians + SIMD_2_PI);
  } else if (angleInRadians > angleUpperLimitInRadians) {
   float diffHi = btFabs(btNormalizeAngle(angleInRadians - angleUpperLimitInRadians));
   float diffLo = btFabs(btNormalizeAngle(angleInRadians - angleLowerLimitInRadians));
   return (diffLo < diffHi) ? (angleInRadians - SIMD_2_PI) : angleInRadians;
  } else {
   return angleInRadians;
  }
 }
 protected int m_userConstraintType;
 protected int m_userConstraintId;
 protected Object m_userConstraintPtr;
 protected float m_breakingImpulseThreshold;
 protected boolean m_isEnabled;
 protected boolean m_needsFeedback;
 protected int m_overrideNumSolverIterations;
 final protected btRigidBody m_rbA;
 final protected btRigidBody m_rbB;
 protected float m_appliedImpulse;
 protected float m_dbgDrawSize;
 protected btJointFeedback m_jointFeedback;

 public btTypedConstraint(int type, btRigidBody rbA) {
  super(type);
  m_userConstraintType = -1;
  m_userConstraintPtr = null;
  m_breakingImpulseThreshold = SIMD_INFINITY;
  m_isEnabled = true;
  m_needsFeedback = false;
  m_overrideNumSolverIterations = -1;
  m_rbA = rbA;
  m_rbB = getFixedBody();
  m_appliedImpulse = 0;
  m_dbgDrawSize = DEFAULT_DEBUGDRAW_SIZE;
  m_jointFeedback = null;
 }

 public btTypedConstraint(int type, btRigidBody rbA, btRigidBody rbB) {
  super(type);
  m_userConstraintType = -1;
  m_userConstraintPtr = null;
  m_breakingImpulseThreshold = SIMD_INFINITY;
  m_isEnabled = true;
  m_needsFeedback = false;
  m_overrideNumSolverIterations = -1;
  m_rbA = rbA;
  m_rbB = rbB;
  m_appliedImpulse = 0.f;
  m_dbgDrawSize = DEFAULT_DEBUGDRAW_SIZE;
  m_jointFeedback = null;
 }
 ///internal method used by the constraint solver, don't use them directly

 public float getMotorFactor(float pos, float lowLim, float uppLim, float vel, float timeFact) {
  if (lowLim > uppLim) {
   return (1.0f);
  } else if (lowLim == uppLim) {
   return (0.0f);
  }
  float lim_fact;
  float delta_max = vel / timeFact;
  if (delta_max < (0.0f)) {
   if ((pos >= lowLim) && (pos < (lowLim - delta_max))) {
    lim_fact = (lowLim - pos) / delta_max;
   } else if (pos < lowLim) {
    lim_fact = (0.0f);
   } else {
    lim_fact = (1.0f);
   }
  } else if (delta_max > (0.0f)) {
   if ((pos <= uppLim) && (pos > (uppLim - delta_max))) {
    lim_fact = (uppLim - pos) / delta_max;
   } else if (pos > uppLim) {
    lim_fact = (0.0f);
   } else {
    lim_fact = (1.0f);
   }
  } else {
   lim_fact = (0.0f);
  }
  return lim_fact;
 }

 public int getOverrideNumSolverIterations() {
  return m_overrideNumSolverIterations;
 }

 ///override the number of constraint solver iterations used to solve this constraint
 ///-1 will use the default number of iterations, as specified in SolverInfo.m_numIterations
 public void setOverrideNumSolverIterations(int overideNumIterations) {
  m_overrideNumSolverIterations = overideNumIterations;
 }

 ///internal method used by the constraint solver, don't use them directly
 public void buildJacobian() {
 }

 ///internal method used by the constraint solver, don't use them directly
 public void setupSolverConstraint(ArrayList<btSolverConstraint> ca, int solverBodyA,
  int solverBodyB, float timeStep) {
 }

 ///internal method used by the constraint solver, don't use them directly
 public abstract void getInfo1(btConstraintInfo1 info);

 ///internal method used by the constraint solver, don't use them directly
 public abstract void getInfo2(btConstraintInfo2 info);

 ///internal method used by the constraint solver, don't use them directly
 public void internalSetAppliedImpulse(float appliedImpulse) {
  m_appliedImpulse = appliedImpulse;
 }

 ///internal method used by the constraint solver, don't use them directly
 public float internalGetAppliedImpulse() {
  return m_appliedImpulse;
 }

 public float getBreakingImpulseThreshold() {
  return m_breakingImpulseThreshold;
 }

 public void setBreakingImpulseThreshold(float threshold) {
  m_breakingImpulseThreshold = threshold;
 }

 public boolean isEnabled() {
  return m_isEnabled;
 }

 public void setEnabled(boolean enabled) {
  m_isEnabled = enabled;
 }

 ///internal method used by the constraint solver, don't use them directly
 public void solveConstraintObsolete(btSolverBody bodyA, btSolverBody bodyB, float timeStep) {
 }

 public btRigidBody getRigidBodyA() {
  return m_rbA;
 }

 public btRigidBody getRigidBodyB() {
  return m_rbB;
 }

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

 public void setUserConstraintPtr(Object ptr) {
  m_userConstraintPtr = ptr;
 }

 public Object getUserConstraintPtr() {
  return m_userConstraintPtr;
 }

 public void setJointFeedback(btJointFeedback jointFeedback) {
  m_jointFeedback = jointFeedback;
 }

 public btJointFeedback getJointFeedback() {
  return m_jointFeedback;
 }

 public int getUid() {
  return m_userConstraintId;
 }

 public boolean needsFeedback() {
  return m_needsFeedback;
 }

 ///enableFeedback will allow to read the applied linear and angular impulse
 ///use getAppliedImpulse, getAppliedLinearImpulse and getAppliedAngularImpulse to read feedback information
 public void enableFeedback(boolean needsFeedback) {
  m_needsFeedback = needsFeedback;
 }

 ///getAppliedImpulse is an estimated total applied impulse. 
 ///This feedback could be used to determine breaking constraints or playing sounds.
 public float getAppliedImpulse() {
  assert (m_needsFeedback);
  return m_appliedImpulse;
 }

 public int getConstraintType() {
  return m_objectType;
 }

 public void setDbgDrawSize(float dbgDrawSize) {
  m_dbgDrawSize = dbgDrawSize;
 }

 public float getDbgDrawSize() {
  return m_dbgDrawSize;
 }

 public final void setParam(int num, float value) {
  setParam(num, value, -1);
 }

 ///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
 ///If no axis is provided, it uses the default axis for this constraint.
 public abstract void setParam(int num, float value, int axis);

 ///return the local value of parameter
 public final float getParam(int num) {
  return getParam(num, -1);
 }

 public abstract float getParam(int num, int axis);

 public static class btConstraintInfo1 implements Serializable {

  public int m_numConstraintRows, nub;
 }
 // returns angle in range [-SIMD_2_PI, SIMD_2_PI], closest to one of the limits 
// all arguments should be normalized angles (i.e. in range [-SIMD_PI, SIMD_PI])
}
