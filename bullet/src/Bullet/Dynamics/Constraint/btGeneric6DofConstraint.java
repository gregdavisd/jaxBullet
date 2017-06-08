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
/// 2009 March: btGeneric6DofConstraint refactored by Roman Ponomarev
/// Added support for generic constraint solver through getInfo1/getInfo2 methods

/*
2007-09-09
btGeneric6DofConstraint Refactored by Francisco Le?n
email: projectileman@yahoo.com
http://gimpact.sf.net
 */
package Bullet.Dynamics.Constraint;

/**
 */// btGeneric6DofConstraint between two rigidbodies each with a pivotpoint that descibes the axis location in local space
/*
 * !
 * btGeneric6DofConstraint can leave any of the 6 degree of freedom 'free' or 'locked'. currently
 * this limit supports rotational motors<br> <ul> <li> For Linear limits, use
 * btGeneric6DofConstraint.setLinearUpperLimit, btGeneric6DofConstraint.setLinearLowerLimit. You can
 * set the parameters with the btTranslationalLimitMotor structure accsesible through the
 * btGeneric6DofConstraint.getTranslationalLimitMotor method. At this moment translational motors
 * are not supported. May be in the future. </li>
 *
 * <li> For Angular limits, use the btRotationalLimitMotor structure for configuring the limit. This
 * is accessible through btGeneric6DofConstraint.getLimitMotor method, This brings support for limit
 * parameters and motors. </li>
 *
 * <li> Angulars limits have these possible ranges: <table border=1 > <tr> <td><b>AXIS</b></td>
 * <td><b>MIN ANGLE</b></td> <td><b>MAX ANGLE</b></td> </tr><tr> <td>X</td> <td>-PI</td> <td>PI</td>
 * </tr><tr> <td>Y</td> <td>-PI/2</td> <td>PI/2</td> </tr><tr> <td>Z</td> <td>-PI</td> <td>PI</td>
 * </tr> </table> </li> </ul>
 *
 *
 * @author Gregery Barton
 */
import Bullet.Dynamics.CollisionObjects.btRigidBody;
import Bullet.Dynamics.btJacobianEntry;
import Bullet.LinearMath.btMatrix3x3;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.SIMD_HALF_PI;
import static Bullet.LinearMath.btScalar.SIMD_INFINITY;
import static Bullet.LinearMath.btScalar.btAsin;
import static Bullet.LinearMath.btScalar.btAtan2;
import static Bullet.LinearMath.btScalar.btNormalizeAngle;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

public class btGeneric6DofConstraint extends btTypedConstraint implements Serializable {

 public static final int BT_6DOF_FLAGS_CFM_NORM = 1;
 public static final int BT_6DOF_FLAGS_CFM_STOP = 2;
 public static final int BT_6DOF_FLAGS_ERP_STOP = 4;
 public static final int BT_6DOF_FLAGS_AXIS_SHIFT = 3; // bits per axis
 //! relative_frames
 //!@{
 protected final btTransform m_frameInA = new btTransform();//!< the constraint space w.r.t body A
 protected final btTransform m_frameInB = new btTransform();//!< the constraint space w.r.t body B
 //!@}
 //! Jacobians
 //!@{
 final protected btJacobianEntry[] m_jacLinear = new btJacobianEntry[3];//!< 3 orthogonal linear constraints
 final protected btJacobianEntry[] m_jacAng = new btJacobianEntry[3];//!< 3 orthogonal angular constraints
 //!@}
 //! Linear_Limit_parameters
 //!@{
 protected btTranslationalLimitMotor m_linearLimits = new btTranslationalLimitMotor();
 //!@}
 //! hinge_parameters
 //!@{
 final protected btRotationalLimitMotor[] m_angularLimits = new btRotationalLimitMotor[]{
  new btRotationalLimitMotor(), new btRotationalLimitMotor(), new btRotationalLimitMotor()};
 //!@}
 //! temporal variables
 //!@{
 protected float m_timeStep;
 protected final btTransform m_calculatedTransformA = new btTransform();
 protected final btTransform m_calculatedTransformB = new btTransform();
 protected final btVector3 m_calculatedAxisAngleDiff = new btVector3();
 protected final btVector3[] m_calculatedAxis = {new btVector3(), new btVector3(), new btVector3()};
 protected final btVector3 m_calculatedLinearDiff = new btVector3();
 protected float m_factA;
 protected float m_factB;
 protected boolean m_hasStaticBody;
 protected final btVector3 m_AnchorPos = new btVector3(); // point betwen pivots of bodies A and B to solve linear axes
 protected boolean m_useLinearReferenceFrameA;
 protected boolean m_useOffsetForConstraintFrame;
 protected int m_flags;

 protected int setAngularLimits(btConstraintInfo2 info, int row_offset, final btTransform transA,
  final btTransform transB, final btVector3 linVelA, final btVector3 linVelB,
  final btVector3 angVelA, final btVector3 angVelB) {
  btGeneric6DofConstraint d6constraint = this;
  int row = row_offset;
  //solve angular limits
  for (int i = 0; i < 3; i++) {
   if (d6constraint.getRotationalLimitMotor(i).needApplyTorques()) {
    final btVector3 axis = d6constraint.getAxis(i);
    int flags = m_flags >>> ((i + 3) * BT_6DOF_FLAGS_AXIS_SHIFT);
    if (0 == (flags & BT_6DOF_FLAGS_CFM_NORM)) {
     m_angularLimits[i].m_normalCFM = info.cfm[0].get();
    }
    if (0 == (flags & BT_6DOF_FLAGS_CFM_STOP)) {
     m_angularLimits[i].m_stopCFM = info.cfm[0].get();
    }
    if (0 == (flags & BT_6DOF_FLAGS_ERP_STOP)) {
     m_angularLimits[i].m_stopERP = info.erp;
    }
    row += get_limit_motor_info2(d6constraint.getRotationalLimitMotor(i),
     transA, transB, linVelA, linVelB, angVelA, angVelB, info, row, axis, 1);
   }
  }
  return row;
 }

 protected int setLinearLimits(btConstraintInfo2 info, int row, final btTransform transA,
  final btTransform transB, final btVector3 linVelA, final btVector3 linVelB,
  final btVector3 angVelA, final btVector3 angVelB) {
  int do_row = row;
//	int row = 0;
  //solve linear limits
  btRotationalLimitMotor limot = new btRotationalLimitMotor();
  for (int i = 0; i < 3; i++) {
   if (m_linearLimits.needApplyForce(i)) { // re-use rotational motor code
    limot.m_bounce = (0.f);
    limot.m_currentLimit = m_linearLimits.m_currentLimit[i];
    limot.m_currentPosition = m_linearLimits.m_currentLinearDiff.getElement(i);
    limot.m_currentLimitError = m_linearLimits.m_currentLimitError.getElement(i);
    limot.m_damping = m_linearLimits.m_damping;
    limot.m_enableMotor = m_linearLimits.m_enableMotor[i];
    limot.m_hiLimit = m_linearLimits.m_upperLimit.getElement(i);
    limot.m_limitSoftness = m_linearLimits.m_limitSoftness;
    limot.m_loLimit = m_linearLimits.m_lowerLimit.getElement(i);
    limot.m_maxLimitForce = (0.f);
    limot.m_maxMotorForce = m_linearLimits.m_maxMotorForce.getElement(i);
    limot.m_targetVelocity = m_linearLimits.m_targetVelocity.getElement(i);
    final btVector3 axis = m_calculatedTransformA.getBasisColumn(i);
    int flags = m_flags >>> (i * BT_6DOF_FLAGS_AXIS_SHIFT);
    limot.m_normalCFM = (flags & BT_6DOF_FLAGS_CFM_NORM) != 0 ? m_linearLimits.m_normalCFM
     .getElement(i) : info.cfm[0].get();
    limot.m_stopCFM =
     (flags & BT_6DOF_FLAGS_CFM_STOP) != 0 ? m_linearLimits.m_stopCFM.getElement(i) : info.cfm[0]
      .get();
    limot.m_stopERP =
     (flags & BT_6DOF_FLAGS_ERP_STOP) != 0 ? m_linearLimits.m_stopERP.getElement(i) : info.erp;
    if (m_useOffsetForConstraintFrame) {
     int indx1 = (i + 1) % 3;
     int indx2 = (i + 2) % 3;
     int rotAllowed = 1; // rotations around orthos to current axis
     if (m_angularLimits[indx1].m_currentLimit != 0 && m_angularLimits[indx2].m_currentLimit != 0) {
      rotAllowed = 0;
     }
     do_row +=
      get_limit_motor_info2(limot, transA, transB, linVelA, linVelB, angVelA, angVelB, info,
       do_row, axis, 0, rotAllowed);
    } else {
     do_row +=
      get_limit_motor_info2(limot, transA, transB, linVelA, linVelB, angVelA, angVelB, info,
       do_row, axis, 0);
    }
   }
  }
  return do_row;
 }

 protected btJacobianEntry buildLinearJacobian(
  final btVector3 normalWorld, final btVector3 pivotAInW, final btVector3 pivotBInW) {
  return new btJacobianEntry(
   m_rbA.getCenterOfMassTransform().getBasis().transpose(),
   m_rbB.getCenterOfMassTransform().getBasis().transpose(),
   new btVector3(pivotAInW).sub(m_rbA.getCenterOfMassPosition()),
   new btVector3(pivotBInW).sub(m_rbB.getCenterOfMassPosition()),
   normalWorld,
   m_rbA.getInvInertiaDiagLocal(),
   m_rbA.getInvMass(),
   m_rbB.getInvInertiaDiagLocal(),
   m_rbB.getInvMass());
 }

 protected btJacobianEntry buildAngularJacobian(final btVector3 jointAxisW) {
  return new btJacobianEntry(jointAxisW,
   m_rbA.getCenterOfMassTransform().getBasis().transpose(),
   m_rbB.getCenterOfMassTransform().getBasis().transpose(),
   m_rbA.getInvInertiaDiagLocal(),
   m_rbB.getInvInertiaDiagLocal());
 }

 // tests linear limits
 protected void calculateLinearInfo() {
  m_calculatedLinearDiff.set(m_calculatedTransformB.getOrigin()).sub(m_calculatedTransformA
   .getOrigin());
  m_calculatedTransformA.getBasis().invert().transform(m_calculatedLinearDiff);
  for (int i = 0; i < 3; i++) {
   m_linearLimits.m_currentLinearDiff.setElement(i, m_calculatedLinearDiff.getElement(i));
   m_linearLimits.testLimitValue(i, m_calculatedLinearDiff.getElement(i));
  }
 }

 static float btGetMatrixElem(final btMatrix3x3 mat, int index) {
  int i = index % 3;
  int j = index / 3;
  return mat.getElement(i, j);
 }

 static boolean matrixToEulerXYZ(final btMatrix3x3 mat, final btVector3 xyz) {
  //	// rot =  cy*cz          -cy*sz           sy
  //	//        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
  //	//       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy
  //
  float fi = btGetMatrixElem(mat, 2);
  if (fi < (1.0f)) {
   if (fi > (-1.0f)) {
    xyz.x = btAtan2(-btGetMatrixElem(mat, 5), btGetMatrixElem(mat, 8));
    xyz.y = btAsin(btGetMatrixElem(mat, 2));
    xyz.z = btAtan2(-btGetMatrixElem(mat, 1), btGetMatrixElem(mat, 0));
    return true;
   } else {
    // WARNING.  Not unique.  XA - ZA = -atan2(r10,r11)
    xyz.x = -btAtan2(btGetMatrixElem(mat, 3), btGetMatrixElem(mat, 4));
    xyz.y = -SIMD_HALF_PI;
    xyz.z = (0.0f);
    return false;
   }
  } else {
   // WARNING.  Not unique.  XAngle + ZAngle = atan2(r10,r11)
   xyz.x = btAtan2(btGetMatrixElem(mat, 3), btGetMatrixElem(mat, 4));
   xyz.y = SIMD_HALF_PI;
   xyz.z = 0.0f;
  }
  return false;
 }
 //! calcs the euler angles between the two bodies.

 protected void calculateAngleInfo() {
  final btMatrix3x3 relative_frame = m_calculatedTransformA.getBasis().invert().mul(
   m_calculatedTransformB.getBasis());
  matrixToEulerXYZ(relative_frame, m_calculatedAxisAngleDiff);
  // in euler angle mode we do not actually constrain the angular velocity
  // along the axes axis[0] and axis[2] (although we do use axis[1]) :
  //
  //    to get			constrain w2-w1 along		...not
  //    ------			---------------------		------
  //    d(angle[0])/dt = 0	ax[1] x ax[2]			ax[0]
  //    d(angle[1])/dt = 0	ax[1]
  //    d(angle[2])/dt = 0	ax[0] x ax[1]			ax[2]
  //
  // constraining w2-w1 along an axis 'a' means that a'*(w2-w1)=0.
  // to prove the result for angle[0], write the expression for angle[0] from
  // GetInfo1 then take the derivative. to prove this for angle[2] it is
  // easier to take the euler rate expression for d(angle[2])/dt with respect
  // to the components of w and set that to 0.
  final btVector3 axis0 = m_calculatedTransformB.getBasisColumn(0);
  final btVector3 axis2 = m_calculatedTransformA.getBasisColumn(2);
  m_calculatedAxis[1].set(axis2).cross(axis0);
  m_calculatedAxis[0].set(m_calculatedAxis[1]).cross(axis2);
  m_calculatedAxis[2].set(axis0).cross(m_calculatedAxis[1]);
  m_calculatedAxis[0].normalize();
  m_calculatedAxis[1].normalize();
  m_calculatedAxis[2].normalize();
 }
 ///for backwards compatibility during the transition to 'getInfo/getInfo2'
 public boolean m_useSolveConstraintObsolete;
 static final boolean D6_USE_OBSOLETE_METHOD = false;
 static final boolean D6_USE_FRAME_OFFSET = true;

 public btGeneric6DofConstraint(btRigidBody rbA, btRigidBody rbB, final btTransform frameInA,
  final btTransform frameInB, boolean useLinearReferenceFrameA) {
  this(D6_CONSTRAINT_TYPE, rbA, rbB, frameInA, frameInB, useLinearReferenceFrameA);
 }

 public btGeneric6DofConstraint(int type, btRigidBody rbA, btRigidBody rbB,
  final btTransform frameInA,
  final btTransform frameInB, boolean useLinearReferenceFrameA) {
  super(type, rbA, rbB);
  m_frameInA.set(frameInA);
  m_frameInB.set(frameInB);
  m_useLinearReferenceFrameA = useLinearReferenceFrameA;
  m_useOffsetForConstraintFrame = D6_USE_FRAME_OFFSET;
  m_flags = 0;
  m_useSolveConstraintObsolete = D6_USE_OBSOLETE_METHOD;
  calculateTransforms();
 }

 public btGeneric6DofConstraint(btRigidBody rbB, final btTransform frameInB,
  boolean useLinearReferenceFrameB) {
  this(D6_CONSTRAINT_TYPE, rbB, frameInB, useLinearReferenceFrameB);
 }

 public btGeneric6DofConstraint(int type, btRigidBody rbB, final btTransform frameInB,
  boolean useLinearReferenceFrameB) {
  super(type, getFixedBody(), rbB);
  m_frameInB.set(frameInB);
  m_useLinearReferenceFrameA = useLinearReferenceFrameB;
  m_useOffsetForConstraintFrame = (D6_USE_FRAME_OFFSET);
  m_flags = 0;
  m_useSolveConstraintObsolete = false;
  ///not providing rigidbody A means implicitly using worldspace for body A
  m_frameInA.set(rbB.getCenterOfMassTransform()).mul(m_frameInB);
  calculateTransforms();
 }
 //! Calcs global transform of the offsets

 /*
  * !
  * Calcs the global transform for the joint offset for body A an B, and also calcs the agle
  * differences between the bodies. \sa btGeneric6DofConstraint.getCalculatedTransformA ,
  * btGeneric6DofConstraint.getCalculatedTransformB, btGeneric6DofConstraint.calculateAngleInfo
  */
 public final void calculateTransforms(final btTransform transA, final btTransform transB) {
  m_calculatedTransformA.set(transA).mul(m_frameInA);
  m_calculatedTransformB.set(transB).mul(m_frameInB);
  calculateLinearInfo();
  calculateAngleInfo();
  if (m_useOffsetForConstraintFrame) {	//  get weight factors depending on masses
   float miA = getRigidBodyA().getInvMass();
   float miB = getRigidBodyB().getInvMass();
   m_hasStaticBody = (miA < SIMD_EPSILON) || (miB < SIMD_EPSILON);
   float miS = miA + miB;
   if (miS > (0.f)) {
    m_factA = miB / miS;
   } else {
    m_factA = (0.5f);
   }
   m_factB = (1.0f) - m_factA;
  }
 }

 public final void calculateTransforms() {
  calculateTransforms(m_rbA.getCenterOfMassTransformPtr(), m_rbB.getCenterOfMassTransformPtr());
 }

 //! Gets the global transform of the offset for body A
 /*
  * !
  * \sa btGeneric6DofConstraint.getFrameOffsetA, btGeneric6DofConstraint.getFrameOffsetB,
  * btGeneric6DofConstraint.calculateAngleInfo.
  */
 public btTransform getCalculatedTransformA() {
  return new btTransform(m_calculatedTransformA);
 }

 //! Gets the global transform of the offset for body B
 /*
  * !
  * \sa btGeneric6DofConstraint.getFrameOffsetA, btGeneric6DofConstraint.getFrameOffsetB,
  * btGeneric6DofConstraint.calculateAngleInfo.
  */
 public btTransform getCalculatedTransformB() {
  return new btTransform(m_calculatedTransformB);
 }

 public btTransform getFrameOffsetA() {
  return new btTransform(m_frameInA);
 }

 public btTransform getFrameOffsetB() {
  return new btTransform(m_frameInB);
 }

 //! performs Jacobian calculation, and also calculates angle differences and axis
 @Override
 public void buildJacobian() {
  if (m_useSolveConstraintObsolete) {
   /* dead code */
   assert(false);
   /*
   // Clear accumulated impulses for the next simulation step
   m_linearLimits.m_accumulatedImpulse.setZero();
   int i;
   for (i = 0; i < 3; i++) {
    m_angularLimits[i].m_accumulatedImpulse = (0.f);
   }
   //calculates transform
   calculateTransforms(m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform());
   //    btVector3& pivotAInW = m_calculatedTransformA.getOrigin();
   //    btVector3& pivotBInW = m_calculatedTransformB.getOrigin();
   calcAnchorPos();
   final btVector3 pivotAInW = new btVector3(m_AnchorPos);
   final btVector3 pivotBInW = new btVector3(m_AnchorPos);
   // not used here
   //    btVector3 rel_pos1 = pivotAInW - m_rbA.getCenterOfMassPosition();
   //    btVector3 rel_pos2 = pivotBInW - m_rbB.getCenterOfMassPosition();
   final btVector3 normalWorld = new btVector3();
   //linear part
   for (i = 0; i < 3; i++) {
    if (m_linearLimits.isLimited(i)) {
     if (m_useLinearReferenceFrameA) {
      normalWorld.set(m_calculatedTransformA.getBasisColumn(i));
     } else {
      normalWorld.set(m_calculatedTransformB.getBasisColumn(i));
     }
     m_jacLinear[i] = buildLinearJacobian(
      normalWorld,
      pivotAInW, pivotBInW);
    }
   }
   // angular part
   for (i = 0; i < 3; i++) {
    //calculates error angle
    if (testAngularLimitMotor(i)) {
     normalWorld.set(getAxis(i));
     // Create angular atom
     m_jacAng[i] = buildAngularJacobian(normalWorld);
    }
   }
   */
  }
 }

 @Override
 public void getInfo1(btConstraintInfo1 info) {
  if (m_useSolveConstraintObsolete) {
   /* dead code */
   assert(false);
    /*
   info.m_numConstraintRows = 0;
   info.nub = 0;
    */
  } else {
   //prepare constraint
   calculateTransforms(m_rbA.getCenterOfMassTransformPtr(), m_rbB.getCenterOfMassTransformPtr());
   info.m_numConstraintRows = 0;
   info.nub = 6;
   int i;
   //test linear limits
   for (i = 0; i < 3; i++) {
    if (m_linearLimits.needApplyForce(i)) {
     info.m_numConstraintRows++;
     info.nub--;
    }
   }
   //test angular limits
   for (i = 0; i < 3; i++) {
    if (testAngularLimitMotor(i)) {
     info.m_numConstraintRows++;
     info.nub--;
    }
   }
  }
 }

 public void getInfo1NonVirtual(btConstraintInfo1 info) {
  if (m_useSolveConstraintObsolete) {
   /* dead code */
   assert(false);
    /*
   info.m_numConstraintRows = 0;
   info.nub = 0;
   */
  } else {
   //pre-allocate all 6
   info.m_numConstraintRows = 6;
   info.nub = 0;
  }
 }

 @Override
 public void getInfo2(btConstraintInfo2 info) {
  assert (!m_useSolveConstraintObsolete);
  final btTransform transA = m_rbA.getCenterOfMassTransformPtr();
  final btTransform transB = m_rbB.getCenterOfMassTransformPtr();
  final btVector3 linVelA = m_rbA.getLinearVelocityPtr();
  final btVector3 linVelB = m_rbB.getLinearVelocityPtr();
  final btVector3 angVelA = m_rbA.getAngularVelocityPtr();
  final btVector3 angVelB = m_rbB.getAngularVelocityPtr();
  if (m_useOffsetForConstraintFrame) { // for stability better to solve angular limits first
   int row = setAngularLimits(info, 0, transA, transB, linVelA, linVelB, angVelA, angVelB);
   setLinearLimits(info, row, transA, transB, linVelA, linVelB, angVelA, angVelB);
  } else { // leave old version for compatibility
   int row = setLinearLimits(info, 0, transA, transB, linVelA, linVelB, angVelA, angVelB);
   setAngularLimits(info, row, transA, transB, linVelA, linVelB, angVelA, angVelB);
  }
 }

 public void getInfo2NonVirtual(btConstraintInfo2 info, final btTransform transA,
  final btTransform transB, final btVector3 linVelA, final btVector3 linVelB,
  final btVector3 angVelA, final btVector3 angVelB) {
  assert (!m_useSolveConstraintObsolete);
  //prepare constraint
  calculateTransforms(transA, transB);
  int i;
  for (i = 0; i < 3; i++) {
   testAngularLimitMotor(i);
  }
  if (m_useOffsetForConstraintFrame) { // for stability better to solve angular limits first
   int row = setAngularLimits(info, 0, transA, transB, linVelA, linVelB, angVelA, angVelB);
   setLinearLimits(info, row, transA, transB, linVelA, linVelB, angVelA, angVelB);
  } else { // leave old version for compatibility
   int row = setLinearLimits(info, 0, transA, transB, linVelA, linVelB, angVelA, angVelB);
   setAngularLimits(info, row, transA, transB, linVelA, linVelB, angVelA, angVelB);
  }
 }

 public void updateRHS(float timeStep) {
 }

 //! Get the rotation axis in global coordinates
 /*
  * !
  * \pre btGeneric6DofConstraint.buildJacobian must be called previously.
  */
 public btVector3 getAxis(int axis_index) {
  return new btVector3(m_calculatedAxis[axis_index]);
 }

 //! Get the relative Euler angle
 /*
  * !
  * \pre btGeneric6DofConstraint.calculateTransforms() must be called previously.
  */
 public float getAngle(int axis_index) {
  return m_calculatedAxisAngleDiff.getElement(axis_index);
 }

 //! Get the relative position of the constraint pivot
 /*
  * !
  * \pre btGeneric6DofConstraint.calculateTransforms() must be called previously.
  */
 public float getRelativePivotPosition(int axis_index) {
  return m_calculatedLinearDiff.getElement(axis_index);
 }

 public void setFrames(final btTransform frameA, final btTransform frameB) {
  m_frameInA.set(frameA);
  m_frameInB.set(frameB);
  buildJacobian();
  calculateTransforms();
 }

 //! Test angular limit.
 /*
  * !
  * Calculates angular correction and returns true if limit needs to be corrected. \pre
  * btGeneric6DofConstraint.calculateTransforms() must be called previously.
  */
 public boolean testAngularLimitMotor(int axis_index) {
  float angle = m_calculatedAxisAngleDiff.getElement(axis_index);
  angle = btAdjustAngleToLimits(angle, m_angularLimits[axis_index].m_loLimit,
   m_angularLimits[axis_index].m_hiLimit);
  m_angularLimits[axis_index].m_currentPosition = angle;
  //test limits
  m_angularLimits[axis_index].testLimitValue(angle);
  return m_angularLimits[axis_index].needApplyTorques();
 }

 final public void setLinearLowerLimit(final btVector3 linearLower) {
  m_linearLimits.m_lowerLimit.set(linearLower);
 }

 final public void getLinearLowerLimit(final btVector3 linearLower) {
  linearLower.set(m_linearLimits.m_lowerLimit);
 }

 final public void setLinearUpperLimit(final btVector3 linearUpper) {
  m_linearLimits.m_upperLimit.set(linearUpper);
 }

 final public void getLinearUpperLimit(final btVector3 linearUpper) {
  linearUpper.set(m_linearLimits.m_upperLimit);
 }

 final public void setAngularLowerLimit(final btVector3 angularLower) {
  for (int i = 0; i < 3; i++) {
   m_angularLimits[i].m_loLimit = btNormalizeAngle(angularLower.getElement(i));
  }
 }

 final public void getAngularLowerLimit(final btVector3 angularLower) {
  for (int i = 0; i < 3; i++) {
   angularLower.setElement(i, m_angularLimits[i].m_loLimit);
  }
 }

 final public void setAngularUpperLimit(final btVector3 angularUpper) {
  for (int i = 0; i < 3; i++) {
   m_angularLimits[i].m_hiLimit = btNormalizeAngle(angularUpper.getElement(i));
  }
 }

 final public void getAngularUpperLimit(final btVector3 angularUpper) {
  for (int i = 0; i < 3; i++) {
   angularUpper.setElement(i, m_angularLimits[i].m_hiLimit);
  }
 }

 //! Retrieves the angular limit informacion
 public btRotationalLimitMotor getRotationalLimitMotor(int index) {
  return m_angularLimits[index];
 }

 //! Retrieves the  limit informacion
 public btTranslationalLimitMotor getTranslationalLimitMotor() {
  return m_linearLimits;
 }

 //first 3 are linear, next 3 are angular
 public void setLimit(int axis, float lo, float hi) {
  if (axis < 3) {
   m_linearLimits.m_lowerLimit.setElement(axis, lo);
   m_linearLimits.m_upperLimit.setElement(axis, hi);
  } else {
   float _lo = btNormalizeAngle(lo);
   float _hi = btNormalizeAngle(hi);
   m_angularLimits[axis - 3].m_loLimit = _lo;
   m_angularLimits[axis - 3].m_hiLimit = _hi;
  }
 }

 //! Test limit
 /*
  * !
  * - free means upper < lower, - locked means upper == lower - limited means upper > lower -
  * limitIndex: first 3 are linear, next 3 are angular
  */
 public boolean isLimited(int limitIndex) {
  if (limitIndex < 3) {
   return m_linearLimits.isLimited(limitIndex);
  }
  return m_angularLimits[limitIndex - 3].isLimited();
 }

 public void calcAnchorPos() {
  float imA = m_rbA.getInvMass();
  float imB = m_rbB.getInvMass();
  float weight;
  if (imB == (0.0f)) {
   weight = (1.0f);
  } else {
   weight = imA / (imA + imB);
  }
  final btVector3 pA = m_calculatedTransformA.getOrigin();
  final btVector3 pB = m_calculatedTransformB.getOrigin();
  m_AnchorPos.mix(pA, pB, weight); // pA * weight + pB * ( (1.0f) - weight);
 }

 public int get_limit_motor_info2(btRotationalLimitMotor limot, final btTransform transA,
  final btTransform transB, final btVector3 linVelA, final btVector3 linVelB,
  final btVector3 angVelA, final btVector3 angVelB,
  btConstraintInfo2 info, int row, final btVector3 ax1, int rotational) {
  return get_limit_motor_info2(limot, transA, transB, linVelA, linVelB, angVelA, angVelB, info, row,
   ax1, rotational, 0);
 }

 public int get_limit_motor_info2(btRotationalLimitMotor limot, final btTransform transA,
  final btTransform transB, final btVector3 linVelA, final btVector3 linVelB,
  final btVector3 angVelA, final btVector3 angVelB,
  btConstraintInfo2 info, int row, final btVector3 ax1, int rotational, int rotAllowed) {
  int srow = row * info.rowskip;
  boolean powered = limot.m_enableMotor;
  int limit = limot.m_currentLimit;
  if (powered || limit != 0) {   // if the joint is powered, or has joint limits, add in the extra row
   btVector3[] J1 = rotational != 0 ? info.m_J1angularAxis : info.m_J1linearAxis;
   btVector3[] J2 = rotational != 0 ? info.m_J2angularAxis : info.m_J2linearAxis;
   J1[srow].set(ax1);
   J2[srow].set(ax1).negate();
   if ((0 == rotational)) {
    if (m_useOffsetForConstraintFrame) {
     final btVector3 tmpA = new btVector3();
     final btVector3 tmpB = new btVector3();
     final btVector3 relA = new btVector3();
     final btVector3 relB = new btVector3();
     // get vector from bodyB to frameB in WCS
     relB.set(m_calculatedTransformB.getOrigin()).sub(transB.getOrigin());
     // get its projection to constraint axis
     final btVector3 projB = new btVector3(ax1).scale(relB.dot(ax1));
     // get vector directed from bodyB to constraint axis (and orthogonal to it)
     final btVector3 orthoB = new btVector3(relB).sub(projB);
     // same for bodyA
     relA.set(m_calculatedTransformA.getOrigin()).sub(transA.getOrigin());
     final btVector3 projA = new btVector3(ax1).scale(relA.dot(ax1));
     final btVector3 orthoA = new btVector3(relA).sub(projA);
     // get desired offset between frames A and B along constraint axis
     float desiredOffs = limot.m_currentPosition - limot.m_currentLimitError;
     // desired vector from projection of center of bodyA to projection of center of bodyB to constraint axis
     final btVector3 totalDist = new btVector3(ax1).scale(desiredOffs).sub(projB).add(projA);
     // get offset vectors relA and relB
     relA.scaleAdd(m_factA, totalDist, orthoA);
     relB.scaleAdd(-m_factB, totalDist, orthoB);
     tmpA.set(relA).cross(ax1);
     tmpB.set(relB).cross(ax1);
     if (m_hasStaticBody && (0 == rotAllowed)) {
      tmpA.scale(m_factA);
      tmpB.scale(m_factB);
     }
     info.m_J1angularAxis[srow].set(tmpA);
     info.m_J2angularAxis[srow].set(tmpB).negate();
    } else {
     final btVector3 ltd = new btVector3();	// Linear Torque Decoupling vector
     final btVector3 c = m_calculatedTransformB.getOrigin().sub(transA.getOrigin());
     ltd.set(c).cross(ax1);
     info.m_J1angularAxis[srow].set(ltd);
     c.set(m_calculatedTransformB.getOrigin()).sub(transB.getOrigin());
     ltd.set(c).cross(ax1).negate();
     info.m_J2angularAxis[srow].set(ltd);
    }
   }
   // if we're limited low and high simultaneously, the joint motor is
   // ineffective
   if (limit != 0 && (limot.m_loLimit == limot.m_hiLimit)) {
    powered = false;
   }
   info.m_constraintError[srow].set(0.f);
   if (powered) {
    info.cfm[srow].set(limot.m_normalCFM);
    if (0 == limit) {
     float tag_vel = rotational != 0 ? limot.m_targetVelocity : -limot.m_targetVelocity;
     float mot_fact = getMotorFactor(limot.m_currentPosition,
      limot.m_loLimit,
      limot.m_hiLimit,
      tag_vel,
      info.fps * limot.m_stopERP);
     info.m_constraintError[srow].plusEquals(mot_fact * limot.m_targetVelocity);
     info.m_lowerLimit[srow].set(-limot.m_maxMotorForce);
     info.m_upperLimit[srow].set(limot.m_maxMotorForce);
    }
   }
   if (limit != 0) {
    float k = info.fps * limot.m_stopERP;
    if (0 == rotational) {
     info.m_constraintError[srow].plusEquals(k * limot.m_currentLimitError);
    } else {
     info.m_constraintError[srow].plusEquals(-k * limot.m_currentLimitError);
    }
    info.cfm[srow].set(limot.m_stopCFM);
    if (limot.m_loLimit == limot.m_hiLimit) {   // limited low and high simultaneously
     info.m_lowerLimit[srow].set(-SIMD_INFINITY);
     info.m_upperLimit[srow].set(SIMD_INFINITY);
    } else {
     if (limit == 1) {
      info.m_lowerLimit[srow].set(0);
      info.m_upperLimit[srow].set(SIMD_INFINITY);
     } else {
      info.m_lowerLimit[srow].set(-SIMD_INFINITY);
      info.m_upperLimit[srow].set(0);
     }
     // deal with bounce
     if (limot.m_bounce > 0) {
      // calculate joint velocity
      float vel;
      if (rotational != 0) {
       vel = angVelA.dot(ax1);
//make sure that if no body . angVelB == zero vec
//                        if (body1)
       vel -= angVelB.dot(ax1);
      } else {
       vel = linVelA.dot(ax1);
//make sure that if no body . angVelB == zero vec
//                        if (body1)
       vel -= linVelB.dot(ax1);
      }
      // only apply bounce if the velocity is incoming, and if the
      // resulting c[] exceeds what we already have.
      if (limit == 1) {
       if (vel < 0) {
        float newc = -limot.m_bounce * vel;
        if (newc > info.m_constraintError[srow].get()) {
         info.m_constraintError[srow].set(newc);
        }
       }
      } else if (vel > 0) {
       float newc = -limot.m_bounce * vel;
       if (newc < info.m_constraintError[srow].get()) {
        info.m_constraintError[srow].set(newc);
       }
      }
     }
    }
   }
   return 1;
  } else {
   return 0;
  }
 }

 // access for UseFrameOffset
 public boolean getUseFrameOffset() {
  return m_useOffsetForConstraintFrame;
 }

 public void setUseFrameOffset(boolean frameOffsetOnOff) {
  m_useOffsetForConstraintFrame = frameOffsetOnOff;
 }

 boolean getUseLinearReferenceFrameA() {
  return m_useLinearReferenceFrameA;
 }

 void setUseLinearReferenceFrameA(boolean linearReferenceFrameA) {
  m_useLinearReferenceFrameA = linearReferenceFrameA;
 }

 ///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
 ///If no axis is provided, it uses the default axis for this constraint.
 @Override
 public void setParam(int num, float value, int axis) {
  if ((axis >= 0) && (axis < 3)) {
   switch (num) {
    case BT_CONSTRAINT_STOP_ERP:
     m_linearLimits.m_stopERP.setElement(axis, value);
     m_flags |= BT_6DOF_FLAGS_ERP_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT);
     break;
    case BT_CONSTRAINT_STOP_CFM:
     m_linearLimits.m_stopCFM.setElement(axis, value);
     m_flags |= BT_6DOF_FLAGS_CFM_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT);
     break;
    case BT_CONSTRAINT_CFM:
     m_linearLimits.m_normalCFM.setElement(axis, value);
     m_flags |= BT_6DOF_FLAGS_CFM_NORM << (axis * BT_6DOF_FLAGS_AXIS_SHIFT);
     break;
    default:
     assert (false);
   }
  } else if ((axis >= 3) && (axis < 6)) {
   switch (num) {
    case BT_CONSTRAINT_STOP_ERP:
     m_angularLimits[axis - 3].m_stopERP = value;
     m_flags |= BT_6DOF_FLAGS_ERP_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT);
     break;
    case BT_CONSTRAINT_STOP_CFM:
     m_angularLimits[axis - 3].m_stopCFM = value;
     m_flags |= BT_6DOF_FLAGS_CFM_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT);
     break;
    case BT_CONSTRAINT_CFM:
     m_angularLimits[axis - 3].m_normalCFM = value;
     m_flags |= BT_6DOF_FLAGS_CFM_NORM << (axis * BT_6DOF_FLAGS_AXIS_SHIFT);
     break;
    default:
     assert (false);
   }
  } else {
   assert (false);
  }
 }

 ///return the local value of parameter
 @Override
 public float getParam(int num, int axis) {
  float retVal = 0;
  if ((axis >= 0) && (axis < 3)) {
   switch (num) {
    case BT_CONSTRAINT_STOP_ERP:
     assert ((m_flags & (BT_6DOF_FLAGS_ERP_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT))) != 0);
     retVal = m_linearLimits.m_stopERP.getElement(axis);
     break;
    case BT_CONSTRAINT_STOP_CFM:
     assert ((m_flags & (BT_6DOF_FLAGS_CFM_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT))) != 0);
     retVal = m_linearLimits.m_stopCFM.getElement(axis);
     break;
    case BT_CONSTRAINT_CFM:
     assert ((m_flags & (BT_6DOF_FLAGS_CFM_NORM << (axis * BT_6DOF_FLAGS_AXIS_SHIFT))) != 0);
     retVal = m_linearLimits.m_normalCFM.getElement(axis);
     break;
    default:
     assert (false);
   }
  } else if ((axis >= 3) && (axis < 6)) {
   switch (num) {
    case BT_CONSTRAINT_STOP_ERP:
     assert ((m_flags & (BT_6DOF_FLAGS_ERP_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT))) != 0);
     retVal = m_angularLimits[axis - 3].m_stopERP;
     break;
    case BT_CONSTRAINT_STOP_CFM:
     assert ((m_flags & (BT_6DOF_FLAGS_CFM_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT))) != 0);
     retVal = m_angularLimits[axis - 3].m_stopCFM;
     break;
    case BT_CONSTRAINT_CFM:
     assert ((m_flags & (BT_6DOF_FLAGS_CFM_NORM << (axis * BT_6DOF_FLAGS_AXIS_SHIFT))) != 0);
     retVal = m_angularLimits[axis - 3].m_normalCFM;
     break;
    default:
     assert (false);
   }
  } else {
   assert (false);
  }
  return retVal;
 }

 public void setAxis(final btVector3 axis1, final btVector3 axis2) {
  final btVector3 zAxis = new btVector3(axis1).normalize();
  final btVector3 yAxis = new btVector3(axis2).normalize();
  final btVector3 xAxis = new btVector3(yAxis).cross(zAxis); // we want right coordinate system
  final btTransform frameInW = new btTransform();
  frameInW.setIdentity();
  frameInW.set3x3(new btMatrix3x3(xAxis.x, yAxis.x, zAxis.x,
   xAxis.y, yAxis.y, zAxis.y,
   xAxis.z, yAxis.z, zAxis.z));
  // now get constraint frame in local coordinate systems
  m_frameInA.set(m_rbA.getCenterOfMassTransform().invert()).mul(frameInW);
  m_frameInB.set(m_rbB.getCenterOfMassTransform().invert()).mul(frameInW);
  calculateTransforms();
 }

 protected int getFlags() {
  return m_flags;
 }
};
