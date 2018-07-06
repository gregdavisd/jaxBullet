/*
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it freely,
 * subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

 /*
 * Added by Roman Ponomarev (rponom@gmail.com)
 * April 04, 2008
 *
 * TODO:
 * - add clamping od accumulated impulse to improve stability
 * - add conversion for ODE constraint solver
 */
package Bullet.Dynamics.Constraint;

import Bullet.Dynamics.CollisionObjects.btRigidBody;
import Bullet.Dynamics.btJacobianEntry;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.SIMD_INFINITY;
import static Bullet.LinearMath.btScalar.btAtan2;
import static Bullet.LinearMath.btScalar.btFabs;
import static Bullet.LinearMath.btScalar.btNormalizeAngle;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.btPlaneSpace1;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btSliderConstraint extends btTypedConstraint implements
 Serializable {

 public static final int BT_SLIDER_FLAGS_CFM_DIRLIN = (1);
 public static final int BT_SLIDER_FLAGS_ERP_DIRLIN = (1 << 1);
 public static final int BT_SLIDER_FLAGS_CFM_DIRANG = (1 << 2);
 public static final int BT_SLIDER_FLAGS_ERP_DIRANG = (1 << 3);
 public static final int BT_SLIDER_FLAGS_CFM_ORTLIN = (1 << 4);
 public static final int BT_SLIDER_FLAGS_ERP_ORTLIN = (1 << 5);
 public static final int BT_SLIDER_FLAGS_CFM_ORTANG = (1 << 6);
 public static final int BT_SLIDER_FLAGS_ERP_ORTANG = (1 << 7);
 public static final int BT_SLIDER_FLAGS_CFM_LIMLIN = (1 << 8);
 public static final int BT_SLIDER_FLAGS_ERP_LIMLIN = (1 << 9);
 public static final int BT_SLIDER_FLAGS_CFM_LIMANG = (1 << 10);
 public static final int BT_SLIDER_FLAGS_ERP_LIMANG = (1 << 11);
 public static final float SLIDER_CONSTRAINT_DEF_SOFTNESS = ((1.0f));
 public static final float SLIDER_CONSTRAINT_DEF_DAMPING = ((1.0f));
 public static final float SLIDER_CONSTRAINT_DEF_RESTITUTION = ((0.7f));
 public static final float SLIDER_CONSTRAINT_DEF_CFM = ((0.f));
 public static final boolean USE_OFFSET_FOR_CONSTANT_FRAME = true;
 ///for backwards compatibility during the transition to 'getInfo/getInfo2'
 boolean m_useSolveConstraintObsolete;
 boolean m_useOffsetForConstraintFrame;
 final btTransform m_frameInA = new btTransform();
 final btTransform m_frameInB = new btTransform();
 // use frameA fo define limits, if true
 boolean m_useLinearReferenceFrameA;
 // linear limits
 float m_lowerLinLimit;
 float m_upperLinLimit;
 // angular limits
 float m_lowerAngLimit;
 float m_upperAngLimit;
 // softness, restitution and damping for different cases
 // DirLin - moving inside linear limits
 // LimLin - hitting linear limit
 // DirAng - moving inside angular limits
 // LimAng - hitting angular limit
 // OrthoLin, OrthoAng - against constraint axis
 float m_softnessDirLin;
 float m_restitutionDirLin;
 float m_dampingDirLin;
 float m_cfmDirLin;
 float m_softnessDirAng;
 float m_restitutionDirAng;
 float m_dampingDirAng;
 float m_cfmDirAng;
 float m_softnessLimLin;
 float m_restitutionLimLin;
 float m_dampingLimLin;
 float m_cfmLimLin;
 float m_softnessLimAng;
 float m_restitutionLimAng;
 float m_dampingLimAng;
 float m_cfmLimAng;
 float m_softnessOrthoLin;
 float m_restitutionOrthoLin;
 float m_dampingOrthoLin;
 float m_cfmOrthoLin;
 float m_softnessOrthoAng;
 float m_restitutionOrthoAng;
 float m_dampingOrthoAng;
 float m_cfmOrthoAng;
 // for interlal use
 boolean m_solveLinLim;
 boolean m_solveAngLim;
 int m_flags;
 final btJacobianEntry[] m_jacLin = {new btJacobianEntry(),
  new btJacobianEntry(),
  new btJacobianEntry()};
 final float[] m_jacLinDiagABInv = new float[3];
 final btJacobianEntry[] m_jacAng = {new btJacobianEntry(),
  new btJacobianEntry(),
  new btJacobianEntry()};
 ;

	float m_timeStep;
 final btTransform m_calculatedTransformA = new btTransform();
 final btTransform m_calculatedTransformB = new btTransform();
 final btVector3 m_sliderAxis = new btVector3();
 final btVector3 m_realPivotAInW = new btVector3();
 final btVector3 m_realPivotBInW = new btVector3();
 final btVector3 m_projPivotInW = new btVector3();
 final btVector3 m_delta = new btVector3();
 final btVector3 m_depth = new btVector3();
 final btVector3 m_relPosA = new btVector3();
 final btVector3 m_relPosB = new btVector3();
 float m_linPos;
 float m_angPos;
 float m_angDepth;
 float m_kAngle;
 boolean m_poweredLinMotor;
 float m_targetLinMotorVelocity;
 float m_maxLinMotorForce;
 float m_accumulatedLinMotorImpulse;
 boolean m_poweredAngMotor;
 float m_targetAngMotorVelocity;
 float m_maxAngMotorForce;
 float m_accumulatedAngMotorImpulse;

 //------------------------    
 final void initParams() {
  m_lowerLinLimit = (1.0f);
  m_upperLinLimit = (-1.0f);
  m_lowerAngLimit = (0.f);
  m_upperAngLimit = (0.f);
  m_softnessDirLin = SLIDER_CONSTRAINT_DEF_SOFTNESS;
  m_restitutionDirLin = SLIDER_CONSTRAINT_DEF_RESTITUTION;
  m_dampingDirLin = (0.f);
  m_cfmDirLin = SLIDER_CONSTRAINT_DEF_CFM;
  m_softnessDirAng = SLIDER_CONSTRAINT_DEF_SOFTNESS;
  m_restitutionDirAng = SLIDER_CONSTRAINT_DEF_RESTITUTION;
  m_dampingDirAng = (0.f);
  m_cfmDirAng = SLIDER_CONSTRAINT_DEF_CFM;
  m_softnessOrthoLin = SLIDER_CONSTRAINT_DEF_SOFTNESS;
  m_restitutionOrthoLin = SLIDER_CONSTRAINT_DEF_RESTITUTION;
  m_dampingOrthoLin = SLIDER_CONSTRAINT_DEF_DAMPING;
  m_cfmOrthoLin = SLIDER_CONSTRAINT_DEF_CFM;
  m_softnessOrthoAng = SLIDER_CONSTRAINT_DEF_SOFTNESS;
  m_restitutionOrthoAng = SLIDER_CONSTRAINT_DEF_RESTITUTION;
  m_dampingOrthoAng = SLIDER_CONSTRAINT_DEF_DAMPING;
  m_cfmOrthoAng = SLIDER_CONSTRAINT_DEF_CFM;
  m_softnessLimLin = SLIDER_CONSTRAINT_DEF_SOFTNESS;
  m_restitutionLimLin = SLIDER_CONSTRAINT_DEF_RESTITUTION;
  m_dampingLimLin = SLIDER_CONSTRAINT_DEF_DAMPING;
  m_cfmLimLin = SLIDER_CONSTRAINT_DEF_CFM;
  m_softnessLimAng = SLIDER_CONSTRAINT_DEF_SOFTNESS;
  m_restitutionLimAng = SLIDER_CONSTRAINT_DEF_RESTITUTION;
  m_dampingLimAng = SLIDER_CONSTRAINT_DEF_DAMPING;
  m_cfmLimAng = SLIDER_CONSTRAINT_DEF_CFM;
  m_poweredLinMotor = false;
  m_targetLinMotorVelocity = (0.f);
  m_maxLinMotorForce = (0.f);
  m_accumulatedLinMotorImpulse = (0.0f);
  m_poweredAngMotor = false;
  m_targetAngMotorVelocity = (0.f);
  m_maxAngMotorForce = (0.f);
  m_accumulatedAngMotorImpulse = (0.0f);
  m_flags = 0;
  m_flags = 0;
  m_useOffsetForConstraintFrame = USE_OFFSET_FOR_CONSTANT_FRAME;
  calculateTransforms(m_rbA.getCenterOfMassTransform(), m_rbB
   .getCenterOfMassTransform());
 }
 // constructors

 public btSliderConstraint(btRigidBody rbA, btRigidBody rbB,
  final btTransform frameInA,
  final btTransform frameInB, boolean useLinearReferenceFrameA) {
  super(SLIDER_CONSTRAINT_TYPE, rbA, rbB);
  m_useSolveConstraintObsolete = false;
  m_frameInA.set(frameInA);
  m_frameInB.set(frameInB);
  m_useLinearReferenceFrameA = useLinearReferenceFrameA;
  initParams();
 }

 public btSliderConstraint(btRigidBody rbB, final btTransform frameInB,
  boolean useLinearReferenceFrameA) {
  super(SLIDER_CONSTRAINT_TYPE, getFixedBody(), rbB);
  m_useSolveConstraintObsolete = false;
  m_frameInB.set(frameInB);
  m_useLinearReferenceFrameA = useLinearReferenceFrameA;
  ///not providing rigidbody A means implicitly using worldspace for body A
  m_frameInA.set(rbB.getCenterOfMassTransform().mul(m_frameInB));
//	m_frameInA.getOrigin() = m_rbA.getCenterOfMassTransform()(m_frameInA.getOrigin());
  initParams();
 }
 // overrides

 @Override
 public void getInfo1(btConstraintInfo1 info) {
  if (m_useSolveConstraintObsolete) {
   /*
    * dead code
    */
   assert (false);
   /*
    * info.m_numConstraintRows = 0; info.nub = 0;
    */
  } else {
   info.m_numConstraintRows = 4; // Fixed 2 linear + 2 angular
   info.nub = 2;
   //prepare constraint
   calculateTransforms(m_rbA.getCenterOfMassTransform(), m_rbB
    .getCenterOfMassTransform());
   testAngLimits();
   testLinLimits();
   if (getSolveLinLimit() || getPoweredLinMotor()) {
    info.m_numConstraintRows++; // limit 3rd linear as well
    info.nub--;
   }
   if (getSolveAngLimit() || getPoweredAngMotor()) {
    info.m_numConstraintRows++; // limit 3rd angular as well
    info.nub--;
   }
  }
 }

 public void getInfo1NonVirtual(btConstraintInfo1 info) {
  info.m_numConstraintRows = 6; // Fixed 2 linear + 2 angular + 1 limit (even if not used)
  info.nub = 0;
 }

 @Override
 public void getInfo2(btConstraintInfo2 info) {
  getInfo2NonVirtual(info, m_rbA.getCenterOfMassTransform(), m_rbB
   .getCenterOfMassTransform(), m_rbA
    .getLinearVelocity(), m_rbB.getLinearVelocity(), m_rbA.getInvMass(), m_rbB
   .getInvMass());
//  getInfo2NonVirtual(info, btTransform.getIdentity(), btTransform.getIdentity(), new btVector3(), 
//   new btVector3(), m_rbA.getInvMass(), m_rbB.getInvMass());
 }

 public void getInfo2NonVirtual(btConstraintInfo2 info, final btTransform transA,
  final btTransform transB,
  final btVector3 linVelA, final btVector3 linVelB, float rbAinvMass,
  float rbBinvMass) {
  final btTransform trA = getCalculatedTransformAPtr();
  final btTransform trB = getCalculatedTransformBPtr();
  assert (!m_useSolveConstraintObsolete);
  int i, s = info.rowskip;
  float signFact = m_useLinearReferenceFrameA ? (1.0f) : (-1.0f);
  // difference between frames in WCS
  final btVector3 ofs = trB.getOrigin().sub(trA.getOrigin());
  // now get weight factors depending on masses
  float miA = rbAinvMass;
  float miB = rbBinvMass;
  boolean hasStaticBody = (miA < SIMD_EPSILON) || (miB < SIMD_EPSILON);
  float miS = miA + miB;
  float factA, factB;
  if (miS > (0.f)) {
   factA = miB / miS;
  } else {
   factA = (0.5f);
  }
  factB = (1.0f) - factA;
  final btVector3 ax1 = new btVector3();
  final btVector3 p = new btVector3();
  final btVector3 q = new btVector3();
  final btVector3 ax1A = trA.getBasisColumn(0);
  final btVector3 ax1B = trB.getBasisColumn(0);
  if (m_useOffsetForConstraintFrame) {
   // get the desired direction of slider axis
   // as weighted sum of X-orthos of frameA and frameB in WCS
   ax1.set(ax1A).scale(factA).add(new btVector3(ax1B).scale(factB));
   ax1.normalize();
   // construct two orthos to slider axis
   btPlaneSpace1(ax1, p, q);
  } else { // old way - use frameA
   trA.getBasisColumn(0, ax1);
   // get 2 orthos to slider axis (Y, Z)
   trA.getBasisColumn(1, p);
   trA.getBasisColumn(2, q);
  }
  // make rotations around these orthos equal
  // the slider axis should be the only unconstrained
  // rotational axis, the angular velocity of the two bodies perpendicular to
  // the slider axis should be equal. thus the constraint equations are
  //    p*w1 - p*w2 = 0
  //    q*w1 - q*w2 = 0
  // where p and q are unit vectors normal to the slider axis, and w1 and w2
  // are the angular velocity vectors of the two bodies.
  info.m_J1angularAxis[0].set(p);
  info.m_J1angularAxis[s].set(q);
  info.m_J2angularAxis[0].set(p).negate();
  info.m_J2angularAxis[s].set(q).negate();
  // compute the right hand side of the constraint equation. set relative
  // body velocities along p and q to bring the slider back into alignment.
  // if ax1A,ax1B are the unit length slider axes as computed from bodyA and
  // bodyB, we need to rotate both bodies along the axis u = (ax1 x ax2).
  // if "theta" is the angle between ax1 and ax2, we need an angular velocity
  // along u to cover angle erp*theta in one step :
  //   |angular_velocity| = angle/time = erp*theta / stepsize
  //                      = (erp*fps) * theta
  //    angular_velocity  = |angular_velocity| * (ax1 x ax2) / |ax1 x ax2|
  //                      = (erp*fps) * theta * (ax1 x ax2) / sin(theta)
  // ...as ax1 and ax2 are unit length. if theta is smallish,
  // theta ~= sin(theta), so
  //    angular_velocity  = (erp*fps) * (ax1 x ax2)
  // ax1 x ax2 is in the plane space of ax1, so we project the angular
  // velocity to p and q to find the right hand side.
//	float k = info.fps * info.erp * getSoftnessOrthoAng();
  float currERP = (m_flags & BT_SLIDER_FLAGS_ERP_ORTANG) != 0 ? m_softnessOrthoAng
   : m_softnessOrthoAng * info.erp;
  float k = info.fps * currERP;
  final btVector3 u = new btVector3(ax1A).cross(ax1B);
  info.m_constraintError[0].set(k * u.dot(p));
  info.m_constraintError[s].set(k * u.dot(q));
  if ((m_flags & BT_SLIDER_FLAGS_CFM_ORTANG) != 0) {
   info.cfm[0].set(m_cfmOrthoAng);
   info.cfm[s].set(m_cfmOrthoAng);
  }
  int nrow = 1; // last filled row
  int srow;
  float limit_err;
  int limit;
  int powered;
  // next two rows. 
  // we want: velA + wA x relA == velB + wB x relB ... but this would
  // result in three equations, so we project along two orthos to the slider axis
  final btTransform bodyA_trans = new btTransform(transA);
  final btTransform bodyB_trans = new btTransform(transB);
  nrow++;
  int s2 = nrow * s;
  nrow++;
  int s3 = nrow * s;
  final btVector3 tmpA = new btVector3();
  final btVector3 tmpB = new btVector3();
  final btVector3 relA = new btVector3();
  final btVector3 relB = new btVector3();
  final btVector3 c = new btVector3();
  if (m_useOffsetForConstraintFrame) {
   // get vector from bodyB to frameB in WCS
   relB.set(trB.getOrigin()).sub(bodyB_trans.getOrigin());
   // get its projection to slider axis
   final btVector3 projB = new btVector3(ax1).scale(relB.dot(ax1));
   // get vector directed from bodyB to slider axis (and orthogonal to it)
   final btVector3 orthoB = new btVector3(relB).sub(projB);
   // same for bodyA
   relA.set(trA.getOrigin()).sub(bodyA_trans.getOrigin());
   final btVector3 projA = new btVector3(ax1).scale(relA.dot(ax1));
   final btVector3 orthoA = new btVector3(relA).sub(projA);
   // get desired offset between frames A and B along slider axis
   float sliderOffs = m_linPos - m_depth.x;
   // desired vector from projection of center of bodyA to projection of center of bodyB to slider axis
   final btVector3 totalDist = new btVector3(ax1).scale(sliderOffs).add(projA)
    .sub(projB);
   // get offset vectors relA and relB
   relA.scaleAdd(factA, totalDist, orthoA);
   relB.scaleAdd(-factB, totalDist, orthoB);
   // now choose average ortho to slider axis
   p.set(orthoB).scale(factA).add(new btVector3(orthoA).scale(factB));
   float len2 = p.lengthSquared();
   if (len2 > SIMD_EPSILON) {
    p.scale(1.0f / btSqrt(len2));
   } else {
    trA.getBasisColumn(1, p);
   }
   // make one more ortho
   q.set(ax1).cross(p);
   // fill two rows
   tmpA.set(relA).cross(p);
   tmpB.set(relB).cross(p);
   info.m_J1angularAxis[s2].set(tmpA);
   info.m_J2angularAxis[s2].set(tmpB).negate();
   tmpA.set(relA).cross(q);
   tmpB.set(relB).cross(q);
   if (hasStaticBody && getSolveAngLimit()) { // to make constraint between static and dynamic objects more rigid
    // remove wA (or wB) from equation if angular limit is hit
    tmpB.scale(factB);
    tmpA.scale(factA);
   }
   info.m_J1angularAxis[s3].set(tmpA);
   info.m_J2angularAxis[s3].set(tmpB).negate();
   info.m_J1linearAxis[s2].set(p);
   info.m_J1linearAxis[s3].set(q);
   info.m_J2linearAxis[s2].set(p).negate();
   info.m_J2linearAxis[s3].set(q).negate();
  } else {	// old way - maybe incorrect if bodies are not on the slider axis
   // see discussion "Bug in slider constraint" http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=4024&start=0
   c.set(bodyB_trans.getOrigin()).sub(bodyA_trans.getOrigin());
   final btVector3 tmp = new btVector3(c).cross(p);
   info.m_J1angularAxis[s2].set(tmp).scale(factA);
   info.m_J2angularAxis[s2].set(tmp).scale(factB);
   tmp.set(c).cross(q);
   info.m_J1angularAxis[s3].set(tmp).scale(factA);
   info.m_J2angularAxis[s3].set(tmp).scale(factB);
   info.m_J1linearAxis[s2].set(p);
   info.m_J1linearAxis[s3].set(q);
   info.m_J2linearAxis[s2].set(p).negate();
   info.m_J2linearAxis[s3].set(q).negate();
  }
  // compute two elements of right hand side
  //	k = info.fps * info.erp * getSoftnessOrthoLin();
  currERP = (m_flags & BT_SLIDER_FLAGS_ERP_ORTLIN) != 0 ? m_softnessOrthoLin : m_softnessOrthoLin
   * info.erp;
  k = info.fps * currERP;
  float rhs = k * p.dot(ofs);
  info.m_constraintError[s2].set(rhs);
  rhs = k * q.dot(ofs);
  info.m_constraintError[s3].set(rhs);
  if ((m_flags & BT_SLIDER_FLAGS_CFM_ORTLIN) != 0) {
   info.cfm[s2].set(m_cfmOrthoLin);
   info.cfm[s3].set(m_cfmOrthoLin);
  }
  // check linear limits
  limit_err = (0.0f);
  limit = 0;
  if (getSolveLinLimit()) {
   limit_err = getLinDepth() * signFact;
   limit = (limit_err > (0.0f)) ? 2 : 1;
  }
  powered = 0;
  if (getPoweredLinMotor()) {
   powered = 1;
  }
  // if the slider has joint limits or motor, add in the extra row
  if (limit != 0 || powered != 0) {
   nrow++;
   srow = nrow * info.rowskip;
   info.m_J1linearAxis[srow].set(ax1);
   info.m_J2linearAxis[srow].set(ax1).negate();
   // linear torque decoupling step:
   //
   // we have to be careful that the linear constraint forces (+/- ax1) applied to the two bodies
   // do not create a torque couple. in other words, the points that the
   // constraint force is applied at must lie along the same ax1 axis.
   // a torque couple will result in limited slider-jointed free
   // bodies from gaining angular momentum.
   if (m_useOffsetForConstraintFrame) {
    // this is needed only when bodyA and bodyB are both dynamic.
    if (!hasStaticBody) {
     tmpA.set(relA).cross(ax1);
     tmpB.set(relB).cross(ax1);
     info.m_J1angularAxis[srow].set(tmpA);
     info.m_J2angularAxis[srow].set(tmpB).negate();
    }
   } else { // The old way. May be incorrect if bodies are not on the slider axis
    final btVector3 ltd = new btVector3();	// Linear Torque Decoupling vector (a torque)
    ltd.set(c).cross(ax1);
    info.m_J1angularAxis[srow].set(ltd).scale(factA);
    info.m_J2angularAxis[srow].set(ltd).scale(factB);
   }
   // right-hand part
   float lostop = getLowerLinLimit();
   float histop = getUpperLinLimit();
   if (limit != 0 && (lostop == histop)) {  // the joint motor is ineffective
    powered = 0;
   }
   info.m_constraintError[srow].set(0.f);
   info.m_lowerLimit[srow].set(0.f);
   info.m_upperLimit[srow].set(0.f);
   currERP = (m_flags & BT_SLIDER_FLAGS_ERP_LIMLIN) != 0 ? m_softnessLimLin : info.erp;
   if (powered != 0) {
    if ((m_flags & BT_SLIDER_FLAGS_CFM_DIRLIN) != 0) {
     info.cfm[srow].set(m_cfmDirLin);
    }
    float tag_vel = getTargetLinMotorVelocity();
    float mot_fact = getMotorFactor(m_linPos, m_lowerLinLimit, m_upperLinLimit,
     tag_vel, info.fps * currERP);
    info.m_constraintError[srow].plusEquals(-(signFact * mot_fact
     * getTargetLinMotorVelocity()));
    info.m_lowerLimit[srow].plusEquals(-getMaxLinMotorForce() / info.fps);
    info.m_upperLimit[srow].plusEquals(getMaxLinMotorForce() / info.fps);
   }
   if (limit != 0) {
    k = info.fps * currERP;
    info.m_constraintError[srow].plusEquals(k * limit_err);
    if ((m_flags & BT_SLIDER_FLAGS_CFM_LIMLIN) != 0) {
     info.cfm[srow].set(m_cfmLimLin);
    }
    if (lostop == histop) {	// limited low and high simultaneously
     info.m_lowerLimit[srow].set(-SIMD_INFINITY);
     info.m_upperLimit[srow].set(SIMD_INFINITY);
    } else if (limit == 1) { // low limit
     info.m_lowerLimit[srow].set(-SIMD_INFINITY);
     info.m_upperLimit[srow].set(0f);
    } else { // high limit
     info.m_lowerLimit[srow].set(0f);
     info.m_upperLimit[srow].set(SIMD_INFINITY);
    }
    // bounce (we'll use slider parameter abs(1.0 - m_dampingLimLin) for that)
    float bounce = btFabs((1.0f) - getDampingLimLin());
    if (bounce > (0.0f)) {
     float vel = linVelA.dot(ax1);
     vel -= linVelB.dot(ax1);
     vel *= signFact;
     // only apply bounce if the velocity is incoming, and if the
     // resulting c[] exceeds what we already have.
     if (limit == 1) {	// low limit
      if (vel < 0) {
       float newc = -bounce * vel;
       if (newc > info.m_constraintError[srow].get()) {
        info.m_constraintError[srow].set(newc);
       }
      }
     } else if (vel > 0) {
      // high limit - all those computations are reversed
      float newc = -bounce * vel;
      if (newc < info.m_constraintError[srow].get()) {
       info.m_constraintError[srow].set(newc);
      }
     }
    }
    info.m_constraintError[srow].timesEquals(getSoftnessLimLin());
   } // if(limit)
  } // if linear limit
  // check angular limits
  limit_err = (0.0f);
  limit = 0;
  if (getSolveAngLimit()) {
   limit_err = getAngDepth();
   limit = (limit_err > (0.0f)) ? 1 : 2;
  }
  // if the slider has joint limits, add in the extra row
  powered = 0;
  if (getPoweredAngMotor()) {
   powered = 1;
  }
  if (limit != 0 || powered != 0) {
   nrow++;
   srow = nrow * info.rowskip;
   info.m_J1angularAxis[srow].set(ax1);
   info.m_J2angularAxis[srow].set(ax1).negate();
   float lostop = getLowerAngLimit();
   float histop = getUpperAngLimit();
   if (limit != 0 && (lostop == histop)) {  // the joint motor is ineffective
    powered = 0;
   }
   currERP = (m_flags & BT_SLIDER_FLAGS_ERP_LIMANG) != 0 ? m_softnessLimAng : info.erp;
   if (powered != 0) {
    if ((m_flags & BT_SLIDER_FLAGS_CFM_DIRANG) != 0) {
     info.cfm[srow].set(m_cfmDirAng);
    }
    float mot_fact = getMotorFactor(m_angPos, m_lowerAngLimit, m_upperAngLimit,
     getTargetAngMotorVelocity(), info.fps * currERP);
    info.m_constraintError[srow].set(mot_fact * getTargetAngMotorVelocity());
    info.m_lowerLimit[srow].set(-getMaxAngMotorForce() / info.fps);
    info.m_upperLimit[srow].set(getMaxAngMotorForce() / info.fps);
   }
   if (limit != 0) {
    k = info.fps * currERP;
    info.m_constraintError[srow].plusEquals(k * limit_err);
    if ((m_flags & BT_SLIDER_FLAGS_CFM_LIMANG) != 0) {
     info.cfm[srow].set(m_cfmLimAng);
    }
    if (lostop == histop) {
     // limited low and high simultaneously
     info.m_lowerLimit[srow].set(-SIMD_INFINITY);
     info.m_upperLimit[srow].set(SIMD_INFINITY);
    } else if (limit == 1) { // low limit
     info.m_lowerLimit[srow].set(0);
     info.m_upperLimit[srow].set(SIMD_INFINITY);
    } else { // high limit
     info.m_lowerLimit[srow].set(-SIMD_INFINITY);
     info.m_upperLimit[srow].set(0);
    }
    // bounce (we'll use slider parameter abs(1.0 - m_dampingLimAng) for that)
    float bounce = btFabs((1.0f) - getDampingLimAng());
    if (bounce > (0.0f)) {
     float vel = m_rbA.getAngularVelocity().dot(ax1);
     vel -= m_rbB.getAngularVelocity().dot(ax1);
     // only apply bounce if the velocity is incoming, and if the
     // resulting c[] exceeds what we already have.
     if (limit == 1) {	// low limit
      if (vel < 0) {
       float newc = -bounce * vel;
       if (newc > info.m_constraintError[srow].get()) {
        info.m_constraintError[srow].set(newc);
       }
      }
     } else if (vel > 0) {
      // high limit - all those computations are reversed
      float newc = -bounce * vel;
      if (newc < info.m_constraintError[srow].get()) {
       info.m_constraintError[srow].set(newc);
      }
     }
    }
    info.m_constraintError[srow].timesEquals(getSoftnessLimAng());
   } // if(limit)
  } // if angular limit or powered
 }

 // access
 public btTransform getCalculatedTransformA() {
  return new btTransform(m_calculatedTransformA);
 }

 public btTransform getCalculatedTransformB() {
  return new btTransform(m_calculatedTransformB);
 }

 public btTransform getCalculatedTransformAPtr() {
  return m_calculatedTransformA;
 }

 public btTransform getCalculatedTransformBPtr() {
  return m_calculatedTransformB;
 }

 public btTransform getFrameOffsetA() {
  return new btTransform(m_frameInA);
 }

 public btTransform getFrameOffsetB() {
  return new btTransform(m_frameInB);
 }

 public btTransform getFrameOffsetAPtr() {
  return m_frameInA;
 }

 public btTransform getFrameOffsetBPtr() {
  return m_frameInB;
 }

 public float getLowerLinLimit() {
  return m_lowerLinLimit;
 }

 public void setLowerLinLimit(float lowerLimit) {
  m_lowerLinLimit = lowerLimit;
 }

 public float getUpperLinLimit() {
  return m_upperLinLimit;
 }

 public void setUpperLinLimit(float upperLimit) {
  m_upperLinLimit = upperLimit;
 }

 public float getLowerAngLimit() {
  return m_lowerAngLimit;
 }

 public void setLowerAngLimit(float lowerLimit) {
  m_lowerAngLimit = btNormalizeAngle(lowerLimit);
 }

 public float getUpperAngLimit() {
  return m_upperAngLimit;
 }

 public void setUpperAngLimit(float upperLimit) {
  m_upperAngLimit = btNormalizeAngle(upperLimit);
 }

 public boolean getUseLinearReferenceFrameA() {
  return m_useLinearReferenceFrameA;
 }

 public float getSoftnessDirLin() {
  return m_softnessDirLin;
 }

 public float getRestitutionDirLin() {
  return m_restitutionDirLin;
 }

 public float getDampingDirLin() {
  return m_dampingDirLin;
 }

 public float getSoftnessDirAng() {
  return m_softnessDirAng;
 }

 public float getRestitutionDirAng() {
  return m_restitutionDirAng;
 }

 public float getDampingDirAng() {
  return m_dampingDirAng;
 }

 public float getSoftnessLimLin() {
  return m_softnessLimLin;
 }

 public float getRestitutionLimLin() {
  return m_restitutionLimLin;
 }

 public float getDampingLimLin() {
  return m_dampingLimLin;
 }

 public float getSoftnessLimAng() {
  return m_softnessLimAng;
 }

 public float getRestitutionLimAng() {
  return m_restitutionLimAng;
 }

 public float getDampingLimAng() {
  return m_dampingLimAng;
 }

 public float getSoftnessOrthoLin() {
  return m_softnessOrthoLin;
 }

 public float getRestitutionOrthoLin() {
  return m_restitutionOrthoLin;
 }

 public float getDampingOrthoLin() {
  return m_dampingOrthoLin;
 }

 public float getSoftnessOrthoAng() {
  return m_softnessOrthoAng;
 }

 public float getRestitutionOrthoAng() {
  return m_restitutionOrthoAng;
 }

 public float getDampingOrthoAng() {
  return m_dampingOrthoAng;
 }

 public void setSoftnessDirLin(float softnessDirLin) {
  m_softnessDirLin = softnessDirLin;
 }

 public void setRestitutionDirLin(float restitutionDirLin) {
  m_restitutionDirLin = restitutionDirLin;
 }

 public void setDampingDirLin(float dampingDirLin) {
  m_dampingDirLin = dampingDirLin;
 }

 public void setSoftnessDirAng(float softnessDirAng) {
  m_softnessDirAng = softnessDirAng;
 }

 public void setRestitutionDirAng(float restitutionDirAng) {
  m_restitutionDirAng = restitutionDirAng;
 }

 public void setDampingDirAng(float dampingDirAng) {
  m_dampingDirAng = dampingDirAng;
 }

 public void setSoftnessLimLin(float softnessLimLin) {
  m_softnessLimLin = softnessLimLin;
 }

 public void setRestitutionLimLin(float restitutionLimLin) {
  m_restitutionLimLin = restitutionLimLin;
 }

 public void setDampingLimLin(float dampingLimLin) {
  m_dampingLimLin = dampingLimLin;
 }

 public void setSoftnessLimAng(float softnessLimAng) {
  m_softnessLimAng = softnessLimAng;
 }

 public void setRestitutionLimAng(float restitutionLimAng) {
  m_restitutionLimAng = restitutionLimAng;
 }

 public void setDampingLimAng(float dampingLimAng) {
  m_dampingLimAng = dampingLimAng;
 }

 public void setSoftnessOrthoLin(float softnessOrthoLin) {
  m_softnessOrthoLin = softnessOrthoLin;
 }

 public void setRestitutionOrthoLin(float restitutionOrthoLin) {
  m_restitutionOrthoLin = restitutionOrthoLin;
 }

 public void setDampingOrthoLin(float dampingOrthoLin) {
  m_dampingOrthoLin = dampingOrthoLin;
 }

 public void setSoftnessOrthoAng(float softnessOrthoAng) {
  m_softnessOrthoAng = softnessOrthoAng;
 }

 public void setRestitutionOrthoAng(float restitutionOrthoAng) {
  m_restitutionOrthoAng = restitutionOrthoAng;
 }

 public void setDampingOrthoAng(float dampingOrthoAng) {
  m_dampingOrthoAng = dampingOrthoAng;
 }

 public void setPoweredLinMotor(boolean onOff) {
  m_poweredLinMotor = onOff;
 }

 public boolean getPoweredLinMotor() {
  return m_poweredLinMotor;
 }

 public void setTargetLinMotorVelocity(float targetLinMotorVelocity) {
  m_targetLinMotorVelocity = targetLinMotorVelocity;
 }

 public float getTargetLinMotorVelocity() {
  return m_targetLinMotorVelocity;
 }

 public void setMaxLinMotorForce(float maxLinMotorForce) {
  m_maxLinMotorForce = maxLinMotorForce;
 }

 public float getMaxLinMotorForce() {
  return m_maxLinMotorForce;
 }

 public void setPoweredAngMotor(boolean onOff) {
  m_poweredAngMotor = onOff;
 }

 public boolean getPoweredAngMotor() {
  return m_poweredAngMotor;
 }

 public void setTargetAngMotorVelocity(float targetAngMotorVelocity) {
  m_targetAngMotorVelocity = targetAngMotorVelocity;
 }

 public float getTargetAngMotorVelocity() {
  return m_targetAngMotorVelocity;
 }

 public void setMaxAngMotorForce(float maxAngMotorForce) {
  m_maxAngMotorForce = maxAngMotorForce;
 }

 public float getMaxAngMotorForce() {
  return m_maxAngMotorForce;
 }

 public float getLinearPos() {
  return m_linPos;
 }

 public float getAngularPos() {
  return m_angPos;
 }

 // access for ODE solver
 public boolean getSolveLinLimit() {
  return m_solveLinLim;
 }

 public float getLinDepth() {
  return m_depth.x;
 }

 public boolean getSolveAngLimit() {
  return m_solveAngLim;
 }

 public float getAngDepth() {
  return m_angDepth;
 }
 // shared code used by ODE solver

 public void calculateTransforms(final btTransform transA,
  final btTransform transB) {
  if (m_useLinearReferenceFrameA || (!m_useSolveConstraintObsolete)) {
   m_calculatedTransformA.set(transA).mul(m_frameInA);
   m_calculatedTransformB.set(transB).mul(m_frameInB);
  } else {
   m_calculatedTransformA.set(transB).mul(m_frameInB);
   m_calculatedTransformB.set(transA).mul(m_frameInA);
  }
  m_realPivotAInW.set(m_calculatedTransformA.getOrigin());
  m_realPivotBInW.set(m_calculatedTransformB.getOrigin());
  m_sliderAxis.set(m_calculatedTransformA.getBasisColumn(0)); // along X
  if (m_useLinearReferenceFrameA || m_useSolveConstraintObsolete) {
   m_delta.set(m_realPivotBInW).sub(m_realPivotAInW);
  } else {
   m_delta.set(m_realPivotAInW).sub(m_realPivotBInW);
  }
  m_projPivotInW.scaleAdd(m_sliderAxis.dot(m_delta), m_sliderAxis,
   m_realPivotAInW);
  final btVector3 normalWorld = new btVector3();
  //linear part
  for (int i = 0; i < 3; i++) {
   m_calculatedTransformA.getBasisColumn(i, normalWorld);
   m_depth.setElement(i, m_delta.dot(normalWorld));
  }
 }

 void testLinLimits() {
  m_solveLinLim = false;
  m_linPos = m_depth.x;
  if (m_lowerLinLimit <= m_upperLinLimit) {
   if (m_depth.x > m_upperLinLimit) {
    m_depth.x -= m_upperLinLimit;
    m_solveLinLim = true;
   } else if (m_depth.x < m_lowerLinLimit) {
    m_depth.x -= m_lowerLinLimit;
    m_solveLinLim = true;
   } else {
    m_depth.x = (0.f);
   }
  } else {
   m_depth.x = (0.f);
  }
 }

 void testAngLimits() {
  m_angDepth = (0.f);
  m_solveAngLim = false;
  if (m_lowerAngLimit <= m_upperAngLimit) {
   final btVector3 axisA0 = m_calculatedTransformA.getBasisColumn(1);
   final btVector3 axisA1 = m_calculatedTransformA.getBasisColumn(2);
   final btVector3 axisB0 = m_calculatedTransformB.getBasisColumn(1);
//		float rot = btAtan2Fast(axisB0.dot(axisA1), axisB0.dot(axisA0));  
   float rot = btAtan2(axisB0.dot(axisA1), axisB0.dot(axisA0));
   rot = btAdjustAngleToLimits(rot, m_lowerAngLimit, m_upperAngLimit);
   m_angPos = rot;
   if (rot < m_lowerAngLimit) {
    m_angDepth = rot - m_lowerAngLimit;
    m_solveAngLim = true;
   } else if (rot > m_upperAngLimit) {
    m_angDepth = rot - m_upperAngLimit;
    m_solveAngLim = true;
   }
  }
 }

 // access for PE Solver
 public btVector3 getAncorInA() {
  final btVector3 ancorInA = new btVector3();
  ancorInA.set(m_realPivotAInW)
   .add(new btVector3(m_sliderAxis)
    .scale((m_lowerLinLimit + m_upperLinLimit) * (0.5f)));
  m_rbA.getCenterOfMassTransform().invert().transform(ancorInA);
  return ancorInA;
 }

 public btVector3 getAncorInB() {
  final btVector3 ancorInB;
  ancorInB = m_frameInB.getOrigin();
  return ancorInB;
 }
 // access for UseFrameOffset

 public boolean getUseFrameOffset() {
  return m_useOffsetForConstraintFrame;
 }

 public void setUseFrameOffset(boolean frameOffsetOnOff) {
  m_useOffsetForConstraintFrame = frameOffsetOnOff;
 }

 public void setFrames(final btTransform frameA, final btTransform frameB) {
  m_frameInA.set(frameA);
  m_frameInB.set(frameB);
  calculateTransforms(m_rbA.getCenterOfMassTransform(), m_rbB
   .getCenterOfMassTransform());
  buildJacobian();
 }

 ///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
 ///If no axis is provided, it uses the default axis for this constraint.
 @Override
 public void setParam(int num, float value, int axis) {
  switch (num) {
   case BT_CONSTRAINT_STOP_ERP:
    if (axis < 1) {
     m_softnessLimLin = value;
     m_flags |= BT_SLIDER_FLAGS_ERP_LIMLIN;
    } else if (axis < 3) {
     m_softnessOrthoLin = value;
     m_flags |= BT_SLIDER_FLAGS_ERP_ORTLIN;
    } else if (axis == 3) {
     m_softnessLimAng = value;
     m_flags |= BT_SLIDER_FLAGS_ERP_LIMANG;
    } else if (axis < 6) {
     m_softnessOrthoAng = value;
     m_flags |= BT_SLIDER_FLAGS_ERP_ORTANG;
    } else {
     assert (false);
    }
    break;
   case BT_CONSTRAINT_CFM:
    if (axis < 1) {
     m_cfmDirLin = value;
     m_flags |= BT_SLIDER_FLAGS_CFM_DIRLIN;
    } else if (axis == 3) {
     m_cfmDirAng = value;
     m_flags |= BT_SLIDER_FLAGS_CFM_DIRANG;
    } else {
     assert (false);
    }
    break;
   case BT_CONSTRAINT_STOP_CFM:
    if (axis < 1) {
     m_cfmLimLin = value;
     m_flags |= BT_SLIDER_FLAGS_CFM_LIMLIN;
    } else if (axis < 3) {
     m_cfmOrthoLin = value;
     m_flags |= BT_SLIDER_FLAGS_CFM_ORTLIN;
    } else if (axis == 3) {
     m_cfmLimAng = value;
     m_flags |= BT_SLIDER_FLAGS_CFM_LIMANG;
    } else if (axis < 6) {
     m_cfmOrthoAng = value;
     m_flags |= BT_SLIDER_FLAGS_CFM_ORTANG;
    } else {
     assert (false);
    }
    break;
  }
 }

 ///return the local value of parameter
 @Override
 public float getParam(int num, int axis) {
  float retVal = SIMD_INFINITY;
  switch (num) {
   case BT_CONSTRAINT_STOP_ERP:
    if (axis < 1) {
     assert ((m_flags & BT_SLIDER_FLAGS_ERP_LIMLIN) != 0);
     retVal = m_softnessLimLin;
    } else if (axis < 3) {
     assert ((m_flags & BT_SLIDER_FLAGS_ERP_ORTLIN) != 0);
     retVal = m_softnessOrthoLin;
    } else if (axis == 3) {
     assert ((m_flags & BT_SLIDER_FLAGS_ERP_LIMANG) != 0);
     retVal = m_softnessLimAng;
    } else if (axis < 6) {
     assert ((m_flags & BT_SLIDER_FLAGS_ERP_ORTANG) != 0);
     retVal = m_softnessOrthoAng;
    } else {
     assert (false);
    }
    break;
   case BT_CONSTRAINT_CFM:
    if (axis < 1) {
     assert ((m_flags & BT_SLIDER_FLAGS_CFM_DIRLIN) != 0);
     retVal = m_cfmDirLin;
    } else if (axis == 3) {
     assert ((m_flags & BT_SLIDER_FLAGS_CFM_DIRANG) != 0);
     retVal = m_cfmDirAng;
    } else {
     assert (false);
    }
    break;
   case BT_CONSTRAINT_STOP_CFM:
    if (axis < 1) {
     assert ((m_flags & BT_SLIDER_FLAGS_CFM_LIMLIN) != 0);
     retVal = m_cfmLimLin;
    } else if (axis < 3) {
     assert ((m_flags & BT_SLIDER_FLAGS_CFM_ORTLIN) != 0);
     retVal = m_cfmOrthoLin;
    } else if (axis == 3) {
     assert ((m_flags & BT_SLIDER_FLAGS_CFM_LIMANG) != 0);
     retVal = m_cfmLimAng;
    } else if (axis < 6) {
     assert ((m_flags & BT_SLIDER_FLAGS_CFM_ORTANG) != 0);
     retVal = m_cfmOrthoAng;
    } else {
     assert (false);
    }
    break;
  }
  return retVal;
 }

 public int getFlags() {
  return m_flags;
 }

};
