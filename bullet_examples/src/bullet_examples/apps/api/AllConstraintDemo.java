/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2015 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
package bullet_examples.apps.api;

import Bullet.Collision.Shape.btBoxShape;
import Bullet.Collision.Shape.btCollisionShape;
import Bullet.Collision.Shape.btCompoundShape;
import Bullet.Collision.Shape.btCylinderShape;
import Bullet.Collision.Shape.btStaticPlaneShape;
import static Bullet.Collision.btCollisionObject.DISABLE_DEACTIVATION;
import static Bullet.Collision.btIDebugDraw.DBG_DrawConstraintLimits;
import static Bullet.Collision.btIDebugDraw.DBG_DrawConstraints;
import Bullet.Dynamics.Constraint.btConeTwistConstraint;
import Bullet.Dynamics.Constraint.btGearConstraint;
import Bullet.Dynamics.Constraint.btGeneric6DofConstraint;
import Bullet.Dynamics.Constraint.btGeneric6DofSpringConstraint;
import Bullet.Dynamics.Constraint.btHinge2Constraint;
import Bullet.Dynamics.Constraint.btHingeConstraint;
import Bullet.Dynamics.Constraint.btPoint2PointConstraint;
import Bullet.Dynamics.Constraint.btSliderConstraint;
import Bullet.Dynamics.Constraint.btTypedConstraint;
import Bullet.Dynamics.Constraint.btUniversalConstraint;
import Bullet.Dynamics.btRigidBody;
import Bullet.Dynamics.btRigidBodyConstructionInfo;
import Bullet.LinearMath.btMatrix3x3;
import Bullet.LinearMath.btQuaternion;
import static Bullet.LinearMath.btScalar.SIMD_HALF_PI;
import static Bullet.LinearMath.btScalar.SIMD_PI;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import bullet_examples.DiscreteDemoContainer;
import javax.swing.JPanel;
import static javax.vecmath.VecMath.cos;
import static javax.vecmath.VecMath.tan;

/**
 *
 * @author Gregery Barton
 */
public class AllConstraintDemo extends DiscreteDemoContainer {

 static final boolean ENABLE_ALL_DEMOS = true;
 static final boolean GEAR_DEMO = false;
 static final boolean POINT_2_POINT = false;
 static final boolean SLIDER = false;
 static final boolean SLIDER_D6 = false;
 static final boolean DOOR_HINGE = false;
 static final boolean GENERIC_6DOF = false;
 static final boolean CONE_TWIST = false;
 static final boolean HINGE_MOTOR = false;
 static final boolean UNIVERSAL = false;
 static final boolean GENERIC_6DOF_SPRINGS = false;
 static final boolean HINGE2 = false;
 static final boolean HINGE_DYNAMIC = false;
 static final boolean MOTOR_6DOF = true;
 static final float CUBE_HALF_EXTENTS = 1.f;
 static final float SIMD_PI_2 = ((SIMD_PI) * 0.5f);
 static final float SIMD_PI_4 = ((SIMD_PI) * 0.25f);
 final btTransform sliderTransform = new btTransform();
 final btVector3 lowerSliderLimit = new btVector3(-10, 0, 0);
 final btVector3 hiSliderLimit = new btVector3(10, 0, 0);
 btRigidBody d6body0 = null;
 btHingeConstraint spDoorHinge = null;
 btHingeConstraint spHingeDynAB = null;
 btGeneric6DofConstraint spSlider6Dof = null;
 static boolean s_bTestConeTwistMotor = false;
 // for cone-twist motor driving
 float m_Time;
 btConeTwistConstraint m_ctc;

 public AllConstraintDemo() {
  super();
 }

 @Override
 public void initPhysics() {
  setUpAxis(1);
  m_Time = 0;
  btCollisionShape groundShape = new btStaticPlaneShape(new btVector3(0f, 1f, 0f), 40f);
  final btTransform groundTransform = new btTransform();
  groundTransform.setIdentity();
  groundTransform.setOrigin(new btVector3(0f, -56f, 0f));
  btRigidBody groundBody = createRigidBody(0, groundTransform, groundShape);
  btCollisionShape shape = new btBoxShape(new btVector3(CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS,
   CUBE_HALF_EXTENTS));
  final btTransform trans = new btTransform();
  trans.setIdentity();
  trans.setOrigin(new btVector3(0f, 20f, 0f));
  float mass = 1.f;
  if (ENABLE_ALL_DEMOS || GEAR_DEMO) {
///gear constraint demo
   final float THETA = SIMD_PI / 4.f;
   final float L_1 = (2 - tan(THETA));
   final float L_2 = (1 / cos(THETA));
   final float RATIO = L_2 / L_1;
   btRigidBody bodyA = null;
   btRigidBody bodyB = null;
   {
    btCollisionShape cylA = new btCylinderShape(new btVector3(0.2f, 0.25f, 0.2f));
    btCollisionShape cylB = new btCylinderShape(new btVector3(L_1, 0.025f, L_1));
    btCompoundShape cyl0 = new btCompoundShape();
    cyl0.addChildShape(btTransform.getIdentity(), cylA);
    cyl0.addChildShape(btTransform.getIdentity(), cylB);
    mass = 6.28f;
    final btVector3 localInertia = new btVector3();
    cyl0.calculateLocalInertia(mass, localInertia);
    btRigidBodyConstructionInfo ci = new btRigidBodyConstructionInfo(mass, null, cyl0, localInertia);
    ci.m_startWorldTransform.setOrigin(new btVector3(-8f, 1f, -8f));
    btRigidBody body = new btRigidBody(ci);//1,0,cyl0,localInertia);
    world().addRigidBody(body);
    body.setLinearFactor(new btVector3(0, 0, 0));
    body.setAngularFactor(new btVector3(0, 1, 0));
    bodyA = body;
   }
   {
    btCollisionShape cylA = new btCylinderShape(new btVector3(0.2f, 0.26f, 0.2f));
    btCollisionShape cylB = new btCylinderShape(new btVector3(L_2, 0.025f, L_2));
    btCompoundShape cyl0 = new btCompoundShape();
    cyl0.addChildShape(btTransform.getIdentity(), cylA);
    cyl0.addChildShape(btTransform.getIdentity(), cylB);
    mass = 6.28f;
    final btVector3 localInertia = new btVector3();
    cyl0.calculateLocalInertia(mass, localInertia);
    btRigidBodyConstructionInfo ci = new btRigidBodyConstructionInfo(mass, null, cyl0, localInertia);
    ci.m_startWorldTransform.setOrigin(new btVector3(-10, 2, -8));
    final btQuaternion orn = new btQuaternion(new btVector3(0, 0, 1), -THETA);
    ci.m_startWorldTransform.set3x3(orn);
    btRigidBody body = new btRigidBody(ci);//1,0,cyl0,localInertia);
    body.setLinearFactor(new btVector3(0f, 0f, 0f));
    btHingeConstraint hinge = new btHingeConstraint(body, new btVector3(0, 0, 0),
     new btVector3(0, 1, 0), true);
    world().addConstraint(hinge);
    bodyB = body;
    body.setAngularVelocity(new btVector3(0, 3, 0));
    world().addRigidBody(body);
   }
   final btVector3 axisA = new btVector3(0, 1, 0);
   final btQuaternion orn = new btQuaternion(new btVector3(0, 0, 1), -THETA);
   final btMatrix3x3 mat = new btMatrix3x3(orn);
   final btVector3 axisB = mat.getRow(1);
   btGearConstraint gear = new btGearConstraint(bodyA, bodyB, axisA, axisB, RATIO);
   world().addConstraint(gear, true);
  }
  if (ENABLE_ALL_DEMOS || POINT_2_POINT) {
   //point to point constraint with a breaking threshold
   {
    mass = 1.f;
    trans.setIdentity();
    trans.setOrigin(new btVector3(1, 30, -5));
    createRigidBody(mass, trans, shape);
    trans.setOrigin(new btVector3(0, 0, -5));
    btRigidBody body0 = createRigidBody(mass, trans, shape);
    trans.setOrigin(new btVector3(2 * CUBE_HALF_EXTENTS, 20, 0));
    mass = 1.f;
    //	btRigidBody* body1 = 0;//createRigidBody( mass,trans,shape);
    final btVector3 pivotInA = new btVector3(CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS, 0);
    btTypedConstraint p2p = new btPoint2PointConstraint(body0, pivotInA);
    world().addConstraint(p2p);
    p2p.setBreakingImpulseThreshold(10.2f);
    p2p.setDbgDrawSize((5.f));
   }
  }
  if (ENABLE_ALL_DEMOS) {
   //point to point constraint (ball socket)
   {
    btRigidBody body0 = createRigidBody(mass, trans, shape);
    trans.setOrigin(new btVector3(2 * CUBE_HALF_EXTENTS, 20, 0));
    mass = 1.f;
//		btRigidBody* body1 = 0;//createRigidBody( mass,trans,shape);
//		btRigidBody* body1 = createRigidBody( 0.0,trans,0);
    //body1.setActivationState(DISABLE_DEACTIVATION);
    //body1.setDamping(0.3,0.3);
    final btVector3 pivotInA = new btVector3(CUBE_HALF_EXTENTS, -CUBE_HALF_EXTENTS,
     -CUBE_HALF_EXTENTS);
    final btVector3 axisInA = new btVector3(0, 0, 1);
    //	btVector3 pivotInB = body1 ? body1.getCenterOfMassTransform().inverse()(body0.getCenterOfMassTransform()(pivotInA)) : pivotInA;
//		btVector3 axisInB = body1?
//			(body1.getCenterOfMassTransform().getBasis().inverse()*(body1.getCenterOfMassTransform().getBasis() * axisInA)) :
    //body0.getCenterOfMassTransform().getBasis() * axisInA;
    final boolean P2P = true;
    if (P2P) {
     btTypedConstraint p2p = new btPoint2PointConstraint(body0, pivotInA);
     //btTypedConstraint* p2p = new btPoint2PointConstraint(*body0,*body1,pivotInA,pivotInB);
     //btTypedConstraint* hinge = new btHingeConstraint(*body0,*body1,pivotInA,pivotInB,axisInA,axisInB);
     world().addConstraint(p2p);
     p2p.setDbgDrawSize((5.f));
    } else {
     btHingeConstraint hinge = new btHingeConstraint(body0, pivotInA, axisInA);
     //use zero targetVelocity and a small maxMotorImpulse to simulate joint friction
     //float	targetVelocity = 0.f;
     //float	maxMotorImpulse = 0.01;
     float targetVelocity = 1.f;
     float maxMotorImpulse = 1.0f;
     hinge.enableAngularMotor(true, targetVelocity, maxMotorImpulse);
     world().addConstraint(hinge);
     hinge.setDbgDrawSize((5.f));
    }
   }
  }
  if (ENABLE_ALL_DEMOS || SLIDER) {
   {
//		final btTransform trans=new btTransform();
    trans.setIdentity();
    final btVector3 worldPos = new btVector3(-20, 0, 30);
    trans.setOrigin(worldPos);
    final btTransform frameInA = new btTransform();
    final btTransform frameInB = new btTransform();
    frameInA.set(btTransform.getIdentity());
    frameInB.set(btTransform.getIdentity());
    btRigidBody pRbA1 = createRigidBody(mass, trans, shape);
//	btRigidBody* pRbA1 = createRigidBody(0.f, trans, shape);
    pRbA1.setActivationState(DISABLE_DEACTIVATION);
    // add dynamic rigid body B1
    worldPos.set(-30, 0, 30);
    trans.setOrigin(worldPos);
    btRigidBody pRbB1 = createRigidBody(mass, trans, shape);
//	btRigidBody* pRbB1 = createRigidBody(0.f, trans, shape);
    pRbB1.setActivationState(DISABLE_DEACTIVATION);
    // create slider constraint between A1 and B1 and add it to world
    btSliderConstraint spSlider1 = new btSliderConstraint(pRbA1, pRbB1, frameInA, frameInB, true);
//	spSlider1 = new btSliderConstraint(*pRbA1, *pRbB1, frameInA, frameInB, false);
    spSlider1.setLowerLinLimit(-15.0F);
    spSlider1.setUpperLinLimit(-5.0F);
//	spSlider1.setLowerLinLimit(5.0F);
//	spSlider1.setUpperLinLimit(15.0F);
//	spSlider1.setLowerLinLimit(-10.0F);
//	spSlider1.setUpperLinLimit(-10.0F);
    spSlider1.setLowerAngLimit(-SIMD_PI / 3.0F);
    spSlider1.setUpperAngLimit(SIMD_PI / 3.0F);
    world().addConstraint(spSlider1, true);
    spSlider1.setDbgDrawSize((5.f));
   }
  }
  if (ENABLE_ALL_DEMOS || SLIDER_D6) {
   //create a slider, using the generic D6 constraint
   {
    mass = 1.f;
    final btVector3 sliderWorldPos = new btVector3(0, 10, 0);
    final btVector3 sliderAxis = new btVector3(1, 0, 0);
    float angle = 0.f;//SIMD_RADS_PER_DEG * 10.f;
    final btMatrix3x3 sliderOrientation = new btMatrix3x3(new btQuaternion(sliderAxis, angle));
    trans.setIdentity();
    trans.setOrigin(sliderWorldPos);
    //trans.setBasis(sliderOrientation);
    sliderTransform.set(trans);
    d6body0 = createRigidBody(mass, trans, shape);
    d6body0.setActivationState(DISABLE_DEACTIVATION);
    btRigidBody fixedBody1 = createRigidBody(0, trans, null);
    world().addRigidBody(fixedBody1);
    final btTransform frameInA = new btTransform();
    final btTransform frameInB = new btTransform();
    frameInA.setIdentity();
    frameInB.setIdentity();
    frameInA.setOrigin(new btVector3(0.f, 5.f, 0.f));
    frameInB.setOrigin(new btVector3(0.f, 5.f, 0.f));
//		bool useLinearReferenceFrameA = false;//use fixed frame B for linear llimits
    boolean useLinearReferenceFrameA = true;//use fixed frame A for linear llimits
    spSlider6Dof = new btGeneric6DofConstraint(fixedBody1, d6body0, frameInA, frameInB,
     useLinearReferenceFrameA);
    spSlider6Dof.setLinearLowerLimit(lowerSliderLimit);
    spSlider6Dof.setLinearUpperLimit(hiSliderLimit);
    //range should be small, otherwise singularities will 'explode' the constraint
//		spSlider6Dof.setAngularLowerLimit(btVector3(-1.5,0,0));
//		spSlider6Dof.setAngularUpperLimit(btVector3(1.5,0,0));
//		spSlider6Dof.setAngularLowerLimit(btVector3(0,0,0));
//		spSlider6Dof.setAngularUpperLimit(btVector3(0,0,0));
    spSlider6Dof.setAngularLowerLimit(new btVector3(-SIMD_PI, 0f, 0f));
    spSlider6Dof.setAngularUpperLimit(new btVector3(1.5f, 0f, 0f));
    spSlider6Dof.getTranslationalLimitMotor().m_enableMotor[0] = true;
    spSlider6Dof.getTranslationalLimitMotor().m_targetVelocity.x = -5.0f;
    spSlider6Dof.getTranslationalLimitMotor().m_maxMotorForce.x = 0.1f;
    world().addConstraint(spSlider6Dof);
    spSlider6Dof.setDbgDrawSize((5.f));
   }
  }
  if (ENABLE_ALL_DEMOS || DOOR_HINGE) {
   { // create a door using hinge constraint attached to the world
    btCollisionShape pDoorShape = new btBoxShape(new btVector3(2.0f, 5.0f, 0.2f));
    final btTransform doorTrans = new btTransform();
    doorTrans.setIdentity();
    doorTrans.setOrigin(new btVector3(-5.0f, -2.0f, 0.0f));
    btRigidBody pDoorBody = createRigidBody(1.0f, doorTrans, pDoorShape);
    pDoorBody.setActivationState(DISABLE_DEACTIVATION);
    final btVector3 btPivotA = new btVector3(10.f + 2.1f, -2.0f, 0.0f); // right next to the door slightly outside
    final btVector3 btAxisA = new btVector3(0.0f, 1.0f, 0.0f); // pointing upwards, aka Y-axis
    spDoorHinge = new btHingeConstraint(pDoorBody, btPivotA, btAxisA);
//		spDoorHinge.setLimit( 0.0f, SIMD_PI_2 );
    // test problem values
//		spDoorHinge.setLimit( -SIMD_PI, SIMD_PI*0.8f);
//		spDoorHinge.setLimit( 1.f, -1.f);
//		spDoorHinge.setLimit( -SIMD_PI*0.8f, SIMD_PI);
//		spDoorHinge.setLimit( -SIMD_PI*0.8f, SIMD_PI, 0.9f, 0.3f, 0.0f);
//		spDoorHinge.setLimit( -SIMD_PI*0.8f, SIMD_PI, 0.9f, 0.01f, 0.0f); // "sticky limits"
    spDoorHinge.setLimit(-SIMD_PI * 0.25f, SIMD_PI * 0.25f);
//		spDoorHinge.setLimit( 0.0f, 0.0f );
    world().addConstraint(spDoorHinge);
    spDoorHinge.setDbgDrawSize((5.f));
    //doorTrans.setOrigin(btVector3(-5.0f, 2.0f, 0.0f));
    //btRigidBody* pDropBody = createRigidBody( 10.0, doorTrans, shape);
   }
  }
  if (ENABLE_ALL_DEMOS || GENERIC_6DOF) {
   { // create a generic 6DOF constraint
    final btTransform tr = new btTransform();
    tr.setIdentity();
    tr.setOrigin(new btVector3((10.f), (6.f), (0.f)));
    tr.setBasis(new btMatrix3x3().setEulerZYX(0, 0, 0));
//		btRigidBody* pBodyA = createRigidBody( mass, tr, shape);
    btRigidBody pBodyA = createRigidBody(0.0f, tr, shape);
//		btRigidBody* pBodyA = createRigidBody( 0.0, tr, 0);
    pBodyA.setActivationState(DISABLE_DEACTIVATION);
    tr.setIdentity();
    tr.setOrigin(new btVector3((0.f), (6.f), (0.f)));
    tr.getBasis().setEulerZYX(0, 0, 0);
    btRigidBody pBodyB = createRigidBody(mass, tr, shape);
//		btRigidBody* pBodyB = createRigidBody(0.f, tr, shape);
    pBodyB.setActivationState(DISABLE_DEACTIVATION);
    final btTransform frameInA = new btTransform();
    final btTransform frameInB = new btTransform();
    frameInA.setIdentity();
    frameInA.setOrigin(new btVector3((-5.f), (0.f), (0.f)));
    frameInB.setIdentity();
    frameInB.setOrigin(new btVector3((5.f), (0.f), (0.f)));
    btGeneric6DofConstraint pGen6DOF = new btGeneric6DofConstraint(pBodyA, pBodyB, frameInA,
     frameInB, true);
//		btGeneric6DofConstraint* pGen6DOF = new btGeneric6DofConstraint(*pBodyA, *pBodyB, frameInA, frameInB, false);
    pGen6DOF.setLinearLowerLimit(new btVector3(-10.f, -2.f, -1.f));
    pGen6DOF.setLinearUpperLimit(new btVector3(10.f, 2.f, 1.f));
//		pGen6DOF.setLinearLowerLimit(btVector3(-10., 0., 0.));
//		pGen6DOF.setLinearUpperLimit(btVector3(10., 0., 0.));
//		pGen6DOF.setLinearLowerLimit(btVector3(0., 0., 0.));
//		pGen6DOF.setLinearUpperLimit(btVector3(0., 0., 0.));
//		pGen6DOF.getTranslationalLimitMotor().m_enableMotor[0] = true;
//		pGen6DOF.getTranslationalLimitMotor().m_targetVelocity[0] = 5.0f;
//		pGen6DOF.getTranslationalLimitMotor().m_maxMotorForce[0] = 0.1f;
//		pGen6DOF.setAngularLowerLimit(btVector3(0., SIMD_HALF_PI*0.9, 0.));
//		pGen6DOF.setAngularUpperLimit(btVector3(0., -SIMD_HALF_PI*0.9, 0.));
//		pGen6DOF.setAngularLowerLimit(btVector3(0., 0., -SIMD_HALF_PI));
//		pGen6DOF.setAngularUpperLimit(btVector3(0., 0., SIMD_HALF_PI));
    pGen6DOF.setAngularLowerLimit(new btVector3(-SIMD_HALF_PI * 0.5f, -0.75f, -SIMD_HALF_PI * 0.8f));
    pGen6DOF.setAngularUpperLimit(new btVector3(SIMD_HALF_PI * 0.5f, 0.75f, SIMD_HALF_PI * 0.8f));
//		pGen6DOF.setAngularLowerLimit(btVector3(0.f, -0.75, SIMD_HALF_PI * 0.8f));
//		pGen6DOF.setAngularUpperLimit(btVector3(0.f, 0.75, -SIMD_HALF_PI * 0.8f));
//		pGen6DOF.setAngularLowerLimit(btVector3(0.f, -SIMD_HALF_PI * 0.8f, SIMD_HALF_PI * 1.98f));
//		pGen6DOF.setAngularUpperLimit(btVector3(0.f, SIMD_HALF_PI * 0.8f,  -SIMD_HALF_PI * 1.98f));
//		pGen6DOF.setAngularLowerLimit(btVector3(-0.75,-0.5, -0.5));
//		pGen6DOF.setAngularUpperLimit(btVector3(0.75,0.5, 0.5));
//		pGen6DOF.setAngularLowerLimit(btVector3(-0.75,0., 0.));
//		pGen6DOF.setAngularUpperLimit(btVector3(0.75,0., 0.));
//		pGen6DOF.setAngularLowerLimit(btVector3(0., -0.7,0.));
//		pGen6DOF.setAngularUpperLimit(btVector3(0., 0.7, 0.));
//		pGen6DOF.setAngularLowerLimit(btVector3(-1., 0.,0.));
//		pGen6DOF.setAngularUpperLimit(btVector3(1., 0., 0.));
    world().addConstraint(pGen6DOF, true);
    pGen6DOF.setDbgDrawSize((5.f));
   }
  }
  if (ENABLE_ALL_DEMOS || CONE_TWIST) {
   { // create a ConeTwist constraint
    final btTransform tr = new btTransform();
    tr.setIdentity();
    tr.setOrigin(new btVector3((-10.f), (5.f), (0.f)));
    tr.setBasis(new btMatrix3x3().setEulerZYX(0, 0, 0));
    btRigidBody pBodyA = createRigidBody(1.0f, tr, shape);
//		btRigidBody* pBodyA = createRigidBody( 0.0, tr, shape);
    pBodyA.setActivationState(DISABLE_DEACTIVATION);
    tr.setIdentity();
    tr.setOrigin(new btVector3((-10.f), (-5.f), (0.f)));
    tr.setBasis(new btMatrix3x3().setEulerZYX(0, 0, 0));
    btRigidBody pBodyB = createRigidBody(0.0f, tr, shape);
//		btRigidBody* pBodyB = createRigidBody(1.0, tr, shape);
    final btTransform frameInA = new btTransform();
    final btTransform frameInB = new btTransform();
    frameInA.setIdentity();
    frameInA.setBasis(new btMatrix3x3().setEulerZYX(0, 0, SIMD_PI_2));
    frameInA.setOrigin(new btVector3((0.f), (-5.f), (0.f)));
    frameInB.setIdentity();
    frameInB.setBasis(new btMatrix3x3().setEulerZYX(0, 0, SIMD_PI_2));
    frameInB.setOrigin(new btVector3((0.f), (5.f), (0.f)));
    m_ctc = new btConeTwistConstraint(pBodyA, pBodyB, frameInA, frameInB);
//		m_ctc.setLimit(float(SIMD_PI_4), float(SIMD_PI_4), float(SIMD_PI) * 0.8f);
//		m_ctc.setLimit(float(SIMD_PI_4*0.6f), float(SIMD_PI_4), float(SIMD_PI) * 0.8f, 1.0f); // soft limit == hard limit
    m_ctc.setLimit((SIMD_PI_4 * 0.6f), (SIMD_PI_4), (SIMD_PI) * 0.8f, 0.5f);
    world().addConstraint(m_ctc, true);
    m_ctc.setDbgDrawSize((5.f));
    // s_bTestConeTwistMotor = true; // use only with old solver for now
    s_bTestConeTwistMotor = false;
   }
  }
  if (ENABLE_ALL_DEMOS || HINGE_MOTOR) { // Hinge connected to the world, with motor (to hinge motor with new and old constraint solver)
   final btTransform tr = new btTransform();
   tr.setIdentity();
   tr.setOrigin(new btVector3((0.f), (0.f), (0.f)));
   btRigidBody pBody = createRigidBody(1.0f, tr, shape);
   pBody.setActivationState(DISABLE_DEACTIVATION);
   final btVector3 btPivotA = new btVector3(10.0f, 0.0f, 0.0f);
   final btVector3 btAxisA = new btVector3(0.0f, 0.0f, 1.0f);
   btHingeConstraint pHinge = new btHingeConstraint(pBody, btPivotA, btAxisA);
//		pHinge.enableAngularMotor(true, -1.0, 0.165); // use for the old solver
   pHinge.enableAngularMotor(true, -1.0f, 1.65f); // use for the new SIMD solver
   world().addConstraint(pHinge);
   pHinge.setDbgDrawSize((5.f));
  }
  if (ENABLE_ALL_DEMOS || UNIVERSAL) {
   {
    // create a universal joint using generic 6DOF constraint
    // create two rigid bodies
    // static bodyA (parent) on top:
    final btTransform tr = new btTransform();
    tr.setIdentity();
    tr.setOrigin(new btVector3((20.f), (4.f), (0.f)));
    btRigidBody pBodyA = createRigidBody(0.0f, tr, shape);
    pBodyA.setActivationState(DISABLE_DEACTIVATION);
    // dynamic bodyB (child) below it :
    tr.setIdentity();
    tr.setOrigin(new btVector3((20.f), (0.f), (0.f)));
    btRigidBody pBodyB = createRigidBody(1.0f, tr, shape);
    pBodyB.setActivationState(DISABLE_DEACTIVATION);
    // add some (arbitrary) data to build constraint frames
    final btVector3 parentAxis = new btVector3(1.f, 0.f, 0.f);
    final btVector3 childAxis = new btVector3(0.f, 0.f, 1.f);
    final btVector3 anchor = new btVector3(20.f, 2.f, 0.f);
    btUniversalConstraint pUniv = new btUniversalConstraint(pBodyA, pBodyB, anchor, parentAxis,
     childAxis);
    pUniv.setLowerLimit(-SIMD_HALF_PI * 0.5f, -SIMD_HALF_PI * 0.5f);
    pUniv.setUpperLimit(SIMD_HALF_PI * 0.5f, SIMD_HALF_PI * 0.5f);
    // add constraint to world
    world().addConstraint(pUniv, true);
    // draw constraint frames and limits for debugging
    pUniv.setDbgDrawSize((5.f));
   }
  }
  if (ENABLE_ALL_DEMOS || GENERIC_6DOF_SPRINGS) {
   { // create a generic 6DOF constraint with springs 
    final btTransform tr = new btTransform();
    tr.setIdentity();
    tr.setOrigin(new btVector3((-20.f), (16.f), (0.f)));
    tr.setBasis(new btMatrix3x3().setEulerZYX(0, 0, 0));
    btRigidBody pBodyA = createRigidBody(0.0f, tr, shape);
    pBodyA.setActivationState(DISABLE_DEACTIVATION);
    tr.setIdentity();
    tr.setOrigin(new btVector3((-10.f), (16.f), (0.f)));
    tr.setBasis(new btMatrix3x3().setEulerZYX(0, 0, 0));
    btRigidBody pBodyB = createRigidBody(1.0f, tr, shape);
    pBodyB.setActivationState(DISABLE_DEACTIVATION);
    final btTransform frameInA = new btTransform();
    final btTransform frameInB = new btTransform();
    frameInA.setIdentity();
    frameInA.setOrigin(new btVector3((10.f), (0.f), (0.f)));
    frameInB.setIdentity();
    frameInB.setOrigin(new btVector3((0.f), (0.f), (0.f)));
    btGeneric6DofSpringConstraint pGen6DOFSpring = new btGeneric6DofSpringConstraint(pBodyA, pBodyB,
     frameInA, frameInB, true);
    pGen6DOFSpring.setLinearUpperLimit(new btVector3(5.f, 0.f, 0.f));
    pGen6DOFSpring.setLinearLowerLimit(new btVector3(-5.f, 0.f, 0.f));
    pGen6DOFSpring.setAngularLowerLimit(new btVector3(0.f, 0.f, -1.5f));
    pGen6DOFSpring.setAngularUpperLimit(new btVector3(0.f, 0.f, 1.5f));
    world().addConstraint(pGen6DOFSpring, true);
    pGen6DOFSpring.setDbgDrawSize((5.f));
    pGen6DOFSpring.enableSpring(0, true);
    pGen6DOFSpring.setStiffness(0, 39.478f);
    pGen6DOFSpring.setDamping(0, 0.5f);
    pGen6DOFSpring.enableSpring(5, true);
    pGen6DOFSpring.setStiffness(5, 39.478f);
    pGen6DOFSpring.setDamping(0, 0.3f);
    pGen6DOFSpring.setEquilibriumPoint();
   }
  }
  if (ENABLE_ALL_DEMOS || HINGE2) {
   // create a Hinge2 joint
   // create two rigid bodies
   // static bodyA (parent) on top:
   final btTransform tr = new btTransform();
   tr.setIdentity();
   tr.setOrigin(new btVector3((-20.f), (4.f), (0.f)));
   btRigidBody pBodyA = createRigidBody(0.0f, tr, shape);
   pBodyA.setActivationState(DISABLE_DEACTIVATION);
   // dynamic bodyB (child) below it :
   tr.setIdentity();
   tr.setOrigin(new btVector3((-20.f), (0.f), (0.f)));
   btRigidBody pBodyB = createRigidBody(1.0f, tr, shape);
   pBodyB.setActivationState(DISABLE_DEACTIVATION);
   // add some data to build constraint frames
   final btVector3 parentAxis = new btVector3(0.f, 1.f, 0.f);
   final btVector3 childAxis = new btVector3(1.f, 0.f, 0.f);
   final btVector3 anchor = new btVector3(-20.f, 0.f, 0.f);
   btHinge2Constraint pHinge2 =
    new btHinge2Constraint(pBodyA, pBodyB, anchor, parentAxis, childAxis);
   pHinge2.setLowerLimit(-SIMD_HALF_PI * 0.5f);
   pHinge2.setUpperLimit(SIMD_HALF_PI * 0.5f);
   // add constraint to world
   world().addConstraint(pHinge2, true);
   // draw constraint frames and limits for debugging
   pHinge2.setDbgDrawSize((5.f));
  }
  if (ENABLE_ALL_DEMOS || HINGE_DYNAMIC) {
   // create a Hinge joint between two dynamic bodies
   // create two rigid bodies
   // static bodyA (parent) on top:
   final btTransform tr = new btTransform();
   tr.setIdentity();
   tr.setOrigin(new btVector3((-20.f), (-2.f), (0.f)));
   btRigidBody pBodyA = createRigidBody(1.0f, tr, shape);
   pBodyA.setActivationState(DISABLE_DEACTIVATION);
   // dynamic bodyB:
   tr.setIdentity();
   tr.setOrigin(new btVector3((-30.f), (-2.f), (0.f)));
   btRigidBody pBodyB = createRigidBody(10.0f, tr, shape);
   pBodyB.setActivationState(DISABLE_DEACTIVATION);
   // add some data to build constraint frames
   final btVector3 axisA = new btVector3(0.f, 1.f, 0.f);
   final btVector3 axisB = new btVector3(0.f, 1.f, 0.f);
   final btVector3 pivotA = new btVector3(-5.f, 0.f, 0.f);
   final btVector3 pivotB = new btVector3(5.f, 0.f, 0.f);
   spHingeDynAB = new btHingeConstraint(pBodyA, pBodyB, pivotA, pivotB, axisA, axisB);
   spHingeDynAB.setLimit(-SIMD_HALF_PI * 0.5f, SIMD_HALF_PI * 0.5f);
   // add constraint to world
   world().addConstraint(spHingeDynAB, true);
   // draw constraint frames and limits for debugging
   spHingeDynAB.setDbgDrawSize((5.f));
  }
  if (ENABLE_ALL_DEMOS || MOTOR_6DOF) { // 6DOF connected to the world, with motor
   final btTransform tr = new btTransform();
   tr.setIdentity();
   tr.setOrigin(new btVector3((10.f), (-15.f), (0.f)));
   btRigidBody pBody = createRigidBody(1.0f, tr, shape);
   pBody.setActivationState(DISABLE_DEACTIVATION);
   final btTransform frameB = new btTransform();
   frameB.setIdentity();
   btGeneric6DofConstraint pGen6Dof = new btGeneric6DofConstraint(pBody, frameB, false);
   world().addConstraint(pGen6Dof);
   pGen6Dof.setDbgDrawSize((5.f));
   pGen6Dof.setAngularLowerLimit(new btVector3(0f, 0f, 0f));
   pGen6Dof.setAngularUpperLimit(new btVector3(0f, 0f, 0f));
   pGen6Dof.setLinearLowerLimit(new btVector3(-10.f, 0f, 0f));
   pGen6Dof.setLinearUpperLimit(new btVector3(10.f, 0f, 0f));
   pGen6Dof.getTranslationalLimitMotor().m_enableMotor[0] = true;
   pGen6Dof.getTranslationalLimitMotor().m_targetVelocity.x = 5.0f;
   pGen6Dof.getTranslationalLimitMotor().m_maxMotorForce.x = 0.1f;
  }
 }

 @Override
 public void resetCamera() {
  camera().set(new btQuaternion().set(2.085825E-4f, -0.9988975f, -0.046732347f, 0.0044579273f),
   new btVector3(-3.541893f, 15.872927f, -142.00476f));
 }

 @Override
 protected JPanel getParams() {
  return null;
 }

 @Override
 protected int getDebugMode() {
  return DBG_DrawConstraintLimits | DBG_DrawConstraints;
 }

 @Override
 public String get_description() {
  return "All the constraints as of an early version of Bullet Physics. Move with WASD keys and mouse to " +
   "get close to constraints for inspection. Manipulate the bodies of the constraint using right mouse button drag.";
 }
};
