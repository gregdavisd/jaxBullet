/*
  * Copyright (c) 2017  
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
package bullet_examples.apps.api;

import Bullet.Collision.Shape.btBoxShape;
import Bullet.Collision.Shape.btCollisionShape;
import Bullet.Collision.btCollisionObject;
import static Bullet.Collision.btCollisionObject.ACTIVE_TAG;
import static Bullet.Collision.btCollisionObject.DISABLE_DEACTIVATION;
import Bullet.Dynamics.Constraint.btGeneric6DofSpring2Constraint;
import static Bullet.Dynamics.Constraint.btTypedConstraint.BT_CONSTRAINT_STOP_CFM;
import static Bullet.Dynamics.Constraint.btTypedConstraint.BT_CONSTRAINT_STOP_ERP;
import Bullet.Dynamics.btRigidBody;
import Bullet.LinearMath.btDefaultMotionState;
import Bullet.LinearMath.btMatrix3x3;
import Bullet.LinearMath.btQuaternion;
import static Bullet.LinearMath.btScalar.SIMD_INFINITY;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import bullet_examples.DiscreteDemoContainer;
import javax.swing.JPanel;

/**
 *
 * @author Gregery Barton
 */
public class Dof6Spring2Demo extends DiscreteDemoContainer {

 Dof6Spring2SetupInternalData m_data = new Dof6Spring2SetupInternalData();

 static class Dof6Spring2SetupInternalData {

  btRigidBody m_TranslateSpringBody;
  btRigidBody m_TranslateSpringBody2;
  btRigidBody m_RotateSpringBody;
  btRigidBody m_RotateSpringBody2;
  btRigidBody m_BouncingTranslateBody;
  btRigidBody m_MotorBody;
  btRigidBody m_ServoMotorBody;
  btRigidBody m_ChainLeftBody;
  btRigidBody m_ChainRightBody;
  btGeneric6DofSpring2Constraint m_ServoMotorConstraint;
  btGeneric6DofSpring2Constraint m_ChainLeftConstraint;
  btGeneric6DofSpring2Constraint m_ChainRightConstraint;
  float mDt;
  int frameID;

  public Dof6Spring2SetupInternalData() {
   mDt = (1.f / 60.f);
   frameID = 0;
  }
 };

 @Override
 public void initPhysics() {
  setUpAxis(1);
  // Setup a big ground box
  {
   btCollisionShape groundShape = new btBoxShape(new btVector3((200.f), (5.f), (200.f)));
   final btTransform groundTransform = new btTransform();
   groundTransform.setIdentity();
   groundTransform.setOrigin(new btVector3(0f, -10f, 0f));
   btCollisionObject fixedGround = new btCollisionObject();
   fixedGround.setCollisionShape(groundShape);
   fixedGround.setWorldTransform(groundTransform);
   world().addCollisionObject(fixedGround);
  }
  world().getSolverInfo().m_numIterations = 100;
  btCollisionShape shape;
  final btVector3 localInertia = new btVector3();
  btDefaultMotionState motionState = new btDefaultMotionState();
  final btTransform bodyTransform = new btTransform();
  float mass;
  final btTransform localA = new btTransform();
  final btTransform localB = new btTransform();
  btGeneric6DofSpring2Constraint constraint;
  //static body centered in the origo
  mass = 0.0f;
  shape = new btBoxShape(new btVector3(0.5f, 0.5f, 0.5f));
  bodyTransform.setIdentity();
  motionState = new btDefaultMotionState(bodyTransform);
  btRigidBody staticBody = new btRigidBody(mass, motionState, shape, localInertia);
/////////// box with undamped translate spring attached to static body
/////////// the box should oscillate left-to-right forever
  {
   mass = 1.0f;
   shape = new btBoxShape(new btVector3(0.5f, 0.5f, 0.5f));
   shape.calculateLocalInertia(mass, localInertia);
   bodyTransform.setIdentity();
   bodyTransform.setOrigin(new btVector3(-2f, 0f, -5f));
   motionState = new btDefaultMotionState(bodyTransform);
   m_data.m_TranslateSpringBody = new btRigidBody(mass, motionState, shape, localInertia);
   m_data.m_TranslateSpringBody.setActivationState(DISABLE_DEACTIVATION);
   world().addRigidBody(m_data.m_TranslateSpringBody);
   localA.setIdentity();
   localA.setOrigin(new btVector3(0, 0, -5));
   localB.setIdentity();
   constraint = new btGeneric6DofSpring2Constraint(staticBody, m_data.m_TranslateSpringBody, localA,
    localB);
   constraint.setLimit(0, 1, -1);
   constraint.setLimit(1, 0, 0);
   constraint.setLimit(2, 0, 0);
   constraint.setLimit(3, 0, 0);
   constraint.setLimit(4, 0, 0);
   constraint.setLimit(5, 0, 0);
   constraint.enableSpring(0, true);
   constraint.setStiffness(0, 100);
   constraint.setDamping(0, 0);
   constraint.setEquilibriumPoint(0, 0);
   constraint.setDbgDrawSize((2.f));
   world().addConstraint(constraint, true);
  }
/////////// box with rotate spring, attached to static body
/////////// box should swing (rotate) left-to-right forever
  {
   mass = 1.0f;
   shape = new btBoxShape(new btVector3(0.5f, 0.5f, 0.5f));
   shape.calculateLocalInertia(mass, localInertia);
   bodyTransform.setIdentity();
   bodyTransform.setBasis(new btMatrix3x3().setEulerZYX(0, 0, (float) Math.PI / 2.0f));
   motionState = new btDefaultMotionState(bodyTransform);
   m_data.m_RotateSpringBody = new btRigidBody(mass, motionState, shape, localInertia);
   m_data.m_RotateSpringBody.setActivationState(DISABLE_DEACTIVATION);
   world().addRigidBody(m_data.m_RotateSpringBody);
   localA.setIdentity();
   localA.setOrigin(new btVector3());
   localB.setIdentity();
   localB.setOrigin(new btVector3(0f, 0.5f, 0f));
   constraint = new btGeneric6DofSpring2Constraint(staticBody, m_data.m_RotateSpringBody, localA,
    localB);
   constraint.setLimit(0, 0, 0);
   constraint.setLimit(1, 0, 0);
   constraint.setLimit(2, 0, 0);
   constraint.setLimit(3, 0, 0);
   constraint.setLimit(4, 0, 0);
   constraint.setLimit(5, 1, -1);
   constraint.enableSpring(5, true);
   constraint.setStiffness(5, 100);
   constraint.setDamping(5, 0);
   constraint.setEquilibriumPoint(0, 0);
   constraint.setDbgDrawSize((2.f));
   world().addConstraint(constraint, true);
  }
/////////// box with bouncing constraint, translation is bounced at the positive x limit, but not at the negative limit
/////////// bouncing can not be set independently at low and high limits, so two constraints will be created: one that defines the low (non bouncing) limit, and one that defines the high (bouncing) limit
/////////// the box should move to the left (as an impulse will be applied to it periodically) until it reaches its limit, then bounce back 
  {
   mass = 1.0f;
   shape = new btBoxShape(new btVector3(0.5f, 0.5f, 0.5f));
   shape.calculateLocalInertia(mass, localInertia);
   bodyTransform.setIdentity();
   bodyTransform.setOrigin(new btVector3(0f, 0f, -3f));
   motionState = new btDefaultMotionState(bodyTransform);
   m_data.m_BouncingTranslateBody = new btRigidBody(mass, motionState, shape, localInertia);
   m_data.m_BouncingTranslateBody.setActivationState(DISABLE_DEACTIVATION);
   m_data.m_BouncingTranslateBody.setDeactivationTime((20000000f));
   world().addRigidBody(m_data.m_BouncingTranslateBody);
   localA.setIdentity();
   localA.setOrigin(new btVector3());
   localB.setIdentity();
   constraint = new btGeneric6DofSpring2Constraint(staticBody, m_data.m_BouncingTranslateBody,
    localA, localB);
   constraint.setLimit(0, -2, SIMD_INFINITY);
   constraint.setLimit(1, 0, 0);
   constraint.setLimit(2, -3, -3);
   constraint.setLimit(3, 0, 0);
   constraint.setLimit(4, 0, 0);
   constraint.setLimit(5, 0, 0);
   constraint.setBounce(0, 0);
   constraint.setParam(BT_CONSTRAINT_STOP_ERP, 0.995f, 0);
   constraint.setParam(BT_CONSTRAINT_STOP_CFM, 0.0f, 0);
   constraint.setDbgDrawSize((2.f));
   world().addConstraint(constraint, true);
   constraint = new btGeneric6DofSpring2Constraint(staticBody, m_data.m_BouncingTranslateBody,
    localA, localB);
   constraint.setLimit(0, -SIMD_INFINITY, 2);
   constraint.setLimit(1, 0, 0);
   constraint.setLimit(2, -3, -3);
   constraint.setLimit(3, 0, 0);
   constraint.setLimit(4, 0, 0);
   constraint.setLimit(5, 0, 0);
   constraint.setBounce(0, 1);
   constraint.setParam(BT_CONSTRAINT_STOP_ERP, 0.995f, 0);
   constraint.setParam(BT_CONSTRAINT_STOP_CFM, 0.0f, 0);
   constraint.setDbgDrawSize((2.f));
   world().addConstraint(constraint, true);
  }
/////////// box with rotational motor, attached to static body
/////////// the box should rotate around the y axis
  {
   mass = 1.0f;
   shape = new btBoxShape(new btVector3(0.5f, 0.5f, 0.5f));
   shape.calculateLocalInertia(mass, localInertia);
   bodyTransform.setIdentity();
   bodyTransform.setOrigin(new btVector3(4f, 0f, 0f));
   motionState = new btDefaultMotionState(bodyTransform);
   m_data.m_MotorBody = new btRigidBody(mass, motionState, shape, localInertia);
   m_data.m_MotorBody.setActivationState(DISABLE_DEACTIVATION);
   world().addRigidBody(m_data.m_MotorBody);
   localA.setIdentity();
   localA.setOrigin(new btVector3(4f, 0f, 0f));
   localB.setIdentity();
   constraint = new btGeneric6DofSpring2Constraint(staticBody, m_data.m_MotorBody, localA, localB);
   constraint.setLimit(0, 0, 0);
   constraint.setLimit(1, 0, 0);
   constraint.setLimit(2, 0, 0);
   constraint.setLimit(3, 0, 0);
   constraint.setLimit(4, 0, 0);
   constraint.setLimit(5, 1, -1);
   constraint.enableMotor(5, true);
   constraint.setTargetVelocity(5, 3.f);
   constraint.setMaxMotorForce(5, 10.f);
   constraint.setDbgDrawSize((2.f));
   world().addConstraint(constraint, true);
  }
/////////// box with rotational servo motor, attached to static body
/////////// the box should rotate around the y axis until it reaches its target
/////////// the target will be negated periodically
  {
   mass = 1.0f;
   shape = new btBoxShape(new btVector3(0.5f, 0.5f, 0.5f));
   shape.calculateLocalInertia(mass, localInertia);
   bodyTransform.setIdentity();
   bodyTransform.setOrigin(new btVector3(7f, 0f, 0f));
   motionState = new btDefaultMotionState(bodyTransform);
   m_data.m_ServoMotorBody = new btRigidBody(mass, motionState, shape, localInertia);
   m_data.m_ServoMotorBody.setActivationState(DISABLE_DEACTIVATION);
   world().addRigidBody(m_data.m_ServoMotorBody);
   localA.setIdentity();
   localA.setOrigin(new btVector3(7, 0, 0));
   localB.setIdentity();
   constraint = new btGeneric6DofSpring2Constraint(staticBody, m_data.m_ServoMotorBody, localA,
    localB);
   constraint.setLimit(0, 0, 0);
   constraint.setLimit(1, 0, 0);
   constraint.setLimit(2, 0, 0);
   constraint.setLimit(3, 0, 0);
   constraint.setLimit(4, 0, 0);
   constraint.setLimit(5, 1, -1);
   constraint.enableMotor(5, true);
   constraint.setTargetVelocity(5, 3.f);
   constraint.setMaxMotorForce(5, 10.f);
   constraint.setServo(5, true);
   constraint.setServoTarget(5, (float) Math.PI / 2.0f);
   constraint.setDbgDrawSize((2.f));
   world().addConstraint(constraint, true);
   m_data.m_ServoMotorConstraint = constraint;
  }
////////// chain of boxes linked together with fully limited rotational and translational constraints
////////// the chain will be pulled to the left and to the right periodically. They should strictly stick together.
  {
   float limitConstraintStrength = 0.6f;
   int bodycount = 10;
   btRigidBody prevBody = null;
   for (int i = 0; i < bodycount; ++i) {
    mass = 1.0f;
    shape = new btBoxShape(new btVector3(0.5f, 0.5f, 0.5f));
    shape.calculateLocalInertia(mass, localInertia);
    bodyTransform.setIdentity();
    bodyTransform.setOrigin(new btVector3(-i, 0f, 3f));
    motionState = new btDefaultMotionState(bodyTransform);
    btRigidBody body = new btRigidBody(mass, motionState, shape, localInertia);
    body.setActivationState(DISABLE_DEACTIVATION);
    world().addRigidBody(body);
    if (prevBody != null) {
     localB.setIdentity();
     localB.setOrigin(new btVector3(0.5f, 0f, 0f));
     localA.setIdentity();
     localA.setOrigin(new btVector3(-0.5f, 0f, 0));
     constraint = new btGeneric6DofSpring2Constraint(prevBody, body, localA, localB);
     constraint.setLimit(0, -0.01f, 0.01f);
     constraint.setLimit(1, 0, 0);
     constraint.setLimit(2, 0, 0);
     constraint.setLimit(3, 0, 0);
     constraint.setLimit(4, 0, 0);
     constraint.setLimit(5, 0, 0);
     for (int a = 0; a < 6; ++a) {
      constraint.setParam(BT_CONSTRAINT_STOP_ERP, 0.9f, a);
      constraint.setParam(BT_CONSTRAINT_STOP_CFM, 0.0f, a);
     }
     constraint.setDbgDrawSize((1.f));
     world().addConstraint(constraint, true);
     if (i < bodycount - 1) {
      localA.setIdentity();
      localA.setOrigin(new btVector3(0f, 0f, 3f));
      localB.setIdentity();
      btGeneric6DofSpring2Constraint constraintZY = new btGeneric6DofSpring2Constraint(staticBody,
       body, localA, localB);
      constraintZY.setLimit(0, 1, -1);
      constraintZY.setDbgDrawSize((1.f));
      world().addConstraint(constraintZY, true);
     }
    } else {
     localA.setIdentity();
     localA.setOrigin(new btVector3(bodycount, 0, 3));
     localB.setIdentity();
     localB.setOrigin(new btVector3());
     m_data.m_ChainLeftBody = body;
     m_data.m_ChainLeftConstraint = new btGeneric6DofSpring2Constraint(staticBody, body, localA,
      localB);
     m_data.m_ChainLeftConstraint.setLimit(3, 0, 0);
     m_data.m_ChainLeftConstraint.setLimit(4, 0, 0);
     m_data.m_ChainLeftConstraint.setLimit(5, 0, 0);
     for (int a = 0; a < 6; ++a) {
      m_data.m_ChainLeftConstraint.setParam(BT_CONSTRAINT_STOP_ERP, limitConstraintStrength, a);
      m_data.m_ChainLeftConstraint.setParam(BT_CONSTRAINT_STOP_CFM, 0.0f, a);
     }
     m_data.m_ChainLeftConstraint.setDbgDrawSize((1.f));
     world().addConstraint(m_data.m_ChainLeftConstraint, true);
    }
    prevBody = body;
   }
   m_data.m_ChainRightBody = prevBody;
   localA.setIdentity();
   localA.setOrigin(new btVector3(-bodycount, 0, 3));
   localB.setIdentity();
   localB.setOrigin(new btVector3());
   m_data.m_ChainRightConstraint = new btGeneric6DofSpring2Constraint(staticBody,
    m_data.m_ChainRightBody, localA, localB);
   m_data.m_ChainRightConstraint.setLimit(3, 0, 0);
   m_data.m_ChainRightConstraint.setLimit(4, 0, 0);
   m_data.m_ChainRightConstraint.setLimit(5, 0, 0);
   for (int a = 0; a < 6; ++a) {
    m_data.m_ChainRightConstraint.setParam(BT_CONSTRAINT_STOP_ERP, limitConstraintStrength, a);
    m_data.m_ChainRightConstraint.setParam(BT_CONSTRAINT_STOP_CFM, 0.0f, a);
   }
  }
 }

 @Override
 public void resetCamera() {
  camera().set(new btQuaternion(5.3701897E-5f, -0.9932216f, -0.11623619f, 4.5962635E-4f),
   new btVector3(0.6305165f, 11.144451f, -44.916996f));
 }

 @Override
 protected JPanel getParams() {
  return null;
 }

 @Override
 public String get_description() {
  return "Show the use of the btGeneric6DofSprint2Constraint. This is a replacement of the btGeneric6DofSpringConstraint, it has various improvements. This includes improved spring implementation and better control over the restitution (bounce) when the constraint hits its limits.\n" +
   "\n" +
   "- Box with undamped translate spring attached to static body the box should oscillate left-to-right forever\n" +
   "\n" +
   "- Box with rotate spring, attached to static body, should swing (rotate) left-to-right forever.\n" +
   "\n" +
   "- Box with bouncing constraint, translation is bounced at the positive x limit, but not at the negative limit bouncing can not be set independently at low and high limits, so two constraints will be created: one that defines the low (non bouncing) limit, and one that defines the high (bouncing) limit the box should move to the left (as an impulse will be applied to it periodically) until it reaches its limit, then bounce back \n" +
   "\n" +
   "- Box with rotational motor, attached to static body. The box should rotate around the y axis.\n" +
   "\n" +
   "- Box with rotational servo motor, attached to static body. The box should rotate around the y axis until it reaches its target. The target will be negated periodically.\n" +
   "\n" +
   "- Chain of boxes linked together with fully limited rotational and translational constraints the chain will be pulled to the left and to the right periodically. They should strictly stick together.\n" +
   "";
 }

 @Override
 protected int getDebugMode() {
  return 0;
 }
 float servoNextFrame = -1;
 float chainNextFrame = -1;
 boolean left = true;
 float bounceNextFrame = -1;

 @Override
 public boolean render_scene() {
/////// servo motor: flip its target periodically
  if (servoNextFrame < 0) {
   m_data.m_ServoMotorConstraint.getRotationalLimitMotor(2).m_servoTarget *= -1;
   servoNextFrame = 3.0f;
  }
  servoNextFrame -= m_data.mDt;
/////// constraint chain: pull the chain left and right periodically
  if (chainNextFrame < 0) {
   if (!left) {
    m_data.m_ChainRightBody.setActivationState(ACTIVE_TAG);
    world().removeConstraint(m_data.m_ChainRightConstraint);
    m_data.m_ChainLeftConstraint.setDbgDrawSize((2.f));
    world().addConstraint(m_data.m_ChainLeftConstraint, true);
   } else {
    m_data.m_ChainLeftBody.setActivationState(ACTIVE_TAG);
    world().removeConstraint(m_data.m_ChainLeftConstraint);
    m_data.m_ChainRightConstraint.setDbgDrawSize((2.f));
    world().addConstraint(m_data.m_ChainRightConstraint, true);
   }
   chainNextFrame = 3.0f;
   left = !left;
  }
  chainNextFrame -= m_data.mDt;
/////// bouncing constraint: push the box periodically
  m_data.m_BouncingTranslateBody.setActivationState(ACTIVE_TAG);
  if (bounceNextFrame < 0) {
   m_data.m_BouncingTranslateBody.applyCentralImpulse(new btVector3(10f, 0f, 0f));
   bounceNextFrame = 3.0f;
  }
  bounceNextFrame -= m_data.mDt;
  m_data.frameID++;
  return super.render_scene(); //To change body of generated methods, choose Tools | Templates.
 }
}
