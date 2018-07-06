/*
 * Bullet Continuous Collision Detection and Physics Library Copyright (c) 2007 Erwin Coumans
 * Motor Demo
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
import Bullet.Collision.Shape.btCapsuleShape;
import Bullet.Collision.Shape.btCollisionShape;
import Bullet.Dynamics.Constraint.btHingeConstraint;
import Bullet.Dynamics.Constraint.btTypedConstraint;
import Bullet.Dynamics.btDynamicsWorld;
import Bullet.Dynamics.btInternalTickCallback;
import Bullet.Dynamics.CollisionObjects.btRigidBody;
import Bullet.Dynamics.CollisionObjects.btRigidBodyConstructionInfo;
import Bullet.LinearMath.btDefaultMotionState;
import Bullet.LinearMath.btMatrix3x3;
import Bullet.LinearMath.btQuaternion;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import bullet_examples.DiscreteDemoContainer;
import java.util.ArrayList;
import javax.swing.JPanel;
import static javax.vecmath.VecMath.cos;
import static javax.vecmath.VecMath.sin;

/**
 *
 * @author Gregery Barton
 */
public class MotorDemo extends DiscreteDemoContainer {

 private static final int NUM_LEGS = 6;
 private static final int BODYPART_COUNT = 2 * NUM_LEGS + 1;
 private static final int JOINT_COUNT = BODYPART_COUNT - 1;
 float m_Time;
 float m_fCyclePeriod; // in milliseconds
 float m_fMuscleStrength;
 ArrayList<TestRig> m_rigs = new ArrayList<>(0);

 void motorPreTickCallback(btDynamicsWorld world, float timeStep) {
  MotorDemo motorDemo = (MotorDemo) world.getWorldUserInfo();
  motorDemo.setMotorTargets(timeStep);
 }

 void setMotorTargets(float deltaTime) {
  float ms = deltaTime * 1000000.f;
  float minFPS = 1000000.f / 60.f;
  if (ms > minFPS) {
   ms = minFPS;
  }
  m_Time += ms;
  //
  // set per-frame sinusoidal position targets using angular motor (hacky?)
  //	
  for (int r = 0; r < m_rigs.size(); r++) {
   for (int i = 0; i < 2 * NUM_LEGS; i++) {
    btHingeConstraint hingeC = (btHingeConstraint) (m_rigs.get(r).GetJoints()[i]);
    float fCurAngle = hingeC.getHingeAngle();
    float fTargetPercent = ((int) (m_Time / 1000) % (int) (m_fCyclePeriod))
     / m_fCyclePeriod;;
    float fTargetAngle = 0.5f
     * (1f + sin(2f * (float) Math.PI * fTargetPercent));
    float fTargetLimitAngle = hingeC.getLowerLimit() + fTargetAngle * (hingeC
     .getUpperLimit() - hingeC.getLowerLimit());
    float fAngleError = fTargetLimitAngle - fCurAngle;
    float fDesiredAngularVel = 1000000.f * fAngleError / ms;
    hingeC.enableAngularMotor(true, fDesiredAngularVel, m_fMuscleStrength);
   }
  }
 }

 @Override
 public void initPhysics() {
  setUpAxis(1);
  // Setup the basic world
  m_Time = 0;
  m_fCyclePeriod = 2000.f; // in milliseconds
//	m_fMuscleStrength = 0.05f;
  // new SIMD solver for joints clips accumulated impulse, so the new limits for the motor
  // should be (numberOfsolverIterations * oldLimits)
  // currently solver uses 10 iterations, so:
  m_fMuscleStrength = 0.5f;
  world().setInternalTickCallback(new btInternalTickCallback() {
   @Override
   public void callback(btDynamicsWorld world, float timeStep) {
    motorPreTickCallback(world, timeStep);
   }

  }, this, true);
  // Setup a big ground box
  {
   btCollisionShape groundShape = new btBoxShape(new btVector3((200.f), (10.f),
    (200.f)));
   final btTransform groundTransform = new btTransform();
   groundTransform.setIdentity();
   groundTransform.setOrigin(new btVector3(0, -10f, 0));
   createRigidBody((0.f), groundTransform, groundShape);
  }
  // Spawn one ragdoll
  final btVector3 startOffset = new btVector3(1f, 0.5f, 0f);
  spawnTestRig(startOffset, false);
  startOffset.set(-2f, 0.5f, 0f);
  spawnTestRig(startOffset, true);
 }

 void spawnTestRig(final btVector3 startOffset, boolean bFixed) {
  TestRig rig = new TestRig(world(), startOffset, bFixed);
  m_rigs.add(rig);
 }

 @Override
 public void resetCamera() {
  camera().set(new btQuaternion(0.07050943f, 0.87723315f, 0.13594612f,
   0.4549826f), new btVector3(
   -17.954006f, 7.447424f, -12.084393f));
 }

 @Override
 protected int getDebugMode() {
  return 0;
 }

 @Override
 protected JPanel getParams() {
  return null;
 }

 @Override
 public String get_description() {
  return "Dynamic control the target velocity of a motor of a btHingeConstraint."
   + " This demo makes use of the 'internal tick callback.";
 }

 static class TestRig {

  btDynamicsWorld m_ownerWorld;
  btCollisionShape[] m_shapes = new btCollisionShape[BODYPART_COUNT];
  btRigidBody[] m_bodies = new btRigidBody[BODYPART_COUNT];
  btTypedConstraint[] m_joints = new btTypedConstraint[JOINT_COUNT];

  private btRigidBody localCreateRigidBody(float mass,
   final btTransform startTransform,
   btCollisionShape shape) {
   boolean isDynamic = (mass != 0.f);
   final btVector3 localInertia = new btVector3();
   if (isDynamic) {
    shape.calculateLocalInertia(mass, localInertia);
   }
   btDefaultMotionState myMotionState = new btDefaultMotionState(startTransform);
   btRigidBodyConstructionInfo rbInfo = new btRigidBodyConstructionInfo(mass,
    myMotionState, shape,
    localInertia);
   btRigidBody body = new btRigidBody(rbInfo);
   m_ownerWorld.addRigidBody(body);
   return body;
  }

  TestRig(btDynamicsWorld ownerWorld, final btVector3 positionOffset,
   boolean bFixed) {
   m_ownerWorld = (ownerWorld);
   final btVector3 vUp = new btVector3(0, 1, 0);
   //
   // Setup geometry
   //
   float fBodySize = 0.25f;
   float fLegLength = 0.45f;
   float fForeLegLength = 0.75f;
   m_shapes[0] = new btCapsuleShape((fBodySize), (0.10f));
   int i;
   for (i = 0; i < NUM_LEGS; i++) {
    m_shapes[1 + 2 * i] = new btCapsuleShape((0.10f), (fLegLength));
    m_shapes[2 + 2 * i] = new btCapsuleShape((0.08f), (fForeLegLength));
   }
   //
   // Setup rigid bodies
   //
   float fHeight = 0.5f;
   final btTransform offset = new btTransform();
   offset.setIdentity();
   offset.setOrigin(positionOffset);
   // root
   final btVector3 vRoot = new btVector3((0.f), (fHeight), (0.f));
   final btTransform transform = new btTransform();
   transform.setIdentity();
   transform.setOrigin(vRoot);
   if (bFixed) {
    m_bodies[0] = localCreateRigidBody((0.f), new btTransform(offset).mul(
     transform), m_shapes[0]);
   } else {
    m_bodies[0] = localCreateRigidBody((1.f), new btTransform(offset).mul(
     transform), m_shapes[0]);
   }
   // legs
   for (i = 0; i < NUM_LEGS; i++) {
    float fAngle = 2f * (float) Math.PI * i / NUM_LEGS;
    float fSin = sin(fAngle);
    float fCos = cos(fAngle);
    transform.setIdentity();
    final btVector3 vBoneOrigin = new btVector3((fCos * (fBodySize + 0.5f
     * fLegLength)), (fHeight),
     (fSin * (fBodySize + 0.5f * fLegLength)));
    transform.setOrigin(vBoneOrigin);
    // thigh
    final btVector3 vToBone = (new btVector3(vBoneOrigin).sub(vRoot))
     .normalize();
    final btVector3 vAxis = new btVector3(vToBone).cross(vUp);
    transform.set3x3(new btQuaternion(vAxis, (float) Math.PI / 2.0f));
    m_bodies[1 + 2 * i] = localCreateRigidBody((1.f), new btTransform(offset)
     .mul(transform),
     m_shapes[1 + 2 * i]);
    // shin
    transform.setIdentity();
    transform.setOrigin(new btVector3((fCos * (fBodySize + fLegLength)),
     (fHeight - 0.5f * fForeLegLength), (fSin * (fBodySize + fLegLength))));
    m_bodies[2 + 2 * i] = localCreateRigidBody((1.f), new btTransform(offset)
     .mul(transform),
     m_shapes[2 + 2 * i]);
   }
   // Setup some damping on the m_bodies
   for (i = 0; i < BODYPART_COUNT; ++i) {
    m_bodies[i].setDamping(0.05f, 0.85f);
    m_bodies[i].setDeactivationTime(0.8f);
    //m_bodies[i].setSleepingThresholds(1.6, 2.5);
    m_bodies[i].setSleepingThresholds(0.5f, 0.5f);
   }
   //
   // Setup the constraints
   //
   btHingeConstraint hingeC;
   //btConeTwistConstraint* coneC;
   final btTransform localA = new btTransform();
   final btTransform localB = new btTransform();
   final btTransform localC = new btTransform();
   for (i = 0; i < NUM_LEGS; i++) {
    float fAngle = 2 * (float) Math.PI * i / NUM_LEGS;
    float fSin = sin(fAngle);
    float fCos = cos(fAngle);
    // hip joints
    localA.setIdentity();
    localB.setIdentity();
    localA.setBasis(new btMatrix3x3().setEulerZYX(0f, -fAngle, 0f));
    localA.setOrigin(
     new btVector3((fCos * fBodySize), (0.f), (fSin * fBodySize)));
    localB.set(m_bodies[1 + 2 * i].getWorldTransform().invert().mul(m_bodies[0]
     .getWorldTransform())
     .mul(localA));
    hingeC = new btHingeConstraint(m_bodies[0], m_bodies[1 + 2 * i], localA,
     localB);
    hingeC.setLimit((-0.75f * (float) Math.PI / 4.0f), ((float) Math.PI / 8.0f));
    //hingeC.setLimit(float(-0.1), float(0.1));
    m_joints[2 * i] = hingeC;
    m_ownerWorld.addConstraint(m_joints[2 * i], true);
    // knee joints
    localA.setIdentity();
    localB.setIdentity();
    localC.setIdentity();
    localA.setBasis(new btMatrix3x3().setEulerZYX(0f, -fAngle, 0f));
    localA.setOrigin(new btVector3((fCos * (fBodySize + fLegLength)), (0.f),
     (fSin * (fBodySize + fLegLength))));
    localB.set(m_bodies[1 + 2 * i].getWorldTransform().invert().mul(m_bodies[0]
     .getWorldTransform())
     .mul(localA));
    localC.set(m_bodies[2 + 2 * i].getWorldTransform().invert().mul(m_bodies[0]
     .getWorldTransform())
     .mul(localA));
    hingeC = new btHingeConstraint(m_bodies[1 + 2 * i], m_bodies[2 + 2 * i],
     localB, localC);
    //hingeC.setLimit(float(-0.01), float(0.01));
    hingeC.setLimit((float) -Math.PI / 8.0f, (0.2f));
    m_joints[1 + 2 * i] = hingeC;
    m_ownerWorld.addConstraint(m_joints[1 + 2 * i], true);
   }
  }

  btTypedConstraint[] GetJoints() {
   return m_joints;
  }

 };
}
