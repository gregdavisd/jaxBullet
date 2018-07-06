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
package bullet_examples.apps.benchmarks;

import Bullet.Collision.Shape.btCapsuleShape;
import Bullet.Collision.Shape.btCollisionShape;
import Bullet.Dynamics.Constraint.btConeTwistConstraint;
import Bullet.Dynamics.Constraint.btHingeConstraint;
import Bullet.Dynamics.Constraint.btTypedConstraint;
import Bullet.Dynamics.btDynamicsWorld;
import Bullet.Dynamics.CollisionObjects.btRigidBody;
import Bullet.Dynamics.CollisionObjects.btRigidBodyConstructionInfo;
import Bullet.LinearMath.btDefaultMotionState;
import Bullet.LinearMath.btMatrix3x3;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;

/**
 *
 * @author Gregery Barton
 */
public class RagDoll {

 private static final int BODYPART_PELVIS = 0;
 private static final int BODYPART_SPINE = 1;
 private static final int BODYPART_HEAD = 2;
 private static final int BODYPART_LEFT_UPPER_LEG = 3;
 private static final int BODYPART_LEFT_LOWER_LEG = 4;
 private static final int BODYPART_RIGHT_UPPER_LEG = 5;
 private static final int BODYPART_RIGHT_LOWER_LEG = 6;
 private static final int BODYPART_LEFT_UPPER_ARM = 7;
 private static final int BODYPART_LEFT_LOWER_ARM = 8;
 private static final int BODYPART_RIGHT_UPPER_ARM = 9;
 private static final int BODYPART_RIGHT_LOWER_ARM = 10;
 private static final int BODYPART_COUNT = 11;
 private static final int JOINT_PELVIS_SPINE = 0;
 private static final int JOINT_SPINE_HEAD = 1;
 private static final int JOINT_LEFT_HIP = 2;
 private static final int JOINT_LEFT_KNEE = 3;
 private static final int JOINT_RIGHT_HIP = 4;
 private static final int JOINT_RIGHT_KNEE = 5;
 private static final int JOINT_LEFT_SHOULDER = 6;
 private static final int JOINT_LEFT_ELBOW = 7;
 private static final int JOINT_RIGHT_SHOULDER = 8;
 private static final int JOINT_RIGHT_ELBOW = 9;
 private static final int JOINT_COUNT = 10;
 btDynamicsWorld m_ownerWorld;
 btCollisionShape[] m_shapes = new btCollisionShape[BODYPART_COUNT];
 btRigidBody[] m_bodies = new btRigidBody[BODYPART_COUNT];
 btTypedConstraint[] m_joints = new btTypedConstraint[JOINT_COUNT];

 private btRigidBody createRigidBody(float mass,
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

 RagDoll(btDynamicsWorld ownerWorld, final btVector3 positionOffset, float scale) {
  m_ownerWorld = (ownerWorld);
  // Setup the geometry
  m_shapes[BODYPART_PELVIS] = new btCapsuleShape((0.15f) * scale, (0.20f)
   * scale);
  m_shapes[BODYPART_SPINE] = new btCapsuleShape((0.15f) * scale, (0.28f) * scale);
  m_shapes[BODYPART_HEAD] = new btCapsuleShape((0.10f) * scale, (0.05f) * scale);
  m_shapes[BODYPART_LEFT_UPPER_LEG] = new btCapsuleShape((0.07f) * scale,
   (0.45f) * scale);
  m_shapes[BODYPART_LEFT_LOWER_LEG] = new btCapsuleShape((0.05f) * scale,
   (0.37f) * scale);
  m_shapes[BODYPART_RIGHT_UPPER_LEG] = new btCapsuleShape((0.07f) * scale,
   (0.45f) * scale);
  m_shapes[BODYPART_RIGHT_LOWER_LEG] = new btCapsuleShape((0.05f) * scale,
   (0.37f) * scale);
  m_shapes[BODYPART_LEFT_UPPER_ARM] = new btCapsuleShape((0.05f) * scale,
   (0.33f) * scale);
  m_shapes[BODYPART_LEFT_LOWER_ARM] = new btCapsuleShape((0.04f) * scale,
   (0.25f) * scale);
  m_shapes[BODYPART_RIGHT_UPPER_ARM] = new btCapsuleShape((0.05f) * scale,
   (0.33f) * scale);
  m_shapes[BODYPART_RIGHT_LOWER_ARM] = new btCapsuleShape((0.04f) * scale,
   (0.25f) * scale);
  // Setup all the rigid bodies
  final btTransform offset = new btTransform();
  offset.setIdentity();
  offset.setOrigin(positionOffset);
  final btTransform transform = new btTransform();
  transform.setIdentity();
  transform.setOrigin(new btVector3((0.f), (1.f), (0.f)).scale(scale));
  m_bodies[BODYPART_PELVIS] = createRigidBody((1.f), new btTransform(offset)
   .mul(transform),
   m_shapes[BODYPART_PELVIS]);
  transform.setIdentity();
  transform.setOrigin(new btVector3((0.f), (1.2f), (0.f)).scale(scale));
  m_bodies[BODYPART_SPINE] = createRigidBody((1.f), new btTransform(offset).mul(
   transform),
   m_shapes[BODYPART_SPINE]);
  transform.setIdentity();
  transform.setOrigin(new btVector3((0.f), (1.6f), (0.f)).scale(scale));
  m_bodies[BODYPART_HEAD] = createRigidBody((1.f), new btTransform(offset).mul(
   transform),
   m_shapes[BODYPART_HEAD]);
  transform.setIdentity();
  transform.setOrigin(new btVector3((-0.18f), (0.65f), (0.f)).scale(scale));
  m_bodies[BODYPART_LEFT_UPPER_LEG] = createRigidBody((1.f), new btTransform(
   offset).mul(transform),
   m_shapes[BODYPART_LEFT_UPPER_LEG]);
  transform.setIdentity();
  transform.setOrigin(new btVector3((-0.18f), (0.2f), (0.f)).scale(scale));
  m_bodies[BODYPART_LEFT_LOWER_LEG] = createRigidBody((1.f), new btTransform(
   offset).mul(transform),
   m_shapes[BODYPART_LEFT_LOWER_LEG]);
  transform.setIdentity();
  transform.setOrigin(new btVector3((0.18f), (0.65f), (0.f)).scale(scale));
  m_bodies[BODYPART_RIGHT_UPPER_LEG]
   = createRigidBody((1.f), new btTransform(offset).mul(transform),
    m_shapes[BODYPART_RIGHT_UPPER_LEG]);
  transform.setIdentity();
  transform.setOrigin(new btVector3((0.18f), (0.2f), (0.f)).scale(scale));
  m_bodies[BODYPART_RIGHT_LOWER_LEG]
   = createRigidBody((1.f), new btTransform(offset).mul(transform),
    m_shapes[BODYPART_RIGHT_LOWER_LEG]);
  transform.setIdentity();
  transform.setOrigin(new btVector3((-0.35f), (1.45f), (0.f)).scale(scale));
  transform
   .setBasis(new btMatrix3x3().setEulerZYX(0, 0, (float) Math.PI / 2.0f));
  m_bodies[BODYPART_LEFT_UPPER_ARM] = createRigidBody((1.f), new btTransform(
   offset).mul(transform),
   m_shapes[BODYPART_LEFT_UPPER_ARM]);
  transform.setIdentity();
  transform.setOrigin(new btVector3((-0.7f), (1.45f), (0.f)).scale(scale));
  transform
   .setBasis(new btMatrix3x3().setEulerZYX(0, 0, (float) Math.PI / 2.0f));
  m_bodies[BODYPART_LEFT_LOWER_ARM] = createRigidBody((1.f), new btTransform(
   offset).mul(transform),
   m_shapes[BODYPART_LEFT_LOWER_ARM]);
  transform.setIdentity();
  transform.setOrigin(new btVector3((0.35f), (1.45f), (0.f)).scale(scale));
  transform.setBasis(new btMatrix3x3()
   .setEulerZYX(0, 0, -(float) Math.PI / 2.0f));
  m_bodies[BODYPART_RIGHT_UPPER_ARM]
   = createRigidBody((1.f), new btTransform(offset).mul(transform),
    m_shapes[BODYPART_RIGHT_UPPER_ARM]);
  transform.setIdentity();
  transform.setOrigin(new btVector3((0.7f), (1.45f), (0.f)).scale(scale));
  transform.setBasis(new btMatrix3x3()
   .setEulerZYX(0, 0, -(float) Math.PI / 2.0f));
  m_bodies[BODYPART_RIGHT_LOWER_ARM]
   = createRigidBody((1.f), new btTransform(offset).mul(transform),
    m_shapes[BODYPART_RIGHT_LOWER_ARM]);
  // Setup some damping on the m_bodies
  for (int i = 0; i < BODYPART_COUNT; ++i) {
   m_bodies[i].setDamping((0.05f), (0.85f));
   m_bodies[i].setDeactivationTime((0.8f));
   m_bodies[i].setSleepingThresholds((1.6f), (2.5f));
  }
  // Now setup the constraints
  btHingeConstraint hingeC;
  btConeTwistConstraint coneC;
  final btTransform localA = new btTransform();
  final btTransform localB = new btTransform();
  localA.setIdentity();
  localB.setIdentity();
  localA.setBasis(new btMatrix3x3().setEulerZYX(0f, (float) Math.PI / 2.0f, 0f));
  localA.setOrigin(new btVector3((0.f), (0.15f), (0.f)).scale(scale));
  localB.setBasis(new btMatrix3x3().setEulerZYX(0, (float) Math.PI / 2.0f, 0));
  localB.setOrigin(new btVector3((0.f), (-0.15f), (0.f)).scale(scale));
  hingeC
   = new btHingeConstraint(m_bodies[BODYPART_PELVIS], m_bodies[BODYPART_SPINE],
    localA, localB);
  hingeC.setLimit((-(float) Math.PI / 4.0f), ((float) Math.PI / 2.0f));
  m_joints[JOINT_PELVIS_SPINE] = hingeC;
  m_ownerWorld.addConstraint(m_joints[JOINT_PELVIS_SPINE], true);
  localA.setIdentity();
  localB.setIdentity();
  localA.setBasis(new btMatrix3x3().setEulerZYX(0, 0, (float) Math.PI / 2.0f));
  localA.setOrigin(new btVector3((0.f), (0.30f), (0.f)).scale(scale));
  localB.setBasis(new btMatrix3x3().setEulerZYX(0f, 0f, (float) Math.PI / 2.0f));
  localB.setOrigin(new btVector3((0.f), (-0.14f), (0.f)).scale(scale));
  coneC = new btConeTwistConstraint(m_bodies[BODYPART_SPINE],
   m_bodies[BODYPART_HEAD], localA,
   localB);
  coneC.setLimit((float) Math.PI / 4.0f, (float) Math.PI / 4.0f, (float) Math.PI
   / 2.0f);
  m_joints[JOINT_SPINE_HEAD] = coneC;
  m_ownerWorld.addConstraint(m_joints[JOINT_SPINE_HEAD], true);
  localA.setIdentity();
  localB.setIdentity();
  localA.setBasis(new btMatrix3x3().setEulerZYX(0, 0, -(float) Math.PI / 4.0f
   * 5));
  localA.setOrigin(new btVector3((-0.18f), (-0.10f), (0.f)).scale(scale));
  localB.setBasis(new btMatrix3x3().setEulerZYX(0f, 0f, -(float) Math.PI / 4.0f
   * 5));
  localB.setOrigin(new btVector3((0.f), (0.225f), (0.f)).scale(scale));
  coneC = new btConeTwistConstraint(m_bodies[BODYPART_PELVIS],
   m_bodies[BODYPART_LEFT_UPPER_LEG],
   localA, localB);
  coneC.setLimit((float) Math.PI / 4.0f, (float) Math.PI / 4.0f, 0);
  m_joints[JOINT_LEFT_HIP] = coneC;
  m_ownerWorld.addConstraint(m_joints[JOINT_LEFT_HIP], true);
  localA.setIdentity();
  localB.setIdentity();
  localA.setBasis(new btMatrix3x3().setEulerZYX(0f, (float) Math.PI / 2.0f, 0f));
  localA.setOrigin(new btVector3((0.f), (-0.225f), (0.f)).scale(scale));
  localB.setBasis(new btMatrix3x3().setEulerZYX(0f, (float) Math.PI / 2.0f, 0f));
  localB.setOrigin(new btVector3((0.f), (0.185f), (0.f)).scale(scale));
  hingeC = new btHingeConstraint(m_bodies[BODYPART_LEFT_UPPER_LEG],
   m_bodies[BODYPART_LEFT_LOWER_LEG], localA, localB);
  hingeC.setLimit((0f), ((float) Math.PI / 2.0f));
  m_joints[JOINT_LEFT_KNEE] = hingeC;
  m_ownerWorld.addConstraint(m_joints[JOINT_LEFT_KNEE], true);
  localA.setIdentity();
  localB.setIdentity();
  localA.setBasis(new btMatrix3x3().setEulerZYX(0f, 0f, (float) Math.PI / 4.0f));
  localA.setOrigin(new btVector3((0.18f), (-0.10f), (0.f)).scale(scale));
  localB.setBasis(new btMatrix3x3().setEulerZYX(0f, 0f, (float) Math.PI / 4.0f));
  localB.setOrigin(new btVector3((0.f), (0.225f), (0.f)).scale(scale));
  coneC = new btConeTwistConstraint(m_bodies[BODYPART_PELVIS],
   m_bodies[BODYPART_RIGHT_UPPER_LEG],
   localA, localB);
  coneC.setLimit((float) Math.PI / 4.0f, (float) Math.PI / 4.0f, 0);
  m_joints[JOINT_RIGHT_HIP] = coneC;
  m_ownerWorld.addConstraint(m_joints[JOINT_RIGHT_HIP], true);
  localA.setIdentity();
  localB.setIdentity();
  localA.setBasis(new btMatrix3x3().setEulerZYX(0f, (float) Math.PI / 2.0f, 0f));
  localA.setOrigin(new btVector3((0.f), (-0.225f), (0.f)).scale(scale));
  localB.setBasis(new btMatrix3x3().setEulerZYX(0, (float) Math.PI / 2.0f, 0));
  localB.setOrigin(new btVector3((0.f), (0.185f), (0.f)).scale(scale));
  hingeC = new btHingeConstraint(m_bodies[BODYPART_RIGHT_UPPER_LEG],
   m_bodies[BODYPART_RIGHT_LOWER_LEG], localA, localB);
  hingeC.setLimit((0f), ((float) Math.PI / 2.0f));
  m_joints[JOINT_RIGHT_KNEE] = hingeC;
  m_ownerWorld.addConstraint(m_joints[JOINT_RIGHT_KNEE], true);
  localA.setIdentity();
  localB.setIdentity();
  localA.setBasis(new btMatrix3x3().setEulerZYX(0, 0, (float) Math.PI));
  localA.setOrigin(new btVector3((-0.2f), (0.15f), (0.f)).scale(scale));
  localB.setBasis(new btMatrix3x3().setEulerZYX(0f, 0f, (float) Math.PI / 2.0f));
  localB.setOrigin(new btVector3((0.f), (-0.18f), (0.f)).scale(scale));
  coneC = new btConeTwistConstraint(m_bodies[BODYPART_SPINE],
   m_bodies[BODYPART_LEFT_UPPER_ARM],
   localA, localB);
  coneC.setLimit((float) Math.PI / 2.0f, (float) Math.PI / 2.0f, 0);
  m_joints[JOINT_LEFT_SHOULDER] = coneC;
  m_ownerWorld.addConstraint(m_joints[JOINT_LEFT_SHOULDER], true);
  localA.setIdentity();
  localB.setIdentity();
  localA.setBasis(new btMatrix3x3().setEulerZYX(0f, (float) Math.PI / 2.0f, 0f));
  localA.setOrigin(new btVector3((0.f), (0.18f), (0.f)).scale(scale));
  localB.setBasis(new btMatrix3x3().setEulerZYX(0f, (float) Math.PI / 2.0f, 0f));
  localB.setOrigin(new btVector3((0.f), (-0.14f), (0.f)).scale(scale));
  hingeC = new btHingeConstraint(m_bodies[BODYPART_LEFT_UPPER_ARM],
   m_bodies[BODYPART_LEFT_LOWER_ARM], localA, localB);
  hingeC.setLimit((-(float) Math.PI / 2.0f), (0f));
  m_joints[JOINT_LEFT_ELBOW] = hingeC;
  m_ownerWorld.addConstraint(m_joints[JOINT_LEFT_ELBOW], true);
  localA.setIdentity();
  localB.setIdentity();
  localA.setBasis(new btMatrix3x3().setEulerZYX(0, 0, 0));
  localA.setOrigin(new btVector3((0.2f), (0.15f), (0.f)).scale(scale));
  localB.setBasis(new btMatrix3x3().setEulerZYX(0, 0, (float) Math.PI / 2.0f));
  localB.setOrigin(new btVector3((0.f), (-0.18f), (0.f)).scale(scale));
  coneC = new btConeTwistConstraint(m_bodies[BODYPART_SPINE],
   m_bodies[BODYPART_RIGHT_UPPER_ARM],
   localA, localB);
  coneC.setLimit((float) Math.PI / 2.0f, (float) Math.PI / 2.0f, 0);
  m_joints[JOINT_RIGHT_SHOULDER] = coneC;
  m_ownerWorld.addConstraint(m_joints[JOINT_RIGHT_SHOULDER], true);
  localA.setIdentity();
  localB.setIdentity();
  localA.setBasis(new btMatrix3x3().setEulerZYX(0, (float) Math.PI / 2.0f, 0));
  localA.setOrigin(new btVector3((0.f), (0.18f), (0.f)).scale(scale));
  localB.setBasis(new btMatrix3x3().setEulerZYX(0, (float) Math.PI / 2.0f, 0));
  localB.setOrigin(new btVector3((0.f), (-0.14f), (0.f)).scale(scale));
  hingeC = new btHingeConstraint(m_bodies[BODYPART_RIGHT_UPPER_ARM],
   m_bodies[BODYPART_RIGHT_LOWER_ARM], localA, localB);
  hingeC.setLimit((-(float) Math.PI / 2.0f), (0f));
  m_joints[JOINT_RIGHT_ELBOW] = hingeC;
  m_ownerWorld.addConstraint(m_joints[JOINT_RIGHT_ELBOW], true);
 }

}
