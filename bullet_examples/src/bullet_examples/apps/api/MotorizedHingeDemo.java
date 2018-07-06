/*
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2015 Erwin Coumans  http://continuousphysics.com/Bullet/
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
import static Bullet.Collision.btCollisionObject.DISABLE_DEACTIVATION;
import static Bullet.Collision.btIDebugDraw.DBG_DrawConstraintLimits;
import static Bullet.Collision.btIDebugDraw.DBG_DrawConstraints;
import static Bullet.Collision.btIDebugDraw.DBG_DrawWireframe;
import Bullet.Dynamics.Constraint.btHingeAccumulatedAngleConstraint;
import Bullet.Dynamics.Constraint.btHingeConstraint;
import Bullet.Dynamics.CollisionObjects.btRigidBody;
import Bullet.LinearMath.btQuaternion;
import static Bullet.LinearMath.btScalar.SIMD_DEGS_PER_RAD;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.btPlaneSpace1;
import bullet_examples.DiscreteDemoContainer;
import javax.swing.JPanel;

/**
 *
 * @author Gregery Barton
 */
public class MotorizedHingeDemo extends DiscreteDemoContainer {

 static float val;
 static float targetVel = 0;
 static float maxImpulse = 10000;
 static btHingeAccumulatedAngleConstraint spDoorHinge = null;
 static float actualHingeVelocity = 0.f;
 MotorizedHingeParams params;
 static btVector3 btAxisA = new btVector3(0f, 1f, 0f);

 @Override
 public void initPhysics() {
  setUpAxis(1);
  set_debug_flag(true, DBG_DrawWireframe);
  { // create a door using hinge constraint attached to the world
   btCollisionShape pDoorShape = new btBoxShape(new btVector3(2.0f, 5.0f, 0.2f));
   final btTransform doorTrans = new btTransform();
   doorTrans.setIdentity();
   doorTrans.setOrigin(new btVector3(-5.0f, -2.0f, 0.0f));
   btRigidBody pDoorBody = createRigidBody(1.0f, doorTrans, pDoorShape);
   pDoorBody.setActivationState(DISABLE_DEACTIVATION);
   final btVector3 btPivotA = new btVector3(10.f + 2.1f, -2.0f, 0.0f); // right next to the door slightly outside
   spDoorHinge = new btHingeAccumulatedAngleConstraint(pDoorBody, btPivotA,
    btAxisA);
   world().addConstraint(spDoorHinge);
   spDoorHinge.setDbgDrawSize((5.f));
  }
 }

 @Override
 public boolean render_scene() {
  val = spDoorHinge.getAccumulatedHingeAngle() * SIMD_DEGS_PER_RAD;
  if (params != null) {
   targetVel = params.get_target_vel();
   maxImpulse = params.get_max_impulse();
  }
  if (world() != null) {
   spDoorHinge.enableAngularMotor(true, targetVel, maxImpulse);
   btHingeConstraint hinge = spDoorHinge;
   if (hinge != null) {
    btRigidBody bodyA = hinge.getRigidBodyA();
    btRigidBody bodyB = hinge.getRigidBodyB();
    final btTransform trA = bodyA.getWorldTransform();
    final btVector3 angVelA = bodyA.getAngularVelocity();
    final btVector3 angVelB = bodyB.getAngularVelocity();
    {
     final btVector3 ax1 = trA.transform3x3(hinge.getFrameOffsetA()
      .getBasisColumn(2));
     float vel = angVelA.dot(ax1);
     vel -= angVelB.dot(ax1);
     //printf("hinge velocity (q) = %f\n", vel);
     actualHingeVelocity = vel;
    }
    final btVector3 ortho0 = new btVector3();
    final btVector3 ortho1 = new btVector3();
    btPlaneSpace1(btAxisA, ortho0, ortho1);
   }
  }
  if (params != null) {
   params.set_actual_vel(actualHingeVelocity);
   params.set_angle(val);
  }
  return super.render_scene();
 }

 @Override
 public void resetCamera() {
  camera().set(new btQuaternion(0.0777956f, -0.056463785f, -0.004413125f,
   0.99535936f),
   new btVector3(16.957514f, 11.647476f, 85.99799f));
 }

 @Override
 protected int getDebugMode() {
  return DBG_DrawConstraintLimits | DBG_DrawConstraints;
 }

 @Override
 protected JPanel getParams() {
  params = new MotorizedHingeParams();
  return params;
 }

 @Override
 public String get_description() {
  return "Adjust the sliders and the hinged door will go round and round and round.  "
   + "The btHingeAccumulatedAngleConstraint accumulates all of its rotation so you know how many times it has rotated"
   + " in a particular direction and the accumulated rotation can be undone by rotating the other way. "
   + "Try to get the door to fly off its hinge by setting the iterations too low"
   + " and manipulating the sliders.";
 }

}
