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
package bullet_examples.apps.api;

import Bullet.Collision.Broadphase.btBroadphaseProxy;
import Bullet.Collision.Shape.btBoxShape;
import Bullet.Collision.Shape.btCollisionShape;
import Bullet.Collision.Shape.btSphereShape;
import static Bullet.Collision.btIDebugDraw.DBG_DrawConstraintLimits;
import static Bullet.Collision.btIDebugDraw.DBG_DrawConstraints;
import static Bullet.Collision.btIDebugDraw.DBG_DrawWireframe;
import Bullet.Dynamics.Constraint.btGeneric6DofSpring2Constraint;
import Bullet.Dynamics.Constraint.btHingeConstraint;
import Bullet.Dynamics.Constraint.btTypedConstraint;
import Bullet.Dynamics.btJointFeedback;
import Bullet.Dynamics.btRigidBody;
import Bullet.LinearMath.btQuaternion;
import static Bullet.LinearMath.btScalar.SIMD_PI;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import bullet_examples.DiscreteDemoContainer;
import java.util.ArrayList;
import javax.swing.JPanel;

/**
 *
 * @author Gregery Barton
 */
public class TestHingeTorque extends DiscreteDemoContainer {

 boolean m_once = true;
 final ArrayList<btJointFeedback> m_jointFeedback = new ArrayList<>(0);
 int collisionFilterGroup = (btBroadphaseProxy.CharacterFilter);
 int collisionFilterMask = (btBroadphaseProxy.AllFilter ^ (btBroadphaseProxy.CharacterFilter));
 static float radius = 0.2f;

 @Override
 protected int getDebugMode() {
  return DBG_DrawConstraints | DBG_DrawConstraintLimits;
 }

 public void initPhysics() {
 int upAxis = 1;
  setUpAxis(upAxis);
  world().getSolverInfo().m_splitImpulse = false;
  world().setGravity(new btVector3(0f, 0f, -10f));
  set_debug_flag(true, DBG_DrawWireframe);
  { // create a door using hinge constraint attached to the world
   int numLinks = 2;
   //    bool selfCollide = false;
   final btVector3 linkHalfExtents = new btVector3(0.05f, 0.37f, 0.1f);
   final btVector3 baseHalfExtents = new btVector3(0.05f, 0.37f, 0.1f);
   btBoxShape baseBox = new btBoxShape(baseHalfExtents);
   final btVector3 basePosition = new btVector3(-0.4f, 3.f, 0.f);
   final btTransform baseWorldTrans = new btTransform();
   baseWorldTrans.setIdentity();
   baseWorldTrans.setOrigin(basePosition);
   //mbC.forceMultiDof();							//if !spherical, you can comment this line to check the 1DoF algorithm
   //init the base
   final btVector3 baseInertiaDiag = new btVector3();
   float baseMass = 0.f;
   float linkMass = 1.f;
   btRigidBody base = createRigidBody(baseMass, baseWorldTrans, baseBox);
   world().removeRigidBody(base);
   base.setDamping(0, 0);
   world().addRigidBody(base, collisionFilterGroup, collisionFilterMask);
   btBoxShape linkBox1 = new btBoxShape(linkHalfExtents);
   btSphereShape linkSphere = new btSphereShape(radius);
   btRigidBody prevBody = base;
   for (int i = 0; i < numLinks; i++) {
    final btTransform linkTrans = new btTransform();
    linkTrans.set(baseWorldTrans);
    linkTrans.setOrigin(new btVector3(basePosition).sub(new btVector3(0, linkHalfExtents.y * 2.f *
     (i + 1), 0)));
    btCollisionShape colOb;
    if (i == 0) {
     colOb = linkBox1;
    } else {
     colOb = linkSphere;
    }
    btRigidBody linkBody = createRigidBody(linkMass, linkTrans, colOb);
    world().removeRigidBody(linkBody);
    world().addRigidBody(linkBody, collisionFilterGroup, collisionFilterMask);
    linkBody.setDamping(0, 0);
    btTypedConstraint con;
    if (i == 0) {
     //create a hinge constraint
     final btVector3 pivotInA = new btVector3(0, -linkHalfExtents.y, 0);
     final btVector3 pivotInB = new btVector3(0, linkHalfExtents.y, 0);
     final btVector3 axisInA = new btVector3(1, 0, 0);
     final btVector3 axisInB = new btVector3(1, 0, 0);
     boolean useReferenceA = true;
     btHingeConstraint hinge = new btHingeConstraint(prevBody, linkBody,
      pivotInA, pivotInB,
      axisInA, axisInB, useReferenceA);
     con = hinge;
    } else {
     final btTransform pivotInA = new btTransform(btQuaternion.getIdentity(), new btVector3(0f,
      -radius, 0f));						//par body's COM to cur body's COM offset
     final btTransform pivotInB = new btTransform(btQuaternion.getIdentity(), new btVector3(0f,
      radius, 0f));							//cur body's COM to cur body's PIV offset
     btGeneric6DofSpring2Constraint fixed = new btGeneric6DofSpring2Constraint(prevBody, linkBody,
      pivotInA, pivotInB);
     fixed.setLinearLowerLimit(new btVector3());
     fixed.setLinearUpperLimit(new btVector3());
     fixed.setAngularLowerLimit(new btVector3());
     fixed.setAngularUpperLimit(new btVector3());
     con = fixed;
    }
    assert (con != null);
//			if (con!=null)
    {
     btJointFeedback fb = new btJointFeedback();
     m_jointFeedback.add(fb);
     con.setJointFeedback(fb);
     world().addConstraint(con, true);
    }
    prevBody = linkBody;
   }
  }
//	if (1)
  {
   final btVector3 groundHalfExtents = new btVector3(1f, 1f, 0.2f);
   groundHalfExtents.setElement(upAxis, 1.f);
   btBoxShape box = new btBoxShape(groundHalfExtents);
   box.initializePolyhedralFeatures();
   final btTransform start = new btTransform();
   start.setIdentity();
   final btVector3 groundOrigin = new btVector3(-0.4f, 3.f, 0.f);
   //	btVector3 basePosition = btVector3(-0.4f, 3.f, 0.f);
   final btQuaternion groundOrn = new btQuaternion(new btVector3(0f, 1f, 0f), 0.25f * SIMD_PI);
   groundOrigin.setElement(upAxis, groundOrigin.getElement(upAxis) - .5f);
   groundOrigin.z -= 0.6;
   start.setOrigin(groundOrigin);
   //	start.setRotation(groundOrn);
   btRigidBody body = createRigidBody(0, start, box);
   body.setFriction(0);
  }
 }

 public void resetCamera() {
  camera().set(new btQuaternion(0.07323081f, -0.69297534f, -0.07110454f, 0.7136992f), new btVector3(9.543488f, 4.484767f, -0.20902833f));
 }

 @Override
 protected JPanel getParams() {
  return null;
 }

 @Override
 public String get_description() {
  return "Apply a torque to in the hinge axis. This example uses a btHingeConstraint and btRigidBody." +
   "The setup is similar to the multi body example TestJointTorque.";
 }
};
