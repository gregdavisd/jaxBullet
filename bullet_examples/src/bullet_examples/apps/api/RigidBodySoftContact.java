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
import Bullet.Collision.Shape.btCompoundShape;
import Bullet.Collision.Shape.btSphereShape;
import static Bullet.Dynamics.ConstraintSolver.btSolverMode.SOLVER_SIMD;
import Bullet.Dynamics.CollisionObjects.btRigidBody;
import Bullet.LinearMath.btQuaternion;
import static Bullet.LinearMath.btScalar.SIMD_PI;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import Bullet.LinearMath.btVector4;
import bullet_examples.DiscreteDemoContainer;
import javax.swing.JPanel;

/**
 *
 * @author Gregery Barton
 */
public class RigidBodySoftContact extends DiscreteDemoContainer {

 private static final int ARRAY_SIZE_Y = 1;
 private static final int ARRAY_SIZE_X = 1;
 private static final int ARRAY_SIZE_Z = 1;

 @Override
 public void initPhysics() {
  setUpAxis(1);
  world().getSolverInfo().m_erp2 = 0.f;
  world().getSolverInfo().m_globalCfm = 0.f;
  world().getSolverInfo().m_numIterations = 3;
  world().getSolverInfo().m_solverMode = SOLVER_SIMD;// | SOLVER_RANDMIZE_ORDER;
  world().getSolverInfo().m_splitImpulse = false;
  ///create a few basic rigid bodies
  btBoxShape groundShape = new btBoxShape(new btVector3((50.f), (50.f), (50.f)));
  final btTransform groundTransform = new btTransform();
  groundTransform.setIdentity();
  groundTransform.setOrigin(new btVector3(0f, -50f, 0f));
  {
   float mass = 0.f;
   btRigidBody body = createRigidBody(mass, groundTransform, groundShape, new btVector4(0f, 0f, 1f,
    1f));
   body.setContactStiffnessAndDamping(300, 10);
  }
  {
   //create a few dynamic rigidbodies
   // Re-using the same collision is better for memory usage and performance
   //btBoxShape* colShape = createBoxShape(btVector3(1,1,1));
   btCollisionShape childShape = new btSphereShape((0.5f));
   btCompoundShape colShape = new btCompoundShape();
   colShape.addChildShape(btTransform.getIdentity(), childShape);
   /// Create Dynamic Objects
   final btTransform startTransform = new btTransform();
   startTransform.setIdentity();
   startTransform.set3x3(new btQuaternion(new btVector3(1f, 1f, 1f), SIMD_PI / 10.f));
   float mass = (1.f);
   //rigidbody is dynamic if and only if mass is non zero, otherwise static
   boolean isDynamic = (mass != 0.f);
   final btVector3 localInertia = new btVector3();
   if (isDynamic) {
    colShape.calculateLocalInertia(mass, localInertia);
   }
   for (int k = 0; k < ARRAY_SIZE_Y; k++) {
    for (int i = 0; i < ARRAY_SIZE_X; i++) {
     for (int j = 0; j < ARRAY_SIZE_Z; j++) {
      startTransform.setOrigin(new btVector3(
       (2.0f * i + 0.1f),
       (3f + 2.0f * k),
       (2.0f * j)));
      btRigidBody body;
      body = createRigidBody(mass, startTransform, colShape);
      //body.setAngularVelocity(btVector3(1,1,1));
     }
    }
   }
  }
 }

 @Override
 public void resetCamera() {
  camera().set(new btQuaternion(0.10903801f, 0.066217855f, 0.007279983f, 0.9918029f), new btVector3(
   -0.9730224f, 3.1672342f, 8.696599f));
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
  return "Using the error correction parameter (ERP) and constraint force mixing (CFM) values for contacts to simulate compliant contact.";
 }
}
