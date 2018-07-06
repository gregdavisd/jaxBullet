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
package bullet_examples.apps.api;

import Bullet.Collision.Shape.btBoxShape;
import Bullet.Collision.Shape.btCapsuleShape;
import Bullet.Collision.Shape.btCapsuleShapeX;
import Bullet.Collision.Shape.btCapsuleShapeZ;
import Bullet.Collision.Shape.btCollisionShape;
import Bullet.Collision.Shape.btConeShape;
import Bullet.Collision.Shape.btConeShapeX;
import Bullet.Collision.Shape.btConeShapeZ;
import Bullet.Collision.Shape.btCylinderShape;
import Bullet.Collision.Shape.btCylinderShapeX;
import Bullet.Collision.Shape.btCylinderShapeZ;
import Bullet.Collision.Shape.btSphereShape;
import static Bullet.Collision.btCollisionObject.CF_ANISOTROPIC_ROLLING_FRICTION;
import Bullet.Dynamics.CollisionObjects.btRigidBody;
import Bullet.Dynamics.CollisionObjects.btRigidBodyConstructionInfo;
import Bullet.LinearMath.btDefaultMotionState;
import Bullet.LinearMath.btQuaternion;
import static Bullet.LinearMath.btScalar.SIMD_PI;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import bullet_examples.DiscreteDemoContainer;
import javax.swing.JPanel;

/**
 *
 * @author Gregery Barton
 */
public class RollingFrictionDemo extends DiscreteDemoContainer {
 ///create 125 (5x5x5) dynamic object

 static final int ARRAY_SIZE_X = 5;
 static final int ARRAY_SIZE_Y = 5;
 static final int ARRAY_SIZE_Z = 5;
//maximum number of objects (and allow user to shoot additional boxes)
 static final int MAX_PROXIES = (ARRAY_SIZE_X * ARRAY_SIZE_Y * ARRAY_SIZE_Z
  + 1024);
///scaling of the objects (0.1 = 20 centimeter boxes )
 static final float SCALING = 1.f;
 static final float START_POS_X = -5;
 static final float START_POS_Y = -5;
 static final float START_POS_Z = -3;

 public void initPhysics() {
  setUpAxis(2);
//	world()->getSolverInfo().m_singleAxisRollingFrictionThreshold = 0.f;//faster but lower quality
  world().setGravity(new btVector3(0, 0, -10));
  {
   ///create a few basic rigid bodies
   btCollisionShape groundShape = new btBoxShape(new btVector3((12.f), (10.f),
    (25.f)));
   final btTransform groundTransform = new btTransform();
   groundTransform.setIdentity();
   groundTransform.setOrigin(new btVector3(0, 0, -28));
   groundTransform.set3x3(new btQuaternion(new btVector3(0, 1, 0), SIMD_PI
    * 0.03f));
   //We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
   float mass = 0;
   //rigidbody is dynamic if and only if mass is non zero, otherwise static
   boolean isDynamic = (mass != 0.f);
   final btVector3 localInertia = new btVector3();
   if (isDynamic) {
    groundShape.calculateLocalInertia(mass, localInertia);
   }
   //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
   btDefaultMotionState myMotionState = new btDefaultMotionState(groundTransform);
   btRigidBodyConstructionInfo rbInfo = new btRigidBodyConstructionInfo(mass,
    myMotionState,
    groundShape, localInertia);
   btRigidBody body = new btRigidBody(rbInfo);
   body.setFriction(.5f);
   //add the body to the dynamics world
   world().addRigidBody(body);
  }
  {
   ///create a few basic rigid bodies
   btCollisionShape groundShape = new btBoxShape(new btVector3((100.f), (100.f),
    (50.f)));
   final btTransform groundTransform = new btTransform();
   groundTransform.setIdentity();
   groundTransform.setOrigin(new btVector3(0f, 0f, -54f));
   //We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
   float mass = 0.f;
   //rigidbody is dynamic if and only if mass is non zero, otherwise static
   boolean isDynamic = (mass != 0.f);
   final btVector3 localInertia = new btVector3();
   if (isDynamic) {
    groundShape.calculateLocalInertia(mass, localInertia);
   }
   //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
   btDefaultMotionState myMotionState = new btDefaultMotionState(groundTransform);
   btRigidBodyConstructionInfo rbInfo = new btRigidBodyConstructionInfo(mass,
    myMotionState,
    groundShape, localInertia);
   btRigidBody body = new btRigidBody(rbInfo);
   body.setFriction(.5f);
   //add the body to the dynamics world
   world().addRigidBody(body);
  }
  {
   //create a few dynamic rigidbodies
   // Re-using the same collision is better for memory usage and performance
   btCollisionShape[] colShapes = {
    new btSphereShape((0.5f)),
    new btCapsuleShape(0.25f, 0.5f),
    new btCapsuleShapeX(0.25f, 0.5f),
    new btCapsuleShapeZ(0.25f, 0.5f),
    new btConeShape(0.25f, 0.5f),
    new btConeShapeX(0.25f, 0.5f),
    new btConeShapeZ(0.25f, 0.5f),
    new btCylinderShape(new btVector3(0.25f, 0.5f, 0.25f)),
    new btCylinderShapeX(new btVector3(0.5f, 0.25f, 0.25f)),
    new btCylinderShapeZ(new btVector3(0.25f, 0.25f, 0.5f))
   };
   final int NUM_SHAPES = colShapes.length;
   /// Create Dynamic Objects
   final btTransform startTransform = new btTransform();
   startTransform.setIdentity();
   float mass = (1.f);
   //rigidbody is dynamic if and only if mass is non zero, otherwise static
   float start_x = START_POS_X - ARRAY_SIZE_X / 2;
   float start_y = START_POS_Y;
   float start_z = START_POS_Z - ARRAY_SIZE_Z / 2;
   {
    int shapeIndex = 0;
    for (int k = 0; k < ARRAY_SIZE_Y; k++) {
     for (int i = 0; i < ARRAY_SIZE_X; i++) {
      for (int j = 0; j < ARRAY_SIZE_Z; j++) {
       startTransform.setOrigin(new btVector3(
        (2.0f * i + start_x),
        (2.0f * j + start_z),
        (20f + 2.0f * k + start_y)).scale(SCALING));
       shapeIndex++;
       btCollisionShape colShape = colShapes[shapeIndex % NUM_SHAPES];
       boolean isDynamic = (mass != 0.f);
       final btVector3 localInertia = new btVector3();
       if (isDynamic) {
        colShape.calculateLocalInertia(mass, localInertia);
       }
       //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
       btDefaultMotionState myMotionState = new btDefaultMotionState(
        startTransform);
       btRigidBodyConstructionInfo rbInfo = new btRigidBodyConstructionInfo(mass,
        myMotionState,
        colShape, localInertia);
       btRigidBody body = new btRigidBody(rbInfo);
       body.setFriction(1.f);
       body.setRollingFriction(.1f);
       body.setSpinningFriction(.1f);
       body.setAnisotropicFriction(colShape
        .getAnisotropicRollingFrictionDirection(),
        CF_ANISOTROPIC_ROLLING_FRICTION);
       world().addRigidBody(body);
      }
     }
    }
   }
  }
 }

 @Override
 public void resetCamera() {
  camera().set(new btQuaternion(-0.10754126f, -0.0019145795f, -2.0712907E-4f,
   -0.99419874f),
   new btVector3(2.3444018f, 25.343372f, 109.90401f));
 }

 @Override
 protected JPanel getParams() {
  return null;
 }

 @Override
 public String get_description() {
  return "Damping is often not good enough to keep rounded objects from rolling down a sloped surface. "
   + "Instead, you can set the rolling friction for a rigid body. Generally it is best to leave the rolling friction "
   + "to zero, to avoid artifacts.";
 }

 @Override
 protected int getDebugMode() {
  return 0;
 }

}
