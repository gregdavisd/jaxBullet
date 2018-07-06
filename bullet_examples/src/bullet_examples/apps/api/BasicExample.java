/*
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2015 Google Inc. http://bulletphysics.org
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
import Bullet.LinearMath.btQuaternion;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import Bullet.LinearMath.btVector4;
import bullet_examples.DiscreteDemoContainer;
import javax.swing.JPanel;

/**
 *
 * @author Gregery Barton
 */
public class BasicExample extends DiscreteDemoContainer {

 public static final int ARRAY_SIZE_Y = 5;
 public static final int ARRAY_SIZE_X = 5;
 public static final int ARRAY_SIZE_Z = 5;

 @Override
 public void initPhysics() {
  setUpAxis(1);
  // world.setGravity(new btVector3());
  ///create a few basic rigid bodies
  btBoxShape groundShape = new btBoxShape(new btVector3((50.f), (50.f), (50.f)));
  final btTransform groundTransform = new btTransform();
  groundTransform.setIdentity();
  groundTransform.setOrigin(new btVector3(0, -50, 0));
  {
   float mass = 0;
   createRigidBody(mass, groundTransform, groundShape, new btVector4(0, 0, 1, 1));
  }
  {
   //create a few dynamic rigidbodies
   // Re-using the same collision is better for memory usage and performance
   btBoxShape colShape = new btBoxShape(new btVector3(.1f, .1f, .1f));
   /// Create Dynamic Objects
   final btTransform startTransform = new btTransform();
   startTransform.setIdentity();
   float mass = 1f;
   //rigidbody is dynamic if and only if mass is non zero, otherwise static
   boolean isDynamic = (mass != 0.f);
   final btVector3 localInertia = new btVector3(0, 0, 0);
   if (isDynamic) {
    colShape.calculateLocalInertia(mass, localInertia);
   }
   for (int k = 0; k < ARRAY_SIZE_Y; k++) {
    for (int i = 0; i < ARRAY_SIZE_X; i++) {
     for (int j = 0; j < ARRAY_SIZE_Z; j++) {
      startTransform.setOrigin(new btVector3(
       (0.2f * i),
       2.0f + .2f * k,
       (0.2f * j)));
      createRigidBody(mass, startTransform, colShape);
     }
    }
   }
  }
 }

 @Override
 public void resetCamera() {
  camera().set(new btQuaternion(0.13435361f, -0.37949604f, -0.055803467f,
   0.91368365f),
   new btVector3(11.757509f, 5.198874f, 11.65576f));
 }

 @Override
 protected JPanel getParams() {
  return null;
 }

 @Override
 public String get_description() {
  return "A block of blocks crashing down. Changing parameters will yield differently shaped piles of blocks. "
   + "At high precisions conservation of momentum will cause a shockwave that bounces the top blocks. "
   + "Box to Box collision is performed by the btBoxBoxDetector class. Select cycle check box for the simulation "
   + "to automatically reset once all blocks are deactivated.";
 }

 @Override
 protected int getDebugMode() {
  return 0;
 }

}
