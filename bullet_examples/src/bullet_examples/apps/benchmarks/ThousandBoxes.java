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

import Bullet.Collision.Shape.btBoxShape;
import Bullet.Dynamics.CollisionObjects.btRigidBody;
import Bullet.LinearMath.btQuaternion;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import javax.swing.JPanel;

/**
 *
 * @author Gregery Barton
 */
public class ThousandBoxes extends BenchmarkDemoContainer {

 @Override
 public void initPhysics() {
  setUpAxis(1);
  create_ground();
  // 3000
  int size = 8;
  float cubeSize = 1.0f;
  float spacing = cubeSize;
  final btVector3 pos = new btVector3(0.0f, cubeSize * 2, 0.f);
  float offset = -size * (cubeSize * 2.0f + spacing) * 0.5f;
  btBoxShape blockShape = new btBoxShape(new btVector3(cubeSize
   - COLLISION_RADIUS, cubeSize - COLLISION_RADIUS, cubeSize - COLLISION_RADIUS));
  final btVector3 localInertia = new btVector3();
  float mass = 2.f;
  blockShape.calculateLocalInertia(mass, localInertia);
  final btTransform trans = new btTransform();
  trans.setIdentity();
  for (int k = 0; k < 47; k++) {
   for (int j = 0; j < size; j++) {
    pos.z = offset + (float) j * (cubeSize * 2.0f + spacing);
    for (int i = 0; i < size; i++) {
     pos.x = offset + (float) i * (cubeSize * 2.0f + spacing);
     trans.setOrigin(pos);
     btRigidBody cmbody;
     cmbody = createRigidBody(mass, trans, blockShape);
    }
   }
   offset -= 0.05f * spacing * (size - 1);
//		spacing *= 1.01f;
   pos.y += (cubeSize * 2.0f + spacing);
  }
 }

 @Override
 public void resetCamera() {
  camera().set(new btQuaternion(0.17830886f, -0.37305543f, -0.07329426f,
   0.90755916f),
   new btVector3(195.00221f, 156.78822f, 193.49379f));
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
  return "";
 }

}
