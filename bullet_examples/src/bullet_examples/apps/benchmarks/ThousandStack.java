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
import Bullet.LinearMath.btQuaternion;
import static Bullet.LinearMath.btScalar.SIMD_PI;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import javax.swing.JPanel;

/**
 *
 * @author Gregery Barton
 */
public class ThousandStack extends BenchmarkDemoContainer {

 @Override
 public void initPhysics() {
  setUpAxis(1);
  create_ground();
  final float cubeSize = 1.0f;
  createPyramid(new btVector3(-20.0f, 0.0f, 0.0f), 12, new btVector3(cubeSize,
   cubeSize, cubeSize));
  createWall(new btVector3(-2.0f, 0.0f, 0.0f), 12, new btVector3(cubeSize,
   cubeSize, cubeSize));
  createWall(new btVector3(4.0f, 0.0f, 0.0f), 12, new btVector3(cubeSize,
   cubeSize, cubeSize));
  createWall(new btVector3(10.0f, 0.0f, 0.0f), 12, new btVector3(cubeSize,
   cubeSize, cubeSize));
  createTowerCircle(new btVector3(25.0f, 0.0f, 0.0f), 8, 24, new btVector3(
   cubeSize, cubeSize,
   cubeSize));
 }

 void createTowerCircle(final btVector3 offsetPosition, int stackSize,
  int rotSize,
  final btVector3 boxSize) {
  btBoxShape blockShape = new btBoxShape(new btVector3(boxSize.x
   - COLLISION_RADIUS, boxSize.y - COLLISION_RADIUS, boxSize.z
   - COLLISION_RADIUS));
  final btTransform trans = new btTransform();
  trans.setIdentity();
  float mass = 1.f;
  final btVector3 localInertia = new btVector3();
  blockShape.calculateLocalInertia(mass, localInertia);
  float radius = 1.3f * rotSize * boxSize.x / SIMD_PI;
  // create active boxes
  final btQuaternion rotY = new btQuaternion(0f, 1f, 0f, 0f);
  float posY = boxSize.y;
  for (int i = 0; i < stackSize; i++) {
   for (int j = 0; j < rotSize; j++) {
    trans.setOrigin(new btVector3(offsetPosition).add(
     rotate(rotY, new btVector3(0.0f, posY, radius))));
    trans.setRotation(rotY);
    createRigidBody(mass, trans, blockShape);
    rotY.mul(new btQuaternion(new btVector3(0, 1, 0), SIMD_PI / (rotSize
     * (0.5f))));
   }
   posY += boxSize.y * 2.0f;
   rotY.mul(new btQuaternion(new btVector3(0, 1, 0), SIMD_PI / (float) rotSize));
  }
 }

 void createWall(final btVector3 offsetPosition, int stackSize,
  final btVector3 boxSize) {
  btBoxShape blockShape = new btBoxShape(new btVector3(boxSize.x
   - COLLISION_RADIUS, boxSize.y - COLLISION_RADIUS, boxSize.z
   - COLLISION_RADIUS));
  float mass = 1.f;
  final btVector3 localInertia = new btVector3();
  blockShape.calculateLocalInertia(mass, localInertia);
//	float  diffX = boxSize[0] * 1.0f;
  float diffY = boxSize.y * 1.0f;
  float diffZ = boxSize.z * 1.0f;
  float offset = -stackSize * (diffZ * 2.0f) * 0.5f;
  final btVector3 pos = new btVector3(0.0f, diffY, 0.0f);
  final btTransform trans = new btTransform();
  trans.setIdentity();
  while (stackSize > 0) {
   for (int i = 0; i < stackSize; i++) {
    pos.z = offset + (float) i * (diffZ * 2.0f);
    trans.setOrigin(new btVector3(offsetPosition).add(pos));
    createRigidBody(mass, trans, blockShape);
   }
   offset += diffZ;
   pos.y += (diffY * 2.0f);
   stackSize--;
  }
 }

 void createPyramid(final btVector3 offsetPosition, int stackSize,
  final btVector3 boxSize) {
  float space = 0.0001f;
  final btVector3 pos = new btVector3(0.0f, boxSize.y, 0.0f);
  btBoxShape blockShape = new btBoxShape(new btVector3(boxSize.x
   - COLLISION_RADIUS, boxSize.y - COLLISION_RADIUS, boxSize.z
   - COLLISION_RADIUS));
  final btTransform trans = new btTransform();
  trans.setIdentity();
  float mass = 1.f;
  final btVector3 localInertia = new btVector3();
  blockShape.calculateLocalInertia(mass, localInertia);
  float diffX = boxSize.x * 1.02f;
  float diffY = boxSize.y * 1.02f;
  float diffZ = boxSize.z * 1.02f;
  float offsetX = -stackSize * (diffX * 2.0f + space) * 0.5f;
  float offsetZ = -stackSize * (diffZ * 2.0f + space) * 0.5f;
  while (stackSize > 0) {
   for (int j = 0; j < stackSize; j++) {
    pos.z = offsetZ + (float) j * (diffZ * 2.0f + space);
    for (int i = 0; i < stackSize; i++) {
     pos.x = offsetX + (float) i * (diffX * 2.0f + space);
     trans.setOrigin(new btVector3(offsetPosition).add(pos));
     this.createRigidBody(mass, trans, blockShape);
    }
   }
   offsetX += diffX;
   offsetZ += diffZ;
   pos.y += (diffY * 2.0f + space);
   stackSize--;
  }
 }

 @Override
 public void resetCamera() {
  camera().set(new btQuaternion(-0.053713836f, -0.9136668f, -0.12852196f,
   -0.38185087f),
   new btVector3(-130.70317f, 61.614944f, -125.73166f));
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
