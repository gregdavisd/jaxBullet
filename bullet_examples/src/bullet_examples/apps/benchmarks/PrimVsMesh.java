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
package bullet_examples.apps.benchmarks;

import Bullet.Collision.Shape.btBoxShape;
import Bullet.Collision.Shape.btCapsuleShape;
import Bullet.Collision.Shape.btSphereShape;
import Bullet.LinearMath.btQuaternion;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.util.concurrent.ThreadLocalRandom;
import javax.swing.JPanel;

/**
 *
 * @author Gregery Barton
 */
public class PrimVsMesh extends BenchmarkDemoContainer {

 @Override
 public void initPhysics() {
  setUpAxis(1);
  final btVector3 boxSize = new btVector3(1.5f, 1.5f, 1.5f);
  float boxMass = 1.0f;
  float sphereRadius = 1.5f;
  float sphereMass = 1.0f;
  float capsuleHalf = 2.0f;
  float capsuleRadius = 1.0f;
  float capsuleMass = 1.0f;
  {
   int size = 10;
   int height = 10;
   float cubeSize = boxSize.x;
   float spacing = 2.0f;
   final btVector3 pos = new btVector3(0.0f, 20.0f, 0.0f);
   float offset = -size * (cubeSize * 2.0f + spacing) * 0.5f;
   int numBodies = 0;
   for (int k = 0; k < height; k++) {
    for (int j = 0; j < size; j++) {
     pos.z = offset + (float) j * (cubeSize * 2.0f + spacing);
     for (int i = 0; i < size; i++) {
      pos.x = offset + (float) i * (cubeSize * 2.0f + spacing);
      final btVector3 bpos = new btVector3(0, 25, 0).add(new btVector3(5.0f, 1.0f, 5.0f).mul(pos));
      int idx = ThreadLocalRandom.current().nextInt() % 9;
      final btTransform trans = new btTransform();
      trans.setIdentity();
      trans.setOrigin(bpos);
      switch (idx) {
       case 0:
       case 1:
       case 2: {
        float r = 0.5f * (idx + 1);
        btBoxShape boxShape = new btBoxShape(new btVector3(boxSize).scale(r));
        createRigidBody(boxMass * r, trans, boxShape);
       }
       break;
       case 3:
       case 4:
       case 5: {
        float r = 0.5f * (idx - 3 + 1);
        btSphereShape sphereShape = new btSphereShape(sphereRadius * r);
        createRigidBody(sphereMass * r, trans, sphereShape);
       }
       break;
       case 6:
       case 7:
       case 8: {
        float r = 0.5f * (idx - 6 + 1);
        btCapsuleShape capsuleShape = new btCapsuleShape(capsuleRadius * r, capsuleHalf * r);
        createRigidBody(capsuleMass * r, trans, capsuleShape);
       }
       break;
      }
      numBodies++;
     }
    }
    offset -= 0.05f * spacing * (size - 1);
    spacing *= 1.1f;
    pos.y += (cubeSize * 2.0f + spacing);
   }
  }
  createLargeMeshBody();
 }

 @Override
 public void resetCamera() {
  camera().set(new btQuaternion(-0.19792123f, -0.3747349f, -0.08222372f, -0.9020201f),new btVector3(-452.2333f, 324.1984f, 456.52335f));
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
