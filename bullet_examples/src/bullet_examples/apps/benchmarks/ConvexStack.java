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
package bullet_examples.apps.benchmarks;

import Bullet.Collision.Shape.btConvexHullShape;
import Bullet.LinearMath.btQuaternion;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import javax.swing.JPanel;

/**
 *
 * @author Gregery Barton
 */
public class ConvexStack extends BenchmarkDemoContainer {

 @Override
 public void initPhysics() {
  setUpAxis(1);
  create_ground();
  int size = 8;
  final float cubeSize = 1.5f;
  float spacing = cubeSize;
  final btVector3 pos = new btVector3(0.0f, cubeSize * 2, 0.0f);
  float offset = -size * (cubeSize * 2.0f + spacing) * 0.5f;
  btConvexHullShape convexHullShape = new btConvexHullShape();
  float scaling = 1f;
  convexHullShape.setLocalScaling(new btVector3(scaling, scaling, scaling));
  int TaruVtxCount = TaruVtx.length / 3;
  for (int i = 0; i < TaruVtxCount; i++) {
   final btVector3 vtx = new btVector3(TaruVtx[i * 3], TaruVtx[i * 3 + 1], TaruVtx[i * 3 + 2]);
   convexHullShape.addPoint(vtx.scale(1.f / scaling));
  }
  //this will enable polyhedral contact clipping, better quality, slightly slower
  //convexHullShape->initializePolyhedralFeatures();
  final btTransform trans = new btTransform();
  trans.setIdentity();
  float mass = 1.f;
  final btVector3 localInertia = new btVector3();
  convexHullShape.calculateLocalInertia(mass, localInertia);
  for (int k = 0; k < 15; k++) {
   for (int j = 0; j < size; j++) {
    pos.z = offset + (float) j * (cubeSize * 2.0f + spacing);
    for (int i = 0; i < size; i++) {
     pos.x = offset + (float) i * (cubeSize * 2.0f + spacing);
     trans.setOrigin(pos);
     createRigidBody(mass, trans, convexHullShape);
    }
   }
   offset -= 0.05f * spacing * (size - 1);
   spacing *= 1.01f;
   pos.y += (cubeSize * 2.0f + spacing);
  }
 }

 @Override
 public void resetCamera() {
  camera().set(new btQuaternion(0.17311957f, -0.0029539443f, -5.194773E-4f, 0.9848963f),
   new btVector3(-19.027628f, 131.53452f, 257.9421f));
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
 static float TaruVtx[] = {
  1.08664f, -1.99237f, 0.0f,
  0.768369f, -1.99237f, -0.768369f,
  1.28852f, 1.34412e-007f, -1.28852f,
  1.82224f, 1.90735e-007f, 0.0f,
  0.0f, -1.99237f, -1.08664f,
  0.0f, 0.0f, -1.82224f,
  0.0f, -1.99237f, -1.08664f,
  -0.768369f, -1.99237f, -0.768369f,
  -1.28852f, 1.34412e-007f, -1.28852f,
  0.0f, 0.0f, -1.82224f,
  -1.08664f, -1.99237f, 1.82086e-007f,
  -1.82224f, 1.90735e-007f, 1.59305e-007f,
  -0.768369f, -1.99237f, 0.76837f,
  -1.28852f, 2.47058e-007f, 1.28852f,
  1.42495e-007f, -1.99237f, 1.08664f,
  2.38958e-007f, 2.70388e-007f, 1.82224f,
  0.768369f, -1.99237f, 0.768369f,
  1.28852f, 2.47058e-007f, 1.28852f,
  0.768369f, 1.99237f, -0.768369f,
  1.08664f, 1.99237f, 0.0f,
  0.0f, 1.99237f, -1.08664f,
  -0.768369f, 1.99237f, -0.768369f,
  0.0f, 1.99237f, -1.08664f,
  -1.08664f, 1.99237f, 0.0f,
  -0.768369f, 1.99237f, 0.768369f,
  1.42495e-007f, 1.99237f, 1.08664f,
  0.768369f, 1.99237f, 0.768369f,
  1.42495e-007f, -1.99237f, 1.08664f,
  -0.768369f, -1.99237f, 0.76837f,
  -1.08664f, -1.99237f, 1.82086e-007f,
  -0.768369f, -1.99237f, -0.768369f,
  0.0f, -1.99237f, -1.08664f,
  0.768369f, -1.99237f, -0.768369f,
  1.08664f, -1.99237f, 0.0f,
  0.768369f, -1.99237f, 0.768369f,
  0.768369f, 1.99237f, -0.768369f,
  0.0f, 1.99237f, -1.08664f,
  -0.768369f, 1.99237f, -0.768369f,
  -1.08664f, 1.99237f, 0.0f,
  -0.768369f, 1.99237f, 0.768369f,
  1.42495e-007f, 1.99237f, 1.08664f,
  0.768369f, 1.99237f, 0.768369f,
  1.08664f, 1.99237f, 0.0f,};
}
