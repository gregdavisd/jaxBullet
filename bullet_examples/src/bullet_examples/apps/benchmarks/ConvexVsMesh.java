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

import Bullet.Collision.Shape.btConvexHullShape;
import Bullet.LinearMath.btQuaternion;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import static bullet_examples.apps.benchmarks.ConvexStack.TaruVtx;
import javax.swing.JPanel;

/**
 *
 * @author Gregery Barton
 */
public class ConvexVsMesh extends BenchmarkDemoContainer {

 @Override
 public void initPhysics() {
  setUpAxis(1);
  final btVector3 boxSize = new btVector3(1.5f, 1.5f, 1.5f);
  btConvexHullShape convexHullShape = new btConvexHullShape();
  int TaruVtxCount = TaruVtx.length / 3;
  for (int i = 0; i < TaruVtxCount; i++) {
   final btVector3 vtx = new btVector3(TaruVtx[i * 3], TaruVtx[i * 3 + 1],
    TaruVtx[i * 3 + 2]);
   convexHullShape.addPoint(vtx);
  }
  final btTransform trans = new btTransform();
  trans.setIdentity();
  float mass = 1.f;
  final btVector3 localInertia = new btVector3();
  convexHullShape.calculateLocalInertia(mass, localInertia);
  {
   int size = 10;
   int height = 10;
   float cubeSize = boxSize.x;
   float spacing = 2.0f;
   final btVector3 pos = new btVector3(0.0f, 20.0f, 0.0f);
   float offset = -size * (cubeSize * 2.0f + spacing) * 0.5f;
   for (int k = 0; k < height; k++) {
    for (int j = 0; j < size; j++) {
     pos.z = offset + (float) j * (cubeSize * 2.0f + spacing);
     for (int i = 0; i < size; i++) {
      pos.x = offset + (float) i * (cubeSize * 2.0f + spacing);
      final btVector3 bpos = new btVector3(0, 25, 0).add(new btVector3(5.0f,
       1.0f, 5.0f).mul(pos));
      trans.setOrigin(bpos);
      createRigidBody(mass, trans, convexHullShape);
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
  camera().set(new btQuaternion(-0.19792123f, -0.3747349f, -0.08222372f,
   -0.9020201f),
   new btVector3(-452.2333f, 324.1984f, 456.52335f));
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
