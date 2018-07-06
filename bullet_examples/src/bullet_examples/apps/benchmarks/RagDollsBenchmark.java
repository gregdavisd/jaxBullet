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

import Bullet.LinearMath.btQuaternion;
import Bullet.LinearMath.btVector3;
import javax.swing.JPanel;

/**
 *
 * @author Gregery Barton
 */
public class RagDollsBenchmark extends BenchmarkDemoContainer {

 @Override
 public void initPhysics() {
  setUpAxis(1);
  create_ground();
  int size = 16;
  float sizeX = 1.f;
  float sizeY = 1.f;
  //int rc=0;
  float scale = 3.5f;
  final btVector3 pos = new btVector3(0.0f, sizeY, 0.0f);
  while (size > 0) {
   float offset = -size * (sizeX * 6.0f) * 0.5f;
   for (int i = 0; i < size; i++) {
    pos.x = offset + (float) i * (sizeX * 6.0f);
    RagDoll ragDoll = new RagDoll(world(), pos, scale);
   }
   offset += sizeX;
   pos.y += (sizeY * 7.0f);
   pos.z -= sizeX * 2.0f;
   size--;
  }
 }

 @Override
 public void resetCamera() {
  camera().set(new btQuaternion(-0.040879007f, -0.92339164f, -0.10268609f,
   -0.36760363f),
   new btVector3(-182.48396f, 94.93444f, -179.77794f));
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
