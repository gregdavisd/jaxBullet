/*
 * Copyright (c) 2017 Gregery Barton
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
package bullet_examples;

import Bullet.LinearMath.btClock;
import Bullet.LinearMath.btQuaternion;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import javax.vecmath.AxisAngle4f;

/**
 * WASD + mouse look camera
 *
 * @author Gregery Barton
 */
public class Camera {

 private final btVector3 eye = new btVector3(0, 3, 20);
 private final btQuaternion dir = new btQuaternion().set(
  new AxisAngle4f(1, 0, 0, 0.1f));
 private btClock clock;

 public btVector3 forward() {
  return dir.conjugateTransform(new btVector3(0, 0, -1));
 }

 public btVector3 eye() {
  return new btVector3(eye);
 }

 public btQuaternion dir() {
  return new btQuaternion(dir);
 }

 public void animate_camera(float speed, boolean forward_pressed,
  boolean backward_pressed,
  boolean left_pressed, boolean right_pressed) {
  final btVector3 forward = forward();
  final btVector3 sideways = new btVector3(forward.z, 0, -forward.x);
  sideways.normalize();
  final btVector3 move = new btVector3();
  if (forward_pressed) {
   move.add(forward);
  }
  if (backward_pressed) {
   move.sub(forward);
  }
  if (left_pressed) {
   move.add(sideways);
  }
  if (right_pressed) {
   move.sub(sideways);
  }
  move.normalize();
  if (clock == null) {
   clock = new btClock();
  }
  float dt = Math.min(clock.getTimeSeconds(), 0.1f);
  clock.reset();
  if (move.lengthSquared() > 0) {
   eye.add(move.scale(dt * speed));
  }
 }

 public void look_around(float dx, float dy) {
  final btQuaternion qdx = new btQuaternion().set(new AxisAngle4f(0, 1, 0, dx));
  final btQuaternion qdy = new btQuaternion().set(new AxisAngle4f(1, 0, 0, -dy));
  dir.mul(qdx);
  dir.set(qdy.mul(dir));
  dir.normalize();
 }

 public void set(final btQuaternion dir, final btVector3 eye) {
  this.dir.set(dir);
  this.eye.set(eye);
 }

 public btTransform get_matrix() {
  final btTransform camera_matrix = new btTransform().set(dir);
  camera_matrix.setOrigin(camera_matrix
   .transform3x3(new btVector3(eye).negate()));
  return camera_matrix;
 }

}
