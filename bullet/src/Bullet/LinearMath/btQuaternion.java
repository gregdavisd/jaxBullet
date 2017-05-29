/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.LinearMath;

import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.btAcos;
import static Bullet.LinearMath.btScalar.btSqrt;
import static Bullet.LinearMath.btVector3.btPlaneSpace1;
import java.io.Serializable;
import javax.vecmath.AxisAngle4f;
import javax.vecmath.Quat4f;

/**
 *
 * @author Gregery Barton
 */
public final class btQuaternion extends Quat4f<btQuaternion> implements Serializable {

 public btQuaternion(final btVector3 t1, float w) {
  set(new AxisAngle4f(t1, w));
 }

 public static btQuaternion getIdentity() {
  return new btQuaternion(0f, 0f, 0f, 1f);
 }

 /**
  *
  * @param x
  * @param y
  * @param z
  * @param w
  */
 public btQuaternion(float x, float y, float z, float w) {
  super(x, y, z, w);
 }

 /**
  *
  * @param q
  */
 public btQuaternion(float[] q) {
  super(q);
 }

 /**
  *
  * @param q1
  */
 public btQuaternion(Quat4f q1) {
  super(q1);
 }

 /**
  * btQuaternion
  */
 public btQuaternion() {
 }

 public float getAngle() {
  return (2.f) * btAcos(w);
 }

 /**
  *
  * @return
  */
 public float x() {
  return x;
 }

 /**
  *
  * @return
  */
 public float y() {
  return y;
 }

 /**
  *
  * @return
  */
 public float z() {
  return z;
 }

 /**
  *
  * @return
  */
 public float w() {
  return w;
 }

 public static btVector3 quatRotate(final btQuaternion rotation, final btVector3 v) {
  return rotation.transform(new btVector3(v));
 }

 public static btQuaternion shortestArcQuat(final btVector3 v0, final btVector3 v1) // Game Programming Gems 2.10. make sure v0,v1 are normalized
 {
  final btVector3 c = new btVector3(v0).cross(v1);
  float d = v0.dot(v1);
  if (d < -1.0 + SIMD_EPSILON) {
   final btVector3 n = new btVector3();
   final btVector3 unused = new btVector3();
   btPlaneSpace1(v0, n, unused);
   return new btQuaternion(n.x(), n.y(), n.z(), 0.0f); // just pick any vector that is orthogonal to v0
  }
  float s = btSqrt((1.0f + d) * 2.0f);
  float rs = 1.0f / s;
  return new btQuaternion(c.getX() * rs, c.getY() * rs, c.getZ() * rs, s * 0.5f);
 }
}
