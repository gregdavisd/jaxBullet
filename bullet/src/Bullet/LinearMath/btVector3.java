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

import static Bullet.LinearMath.btScalar.B3_INFINITY;
import static Bullet.LinearMath.btScalar.SIMDSQRT12;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.btFabs;
import static Bullet.LinearMath.btScalar.btRecipSqrt;
import java.io.Serializable;
import javax.vecmath.Tuple3f;

/**
 *
 * @author Gregery Barton
 */
public final class btVector3 extends Tuple3f<btVector3> implements Serializable {

 /**
  *
  * @param n
  * @param p
  * @param q
  */
 public static void btPlaneSpace1(final btVector3 n, final btVector3 p, final btVector3 q) {
  if (btFabs(n.z) > SIMDSQRT12) {
   // choose p in y-z plane
   float a = n.y * n.y + n.z * n.z;
   float k = btRecipSqrt(a);
   p.x = 0;
   p.y = -n.z * k;
   p.z = n.y * k;
   // set q = n x p
   q.x = a * k;
   q.y = -n.x * p.z;
   q.z = n.x * p.y;
  } else {
   // choose p in x-y plane
   float a = n.x * n.x + n.y * n.y;
   float k = btRecipSqrt(a);
   p.x = -n.y * k;
   p.y = n.x * k;
   p.z = 0;
   // set q = n x p
   q.x = -n.z * p.y;
   q.y = n.z * p.x;
   q.z = a * k;
  }
 }

 /**
  * @param v1
  * @param v2
  * @return
  * @brief Return the dot product between two vectors
  */
 public static float btDot(final btVector3 v1, final btVector3 v2) {
  return v1.dot(v2);
 }

 /**
  *
  * @param array
  */
 public static void init(btVector3[] array) {
  for (int i = 0; i < array.length; i++) {
   if (array[i] == null) {
    array[i] = new btVector3();
   }
  }
 }

 /**
  *
  * @param v1
  * @param v2
  * @return
  */
 public static btVector3 btCross(final btVector3 v1, final btVector3 v2) {
  return new btVector3(v1).cross(v2);
 }

 /**
  *
  * @param a
  * @param b
  * @param c
  * @return
  */
 public static float det(final btVector3 a, final btVector3 b, final btVector3 c) {
  return (a.y() * b.z() * c.x() + a.z() * b.x() * c.y() -
   a.x() * b.z() * c.y() - a.y() * b.x() * c.z() +
   a.x() * b.y() * c.z() - a.z() * b.y() * c.x());
 }

 /**
  *
  * @param a
  * @param b
  * @return
  */
 public static btVector3 btMul(final btVector3 a, final btVector3 b) {
  return new btVector3(a).mul(b);
 }

 // Bullet physics 3 vectors actually have a fourth component
 /**
  *
  */
 //public float w;
 /**
  * btVector3
  */
 public btVector3() {
 }

 /**
  *
  * @param x
  * @param y
  * @param z
  */
 public btVector3(float x, float y, float z) {
  this.x = x;
  this.y = y;
  this.z = z;
 }

 /**
  *
  * @param v
  */
 public btVector3(float[] v) {
  super(v);
 }

 /**
  *
  * @param v1
  */
 public btVector3(final btVector3 v1) {
  x = v1.x;
  y = v1.y;
  z = v1.z;
 }

 /**
  *
  * @param t1
  */
 public btVector3(Tuple3f t1) {
  x = t1.x;
  y = t1.y;
  z = t1.z;
 }

 /**
  *
  * @param v0
  * @param v1
  * @param rt
  */
 public void setInterpolate3(final btVector3 v0, final btVector3 v1, float rt) {
  float s = 1.0f - rt;
  x = s * v0.x + rt * v1.x;
  y = s * v0.y + rt * v1.y;
  z = s * v0.z + rt * v1.z;
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
// public float w() {
//  return w;
// }
 /**
  * create a vector as btVector3( this->dot( btVector3 v0 ), this->dot( btVector3 v1), this->dot(
  * btVector3 v2 ))
  *
  * @param v0
  * @param v1
  * @param v2
  * @return
  */
 public btVector3 dot3(final btVector3 v0, final btVector3 v1, final btVector3 v2) {
  return new btVector3(dot(v0), dot(v1), dot(v2));
 }

 /**
  *
  * @param w
  * @return
  */
// public btVector3 setW(float w) {
//  this.w = w;
//  return this;
// }
 /**
  *
  * @return
  */
// public float getW() {
//  return w;
// }
 /**
  *
  * @param array
  * @param array_count
  * @param dotOut
  * @return
  */
 public int maxDot(btVector3[] array, int array_count, float[] dotOut) {
  {
   float maxDot = -B3_INFINITY;
   int ptIndex = -1;
   for (int i = 0; i < array_count; i++) {
    float dot = array[i].dot(this);
    if (dot > maxDot) {
     maxDot = dot;
     ptIndex = i;
    }
   }
   assert (ptIndex >= 0);
   if (ptIndex < 0) {
    ptIndex = 0;
   }
   dotOut[0] = maxDot;
   return ptIndex;
  }
 }

 /**
  *
  * @param v1
  * @param v2
  * @return
  */
 public float triple(final btVector3 v1, final btVector3 v2) {
  return x * (v1.y * v2.z - v1.z * v2.y) + y * (v1.z * v2.x - v1.x * v2.z) + z * (v1.x * v2.y -
   v1.y * v2.x);
 }

 public void getSkewSymmetricMatrix(final btMatrix3x3 m) {
  m.set(0.f, -z(), y(),
   z(), 0.f, -x(),
   -y(), x(), 0.f);
 }

 public void getSkewSymmetricMatrix(final btVector3 x, final btVector3 y, final btVector3 z) {
  x.set(0f, -z(), y());
  y.set(z(), 0.f, -x());
  z.set(-y(), x(), 0.f);
 }

 public boolean fuzzyZero() {
  return lengthSquared() < SIMD_EPSILON * SIMD_EPSILON;
 }
}
