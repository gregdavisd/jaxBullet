/*
 * Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/
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
package Bullet.LinearMath;

import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.btCos;
import static Bullet.LinearMath.btScalar.btFabs;
import static Bullet.LinearMath.btScalar.btSin;
import static Bullet.LinearMath.btScalar.btSqrt;
import static Bullet.LinearMath.btVector3.btDot;
import java.io.Serializable;
import javax.vecmath.Matrix3f;
import javax.vecmath.Matrix4f;

/**
 *
 * @author Gregery Barton
 */
public final class btMatrix3x3 extends Matrix3f<btMatrix3x3> implements
 Serializable {

 static final int M00 = 0;
 static final int M01 = 4;
 static final int M02 = 8;
 static final int M03 = 12;
 static final int M10 = 1;
 static final int M11 = 5;
 static final int M12 = 9;
 static final int M13 = 13;
 static final int M20 = 2;
 static final int M21 = 6;
 static final int M22 = 10;
 static final int M23 = 14;
 static final int M30 = 3;
 static final int M31 = 7;
 static final int M32 = 11;
 static final int M33 = 15;
 static final int C4X4 = 16;

 /**
  *
  * @param v
  */
 public btMatrix3x3(float[][] v) {
  super(v);
 }

 /**
  *
  * @param m00
  * @param m01
  * @param m02
  * @param m10
  * @param m11
  * @param m12
  * @param m20
  * @param m21
  * @param m22
  */
 public btMatrix3x3(float m00, float m01, float m02, float m10, float m11,
  float m12, float m20,
  float m21, float m22) {
  super(m00, m01, m02, m10, m11, m12, m20, m21, m22);
 }

 /**
  *
  * @param v
  */
 public btMatrix3x3(float[] v) {
  super(v);
 }

 /**
  *
  * @param m1
  */
 public btMatrix3x3(Matrix3f m1) {
  super(m1);
 }

 /**
  *
  * @param m1
  */
 public btMatrix3x3(Matrix4f m1) {
  super(m1);
 }

 /**
  * btMatrix3x3
  */
 public btMatrix3x3() {
 }

 /**
  *
  * @param i
  * @return
  */
 public btVector3 getColumn(int i) {
  final btVector3 v = new btVector3();
  getColumn(i, v);
  return v;
 }

 public btMatrix3x3(final btQuaternion q) {
  set(q);
 }

 /**
  *
  * @param i
  * @return
  */
 public btVector3 getRow(int i) {
  final btVector3 v = new btVector3();
  getRow(i, v);
  return v;
 }

 /**
  * @param maxSteps
  * @brief diagonalizes this matrix by the Jacobi method.
  * @param rot stores the rotation from the coordinate system in which the
  * matrix is diagonal to the original coordinate system, i.e., old_this = rot *
  * new_this * rot^T.
  * @param threshold See iteration
  */
 public void diagonalize(final btMatrix3x3 rot, float threshold, int maxSteps) {
  rot.setIdentity();
  for (int step = maxSteps; step > 0; step--) {
   // find off-diagonal element [p][q] with largest magnitude
   int p = 0;
   int q = 1;
   int r = 2;
   float max = btFabs(m01);
   float v = btFabs(m02);
   if (v > max) {
    q = 2;
    r = 1;
    max = v;
   }
   v = btFabs(m12);
   if (v > max) {
    p = 1;
    q = 2;
    r = 0;
    max = v;
   }
   float t = threshold * (btFabs(m00) + btFabs(m11) + btFabs(m22));
   if (max <= t) {
    if (max <= SIMD_EPSILON * t) {
     return;
    }
    step = 1;
   }
   // compute Jacobi rotation J which leads to a zero for element [p][q] 
   float mpq = getElement(p, q);
   float theta = (getElement(q, q) - getElement(p, p)) / (2 * mpq);
   float theta2 = theta * theta;
   float cos;
   float sin;
   if (theta2 * theta2 < (10f / SIMD_EPSILON)) {
    t = (theta >= 0) ? 1f / (theta + btSqrt(1 + theta2))
     : 1f / (theta - btSqrt(1f + theta2));
    cos = 1f / btSqrt(1f + t * t);
    sin = cos * t;
   } else {
    // approximation for large theta-value, i.e., a nearly diagonal matrix
    t = 1f / (theta * (2f + (0.5f) / theta2));
    cos = 1f - (0.5f) * t * t;
    sin = cos * t;
   }
   // apply rotation to matrix (this = J^T * this * J)
   setElement(p, q, 0);
   setElement(q, p, 0);
   setElement(p, p, getElement(p, p) - t * mpq);
   setElement(q, q, getElement(q, q) + t * mpq);
   float mrp = getElement(r, p);
   float mrq = getElement(r, q);
   {
    float c = cos * mrp - sin * mrq;
    setElement(r, p, c);
    setElement(p, r, c);
   }
   {
    float c = cos * mrq + sin * mrp;
    setElement(r, q, c);
    setElement(q, r, c);
   }
   // apply rotation to rot (rot = rot * J)
   for (int i = 0; i < 3; i++) {
    mrp = rot.getElement(i, p);
    mrq = rot.getElement(i, q);
    setElement(i, p, cos * mrp - sin * mrq);
    setElement(i, q, cos * mrq + sin * mrp);
   }
  }
 }

 /**
  *
  * @param m
  * @return
  */
 public btMatrix3x3 transposeTimes(final btMatrix3x3 m) {
  return new btMatrix3x3().mulTransposeLeft(this, m);
 }

 /// Solve A * x = b, where b is a column vector. This is more efficient
 /// than computing the inverse in one-shot cases.
 ///Solve33 is from Box2d, thanks to Erin Catto,
 public btVector3 solve33(final btVector3 b) {
  final btVector3 col1 = getColumn(0);
  final btVector3 col2 = getColumn(1);
  final btVector3 col3 = getColumn(2);
  float det = btDot(col1, new btVector3(col2).cross(col3));
  if (btFabs(det) > SIMD_EPSILON) {
   det = 1.0f / det;
  }
  final btVector3 x = new btVector3();
  x.x = det * btDot(b, new btVector3(col2).cross(col3));
  x.y = det * btDot(col1, new btVector3(b).cross(col3));
  x.z = det * btDot(col1, new btVector3(col2).cross(b));
  return x;
 }

 public btMatrix3x3 inverse() {
  throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
 }

 public void getOpenGLMatrix(float[] m) {
  m[M00] = m00;
  m[M01] = m01;
  m[M02] = m02;
  m[M03] = 0;
  m[M10] = m10;
  m[M11] = m11;
  m[M12] = m12;
  m[M13] = 0;
  m[M20] = m20;
  m[M21] = m21;
  m[M22] = m22;
  m[M23] = 0;
  m[M30] = 0;
  m[M31] = 0;
  m[M32] = 0;
  m[M33] = 1.0f;
 }

 /**
  * @brief Set the matrix from euler angles YPR around ZYX axes
  * @param eulerX Roll about X axis
  * @param eulerY Pitch around Y axis
  * @param eulerZ Yaw aboud Z axis
  *
  * These angles are used to produce a rotation matrix. The euler angles are
  * applied in ZYX order. I.e a vector is first rotated about X then Y and then
  * Z
  *
  */
 public btMatrix3x3 setEulerZYX(float eulerX, float eulerY, float eulerZ) {
  ///@todo proposed to reverse this since it's labeled zyx but takes arguments xyz and it will match all other parts of the code
  float ci = btCos(eulerX);
  float cj = btCos(eulerY);
  float ch = btCos(eulerZ);
  float si = btSin(eulerX);
  float sj = btSin(eulerY);
  float sh = btSin(eulerZ);
  float cc = ci * ch;
  float cs = ci * sh;
  float sc = si * ch;
  float ss = si * sh;
  set(cj * ch, sj * sc - cs, sj * cc + ss,
   cj * sh, sj * ss + cc, sj * cs - sc,
   -sj, cj * si, cj * ci);
  return this;
 }

}
