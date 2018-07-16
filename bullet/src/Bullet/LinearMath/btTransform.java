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

import java.io.Serializable;
import javax.vecmath.Matrix4f;

/**
 *
 * @author Gregery Barton
 */
public class btTransform extends Matrix4f<btTransform> implements Serializable {

 private static final btTransform IDENTITY = new btTransform().setIdentity();

 /*
  * corresponding index in a column major OpenGL matrix   *
  */
private static final int M00 = 0;
private static final int M01 = 4;
private static final int M02 = 8;
private static final int M03 = 12;
private static final int M10 = 1;
private static final int M11 = 5;
private static final int M12 = 9;
private static final int M13 = 13;
private static final int M20 = 2;
private static final int M21 = 6;
private static final int M22 = 10;
private static final int M23 = 14;
private static final int M30 = 3;
private static final int M31 = 7;
private static final int M32 = 11;
private static final int M33 = 15;
private static final int C4X4 = 16;

 /**
  *
  * @param m00
  * @param m01
  * @param m02
  * @param m03
  * @param m10
  * @param m11
  * @param m12
  * @param m13
  * @param m20
  * @param m21
  * @param m22
  * @param m23
  * @param m30
  * @param m31
  * @param m32
  * @param m33
  */
 public btTransform(float m00, float m01, float m02, float m03, float m10,
  float m11, float m12,
  float m13, float m20, float m21, float m22, float m23, float m30, float m31,
  float m32, float m33) {
  super(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31,
   m32, m33);
 }

 /**
  *
  * @param v
  */
 public btTransform(float[] v) {
  super(v);
 }

 /**
  *
  * @param m1
  */
 public btTransform(final btTransform m1) {
  super(m1);
 }

 public btTransform(final Matrix4f m1) {
  super(m1);
 }

 /**
  *
  * @param m1
  * @param t1
  * @param s
  */
 public btTransform(final btMatrix3x3 m1, final btVector3 t1, float s) {
  super(m1, t1, s);
 }

 /**
  *
  * @param m1
  */
 public btTransform(final btMatrix3x3 m1) {
  super(m1);
 }

 /**
  *
  * @param m1
  * @param t1
  */
 public btTransform(final btMatrix3x3 m1, final btVector3 t1) {
  super(m1, t1);
 }

 /**
  * btTransform
  */
 public btTransform() {
  m33 = 1.0f;
 }

 /**
  *
  * @param quat
  * @param position
  */
 public btTransform(final btQuaternion quat, final btVector3 position) {
  set(quat);
  setOrigin(position);
 }

 /**
  *
  * @param v
  */
 public btTransform setOrigin(final btVector3 v) {
  m03 = v.x;
  m13 = v.y;
  m23 = v.z;
  return this;
 }

 /**
  *
  * @return
  */
 public btVector3 getOrigin() {
  return new btVector3(m03, m13, m23);
 }

 /**
  * The returned matrix does not back the original transform unlike in the c++
  * version. Use setBasis instead.
  *
  * @return
  */
 public btMatrix3x3 getBasis() {
  return new btMatrix3x3(this);
 }

 /**
  * Get a 3 component vector from a columns. The returned vector is independent
  * of the matrix.
  *
  * @param column
  * @return
  */
 public btVector3 getBasisColumn(int column) {
  final btVector3 v = new btVector3();
  getBasisColumn(column, v);
  return v;
 }

 /**
  * Get a 3 component vector from a columns. The returned vector is independent
  * of the matrix.
  *
  * @param column number
  * @param v vector output
  * @return
  */
 public btVector3 getBasisColumn(int column, final btVector3 v) {
  switch (column) {
   case 0:
    v.x = m00;
    v.y = m10;
    v.z = m20;
    break;
   case 1:
    v.x = m01;
    v.y = m11;
    v.z = m21;
    break;
   case 2:
    v.x = m02;
    v.y = m12;
    v.z = m22;
    break;
   default:
    throw new ArrayIndexOutOfBoundsException();
  }
  return v;
 }

 /**
  * Get a 3 component vector from a row. The returned vector is independent of
  * the matrix.
  *
  * @param row number
  * @return
  */
 public btVector3 getBasisRow(int row) {
  final btVector3 r = new btVector3();
  return getBasisRow(row, r);
 }

 /**
  * Get a 3 component vector from a row. The returned vector is independent of
  * the matrix.
  *
  * @param row number
  * @param v output vector
  * @return
  */
 public btVector3 getBasisRow(int row, final btVector3 v) {
  switch (row) {
   case 0:
    v.x = m00;
    v.y = m01;
    v.z = m02;
    break;
   case 1:
    v.x = m10;
    v.y = m11;
    v.z = m12;
    break;
   case 2:
    v.x = m20;
    v.y = m21;
    v.z = m22;
    break;
   default:
    throw new ArrayIndexOutOfBoundsException();
  }
  return v;
 }

 /**
  * Replace the upper 3x3 components of this matrix
  *
  * @param basis
  */
 public void setBasis(final btMatrix3x3 basis) {
  set3x3(basis);
 }

 /**
  *
  * @param t
  * @return
  */
 public btTransform inverseTimes(final btTransform t) {
  final btVector3 v = t.getOrigin().sub(getOrigin());
  final btMatrix3x3 basis = getBasis();
  return new btTransform(basis.transposeTimes(t.getBasis()), basis
   .transposeTransform(v));
 }

 /**
  * Return a normalized quaternion that represents the rotation of this matrix.
  *
  * @return
  */
 public btQuaternion getRotation() {
  return new btQuaternion().set(this);
 }

 /**
  *
  * @param pointA
  * @return
  */
 public btVector3 invXform(final btVector3 pointA) {
  final btVector3 v = new btVector3(pointA).sub(getOrigin());
  return getBasis().transposeTransform(v);
 }

 public void getOpenGLMatrix(float[] m) {
  m[M00] = m00;
  m[M01] = m01;
  m[M02] = m02;
  m[M03] = m03;
  m[M10] = m10;
  m[M11] = m11;
  m[M12] = m12;
  m[M13] = m13;
  m[M20] = m20;
  m[M21] = m21;
  m[M22] = m22;
  m[M23] = m23;
  m[M30] = m30;
  m[M31] = m31;
  m[M32] = m32;
  m[M33] = m33;
 }

 public static btTransform getIdentity() {
  return new btTransform(IDENTITY);
 }

}
