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
import javax.vecmath.Tuple3f;
import javax.vecmath.Tuple4f;
import javax.vecmath.Vector4f;

/**
 *
 * @author Gregery Barton
 */
public class btVector4 extends Tuple4f<btVector4> implements Serializable {

 /**
  *
  * @param x
  * @param y
  * @param z
  * @param w
  */
 public btVector4(float x, float y, float z, float w) {
  super(x, y, z, w);
 }

 /**
  *
  * @param v
  */
 public btVector4(float[] v) {
  super(v);
 }

 /**
  *
  * @param v1
  */
 public btVector4(Vector4f v1) {
  super(v1);
 }

 /**
  *
  * @param t1
  */
 public btVector4(Tuple4f t1) {
  super(t1);
 }

 /**
  *
  * @param t1
  * @param w
  */
 public btVector4(Tuple3f t1, float w) {
  super(t1, w);
 }

 /**
  * btVector4
  */
 public btVector4() {
 }

 /**
  *
  * @return
  */
 public btVector4 absolute4() {
  return (btVector4) new btVector4(this).abs();
 }

 /**
  *
  * @return
  */
 public int closestAxis4() {
  return absolute4().maxAxis();
 }

 public float x() {
  return x;
 }
 
 public float y() {
  return y;
 }
 
 public float z() {
  return z;
 }
 
 public float w() {
  return w;
 }
 
}
