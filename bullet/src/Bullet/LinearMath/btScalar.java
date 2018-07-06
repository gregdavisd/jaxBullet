/*
 * Copyright (c) 2003-2009 Erwin Coumans  http://bullet.googlecode.com
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
import static javax.vecmath.VecMath.acos;
import static javax.vecmath.VecMath.asin;
import static javax.vecmath.VecMath.atan2;
import static javax.vecmath.VecMath.cos;
import static javax.vecmath.VecMath.pow;
import static javax.vecmath.VecMath.sin;
import static javax.vecmath.VecMath.sqrt;

/**
 *
 * @author Gregery Barton
 */
public class btScalar implements Serializable {

 /**
  *
  */
 //public static final float FLT_EPSILON = 1.192092896e-07F;
 public static final float FLT_EPSILON = 1.192092896e-07F;
 /**
  *
  */
 public static final float FLT_MAX = Float.MAX_VALUE;
 /**
  *
  */
 public static final float BT_LARGE_FLOAT = 1e18f;
 /**
  *
  */
 public static final float SIMD_EPSILON = FLT_EPSILON;
 /**
  *
  */
 public static final float SIMD_INFINITY = FLT_MAX;
 /**
  *
  */
 public static final float SIMD_PI = 3.1415926535897932384626433832795029f;
 /**
  *
  */
 public static final float SIMD_2_PI = 2.0f * SIMD_PI;
 /**
  *
  */
 public static final float SIMD_HALF_PI = SIMD_PI * 0.5f;
 /**
  *
  */
 public static final float SIMD_RADS_PER_DEG = (SIMD_2_PI / 360.0f);
 /**
  *
  */
 public static final float SIMDSQRT12 = 0.7071067811865475244008443621048490f;
 /**
  *
  */
 public static final float B3_INFINITY = FLT_MAX;
 public static final float SIMD_DEGS_PER_RAD = ((360.0f) / SIMD_2_PI);
 public static final int BT_BULLET_VERSION = 286;
 public static final float BT_ONE = 1.0f;
 public static final float BT_ZERO = 0.0f;
 public static final float M_PI = (float) Math.PI;
 public static final float M_PI_2 = M_PI / 2.0f;

 /**
  *
  * @param r
  * @return
  */
 public static float btSin(float r) {
  return sin(r);
 }

 /**
  *
  * @param x
  * @return
  */
 public static float btAcos(float x) {
  float rx = x;
  if (rx < (-1f)) {
   rx = (-1f);
  }
  if (rx > (1f)) {
   rx = (1f);
  }
  return acos(rx);
 }

 public static float btAsin(float x) {
  float rx = x;
  if (rx < (-1f)) {
   rx = (-1f);
  }
  if (rx > (1f)) {
   rx = (1f);
  }
  return asin(rx);
 }

 public static float btAtan2(float y, float x) {
  return atan2(y, x);
 }

 /**
  *
  * @param r
  * @return
  */
 public static float btCos(float r) {
  return cos(r);
 }

 /**
  *
  * @param x
  * @return
  */
 public static float btFabs(float x) {
  return Math.abs(x);
 }

 /**
  *
  * @param x
  * @return
  */
 public static float btRecipSqrt(float x) {
  return 1.0f / btSqrt(x);
 }

 /**
  *
  * @param x
  * @return
  */
 public static float btSqrt(float x) {
  return sqrt(x);
 }

 /**
  *
  * @param a
  * @param b
  * @param c
  * @return
  */
 public static float btFsels(float a, float b, float c) {
  return a >= 0 ? b : c;
 }

 public static float btPow(float a, float b) {
  return pow(a, b);
 }
///btSelect avoids branches, which makes performance much better for consoles like Playstation 3 and XBox 360
///Thanks Phil Knight. See also http://www.cellperformance.com/articles/2006/04/more_techniques_for_eliminatin_1.html

 /**
  *
  * @param condition
  * @param valueIfConditionNonZero
  * @param valueIfConditionZero
  * @return
  */
 public static int btSelect(int condition, int valueIfConditionNonZero,
  int valueIfConditionZero) {
  // Set testNz to 0xFFFFFFFF if condition is nonzero, 0x00000000 if condition is zero
  // Rely on positive value or'ed with its negative having sign bit on
  // and zero value or'ed with its negative (which is still zero) having sign bit off
  // Use arithmetic shift right, shifting the sign bit through all 32 bits
  int testNz = ((condition | -condition) >>> 31);
  int testEqz = ~testNz;
  return ((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz));
 }

 public static boolean btFuzzyZero(float x) {
  return btFabs(x) < SIMD_EPSILON;
 }

 public static float btFmod(float x, float y) {
  return x % y;
 }

 // returns normalized value in range [-SIMD_PI, SIMD_PI]
 public static float btNormalizeAngle(float angleInRadians) {
  float _angleInRadians = btFmod(angleInRadians, SIMD_2_PI);
  if (_angleInRadians < -SIMD_PI) {
   return _angleInRadians + SIMD_2_PI;
  } else if (_angleInRadians > SIMD_PI) {
   return _angleInRadians - SIMD_2_PI;
  } else {
   return _angleInRadians;
  }
 }

 public static boolean btEqual(float a, float eps) {
  return (((a) <= eps) && !((a) < -eps));
 }

 public static float btAtan2Fast(float y, float x) {
  float coeff_1 = SIMD_PI / 4.0f;
  float coeff_2 = 3.0f * coeff_1;
  float abs_y = btFabs(y);
  float angle;
  if (x >= 0.0f) {
   float r = (x - abs_y) / (x + abs_y);
   angle = coeff_1 - coeff_1 * r;
  } else {
   float r = (x + abs_y) / (abs_y - x);
   angle = coeff_2 - coeff_1 * r;
  }
  return (y < 0.0f) ? -angle : angle;
 }

}
