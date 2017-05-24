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

package Bullet.Extras;

import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btMinMax  implements Serializable {

 /**
  *
  * @param a
  * @param b
  * @return
  */
 public static float btMin(float a, float b) {
  return a < b ? a : b;
 }

 /**
  *
  * @param a
  * @param b
  * @return
  */
 public static int btMin(int a, int b) {
  return a < b ? a : b;
 }

 /**
  *
  * @param a
  * @param b
  * @return
  */
 public static float btMax(float a, float b) {
  return a > b ? a : b;
 }

 /**
  *
  * @param a
  * @param b
  * @return
  */
 public static int btMax(int a, int b) {
  return a > b ? a : b;
 }

 /**
  *
  * @param a
  * @param lb
  * @param ub
  * @return
  */
 public static float btClamped(float a, float lb, float ub) {
  return a < lb ? lb : (ub < a ? ub : a);
 }
}
