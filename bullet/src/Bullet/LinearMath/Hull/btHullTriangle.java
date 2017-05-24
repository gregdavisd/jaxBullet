/*
Stan Melax Convex Hull Computation
Copyright (c) 2008 Stan Melax http://www.melax.com/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.LinearMath.Hull;


import java.io.Serializable;
import javax.vecmath.Tuple3i;

/**
 *
 * @author Gregery Barton
 */
class btHullTriangle extends Int3 implements Serializable  {

 final Tuple3i n = new Int3();
 int id;
 int vmax;
 float rise;

 btHullTriangle(int a, int b, int c) {
  super(a, b, c);
  n.set(-1, -1, -1);
  vmax = -1;
 }

 int _neib(int a, int b) {
  int i;
  for (i = 0; i < 3; i++) {
   int i1 = (i + 1) % 3;
   int i2 = (i + 2) % 3;
   if (getElement(i) == a && getElement(i1) == b) {
    return i2;
   }
   if (getElement(i) == b && getElement(i1) == a) {
    return i2;
   }
  }
  assert(false);
  return -1;
 }

 int neib(int a, int b) {
  return n.getElement(_neib(a, b));
 }

 void neib_set(int a, int b, int value) {
  n.setElement(_neib(a, b), value);
 }
}
