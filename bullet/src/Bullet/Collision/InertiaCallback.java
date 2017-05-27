/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

package Bullet.Collision;

import Bullet.LinearMath.btMatrix3x3;
import static Bullet.LinearMath.btScalar.btFabs;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
 public class InertiaCallback implements btTriangleCallback  , Serializable {

 final btMatrix3x3 sum = new btMatrix3x3();
 final btVector3 center = new btVector3();

  public InertiaCallback(final btVector3 center) {
  this.center.set(center);
 }

 @Override
 public boolean processTriangle(btVector3[] triangle, int partId, int triangleIndex) {
  final btVector3 _a = new btVector3(triangle[0]).sub(center);
  final btVector3 _b = new btVector3(triangle[1]).sub(center);
  final btVector3 _c = new btVector3(triangle[2]).sub(center);
  float volNeg = -btFabs(_a.triple(_b, _c)) * (1.f / 6f);
  float[] a = new float[3];
  _a.get(a);
  float[] b = new float[3];
  _b.get(b);
  float[] c = new float[3];
  _c.get(c);
  float[][] i = new float[3][3];
  for (int j = 0; j < 3; j++) {
   for (int k = 0; k <= j; k++) {
    i[j][k] = i[k][j] = volNeg * ((0.1f) * (a[j] * a[k] + b[j] * b[k] + c[j] * c[k]) +
     (0.05f) * (a[j] * b[k] + a[k] * b[j] + a[j] * c[k] + a[k] * c[j] + b[j] * c[k] + b[k] * c[j]));
   }
  }
  float i00 = -i[0][0];
  float i11 = -i[1][1];
  float i22 = -i[2][2];
  i[0][0] = i11 + i22;
  i[1][1] = i22 + i00;
  i[2][2] = i00 + i11;
  final btMatrix3x3 _i = new btMatrix3x3(i);
  sum.setRow(0, sum.getRow(0).add(_i.getRow(0)));
  sum.setRow(1, sum.getRow(1).add(_i.getRow(1)));
  sum.setRow(2, sum.getRow(2).add(_i.getRow(2)));
  return true;
 }

 public  btMatrix3x3 getInertia() {
  return new btMatrix3x3(sum);
 }
};
