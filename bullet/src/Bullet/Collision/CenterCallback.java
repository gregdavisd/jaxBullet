/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

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

import static Bullet.LinearMath.btScalar.btFabs;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
 public class CenterCallback implements btTriangleCallback,   Serializable  {

 boolean first;
 final btVector3 ref = new btVector3();
 final btVector3 sum = new btVector3();
 float volume;

  public CenterCallback() {
  first = true;
 }

 @Override
 public boolean processTriangle(btVector3[] triangle, int partId, int triangleIndex) {
  if (first) {
   ref.set(triangle[0]);
   first = false;
  } else {
   float vol = btFabs(new btVector3(triangle[0]).sub(ref)
    .triple(new btVector3(triangle[1]).sub(ref),
     new btVector3(triangle[2]).sub(ref)));
   sum.add(new btVector3(triangle[0]).add(triangle[1]).add(triangle[2]).add(ref)).scale(((0.25f) *
    vol));
   volume += vol;
  }
  return true;
 }

  public btVector3 getCenter() {
  return (volume > 0f) ? new btVector3(sum).scale(1.0f / volume) : new btVector3(ref);
 }

  public float getVolume() {
  return volume * (1.f / 6f);
 }
};
