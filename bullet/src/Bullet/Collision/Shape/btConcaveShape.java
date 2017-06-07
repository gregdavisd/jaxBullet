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
package Bullet.Collision.Shape;

import Bullet.Collision.btTriangleCallback;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public abstract class btConcaveShape extends btCollisionShape implements Serializable {

 protected float m_collisionMargin;

 btConcaveShape() {
  m_collisionMargin = 0;
 }

 public abstract void processAllTriangles(btTriangleCallback callback, final btVector3 aabbMin,
  final btVector3 aabbMax);

 @Override
 public final float getMargin() {
  return m_collisionMargin;
 }

 @Override
 public final void setMargin(float collisionMargin) {
  m_collisionMargin = collisionMargin;
 }

 @Override
 public int hashCode() {
  int hash = 7;
  hash = super.hashCode() + (83 * hash + Float.floatToIntBits(this.m_collisionMargin));
  return hash;
 }

 @Override
 public boolean equals(Object obj) {
  if (this == obj) {
   return true;
  }
  if (obj == null) {
   return false;
  }
  if (getClass() != obj.getClass()) {
   return false;
  }
  final btConcaveShape other = (btConcaveShape) obj;
  if (Float.floatToIntBits(this.m_collisionMargin) != Float.floatToIntBits(other.m_collisionMargin)) {
   return false;
  }
  return super.equals(obj);
 }
}
