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

import Bullet.Collision.Broadphase.btDbvtNode;
import Bullet.LinearMath.btTransform;
import java.io.Serializable;
import java.util.Objects;

/**
 *
 * @author Gregery Barton
 */
public class btCompoundShapeChild  implements Serializable {

 final btTransform m_transform = new btTransform();
 btCollisionShape m_childShape;
 int m_childShapeType;
 float m_childMargin;
 btDbvtNode m_node;

 @Override
 public int hashCode() {
  int hash = 7;
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
  final btCompoundShapeChild other = (btCompoundShapeChild) obj;
  if (this.m_childShapeType != other.m_childShapeType) {
   return false;
  }
  if (Float.floatToIntBits(this.m_childMargin) != Float.floatToIntBits(other.m_childMargin)) {
   return false;
  }
  if (!Objects.equals(this.m_transform, other.m_transform)) {
   return false;
  }
  return Objects.equals(this.m_childShape, other.m_childShape);
 }
}
