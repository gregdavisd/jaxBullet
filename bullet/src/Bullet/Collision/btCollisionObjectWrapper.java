/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

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

import Bullet.Collision.Shape.btCollisionShape;
import Bullet.LinearMath.btTransform;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btCollisionObjectWrapper  implements Serializable {

 final btCollisionObjectWrapper m_parent;
 final btCollisionShape m_shape;
 final btCollisionObject m_collisionObject;
 final btTransform m_worldTransform;
 int m_partId;
 int m_index;

 // not implemented. Not allowed.
 public btCollisionObjectWrapper(btCollisionObjectWrapper w) {
  throw new AssertionError();
 }

 public btCollisionObjectWrapper(btCollisionObjectWrapper parent, btCollisionShape shape,
  btCollisionObject collisionObject, final btTransform worldTransform, int partId, int index) {
  m_parent = parent;
  m_shape = shape;
  m_collisionObject = collisionObject;
  m_worldTransform = worldTransform;
  m_partId = partId;
  m_index = index;
 }

 public btTransform getWorldTransform() {
  return new btTransform(m_worldTransform);
 }

 public btCollisionObject getCollisionObject() {
  return m_collisionObject;
 }

 public btCollisionShape getCollisionShape() {
  return m_shape;
 }
}
