/*
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org
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
package Bullet.Collision;

import Bullet.Collision.Broadphase.btDbvt;
import Bullet.Collision.Broadphase.btDbvtNode;
import Bullet.Collision.Shape.btCollisionShape;
import Bullet.Collision.Shape.btCompoundShape;
import static Bullet.Collision.btCollisionWorld.rayTestSingleInternal;
import Bullet.LinearMath.btTransform;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class RayTester extends btDbvt.ICollide implements Serializable {

 final btCollisionObject m_collisionObject;
 final btCompoundShape m_compoundShape;
 final btTransform m_colObjWorldTransform;
 final btTransform m_rayFromTrans;
 final btTransform m_rayToTrans;
 final RayResultCallback m_resultCallback;

 RayTester(btCollisionObject collisionObject,
  btCompoundShape compoundShape, final btTransform colObjWorldTransform,
  final btTransform rayFromTrans, final btTransform rayToTrans,
  RayResultCallback resultCallback) {
  m_collisionObject = collisionObject;
  m_compoundShape = compoundShape;
  m_colObjWorldTransform = colObjWorldTransform;
  m_rayFromTrans = rayFromTrans;
  m_rayToTrans = rayToTrans;
  m_resultCallback = resultCallback;
 }

 void processLeaf(int i) {
  btCollisionShape childCollisionShape = m_compoundShape.getChildShape(i);
  final btTransform childTrans = m_compoundShape.getChildTransform(i);
  final btTransform childWorldTrans = new btTransform(m_colObjWorldTransform)
   .mul(childTrans);
  btCollisionObjectWrapper tmpOb = new btCollisionObjectWrapper(null,
   childCollisionShape,
   m_collisionObject, childWorldTrans, -1, i);
  // replace collision shape so that callback can determine the triangle
  LocalInfoAdder2 my_cb = new LocalInfoAdder2(i, m_resultCallback);
  rayTestSingleInternal(
   m_rayFromTrans,
   m_rayToTrans,
   tmpOb,
   my_cb);
 }

 @Override
 public void process(btDbvtNode leaf) {
  processLeaf(leaf.dataAsInt());
 }

};
