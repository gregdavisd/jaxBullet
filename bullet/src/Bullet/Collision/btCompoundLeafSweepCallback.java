/*
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
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
import Bullet.Collision.Shape.btConvexShape;
import static Bullet.Collision.btCollisionWorld.objectQuerySingleInternal;
import Bullet.LinearMath.btTransform;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btCompoundLeafSweepCallback extends btDbvt.ICollide implements
 Serializable {

 btCompoundLeafSweepCallback(
  btCollisionObjectWrapper colObjWrap,
  btConvexShape castShape, final btTransform convexFromTrans,
  final btTransform convexToTrans,
  float allowedPenetration,
  btCompoundShape compoundShape, final btTransform colObjWorldTransform,
  ConvexResultCallback resultCallback) {
  m_colObjWrap = colObjWrap;
  m_castShape = castShape;
  m_convexFromTrans = convexFromTrans;
  m_convexToTrans = convexToTrans;
  m_allowedPenetration = allowedPenetration;
  m_compoundShape = compoundShape;
  m_colObjWorldTransform = colObjWorldTransform;
  m_resultCallback = resultCallback;
 }

 final btCollisionObjectWrapper m_colObjWrap;
 final btConvexShape m_castShape;
 final btTransform m_convexFromTrans;
 final btTransform m_convexToTrans;
 float m_allowedPenetration;
 final btCompoundShape m_compoundShape;
 final btTransform m_colObjWorldTransform;
 final ConvexResultCallback m_resultCallback;

 void processChild(int index, final btTransform childTrans,
  btCollisionShape childCollisionShape) {
  final btTransform childWorldTrans = new btTransform(m_colObjWorldTransform)
   .mul(childTrans);
  LocalInfoAdder my_cb = new LocalInfoAdder(index, m_resultCallback);
  btCollisionObjectWrapper tmpObj = new btCollisionObjectWrapper(m_colObjWrap,
   childCollisionShape,
   m_colObjWrap.getCollisionObject(), childWorldTrans, -1, index);
  objectQuerySingleInternal(m_castShape, m_convexFromTrans, m_convexToTrans,
   tmpObj, my_cb,
   m_allowedPenetration);
 }

 @Override
 public void process(btDbvtNode leaf) {
  // Processing leaf node
  int index = leaf.dataAsInt();
  final btTransform childTrans = m_compoundShape.getChildTransform(index);
  btCollisionShape childCollisionShape = m_compoundShape.getChildShape(index);
  processChild(index, childTrans, childCollisionShape);
 }

};
