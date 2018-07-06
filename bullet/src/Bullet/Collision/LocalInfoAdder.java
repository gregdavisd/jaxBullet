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

import Bullet.Collision.Broadphase.btBroadphaseProxy;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class LocalInfoAdder extends ConvexResultCallback implements Serializable {

 final ConvexResultCallback m_userCallback;
 int m_i;

 LocalInfoAdder(int i, ConvexResultCallback user) {
  m_userCallback = user;
  m_i = i;
  m_closestHitFraction = m_userCallback.m_closestHitFraction;
 }

 @Override
 public boolean needsCollision(btBroadphaseProxy p) {
  return m_userCallback.needsCollision(p);
 }

 @Override
 public float addSingleResult(LocalConvexResult r, boolean b) {
  LocalShapeInfo shapeInfo = new LocalShapeInfo();
  shapeInfo.m_shapePart = -1;
  shapeInfo.m_triangleIndex = m_i;
  if (r.m_localShapeInfo == null) {
   r.m_localShapeInfo = shapeInfo;
  }
  float result = m_userCallback.addSingleResult(r, b);
  m_closestHitFraction = m_userCallback.m_closestHitFraction;
  return result;
 }

};
