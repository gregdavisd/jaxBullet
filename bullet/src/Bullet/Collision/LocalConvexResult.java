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

import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class LocalConvexResult implements Serializable {

 public LocalConvexResult(btCollisionObject hitCollisionObject,
  LocalShapeInfo localShapeInfo, final btVector3 hitNormalLocal,
  final btVector3 hitPointLocal,
  float hitFraction
 ) {
  m_hitCollisionObject = hitCollisionObject;
  m_localShapeInfo = localShapeInfo;
  m_hitNormalLocal.set(hitNormalLocal);
  m_hitPointLocal.set(hitPointLocal);
  m_hitFraction = hitFraction;
 }

 final public btCollisionObject m_hitCollisionObject;
 public LocalShapeInfo m_localShapeInfo;
 public final btVector3 m_hitNormalLocal = new btVector3();
 public final btVector3 m_hitPointLocal = new btVector3();
 public float m_hitFraction;
}
