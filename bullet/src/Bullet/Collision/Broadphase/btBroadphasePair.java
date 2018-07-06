
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
package Bullet.Collision.Broadphase;

import Bullet.Collision.Algorithm.btCollisionAlgorithm;
import java.io.Serializable;
import java.util.Objects;

/**
 *
 * @author Gregery Barton
 */
public class btBroadphasePair implements Serializable {

 public btBroadphaseProxy m_pProxy0;
 public btBroadphaseProxy m_pProxy1;
 public btCollisionAlgorithm m_algorithm;

 btBroadphasePair() {
  m_pProxy0 = null;
  m_pProxy1 = null;
  m_algorithm = null;
 }

 btBroadphasePair(btBroadphasePair other) {
  m_pProxy0 = other.m_pProxy0;
  m_pProxy1 = other.m_pProxy1;
  m_algorithm = other.m_algorithm;
 }

 btBroadphasePair(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1) {
  //keep them sorted, so the std.set operations work
  if (proxy0.m_uniqueId < proxy1.m_uniqueId) {
   m_pProxy0 = proxy0;
   m_pProxy1 = proxy1;
  } else {
   m_pProxy0 = proxy1;
   m_pProxy1 = proxy0;
  }
  m_algorithm = null;
 }

 @Override
 public int hashCode() {
  int hash = 7;
  hash = 41 * hash + Objects.hashCode(this.m_pProxy0.m_uniqueId);
  hash = 41 * hash + Objects.hashCode(this.m_pProxy1.m_uniqueId);
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
  final btBroadphasePair other = (btBroadphasePair) obj;
  return (this.m_pProxy0 == other.m_pProxy0) && (this.m_pProxy1
   == other.m_pProxy1);
 }

};
