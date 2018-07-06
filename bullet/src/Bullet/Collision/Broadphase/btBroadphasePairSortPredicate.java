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

import java.io.Serializable;
import java.util.Comparator;
import java.util.Objects;

/**
 *
 * @author Gregery Barton
 */
public class btBroadphasePairSortPredicate implements
 Comparator<btBroadphasePair>, Serializable {

 @Override
 public int compare(btBroadphasePair o1, btBroadphasePair o2) {
  btBroadphasePair a = (btBroadphasePair) o1;
  btBroadphasePair b = (btBroadphasePair) o2;
  int uidA0 = a.m_pProxy0 != null ? a.m_pProxy0.m_uniqueId : Integer.MAX_VALUE;
  int uidB0 = b.m_pProxy0 != null ? b.m_pProxy0.m_uniqueId : Integer.MAX_VALUE;
  int uidA1 = a.m_pProxy1 != null ? a.m_pProxy1.m_uniqueId : Integer.MAX_VALUE;
  int uidB1 = b.m_pProxy1 != null ? b.m_pProxy1.m_uniqueId : Integer.MAX_VALUE;
  int diff = uidA0 - uidB0;
  if (diff != 0) {
   return diff;
  }
  diff = uidA1 - uidB1;
  if (diff != 0) {
   return diff;
  }
  assert (a.m_pProxy0 == b.m_pProxy0 && a.m_pProxy1 == b.m_pProxy1);
  diff = (a.m_algorithm != null ? Objects.hashCode(a.m_algorithm) : Integer.MAX_VALUE)
   - (b.m_algorithm != null ? Objects.hashCode(b.m_algorithm) : Integer.MAX_VALUE);
  return diff;
 }

}
