/*
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2011 Advanced Micro Devices, Inc.  http://bulletphysics.org
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

import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btInternalEdge implements Serializable {

 btInternalEdge() {
  m_face0 = -1;
  m_face1 = -1;
 }

 int m_face0;
 int m_face1;

 @Override
 public int hashCode() {
  int hash = 7;
  hash = 89 * hash + this.m_face0;
  hash = 89 * hash + this.m_face1;
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
  final btInternalEdge other = (btInternalEdge) obj;
  if (this.m_face0 != other.m_face0) {
   return false;
  }
  return this.m_face1 == other.m_face1;
 }

}
