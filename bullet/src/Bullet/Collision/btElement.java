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

import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btElement implements Comparable, Serializable {

 public int m_id;
 public int m_sz;

 @Override
 public int hashCode() {
  int hash = 3;
  hash = 53 * hash + this.m_id;
  hash = 53 * hash + this.m_sz;
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
  final btElement other = (btElement) obj;
  if (this.m_id != other.m_id) {
   return false;
  }
  return this.m_sz == other.m_sz;
 }

 @Override
 public int compareTo(Object o) {
  return m_id - ((btElement) o).m_id;
 }
}
