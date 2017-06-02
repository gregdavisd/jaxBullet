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

import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
class btOptimizedBvhNode implements   Serializable {

 //32 bytes
 final btVector3 m_aabbMinOrg = new btVector3();
 final btVector3 m_aabbMaxOrg = new btVector3();
 //4
 int m_escapeIndex;
 //8
 //for child nodes
 int m_subPart;
 int m_triangleIndex;

 btOptimizedBvhNode(btOptimizedBvhNode node) {
  m_aabbMinOrg.set(node.m_aabbMinOrg);
  m_aabbMaxOrg.set(node.m_aabbMaxOrg);
  m_escapeIndex=node.m_escapeIndex;
  m_subPart=node.m_subPart;
  m_triangleIndex=node.m_triangleIndex;
 }

 public btOptimizedBvhNode() {
 }
 
} 
