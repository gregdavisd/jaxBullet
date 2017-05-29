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
class btBvhSubtreeInfo implements Serializable {

 //12 bytes
 final int[] m_quantizedAabbMin = new int[3];
 final int[] m_quantizedAabbMax = new int[3];
 //4 bytes, points to the root of the subtree
 int m_rootNodeIndex;
 //4 bytes
 int m_subtreeSize;

 btBvhSubtreeInfo() {
 }

 void setAabbFromQuantizeNode(btQuantizedBvhNode quantizedNode) {
  m_quantizedAabbMin[0] = quantizedNode.m_quantizedAabbMin[0];
  m_quantizedAabbMin[1] = quantizedNode.m_quantizedAabbMin[1];
  m_quantizedAabbMin[2] = quantizedNode.m_quantizedAabbMin[2];
  m_quantizedAabbMax[0] = quantizedNode.m_quantizedAabbMax[0];
  m_quantizedAabbMax[1] = quantizedNode.m_quantizedAabbMax[1];
  m_quantizedAabbMax[2] = quantizedNode.m_quantizedAabbMax[2];
 }
}
