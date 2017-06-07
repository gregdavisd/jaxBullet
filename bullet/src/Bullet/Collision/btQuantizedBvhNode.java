/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

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
public class btQuantizedBvhNode implements  Serializable {

//http://msdn.microsoft.com/library/default.asp?url=/library/en-us/vclang/html/vclrf__m128.asp
//Note: currently we have 16 bytes per quantized node
 static final int MAX_SUBTREE_SIZE_IN_BYTES = 2048;
// 10 gives the potential for 1024 parts, with at most 2^21 (2097152) (minus one
// actually) triangles each (since the sign bit is reserved
 static final int MAX_NUM_PARTS_IN_BITS = 10;
 /* TODO: short or int here doesn't matter its the array overhead that takes up memory
 
 */
 //12 bytes
 final short[] m_quantizedAabbMin = new short[3]; 
 final short[] m_quantizedAabbMax = new short[3];
 //4 bytes
 int m_escapeIndexOrTriangleIndex;

 btQuantizedBvhNode(btQuantizedBvhNode node) {
 m_escapeIndexOrTriangleIndex=node.m_escapeIndexOrTriangleIndex;
 System.arraycopy(node.m_quantizedAabbMin, 0, m_quantizedAabbMin, 0, 3);
 System.arraycopy(node.m_quantizedAabbMax, 0, m_quantizedAabbMax, 0, 3);
 }

 public btQuantizedBvhNode() {
 }

 boolean isLeafNode() {
  //skipindex is negative (internal node), triangleindex >=0 (leafnode)
  return (m_escapeIndexOrTriangleIndex >= 0);
 }

 int getEscapeIndex() {
  assert (!isLeafNode());
  return -m_escapeIndexOrTriangleIndex;
 }

 int getTriangleIndex() {
  assert (isLeafNode());
  int x = 0;
  int y = (~(x)) << (31 - MAX_NUM_PARTS_IN_BITS);
  // Get only the lower bits where the triangle index is stored
  return (m_escapeIndexOrTriangleIndex & ~(y));
 }

 int getPartId() {
  assert (isLeafNode());
  // Get only the highest bits where the part index is stored
  return (m_escapeIndexOrTriangleIndex >>> (31 - MAX_NUM_PARTS_IN_BITS));
 }
 
}
