/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

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

import static Bullet.Collision.btQuantizedBvhNode.MAX_NUM_PARTS_IN_BITS;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author Gregery Barton
 */
class QuantizedNodeTriangleCallback implements btTriangleCallback, Serializable {

 final ArrayList<btQuantizedBvhNode> m_triangleNodes;
 final btQuantizedBvh m_optimizedTree; // for quantization

 QuantizedNodeTriangleCallback(ArrayList<btQuantizedBvhNode> triangleNodes, btQuantizedBvh tree) {
  m_triangleNodes = triangleNodes;
  m_optimizedTree = tree;
 }

 QuantizedNodeTriangleCallback set(QuantizedNodeTriangleCallback other) {
  m_triangleNodes.ensureCapacity(other.m_triangleNodes.size());
  m_triangleNodes.clear();
  for (btQuantizedBvhNode node : other.m_triangleNodes) {
    m_triangleNodes.add(new btQuantizedBvhNode(node));
  }
  return this;
 }

 @Override
 public boolean processTriangle(btVector3[] triangle, int partId, int triangleIndex) {
  // The partId and triangle index must fit in the same (positive) integer
  assert (partId < (1 << MAX_NUM_PARTS_IN_BITS));
  assert (triangleIndex < (1 << (31 - MAX_NUM_PARTS_IN_BITS)));
  //negative indices are reserved for escapeIndex
  assert (triangleIndex >= 0);
  btQuantizedBvhNode node = new btQuantizedBvhNode();
  final btVector3 aabbMin = new btVector3();
  final btVector3 aabbMax = new btVector3();
  aabbMin.set((BT_LARGE_FLOAT), (BT_LARGE_FLOAT), (BT_LARGE_FLOAT));
  aabbMax.set((-BT_LARGE_FLOAT), (-BT_LARGE_FLOAT), (-BT_LARGE_FLOAT));
  aabbMin.setMin(triangle[0]);
  aabbMax.setMax(triangle[0]);
  aabbMin.setMin(triangle[1]);
  aabbMax.setMax(triangle[1]);
  aabbMin.setMin(triangle[2]);
  aabbMax.setMax(triangle[2]);
  //PCK: add these checks for zero dimensions of aabb
  float MIN_AABB_DIMENSION = (0.002f);
  float MIN_AABB_HALF_DIMENSION = (0.001f);
  if (aabbMax.x() - aabbMin.x() < MIN_AABB_DIMENSION) {
   aabbMax.setX(aabbMax.x() + MIN_AABB_HALF_DIMENSION);
   aabbMin.setX(aabbMin.x() - MIN_AABB_HALF_DIMENSION);
  }
  if (aabbMax.y() - aabbMin.y() < MIN_AABB_DIMENSION) {
   aabbMax.setY(aabbMax.y() + MIN_AABB_HALF_DIMENSION);
   aabbMin.setY(aabbMin.y() - MIN_AABB_HALF_DIMENSION);
  }
  if (aabbMax.z() - aabbMin.z() < MIN_AABB_DIMENSION) {
   aabbMax.setZ(aabbMax.z() + MIN_AABB_HALF_DIMENSION);
   aabbMin.setZ(aabbMin.z() - MIN_AABB_HALF_DIMENSION);
  }
  m_optimizedTree.quantize(node.m_quantizedAabbMin, aabbMin, 0);
  m_optimizedTree.quantize(node.m_quantizedAabbMax, aabbMax, 1);
  node.m_escapeIndexOrTriangleIndex = (partId << (31 - MAX_NUM_PARTS_IN_BITS)) | triangleIndex;
  m_triangleNodes.add(node);
  return true;
 }
};
