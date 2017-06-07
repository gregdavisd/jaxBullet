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

import Bullet.Collision.Shape.btStridingMeshLock;
import Bullet.Collision.Shape.btStridingMeshInterface;
import static Bullet.LinearMath.btAabbUtil2.testQuantizedAabbAgainstQuantizedAabb;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btOptimizedBvh extends btQuantizedBvh implements Serializable {

 public btOptimizedBvh() {
 }

 public void build(btStridingMeshInterface triangles, boolean useQuantizedAabbCompression,
  final btVector3 bvhAabbMin, final btVector3 bvhAabbMax) {
  m_useQuantization = useQuantizedAabbCompression;
  int numLeafNodes;
  if (m_useQuantization) {
   //initialize quantization values
   setQuantizationValues(bvhAabbMin, bvhAabbMax);
   QuantizedNodeTriangleCallback callback = new QuantizedNodeTriangleCallback(m_quantizedLeafNodes,
    this);
   triangles.InternalProcessAllTriangles(callback, new btVector3(), new btVector3());
   //now we have an array of leafnodes in m_leafNodes
   numLeafNodes = m_quantizedLeafNodes.size();
   int new_size = numLeafNodes * 2;
   m_quantizedContiguousNodes.ensureCapacity(new_size);
   m_quantizedContiguousNodes.clear();
   while (m_quantizedContiguousNodes.size() < new_size) {
    m_quantizedContiguousNodes.add(new btQuantizedBvhNode());
   }
  } else {
   BuildOptimizedBvhNodeCallback callback = new BuildOptimizedBvhNodeCallback(m_leafNodes);
   final btVector3 aabbMin = new btVector3((-BT_LARGE_FLOAT), (-BT_LARGE_FLOAT), (-BT_LARGE_FLOAT));
   final btVector3 aabbMax = new btVector3((BT_LARGE_FLOAT), (BT_LARGE_FLOAT), (BT_LARGE_FLOAT));
   triangles.InternalProcessAllTriangles(callback, aabbMin, aabbMax);
   //now we have an array of leafnodes in m_leafNodes
   numLeafNodes = m_leafNodes.size();
   int new_size = numLeafNodes * 2;
   m_contiguousNodes.ensureCapacity(new_size);
   m_contiguousNodes.clear();
   while (m_contiguousNodes.size() < new_size) {
    m_contiguousNodes.add(new btOptimizedBvhNode());
   }
  }
  m_curNodeIndex = 0;
  buildTree(0, numLeafNodes);
  ///if the entire tree is small then subtree size, we need to create a header info for the tree
  if (m_useQuantization && m_SubtreeHeaders.isEmpty()) {
   btBvhSubtreeInfo subtree = new btBvhSubtreeInfo();
   m_SubtreeHeaders.add(subtree);
   subtree.setAabbFromQuantizeNode(m_quantizedContiguousNodes.get(0));
   subtree.m_rootNodeIndex = 0;
   subtree.m_subtreeSize = m_quantizedContiguousNodes.get(0).isLeafNode() ? 1 :
    m_quantizedContiguousNodes.get(0).getEscapeIndex();
  }
  //PCK: clear m_quantizedLeafNodes and m_leafNodes, they are temporary
  m_quantizedLeafNodes.clear();
  m_quantizedLeafNodes.trimToSize();
  m_leafNodes.clear();
  m_leafNodes.trimToSize();
 }

 public void refit(btStridingMeshInterface meshInterface, final btVector3 aabbMin,
  final btVector3 aabbMax) {
  if (m_useQuantization) {
   setQuantizationValues(aabbMin, aabbMax);
   updateBvhNodes(meshInterface, 0, m_curNodeIndex, 0);
   ///now update all subtree headers
   int i;
   for (i = 0; i < m_SubtreeHeaders.size(); i++) {
    btBvhSubtreeInfo subtree = m_SubtreeHeaders.get(i);
    subtree.setAabbFromQuantizeNode(m_quantizedContiguousNodes.get(subtree.m_rootNodeIndex));
   }
  } else {
   // don't know why this is empty
   assert (false);
  }
 }

 public void refitPartial(btStridingMeshInterface meshInterface, final btVector3 aabbMin,
  final btVector3 aabbMax) {
  //incrementally initialize quantization values
  assert (m_useQuantization);
  assert (aabbMin.getX() > m_bvhAabbMin.getX());
  assert (aabbMin.getY() > m_bvhAabbMin.getY());
  assert (aabbMin.getZ() > m_bvhAabbMin.getZ());
  assert (aabbMax.getX() < m_bvhAabbMax.getX());
  assert (aabbMax.getY() < m_bvhAabbMax.getY());
  assert (aabbMax.getZ() < m_bvhAabbMax.getZ());
  ///we should update all quantization values, using updateBvhNodes(meshInterface);
  ///but we only update chunks that overlap the given aabb
  short[] quantizedQueryAabbMin = new short[3];
  short[] quantizedQueryAabbMax = new short[3];
  quantize(quantizedQueryAabbMin, aabbMin, 0);
  quantize(quantizedQueryAabbMax, aabbMax, 1);
  int i;
  for (i = 0; i < this.m_SubtreeHeaders.size(); i++) {
   btBvhSubtreeInfo subtree = m_SubtreeHeaders.get(i);
   //PCK: unsigned instead of boolean
   boolean overlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,
    quantizedQueryAabbMax, subtree.m_quantizedAabbMin, subtree.m_quantizedAabbMax) != 0;
   if (overlap) {
    updateBvhNodes(meshInterface, subtree.m_rootNodeIndex, subtree.m_rootNodeIndex +
     subtree.m_subtreeSize, i);
    subtree.setAabbFromQuantizeNode(m_quantizedContiguousNodes.get(subtree.m_rootNodeIndex));
   }
  }
 }

 public void updateBvhNodes(btStridingMeshInterface meshInterface, int firstNode, int endNode,
  int index) {
  assert (m_useQuantization);
  int curNodeSubPart = -1;
  //get access info to trianglemesh data
  // InternalProcessSubPart does the scaling for us
//  final btVector3 meshScaling = meshInterface.getScaling();
  UpdateOptimizedBvhNodeCallback callback = new UpdateOptimizedBvhNodeCallback(this);
  final btVector3 aabbMin = new btVector3();
  final btVector3 aabbMax = new btVector3();
  btStridingMeshLock mesh_lock = null;
  int i;
  for (i = endNode - 1; i >= firstNode; i--) {
   btQuantizedBvhNode curNode = m_quantizedContiguousNodes.get(i);
   callback.setCurNode(curNode);
   if (curNode.isLeafNode()) {
    //recalc aabb from triangle data
    int nodeSubPart = curNode.getPartId();
    int nodeTriangleIndex = curNode.getTriangleIndex();
    if (nodeSubPart != curNodeSubPart) {
//					if (curNodeSubPart >= 0)
//						meshInterface.unLockReadOnlyVertexBase(curNodeSubPart);
     mesh_lock = meshInterface.getLockedReadOnlyVertexIndexBase(nodeSubPart);
     curNodeSubPart = nodeSubPart;
    }
    meshInterface.InternalProcessSubPart(callback, mesh_lock, aabbMin, aabbMax, nodeSubPart,
     nodeTriangleIndex);
   } else {
    //combine aabb from both children
    btQuantizedBvhNode leftChildNode = m_quantizedContiguousNodes.get(i + 1);
    btQuantizedBvhNode rightChildNode = leftChildNode.isLeafNode() ? m_quantizedContiguousNodes.get(
     i + 2) :
     m_quantizedContiguousNodes.get(i + 1 + leftChildNode.getEscapeIndex());
    {
     for (int k = 0; k < 3; k++) {
      curNode.m_quantizedAabbMin[k] = leftChildNode.m_quantizedAabbMin[k];
      if (curNode.m_quantizedAabbMin[k] > rightChildNode.m_quantizedAabbMin[k]) {
       curNode.m_quantizedAabbMin[k] = rightChildNode.m_quantizedAabbMin[k];
      }
      curNode.m_quantizedAabbMax[k] = leftChildNode.m_quantizedAabbMax[k];
      if (curNode.m_quantizedAabbMax[k] < rightChildNode.m_quantizedAabbMax[k]) {
       curNode.m_quantizedAabbMax[k] = rightChildNode.m_quantizedAabbMax[k];
      }
     }
    }
   }
  }
//		if (curNodeSubPart >= 0)
//			meshInterface.unLockReadOnlyVertexBase(curNodeSubPart);
 }
}
