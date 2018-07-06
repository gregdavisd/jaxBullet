/*
 * Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/
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

import Bullet.Collision.Broadphase.btNodeOverlapCallback;
import static Bullet.LinearMath.btAabbUtil2.TestAabbAgainstAabb2;
import static Bullet.LinearMath.btAabbUtil2.btRayAabb2;
import static Bullet.LinearMath.btAabbUtil2.testQuantizedAabbAgainstQuantizedAabb;
import static Bullet.LinearMath.btScalar.BT_BULLET_VERSION;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import static Bullet.LinearMath.btScalar.SIMD_INFINITY;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.init;
import java.io.Serializable;
import java.util.ArrayList;

/**
 *
 * @author Gregery Barton
 */
class btQuantizedBvh implements Serializable {

 static final int MAX_SUBTREE_SIZE_IN_BYTES = 2048;
 public static final int TRAVERSAL_STACKLESS = 0;
 public static final int TRAVERSAL_STACKLESS_CACHE_FRIENDLY = 1;
 public static final int TRAVERSAL_RECURSIVE = 2;
 private static final float QUANT_FLOAT_RANGE = 32765.0f;
 final btVector3 m_bvhAabbMin = new btVector3();
 final btVector3 m_bvhAabbMax = new btVector3();
 final btVector3 m_bvhQuantization = new btVector3();
 int m_bulletVersion;	//for serialization versioning. It could also be used to detect endianess.
 int m_curNodeIndex;
 //quantization data
 boolean m_useQuantization;
 final ArrayList<btOptimizedBvhNode> m_leafNodes = new ArrayList<>(0);
 final ArrayList<btOptimizedBvhNode> m_contiguousNodes = new ArrayList<>(0);
 final ArrayList<btQuantizedBvhNode> m_quantizedLeafNodes = new ArrayList<>(0);
 final ArrayList<btQuantizedBvhNode> m_quantizedContiguousNodes = new ArrayList<>(
  0);
 int m_traversalMode;
 final ArrayList<btBvhSubtreeInfo> m_SubtreeHeaders = new ArrayList<>(0);
 int maxIterations = 0;

 btQuantizedBvh() {
  m_bulletVersion = BT_BULLET_VERSION;
  m_useQuantization = false;
  //m_traversalMode(TRAVERSAL_STACKLESS_CACHE_FRIENDLY)
  m_traversalMode = TRAVERSAL_STACKLESS;
  //m_traversalMode(TRAVERSAL_RECURSIVE)
  m_bvhAabbMin.set(-SIMD_INFINITY, -SIMD_INFINITY, -SIMD_INFINITY);
  m_bvhAabbMax.set(SIMD_INFINITY, SIMD_INFINITY, SIMD_INFINITY);
 }

 ///two versions, one for quantized and normal nodes. This allows code-reuse while maintaining readability (no template/macro!)
 ///this might be refactored into a  const, it is usually not calculated at run-time
 void setInternalNodeAabbMin(int nodeIndex, final btVector3 aabbMin) {
  if (m_useQuantization) {
   quantize(m_quantizedContiguousNodes.get(nodeIndex).m_quantizedAabbMin,
    aabbMin, 0);
  } else {
   m_contiguousNodes.get(nodeIndex).m_aabbMinOrg.set(aabbMin);
  }
 }

 void setInternalNodeAabbMax(int nodeIndex, final btVector3 aabbMax) {
  if (m_useQuantization) {
   quantize(m_quantizedContiguousNodes.get(nodeIndex).m_quantizedAabbMax,
    aabbMax, 1);
  } else {
   m_contiguousNodes.get(nodeIndex).m_aabbMaxOrg.set(aabbMax);
  }
 }

 btVector3 getAabbMin(int nodeIndex) {
  if (m_useQuantization) {
   return unQuantize(m_quantizedLeafNodes.get(nodeIndex).m_quantizedAabbMin);
  }
  //non-quantized
  return new btVector3(m_leafNodes.get(nodeIndex).m_aabbMinOrg);
 }

 btVector3 getAabbMax(int nodeIndex) {
  if (m_useQuantization) {
   return unQuantize(m_quantizedLeafNodes.get(nodeIndex).m_quantizedAabbMax);
  }
  //non-quantized
  return new btVector3(m_leafNodes.get(nodeIndex).m_aabbMaxOrg);
 }

 void setInternalNodeEscapeIndex(int nodeIndex, int escapeIndex) {
  if (m_useQuantization) {
   m_quantizedContiguousNodes.get(nodeIndex).m_escapeIndexOrTriangleIndex = -escapeIndex;
  } else {
   m_contiguousNodes.get(nodeIndex).m_escapeIndex = escapeIndex;
  }
 }

 void mergeInternalNodeAabb(int nodeIndex, final btVector3 newAabbMin,
  final btVector3 newAabbMax) {
  if (m_useQuantization) {
   short[] quantizedAabbMin = new short[3];
   short[] quantizedAabbMax = new short[3];
   quantize(quantizedAabbMin, newAabbMin, 0);
   quantize(quantizedAabbMax, newAabbMax, 1);
   for (int i = 0; i < 3; i++) {
    if (m_quantizedContiguousNodes.get(nodeIndex).m_quantizedAabbMin[i]
     > quantizedAabbMin[i]) {
     m_quantizedContiguousNodes.get(nodeIndex).m_quantizedAabbMin[i] = quantizedAabbMin[i];
    }
    if (m_quantizedContiguousNodes.get(nodeIndex).m_quantizedAabbMax[i]
     < quantizedAabbMax[i]) {
     m_quantizedContiguousNodes.get(nodeIndex).m_quantizedAabbMax[i] = quantizedAabbMax[i];
    }
   }
  } else {
   //non-quantized
   m_contiguousNodes.get(nodeIndex).m_aabbMinOrg.setMin(newAabbMin);
   m_contiguousNodes.get(nodeIndex).m_aabbMaxOrg.setMax(newAabbMax);
  }
 }

 void swapLeafNodes(int firstIndex, int secondIndex) {
  if (m_useQuantization) {
   btQuantizedBvhNode tmp = m_quantizedLeafNodes.get(firstIndex);
   m_quantizedLeafNodes.set(firstIndex, m_quantizedLeafNodes.get(secondIndex));
   m_quantizedLeafNodes.set(secondIndex, tmp);
  } else {
   btOptimizedBvhNode tmp = m_leafNodes.get(firstIndex);
   m_leafNodes.set(firstIndex, m_leafNodes.get(secondIndex));
   m_leafNodes.set(secondIndex, tmp);
  }
 }

 void assignInternalNodeFromLeafNode(int internalNode, int leafNodeIndex) {
  if (m_useQuantization) {
   m_quantizedContiguousNodes.set(internalNode, m_quantizedLeafNodes.get(
    leafNodeIndex));
  } else {
   m_contiguousNodes.set(internalNode, m_leafNodes.get(leafNodeIndex));
  }
 }

 void buildTree(int startIndex, int endIndex) {
  int splitAxis, splitIndex, i;
  int numIndices = endIndex - startIndex;
  int curIndex = m_curNodeIndex;
  assert (numIndices > 0);
  if (numIndices == 1) {
   assignInternalNodeFromLeafNode(m_curNodeIndex, startIndex);
   m_curNodeIndex++;
   return;
  }
  //calculate Best Splitting Axis and where to split it. Sort the incoming 'leafNodes' array within range 'startIndex/endIndex'.
  splitAxis = calcSplittingAxis(startIndex, endIndex);
  splitIndex = sortAndCalcSplittingIndex(startIndex, endIndex, splitAxis);
  int internalNodeIndex = m_curNodeIndex;
  //set the min aabb to 'inf' or a max value, and set the max aabb to a -inf/minimum value.
  //the aabb will be expanded during buildTree/mergeInternalNodeAabb with actual node values
  setInternalNodeAabbMin(m_curNodeIndex, m_bvhAabbMax);//can't use btVector3(SIMD_INFINITY,SIMD_INFINITY,SIMD_INFINITY)) because of quantization
  setInternalNodeAabbMax(m_curNodeIndex, m_bvhAabbMin);//can't use btVector3(-SIMD_INFINITY,-SIMD_INFINITY,-SIMD_INFINITY)) because of quantization
  for (i = startIndex; i < endIndex; i++) {
   mergeInternalNodeAabb(m_curNodeIndex, getAabbMin(i), getAabbMax(i));
  }
  m_curNodeIndex++;
  //internalNode.m_escapeIndex;
  int leftChildNodexIndex = m_curNodeIndex;
  //build left child tree
  buildTree(startIndex, splitIndex);
  int rightChildNodexIndex = m_curNodeIndex;
  //build right child tree
  buildTree(splitIndex, endIndex);
  int escapeIndex = m_curNodeIndex - curIndex;
  if (m_useQuantization) {
   //escapeIndex is the number of nodes of this subtree
   int sizeQuantizedNode = 16;//sizeof(btQuantizedBvhNode);
   int treeSizeInBytes = escapeIndex * sizeQuantizedNode;
   if (treeSizeInBytes > MAX_SUBTREE_SIZE_IN_BYTES) {
    updateSubtreeHeaders(leftChildNodexIndex, rightChildNodexIndex);
   }
  } else {
  }
  setInternalNodeEscapeIndex(internalNodeIndex, escapeIndex);
 }

 int calcSplittingAxis(int startIndex, int endIndex) {
  int i;
  final btVector3 means = new btVector3();
  final btVector3 variance = new btVector3();
  int numIndices = endIndex - startIndex;
  final btVector3 center = new btVector3();
  for (i = startIndex; i < endIndex; i++) {
   center.set(getAabbMax(i)).add(getAabbMin(i)).scale(0.5f);
   means.add(center);
  }
  means.scale(1.f / numIndices);
  final btVector3 diff2 = new btVector3();
  for (i = startIndex; i < endIndex; i++) {
   center.set(getAabbMax(i)).add(getAabbMin(i)).scale(0.5f);
   diff2.set(center).sub(means);
   diff2.mul(diff2);
   variance.add(diff2);
  }
  variance.scale(((1.f) / ((float) numIndices - 1)));
  return variance.maxAxis();
 }

 int sortAndCalcSplittingIndex(int startIndex, int endIndex, int splitAxis) {
  int i;
  int splitIndex = startIndex;
  int numIndices = endIndex - startIndex;
  float splitValue;
  final btVector3 means = new btVector3();
  final btVector3 center = new btVector3();
  for (i = startIndex; i < endIndex; i++) {
   center.set(getAabbMax(i)).add(getAabbMin(i)).scale(0.5f);
   means.add(center);
  }
  means.scale(((1.f) / numIndices));
  splitValue = means.getElement(splitAxis);
  //sort leafNodes so all values larger then splitValue comes first, and smaller values start from 'splitIndex'.
  for (i = startIndex; i < endIndex; i++) {
   center.set(getAabbMax(i)).add(getAabbMin(i)).scale(0.5f);
   if (center.getElement(splitAxis) > splitValue) {
    //swap
    swapLeafNodes(i, splitIndex);
    splitIndex++;
   }
  }
  //if the splitIndex causes unbalanced trees, fix this by using the center in between startIndex and endIndex
  //otherwise the tree-building might fail due to stack-overflows in certain cases.
  //unbalanced1 is unsafe: it can cause stack overflows
  //boolean unbalanced1 = ((splitIndex==startIndex) || (splitIndex == (endIndex-1)));
  //unbalanced2 should work too: always use center (perfect balanced trees)	
  //boolean unbalanced2 = true;
  //this should be safe too:
  int rangeBalancedIndices = numIndices / 3;
  boolean unbalanced = ((splitIndex <= (startIndex + rangeBalancedIndices))
   || (splitIndex >= (endIndex - 1 - rangeBalancedIndices)));
  if (unbalanced) {
   splitIndex = startIndex + (numIndices >> 1);
  }
  boolean unbal = (splitIndex == startIndex) || (splitIndex == (endIndex));
  assert (!unbal);
  return splitIndex;
 }

 void walkStacklessTree(btNodeOverlapCallback nodeCallback,
  final btVector3 aabbMin,
  final btVector3 aabbMax) {
  assert (!m_useQuantization);
  btOptimizedBvhNode rootNode = m_contiguousNodes.get(0);
  int escapeIndex, curIndex = 0;
  int walkIterations = 0;
  boolean isLeafNode;
  //PCK: unsigned instead of boolean
  boolean aabbOverlap;
  while (curIndex < m_curNodeIndex) {
   //catch bugs in tree data
   assert (walkIterations < m_curNodeIndex);
   walkIterations++;
   aabbOverlap
    = TestAabbAgainstAabb2(aabbMin, aabbMax, rootNode.m_aabbMinOrg,
     rootNode.m_aabbMaxOrg);
   isLeafNode = rootNode.m_escapeIndex == -1;
   //PCK: unsigned instead of boolean
   if (isLeafNode && (aabbOverlap)) {
    nodeCallback.processNode(rootNode.m_subPart, rootNode.m_triangleIndex);
   }
   //PCK: unsigned instead of boolean
   if ((aabbOverlap) || isLeafNode) {
    curIndex++;
    rootNode = m_contiguousNodes.get(curIndex);
   } else {
    escapeIndex = rootNode.m_escapeIndex;
    curIndex += escapeIndex;
    rootNode = m_contiguousNodes.get(curIndex);
   }
  }
  if (maxIterations < walkIterations) {
   maxIterations = walkIterations;
  }
 }

 void walkStacklessQuantizedTreeAgainstRay(btNodeOverlapCallback nodeCallback,
  final btVector3 raySource, final btVector3 rayTarget, final btVector3 aabbMin,
  final btVector3 aabbMax, int startNodeIndex, int endNodeIndex) {
  assert (m_useQuantization);
  int curIndex = startNodeIndex;
  int walkIterations = 0;
  int subTreeSize = endNodeIndex - startNodeIndex;
  btQuantizedBvhNode rootNode = m_quantizedContiguousNodes.get(startNodeIndex);
  int escapeIndex;
  boolean isLeafNode;
  //PCK: unsigned instead of boolean
  int boxBoxOverlap;
  int rayBoxOverlap = 0;
  float lambda_max;
  final btVector3 rayDirection = new btVector3(rayTarget).sub(raySource);
  rayDirection.normalize();
  lambda_max = rayDirection.dot(new btVector3(rayTarget).sub(raySource));
  ///what about division by zero? -. just set rayDirection[i] to 1.0
  rayDirection.x = rayDirection.x == (0.0f) ? (BT_LARGE_FLOAT) : (1.0f)
   / rayDirection.x;
  rayDirection.y = rayDirection.y == (0.0f) ? (BT_LARGE_FLOAT) : (1.0f)
   / rayDirection.y;
  rayDirection.z = rayDirection.z == (0.0f) ? (BT_LARGE_FLOAT) : (1.0f)
   / rayDirection.z;
  int[] sign = new int[]{rayDirection.x < 0.0f ? 1 : 0,
   rayDirection.y < 0.0f ? 1 : 0,
   rayDirection.z < 0.0f ? 1 : 0};
  /*
   * Quick pruning by quantized box
   */
  final btVector3 rayAabbMin = new btVector3(raySource);
  final btVector3 rayAabbMax = new btVector3(raySource);
  rayAabbMin.setMin(rayTarget);
  rayAabbMax.setMax(rayTarget);

  /*
   * Add box cast extents to bounding box
   */
  rayAabbMin.add(aabbMin);
  rayAabbMax.add(aabbMax);
  short[] quantizedQueryAabbMin = new short[3];
  short[] quantizedQueryAabbMax = new short[3];
  quantizeWithClamp(quantizedQueryAabbMin, rayAabbMin, 0);
  quantizeWithClamp(quantizedQueryAabbMax, rayAabbMax, 1);
  float[] param = new float[1];
  while (curIndex < endNodeIndex) {
   //catch bugs in tree data
   assert (walkIterations < subTreeSize);
   walkIterations++;
   //PCK: unsigned instead of boolean
   // only interested if this is closer than any previous hit
   param[0] = 1.0f;
   rayBoxOverlap = 0;
   boxBoxOverlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,
    quantizedQueryAabbMax, rootNode.m_quantizedAabbMin,
    rootNode.m_quantizedAabbMax);
   isLeafNode = rootNode.isLeafNode();
   if (boxBoxOverlap != 0) {
    btVector3[] bounds = new btVector3[2];
    bounds[0] = (unQuantize(rootNode.m_quantizedAabbMin));
    bounds[1] = (unQuantize(rootNode.m_quantizedAabbMax));
    /*
     * Add box cast extents
     */
    bounds[0].sub(aabbMax);
    bounds[1].sub(aabbMin);
    final btVector3 normal;
    ///careful with this check: need to check division by zero (above) and fix the unQuantize method
    ///thanks Joerg/hiker for the reproduction case!
    ///http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=9&t=1858
    //BT_PROFILE("btRayAabb2");
    rayBoxOverlap = btRayAabb2(raySource, rayDirection, sign, bounds, param,
     0.0f, lambda_max) ? 1
      : 0;
   }
   if (isLeafNode && (rayBoxOverlap != 0)) {
    nodeCallback.processNode(rootNode.getPartId(), rootNode.getTriangleIndex());
   }
   //PCK: unsigned instead of boolean
   if ((rayBoxOverlap != 0) || isLeafNode) {
    curIndex++;
    rootNode = m_quantizedContiguousNodes.get(curIndex);
   } else {
    escapeIndex = rootNode.getEscapeIndex();
    curIndex += escapeIndex;
    rootNode = m_quantizedContiguousNodes.get(curIndex);
   }
  }
  if (maxIterations < walkIterations) {
   maxIterations = walkIterations;
  }
 }

 void walkStacklessQuantizedTree(btNodeOverlapCallback nodeCallback,
  short[] quantizedQueryAabbMin,
  short[] quantizedQueryAabbMax, int startNodeIndex, int endNodeIndex) {
  assert (m_useQuantization);
  int curIndex = startNodeIndex;
  int walkIterations = 0;
  int subTreeSize = endNodeIndex - startNodeIndex;
  btQuantizedBvhNode rootNode = m_quantizedContiguousNodes.get(startNodeIndex);
  int escapeIndex;
  boolean isLeafNode;
  //PCK: unsigned instead of boolean
  int aabbOverlap;
  while (curIndex < endNodeIndex) {
   //catch bugs in tree data
   assert (walkIterations < subTreeSize);
   walkIterations++;
   //PCK: unsigned instead of boolean
   aabbOverlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,
    quantizedQueryAabbMax,
    rootNode.m_quantizedAabbMin, rootNode.m_quantizedAabbMax);
   isLeafNode = rootNode.isLeafNode();
   if (isLeafNode && (aabbOverlap != 0)) {
    nodeCallback.processNode(rootNode.getPartId(), rootNode.getTriangleIndex());
   }
   //PCK: unsigned instead of boolean
   if ((aabbOverlap != 0) || isLeafNode) {
    curIndex++;
    rootNode = m_quantizedContiguousNodes.get(curIndex);
   } else {
    escapeIndex = rootNode.getEscapeIndex();
    curIndex += escapeIndex;
    rootNode = m_quantizedContiguousNodes.get(curIndex);
   }
  }
  if (maxIterations < walkIterations) {
   maxIterations = walkIterations;
  }
 }

 void walkStacklessTreeAgainstRay(btNodeOverlapCallback nodeCallback,
  final btVector3 raySource,
  final btVector3 rayTarget, final btVector3 aabbMin, final btVector3 aabbMax,
  int startNodeIndex,
  int endNodeIndex) {
  assert (!m_useQuantization);
  btOptimizedBvhNode rootNode = m_contiguousNodes.get(0);
  int escapeIndex, curIndex = 0;
  int walkIterations = 0;
  boolean isLeafNode;
  //PCK: unsigned instead of boolean
  boolean aabbOverlap;
  boolean rayBoxOverlap;
  float lambda_max;

  /*
   * Quick pruning by quantized box
   */
  final btVector3 rayAabbMin = new btVector3(raySource);
  final btVector3 rayAabbMax = new btVector3(raySource);
  rayAabbMin.setMin(rayTarget);
  rayAabbMax.setMax(rayTarget);

  /*
   * Add box cast extents to bounding box
   */
  rayAabbMin.add(aabbMin);
  rayAabbMax.add(aabbMax);
  final btVector3 rayDir = new btVector3(rayTarget).sub(raySource);
  rayDir.normalize();
  lambda_max = rayDir.dot(new btVector3(rayTarget).sub(raySource));
  ///what about division by zero? -. just set rayDirection[i] to 1.0
  final btVector3 rayDirectionInverse = new btVector3();
  rayDirectionInverse.x = rayDir.x == (0.0f) ? (BT_LARGE_FLOAT) : (1.0f)
   / rayDir.x;
  rayDirectionInverse.y = rayDir.y == (0.0f) ? (BT_LARGE_FLOAT) : (1.0f)
   / rayDir.y;
  rayDirectionInverse.z = rayDir.z == (0.0f) ? (BT_LARGE_FLOAT) : (1.0f)
   / rayDir.z;
  int[] sign = new int[]{rayDirectionInverse.x < 0.0f ? 1 : 0,
   rayDirectionInverse.y < 0.0f ? 1 : 0,
   rayDirectionInverse.z < 0.0f ? 1 : 0};
  btVector3[] bounds = new btVector3[2];
  init(bounds);
  float[] param = new float[1];
  while (curIndex < m_curNodeIndex) {
   param[0] = 1.0f;
   //catch bugs in tree data
   assert (walkIterations < m_curNodeIndex);
   walkIterations++;
   bounds[0].set(rootNode.m_aabbMinOrg);
   bounds[1].set(rootNode.m_aabbMaxOrg);
   /*
    * Add box cast extents
    */
   bounds[0].sub(aabbMax);
   bounds[1].sub(aabbMin);
   aabbOverlap = TestAabbAgainstAabb2(rayAabbMin, rayAabbMax,
    rootNode.m_aabbMinOrg,
    rootNode.m_aabbMaxOrg);
   //perhaps profile if it is worth doing the aabbOverlap test first
   ///careful with this check: need to check division by zero (above) and fix the unQuantize method
   ///thanks Joerg/hiker for the reproduction case!
   ///http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=9&t=1858
   rayBoxOverlap = (aabbOverlap) ? btRayAabb2(raySource, rayDirectionInverse,
    sign, bounds, param,
    0.0f, lambda_max) : false;
   isLeafNode = rootNode.m_escapeIndex == -1;
   //PCK: unsigned instead of boolean
   if (isLeafNode && rayBoxOverlap) {
    nodeCallback.processNode(rootNode.m_subPart, rootNode.m_triangleIndex);
   }
   //PCK: unsigned instead of boolean
   if (rayBoxOverlap || isLeafNode) {
    curIndex++;
    rootNode = m_contiguousNodes.get(curIndex);
   } else {
    escapeIndex = rootNode.m_escapeIndex;
    curIndex += escapeIndex;
    rootNode = m_contiguousNodes.get(curIndex);
   }
  }
  if (maxIterations < walkIterations) {
   maxIterations = walkIterations;
  }
 }

 ///tree traversal designed for small-memory processors like PS3 SPU
 void walkStacklessQuantizedTreeCacheFriendly(btNodeOverlapCallback nodeCallback,
  short[] quantizedQueryAabbMin, short[] quantizedQueryAabbMax) {
  assert (m_useQuantization);
  int i;
  for (i = 0; i < m_SubtreeHeaders.size(); i++) {
   btBvhSubtreeInfo subtree = m_SubtreeHeaders.get(i);
   //PCK: unsigned instead of boolean
   boolean overlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,
    quantizedQueryAabbMax, subtree.m_quantizedAabbMin,
    subtree.m_quantizedAabbMax) != 0;
   if (overlap) {
    walkStacklessQuantizedTree(nodeCallback, quantizedQueryAabbMin,
     quantizedQueryAabbMax,
     subtree.m_rootNodeIndex,
     subtree.m_rootNodeIndex + subtree.m_subtreeSize);
   }
  }
 }

 ///use the 16-byte stackless 'skipindex' node tree to do a recursive traversal
 void walkRecursiveQuantizedTreeAgainstQueryAabb(int curIndex,
  btNodeOverlapCallback nodeCallback,
  short[] quantizedQueryAabbMin, short[] quantizedQueryAabbMax) {
  assert (m_useQuantization);
  btQuantizedBvhNode currentNode = m_quantizedContiguousNodes.get(curIndex);
  boolean isLeafNode;
  //PCK: unsigned instead of boolean
  boolean aabbOverlap;
  //PCK: unsigned instead of boolean
  aabbOverlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,
   quantizedQueryAabbMax,
   currentNode.m_quantizedAabbMin, currentNode.m_quantizedAabbMax) != 0;
  isLeafNode = currentNode.isLeafNode();
  //PCK: unsigned instead of boolean
  if (aabbOverlap) {
   if (isLeafNode) {
    nodeCallback.processNode(currentNode.getPartId(), currentNode
     .getTriangleIndex());
   } else {
    //process left and right children
    int leftIndex = curIndex + 1;
    btQuantizedBvhNode leftChildNode = m_quantizedContiguousNodes.get(leftIndex);
    walkRecursiveQuantizedTreeAgainstQueryAabb(leftIndex, nodeCallback,
     quantizedQueryAabbMin,
     quantizedQueryAabbMax);
    int rightIndex = leftChildNode.isLeafNode() ? leftIndex + 1 : leftIndex
     + leftChildNode
      .getEscapeIndex();
//			  btQuantizedBvhNode  rightChildNode = leftChildNode.isLeafNode() ? leftChildNode+1:leftChildNode+leftChildNode.getEscapeIndex();
    walkRecursiveQuantizedTreeAgainstQueryAabb(rightIndex, nodeCallback,
     quantizedQueryAabbMin,
     quantizedQueryAabbMax);
   }
  }
 }

 ///use the 16-byte stackless 'skipindex' node tree to do a recursive traversal
 void walkRecursiveQuantizedTreeAgainstQuantizedTree(
  btQuantizedBvhNode treeNodeA,
  btQuantizedBvhNode treeNodeB, btNodeOverlapCallback nodeCallback) {
  // can't find definition source code
  throw new AssertionError();
 }

 void updateSubtreeHeaders(int leftChildNodexIndex, int rightChildNodexIndex) {
  assert (m_useQuantization);
  btQuantizedBvhNode leftChildNode = m_quantizedContiguousNodes.get(
   leftChildNodexIndex);
  int leftSubTreeSize = leftChildNode.isLeafNode() ? 1 : leftChildNode
   .getEscapeIndex();
  int leftSubTreeSizeInBytes = leftSubTreeSize * 16;//static_cast<int>(sizeof(btQuantizedBvhNode));
  btQuantizedBvhNode rightChildNode = m_quantizedContiguousNodes.get(
   rightChildNodexIndex);
  int rightSubTreeSize = rightChildNode.isLeafNode() ? 1 : rightChildNode
   .getEscapeIndex();
  int rightSubTreeSizeInBytes = rightSubTreeSize * 16;//  static_cast<int>(sizeof(btQuantizedBvhNode));
  if (leftSubTreeSizeInBytes <= MAX_SUBTREE_SIZE_IN_BYTES) {
   btBvhSubtreeInfo subtree = new btBvhSubtreeInfo();
   m_SubtreeHeaders.add(subtree);
   subtree.setAabbFromQuantizeNode(leftChildNode);
   subtree.m_rootNodeIndex = leftChildNodexIndex;
   subtree.m_subtreeSize = leftSubTreeSize;
  }
  if (rightSubTreeSizeInBytes <= MAX_SUBTREE_SIZE_IN_BYTES) {
   btBvhSubtreeInfo subtree = new btBvhSubtreeInfo();
   m_SubtreeHeaders.add(subtree);
   subtree.setAabbFromQuantizeNode(rightChildNode);
   subtree.m_rootNodeIndex = rightChildNodexIndex;
   subtree.m_subtreeSize = rightSubTreeSize;
  }
 }

 ///***************************************** expert/internal use only ************************* 
 void setQuantizationValues(final btVector3 bvhAabbMin,
  final btVector3 bvhAabbMax) {
  setQuantizationValues(bvhAabbMin, bvhAabbMax, 1.0f);
 }

 void setQuantizationValues(final btVector3 bvhAabbMin,
  final btVector3 bvhAabbMax,
  float quantizationMargin) {
  //enlarge the AABB to avoid division by zero when initializing the quantization values
  final btVector3 clampValue = new btVector3(quantizationMargin,
   quantizationMargin,
   quantizationMargin);
  m_bvhAabbMin.set(new btVector3(bvhAabbMin).sub(clampValue));
  m_bvhAabbMax.set(new btVector3(bvhAabbMax).add(clampValue));
  final btVector3 aabbSize = new btVector3(m_bvhAabbMax).sub(m_bvhAabbMin);
  m_bvhQuantization.set(new btVector3((QUANT_FLOAT_RANGE), (QUANT_FLOAT_RANGE),
   (QUANT_FLOAT_RANGE))
   .div(aabbSize));
  m_useQuantization = true;
  {
   short[] vecIn = new short[3];
   final btVector3 v;
   {
    quantize(vecIn, m_bvhAabbMin, 0);
    v = unQuantize(vecIn);
    m_bvhAabbMin.setMin(new btVector3(v).sub(clampValue));
   }
   aabbSize.set(m_bvhAabbMax).sub(m_bvhAabbMin);
   m_bvhQuantization.set(
    new btVector3((QUANT_FLOAT_RANGE), (QUANT_FLOAT_RANGE), (QUANT_FLOAT_RANGE))
     .div(aabbSize));
   {
    quantize(vecIn, m_bvhAabbMax, 1);
    v.set(unQuantize(vecIn));
    m_bvhAabbMax.setMax(new btVector3(v).add(clampValue));
   }
   aabbSize.set(m_bvhAabbMax).sub(m_bvhAabbMin);
   m_bvhQuantization.set(
    new btVector3((QUANT_FLOAT_RANGE), (QUANT_FLOAT_RANGE), (QUANT_FLOAT_RANGE)))
    .div(aabbSize);
  }
 }

 ArrayList<btQuantizedBvhNode> getLeafNodeArray() {
  return m_quantizedLeafNodes;
 }

 ///buildInternal is expert use only: assumes that setQuantizationValues and LeafNodeArray are initialized
 void buildInternal() {
  ///assumes that caller filled in the m_quantizedLeafNodes
  m_useQuantization = true;
  int numLeafNodes = 0;
  if (m_useQuantization) {
   //now we have an array of leafnodes in m_leafNodes
   numLeafNodes = m_quantizedLeafNodes.size();
   int new_size = numLeafNodes * 2;
   m_quantizedContiguousNodes.clear();
   m_quantizedContiguousNodes.ensureCapacity(new_size);
   while (m_quantizedContiguousNodes.size() < new_size) {
    m_quantizedContiguousNodes.add(new btQuantizedBvhNode());
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
   subtree.m_subtreeSize = m_quantizedContiguousNodes.get(0).isLeafNode() ? 1
    : m_quantizedContiguousNodes.get(0).getEscapeIndex();
  }
  //PCK: clear m_quantizedLeafNodes and m_leafNodes, they are temporary
  m_quantizedLeafNodes.clear();
  m_quantizedLeafNodes.trimToSize();
  m_leafNodes.clear();
  m_leafNodes.trimToSize();
 }

 ///***************************************** expert/internal use only *************************
 public void reportAabbOverlappingNodex(btNodeOverlapCallback nodeCallback,
  final btVector3 aabbMin,
  final btVector3 aabbMax) {
  //either choose recursive traversal (walkTree) or stackless (walkStacklessTree)
  if (m_useQuantization) {
   ///quantize query AABB
   short[] quantizedQueryAabbMin = new short[3];
   short[] quantizedQueryAabbMax = new short[3];
   quantizeWithClamp(quantizedQueryAabbMin, aabbMin, 0);
   quantizeWithClamp(quantizedQueryAabbMax, aabbMax, 1);
   switch (m_traversalMode) {
    case TRAVERSAL_STACKLESS:
     walkStacklessQuantizedTree(nodeCallback, quantizedQueryAabbMin,
      quantizedQueryAabbMax, 0,
      m_curNodeIndex);
     break;
    case TRAVERSAL_STACKLESS_CACHE_FRIENDLY:
     walkStacklessQuantizedTreeCacheFriendly(nodeCallback, quantizedQueryAabbMin,
      quantizedQueryAabbMax);
     break;
    case TRAVERSAL_RECURSIVE: {
     walkRecursiveQuantizedTreeAgainstQueryAabb(0, nodeCallback,
      quantizedQueryAabbMin,
      quantizedQueryAabbMax);
    }
    break;
    default:
     //unsupported
     assert (false);
   }
  } else {
   walkStacklessTree(nodeCallback, aabbMin, aabbMax);
  }
 }

 public void reportRayOverlappingNodex(btNodeOverlapCallback nodeCallback,
  final btVector3 raySource,
  final btVector3 rayTarget) {
  reportBoxCastOverlappingNodex(nodeCallback, raySource, rayTarget,
   new btVector3(), new btVector3());
 }

 public void reportBoxCastOverlappingNodex(btNodeOverlapCallback nodeCallback,
  final btVector3 raySource,
  final btVector3 rayTarget, final btVector3 aabbMin, final btVector3 aabbMax) {
  //always use stackless
  if (m_useQuantization) {
   walkStacklessQuantizedTreeAgainstRay(nodeCallback, raySource, rayTarget,
    aabbMin, aabbMax, 0,
    m_curNodeIndex);
  } else {
   walkStacklessTreeAgainstRay(nodeCallback, raySource, rayTarget, aabbMin,
    aabbMax, 0,
    m_curNodeIndex);
  }
 }

 public void quantize(short[] out, final btVector3 point, int isMax) {
  assert (m_useQuantization);
  assert (point.getX() <= m_bvhAabbMax.getX());
  assert (point.getY() <= m_bvhAabbMax.getY());
  assert (point.getZ() <= m_bvhAabbMax.getZ());
  assert (point.getX() >= m_bvhAabbMin.getX());
  assert (point.getY() >= m_bvhAabbMin.getY());
  assert (point.getZ() >= m_bvhAabbMin.getZ());
  final btVector3 v = new btVector3(point).sub(m_bvhAabbMin).mul(
   m_bvhQuantization);
  ///Make sure rounding is done in a way that unQuantize(quantizeWithClamp(...)) is conservative
  ///end-points always set the first bit, so that they are sorted properly (so that neighbouring AABBs overlap properly)
  ///@todo: double-check this
  if (isMax != 0) {
   out[0] = (short) (((short) (v.getX() + (1.f)) | 1));
   out[1] = (short) (((short) (v.getY() + (1.f)) | 1));
   out[2] = (short) (((short) (v.getZ() + (1.f)) | 1));
  } else {
   out[0] = (short) (((short) (v.getX()) & 0xfffe));
   out[1] = (short) (((short) (v.getY()) & 0xfffe));
   out[2] = (short) (((short) (v.getZ()) & 0xfffe));
  }
  assert (out[0] >= 0);
  assert (out[1] >= 0);
  assert (out[2] >= 0);
 }

 public void quantizeWithClamp(short[] out, final btVector3 point2, int isMax) {
  assert (m_useQuantization);
  final btVector3 clampedPoint = new btVector3(point2);
  clampedPoint.setMax(m_bvhAabbMin);
  clampedPoint.setMin(m_bvhAabbMax);
  quantize(out, clampedPoint, isMax);
 }

 public btVector3 unQuantize(short[] vecIn) {
  final btVector3 vecOut = new btVector3();
  vecOut.set(
   (vecIn[0]) / (m_bvhQuantization.getX()),
   (vecIn[1]) / (m_bvhQuantization.getY()),
   (vecIn[2]) / (m_bvhQuantization.getZ()));
  vecOut.add(m_bvhAabbMin);
  return vecOut;
 }

 ///setTraversalMode let's you choose between stackless, recursive or stackless cache friendly tree traversal. Note this is only implemented for quantized trees.
 public void setTraversalMode(int traversalMode) {
  m_traversalMode = traversalMode;
 }

 ArrayList<btQuantizedBvhNode> getQuantizedNodeArray() {
  return m_quantizedContiguousNodes;
 }

 ArrayList<btBvhSubtreeInfo> getSubtreeInfoArray() {
  return m_SubtreeHeaders;
 }

 boolean isQuantized() {
  return m_useQuantization;
 }

}
