/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
///btDbvt implementation by Nathanael Presson

package Bullet.Collision.Broadphase;
///btDbvt implementation by Nathanael Presson

import static Bullet.Extras.btMinMax.btMax;
import static Bullet.LinearMath.btAabbUtil2.btRayAabb2;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import static Bullet.LinearMath.btScalar.SIMD_INFINITY;
import static Bullet.LinearMath.btScalar.btFabs;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.btDot;
import static Bullet.LinearMath.btVector3.init;
import static Bullet.common.btAlignedObjectArray.findLinearSearch;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collections;
import static java.util.Collections.swap;
import org.apache.commons.collections.primitives.ArrayIntList;

/**
 *
 * @author Gregery Barton
 */
public class btDbvt  implements Serializable  {

 /* Stack element	*/
 public static class sStkNN  implements Serializable{

  public btDbvtNode a;
  public btDbvtNode b;

  public sStkNN() {
  }

  public sStkNN(btDbvtNode na, btDbvtNode nb) {
   a = na;
   b = nb;
  }
 };

 public static class sStkNP  implements Serializable{

  final btDbvtNode node;
  int mask;

  sStkNP(btDbvtNode n, int m) {
   node = n;
   mask = m;
  }
 };

 public static class sStkNPS  implements Serializable{

  btDbvtNode node;
  int mask;
  float value;

  sStkNPS() {
  }

  sStkNPS(btDbvtNode n, int m, float v) {
   node = n;
   mask = m;
   value = v;
  }
 };

 public static class sStkCLN  implements Serializable{

  final btDbvtNode node;
  final btDbvtNode parent;

  sStkCLN(btDbvtNode n, btDbvtNode p) {
   node = n;
   parent = p;
  }
 };
 // Policies/Interfaces

 /* ICollide	*/
 public static class ICollide  implements Serializable{

  public void process(btDbvtNode a, btDbvtNode b) {
  }

  public void process(btDbvtNode n) {
  }

  public void process(btDbvtNode n, float f) {
   ICollide.this.process(n);
  }

  public boolean descent(btDbvtNode n) {
   return (true);
  }

  public boolean allLeaves(btDbvtNode n) {
   return (true);
  }
 }

 /* IWriter	*/
 public static abstract class IWriter  implements Serializable{

  abstract void prepare(btDbvtNode root, int numnodes);

  abstract void writeNode(btDbvtNode n, int index, int parent, int child0, int child1);

  abstract void writeLeaf(btDbvtNode n, int index, int parent);
 }

 /* IClone	*/
 public static class IClone  implements Serializable{

  void cloneLeaf(btDbvtNode n) {
  }
 };
 // Constants
 static final int SIMPLE_STACKSIZE = 64;
 static final int DOUBLE_STACKSIZE = SIMPLE_STACKSIZE * 2;
 // Fields
 public btDbvtNode m_root;
 public int m_lkhd;
 public int m_leaves;
 public int m_opath;
 final ArrayList<sStkNN> m_stkStack = new ArrayList<>(0);

 // Methods
  public btDbvt() {
  {
   m_root = null;
   m_lkhd = -1;
   m_leaves = 0;
   m_opath = 0;
  }
 }

  public void clear() {
  if (m_root != null) {
   recursedeletenode(this, m_root);
  }
  m_lkhd = -1;
  m_stkStack.clear();
  m_opath = 0;
 }

  public boolean empty() {
  return (null == m_root);
 }

  public void optimizeBottomUp() {
  if (m_root != null) {
   ArrayList<btDbvtNode> leaves = new ArrayList<>(m_leaves);
   fetchleaves(this, m_root, leaves);
   bottomup(this, leaves);
   m_root = leaves.get(0);
  }
 }

  public void optimizeTopDown(int bu_treshold) {
  if (m_root != null) {
   ArrayList<btDbvtNode> leaves = new ArrayList<>(m_leaves);
   fetchleaves(this, m_root, leaves);
   m_root = topdown(this, leaves, bu_treshold);
  }
 }

  public void optimizeTopDown() {
  optimizeTopDown(128);
 }

 public  void optimizeIncremental(int passes) {
  return;
//  int do_passes = passes;
//  if (do_passes < 0) {
//   do_passes = m_leaves;
//  }
//  if (m_root != null && (do_passes > 0)) {
//   do {
//    btDbvtNode node = m_root;
//    int bit = 0;
//    while (node.isinternal()) {
//     btDbvtNode[] ref = new btDbvtNode[]{m_root};
//     node = sort(node, ref).childs[(m_opath >> bit) & 1];
//     m_root = ref[0];
//     bit = (bit + 1) & (4 * 8 - 1);
//    }
//    update(node);
//    ++m_opath;
//   } while (--do_passes > 0);
//  }
 }

  public btDbvtNode insert(btDbvtAabbMm volume, int dataAsInt, Object data) {
  btDbvtNode leaf = createnode(this, null, volume, dataAsInt, data);
  insertleaf(this, m_root, leaf);
  ++m_leaves;
  return (leaf);
 }

  public void update(btDbvtNode leaf, int lookahead) {
  btDbvtNode root = removeleaf(this, leaf);
  if (root != null) {
   if (lookahead >= 0) {
    for (int i = 0; (i < lookahead) && root.parent != null; ++i) {
     root = root.parent;
    }
   } else {
    root = m_root;
   }
  }
  insertleaf(this, root, leaf);
 }

  public void update(btDbvtNode leaf) {
  update(leaf, -1);
 }

  public void update(btDbvtNode leaf, btDbvtAabbMm volume) {
  btDbvtNode root = removeleaf(this, leaf);
  if (root != null) {
   if (m_lkhd >= 0) {
    for (int i = 0; (i < m_lkhd) && root.parent != null; ++i) {
     root = root.parent;
    }
   } else {
    root = m_root;
   }
  }
  leaf.volume.set(volume);
  insertleaf(this, root, leaf);
 }

  public boolean update(btDbvtNode leaf, btDbvtAabbMm volume, final btVector3 velocity, float margin) {
  if (leaf.volume.contain(volume)) {
   return (false);
  }
  volume.expand(new btVector3(margin, margin, margin));
  volume.signedExpand(velocity);
  update(leaf, volume);
  return (true);
 }

  public boolean update(btDbvtNode leaf, btDbvtAabbMm volume, final btVector3 velocity) {
  if (leaf.volume.contain(volume)) {
   return (false);
  }
  volume.signedExpand(velocity);
  update(leaf, volume);
  return (true);
 }

  public boolean update(btDbvtNode leaf, btDbvtAabbMm volume, float margin) {
  if (leaf.volume.contain(volume)) {
   return (false);
  }
  volume.expand(new btVector3(margin, margin, margin));
  update(leaf, volume);
  return (true);
 }

  public void remove(btDbvtNode leaf) {
  removeleaf(this, leaf);
  deletenode(this, leaf);
  --m_leaves;
 }

 void write(IWriter iwriter) {
  btDbvtNodeEnumerator nodes = new btDbvtNodeEnumerator();
  nodes.nodes.ensureCapacity(m_leaves * 2);
  enumNodes(m_root, nodes);
  iwriter.prepare(m_root, nodes.nodes.size());
  for (int i = 0; i < nodes.nodes.size(); ++i) {
   btDbvtNode n = nodes.nodes.get(i);
   int p = -1;
   if (n.parent != null) {
    p = findLinearSearch(nodes.nodes, n.parent);
   }
   if (n.isinternal()) {
    int c0 = findLinearSearch(nodes.nodes, n.childs[0]);
    int c1 = findLinearSearch(nodes.nodes, n.childs[1]);
    iwriter.writeNode(n, i, p, c0, c1);
   } else {
    iwriter.writeLeaf(n, i, p);
   }
  }
 }

 void clone(btDbvt dest, IClone iclone) {
  dest.clear();
  if (m_root != null) {
   final ArrayList<sStkCLN> stack = new ArrayList<>(m_leaves);
   stack.add(new sStkCLN(m_root, null));
   do {
    int i = stack.size() - 1;
    sStkCLN e = stack.get(i);
    btDbvtNode n = createnode(dest, e.parent, e.node.volume, e.node.dataAsInt, e.node.data);
    System.arraycopy(e.node.childs, 0, n.childs, 0, n.childs.length);
    stack.remove(stack.size() - 1);
    if (e.parent != null) {
     e.parent.childs[i & 1] = n;
    } else {
     dest.m_root = n;
    }
    if (e.node.isinternal()) {
     stack.add(new sStkCLN(e.node.childs[0], n));
     stack.add(new sStkCLN(e.node.childs[1], n));
    } else {
     iclone.cloneLeaf(n);
    }
   } while (stack.size() > 0);
  }
 }

 void clone(btDbvt dest) {
  clone(dest, null);
 }

 static int maxdepth(btDbvtNode node) {
  int depth[] = new int[]{0};
  if (node != null) {
   getmaxdepth(node, 1, depth);
  }
  return (depth[0]);
 }

 static int countLeaves(btDbvtNode node) {
  if (node.isinternal()) {
   return (countLeaves(node.childs[0]) + countLeaves(node.childs[1]));
  } else {
   return (1);
  }
 }

 static void extractLeaves(btDbvtNode node, ArrayList<btDbvtNode> leaves) {
  if (node.isinternal()) {
   extractLeaves(node.childs[0], leaves);
   extractLeaves(node.childs[1], leaves);
  } else {
   leaves.add(node);
  }
 }

 static void benchmark() {
 }

 // DBVT_IPOLICY must support ICollide policy/interface
 static void enumNodes(btDbvtNode root,
  ICollide policy) {
  policy.process(root);
  if (root.isinternal()) {
   enumNodes(root.childs[0], policy);
   enumNodes(root.childs[1], policy);
  }
 }

 static void enumLeaves(btDbvtNode root,
  ICollide policy) {
  if (root.isinternal()) {
   enumLeaves(root.childs[0], policy);
   enumLeaves(root.childs[1], policy);
  } else {
   policy.process(root);
  }
 }

 void collideTT(btDbvtNode root0,
  btDbvtNode root1,
  ICollide policy) {
  if (root0 != null && root1 != null) {
   int depth = 1;
   int treshold = DOUBLE_STACKSIZE - 4;
   final ArrayList<sStkNN> stkStack = new ArrayList<>(0);
   stkStack.addAll(Collections.nCopies(DOUBLE_STACKSIZE, (sStkNN) null));
   stkStack.set(0, new sStkNN(root0, root1));
   do {
    sStkNN p = stkStack.get(--depth);
    if (depth > treshold) {
     stkStack.addAll(Collections.nCopies(stkStack.size(), (sStkNN) null));
     treshold = stkStack.size() - 4;
    }
    if (p.a == p.b) {
     if (p.a.isinternal()) {
      stkStack.set(depth++, new sStkNN(p.a.childs[0], p.a.childs[0]));
      stkStack.set(depth++, new sStkNN(p.a.childs[1], p.a.childs[1]));
      stkStack.set(depth++, new sStkNN(p.a.childs[0], p.a.childs[1]));
     }
    } else if (Intersect(p.a.volume, p.b.volume)) {
     if (p.a.isinternal()) {
      if (p.b.isinternal()) {
       stkStack.set(depth++, new sStkNN(p.a.childs[0], p.b.childs[0]));
       stkStack.set(depth++, new sStkNN(p.a.childs[1], p.b.childs[0]));
       stkStack.set(depth++, new sStkNN(p.a.childs[0], p.b.childs[1]));
       stkStack.set(depth++, new sStkNN(p.a.childs[1], p.b.childs[1]));
      } else {
       stkStack.set(depth++, new sStkNN(p.a.childs[0], p.b));
       stkStack.set(depth++, new sStkNN(p.a.childs[1], p.b));
      }
     } else if (p.b.isinternal()) {
      stkStack.set(depth++, new sStkNN(p.a, p.b.childs[0]));
      stkStack.set(depth++, new sStkNN(p.a, p.b.childs[1]));
     } else {
      policy.process(p.a, p.b);
     }
    }
   } while (depth > 0);
  }
 }

   void collideTTpersistentStack(btDbvtNode root0,
  btDbvtNode root1,
  ICollide policy) {
  if (root0 != null && root1 != null) {
   int depth = 1;
   while (m_stkStack.size() < DOUBLE_STACKSIZE) {
    m_stkStack.add(null);
   }
   int treshold = m_stkStack.size() - 4;
   m_stkStack.set(0, new sStkNN(root0, root1));
   do {
    sStkNN p = m_stkStack.get(--depth);
    if (depth > treshold) {
     m_stkStack.addAll(Collections.nCopies(m_stkStack.size(), (sStkNN) null));
     treshold = m_stkStack.size() - 4;
    }
    if (p.a == p.b) {
     if (p.a.isinternal()) {
      m_stkStack.set(depth++, new sStkNN(p.a.childs[0], p.a.childs[0]));
      m_stkStack.set(depth++, new sStkNN(p.a.childs[1], p.a.childs[1]));
      m_stkStack.set(depth++, new sStkNN(p.a.childs[0], p.a.childs[1]));
     }
    } else if (Intersect(p.a.volume, p.b.volume)) {
     if (p.a.isinternal()) {
      if (p.b.isinternal()) {
       m_stkStack.set(depth++, new sStkNN(p.a.childs[0], p.b.childs[0]));
       m_stkStack.set(depth++, new sStkNN(p.a.childs[1], p.b.childs[0]));
       m_stkStack.set(depth++, new sStkNN(p.a.childs[0], p.b.childs[1]));
       m_stkStack.set(depth++, new sStkNN(p.a.childs[1], p.b.childs[1]));
      } else {
       m_stkStack.set(depth++, new sStkNN(p.a.childs[0], p.b));
       m_stkStack.set(depth++, new sStkNN(p.a.childs[1], p.b));
      }
     } else if (p.b.isinternal()) {
      m_stkStack.set(depth++, new sStkNN(p.a, p.b.childs[0]));
      m_stkStack.set(depth++, new sStkNN(p.a, p.b.childs[1]));
     } else {
      policy.process(p.a, p.b);
     }
    }
   } while (depth > 0);
  }
 }

 public void collideTV(btDbvtNode root,
  btDbvtAabbMm volume,
  ICollide policy) {
  if (root != null) {
   ArrayList<btDbvtNode> stack = new ArrayList<>(SIMPLE_STACKSIZE);
   stack.add(root);
   do {
    btDbvtNode n = stack.get(stack.size() - 1);
    stack.remove(stack.size() - 1);
    if (Intersect(n.volume, volume)) {
     if (n.isinternal()) {
      stack.add(n.childs[0]);
      stack.add(n.childs[1]);
     } else {
      policy.process(n);
     }
    }
   } while (stack.size() > 0);
  }
 }

 public void collideTVNoStackAlloc(btDbvtNode root,
  btDbvtAabbMm volume,
  ArrayList<btDbvtNode> stack,
  ICollide policy) {
  if (root != null) {
   stack.ensureCapacity(SIMPLE_STACKSIZE);
   stack.clear();
   stack.add(root);
   do {
    btDbvtNode n = stack.get(stack.size() - 1);
    stack.remove(stack.size() - 1);
    if (Intersect(n.volume, volume)) {
     if (n.isinternal()) {
      stack.add(n.childs[0]);
      stack.add(n.childs[1]);
     } else {
      policy.process(n);
     }
    }
   } while (stack.size() > 0);
  }
 }

 ///rayTest is a re-entrant ray test, and can be called in parallel as long as the btAlignedAlloc is thread-safe (uses locking etc)
 ///rayTest is slower than rayTestInternal, because it builds a local stack, using memory allocations, and it recomputes signs/rayDirectionInverses each time
public  static void rayTest(btDbvtNode root, final btVector3 rayFrom, final btVector3 rayTo,
  ICollide policy) {
  if (root != null) {
   final btVector3 rayDir = new btVector3(rayTo).sub(rayFrom);
   rayDir.normalize();
   ///what about division by zero? -. just set rayDirection[i] to INF/BT_LARGE_FLOAT
   final btVector3 rayDirectionInverse = new btVector3();
   rayDirectionInverse.x = rayDir.x == (0.0f) ? (BT_LARGE_FLOAT) : (1.0f) / rayDir.x;
   rayDirectionInverse.y = rayDir.y == (0.0f) ? (BT_LARGE_FLOAT) : (1.0f) / rayDir.y;
   rayDirectionInverse.z = rayDir.z == (0.0f) ? (BT_LARGE_FLOAT) : (1.0f) / rayDir.z;
   int[] signs = new int[]{rayDirectionInverse.x < 0.0 ? 1 : 0, rayDirectionInverse.y < 0.0 ? 1 : 0,
    rayDirectionInverse.z < 0.0 ? 1 : 0};
   float lambda_max = rayDir.dot(new btVector3(rayTo).sub(rayFrom));
   //final btVector3 resultNormal = new btVector3();
   final ArrayList<  btDbvtNode> stack = new ArrayList<>(0);
   int depth = 1;
   int treshold = DOUBLE_STACKSIZE - 2;
   stack.addAll(Collections.nCopies(DOUBLE_STACKSIZE, (btDbvtNode) null));
   stack.set(0, root);
   final btVector3[] bounds = new btVector3[2];
   init(bounds);
   do {
    btDbvtNode node = stack.get(--depth);
    bounds[0].set(node.volume.mins());
    bounds[1].set( node.volume.maxs());
    float lambda_min = 0.f;
    float[] tmin = new float[]{1.f};
    boolean result1 = btRayAabb2(rayFrom, rayDirectionInverse, signs, bounds, tmin, lambda_min,
     lambda_max);
    if (result1) {
     if (node.isinternal()) {
      if (depth > treshold) {
       stack.addAll(Collections.nCopies(stack.size(), (btDbvtNode) null));
       treshold = stack.size() - 2;
      }
      stack.set(depth++, node.childs[0]);
      stack.set(depth++, node.childs[1]);
     } else {
      policy.process(node);
     }
    }
   } while (depth > 0);
  }
 }

 ///rayTestInternal is faster than rayTest, because it uses a persistent stack (to reduce dynamic memory allocations to a minimum) and it uses precomputed signs/rayInverseDirections
 ///rayTestInternal is used by btDbvtBroadphase to accelerate world ray casts
 void rayTestInternal(btDbvtNode root, final btVector3 rayFrom, final btVector3 rayTo,
  final btVector3 rayDirectionInverse,
  int[] signs,
  float lambda_max, final btVector3 aabbMin, final btVector3 aabbMax,
  ArrayList<  btDbvtNode> stack,
  ICollide policy) {
  if (root != null) {
   //btVector3 resultNormal;
   int depth = 1;
   while (stack.size() < DOUBLE_STACKSIZE) {
    stack.add((btDbvtNode) null);
   }
   int treshold = stack.size() - 2;
   stack.set(0, root);
   btVector3[] bounds = new btVector3[2];
   init(bounds);
   do {
    btDbvtNode node = stack.get(--depth);
    bounds[0].set(node.volume.mins()).sub(aabbMax);
    bounds[1].set(node.volume.maxs()).sub(aabbMin);
    float[] tmin = new float[]{1.f};
    float lambda_min = 0.f;
    boolean result1 = btRayAabb2(rayFrom, rayDirectionInverse, signs, bounds, tmin, lambda_min,
     lambda_max);
    if (result1) {
     if (node.isinternal()) {
      if (depth > treshold) {
       stack.addAll(Collections.nCopies(stack.size(), (btDbvtNode) null));
       treshold = stack.size() - 2;
      }
      stack.set(depth++, node.childs[0]);
      stack.set(depth++, node.childs[1]);
     } else {
      policy.process(node);
     }
    }
   } while (depth > 0);
  }
 }

 static void collideKDOP(btDbvtNode root,
  btVector3[] normals,
  float[] offsets,
  int count,
  ICollide policy) {
  if (root != null) {
   int inside = (1 << count) - 1;
   ArrayList<sStkNP> stack = new ArrayList<>(0);
   int[] signs = new int[4 * 8];
   assert(count < signs.length);
   for (int i = 0; i < count; ++i) {
    signs[i] = ((normals[i].x() >= 0) ? 1 : 0) +
     ((normals[i].y() >= 0) ? 2 : 0) +
     ((normals[i].z() >= 0) ? 4 : 0);
   }
   stack.ensureCapacity(SIMPLE_STACKSIZE);
   stack.add(new sStkNP(root, 0));
   do {
    sStkNP se = stack.get(stack.size() - 1);
    boolean out = false;
    stack.remove(stack.size() - 1);
    for (int i = 0, j = 1; (!out) && (i < count); ++i, j <<= 1) {
     if (0 == (se.mask & j)) {
      int side = se.node.volume.classify(normals[i], offsets[i], signs[i]);
      switch (side) {
       case -1:
        out = true;
        break;
       case +1:
        se.mask |= j;
        break;
      }
     }
    }
    if (!out) {
     if ((se.mask != inside) && (se.node.isinternal())) {
      stack.add(new sStkNP(se.node.childs[0], se.mask));
      stack.add(new sStkNP(se.node.childs[1], se.mask));
     } else if (policy.allLeaves(se.node)) {
      enumLeaves(se.node, policy);
     }
    }
   } while (stack.size() > 0);
  }
 }

 static void collideOCL(btDbvtNode root,
  btVector3[] normals,
  float[] offsets, final btVector3 sortaxis,
  int count,
  ICollide policy,
  boolean fullsort) {
  if (root != null) {
   int srtsgns = (sortaxis.x >= 0 ? 1 : 0) +
    (sortaxis.y >= 0 ? 2 : 0) +
    (sortaxis.z >= 0 ? 4 : 0);
   int inside = (1 << count) - 1;
   final ArrayList<sStkNPS> stock = new ArrayList<>(0);
   final ArrayIntList ifree = new ArrayIntList();
   final ArrayIntList stack = new ArrayIntList();
   int[] signs = new int[4 * 8];
   assert(count < signs.length);
   for (int i = 0; i < count; ++i) {
    signs[i] = ((normals[i].x() >= 0) ? 1 : 0) +
     ((normals[i].y() >= 0) ? 2 : 0) +
     ((normals[i].z() >= 0) ? 4 : 0);
   }
   stock.ensureCapacity(SIMPLE_STACKSIZE);
   stack.ensureCapacity(SIMPLE_STACKSIZE);
   ifree.ensureCapacity(SIMPLE_STACKSIZE);
   stack.add(allocate(ifree, stock, new sStkNPS(root, 0, root.volume.projectMinimum(sortaxis,
    srtsgns))));
   do {
    int id = stack.get(stack.size() - 1);
    sStkNPS se = stock.get(id);
    stack.removeElement(stack.size() - 1);
    ifree.add(id);
    if (se.mask != inside) {
     boolean out = false;
     for (int i = 0, j = 1; (!out) && (i < count); ++i, j <<= 1) {
      if (0 == (se.mask & j)) {
       int side = se.node.volume.classify(normals[i], offsets[i], signs[i]);
       switch (side) {
        case -1:
         out = true;
         break;
        case +1:
         se.mask |= j;
         break;
       }
      }
     }
     if (out) {
      continue;
     }
    }
    if (policy.descent(se.node)) {
     if (se.node.isinternal()) {
      btDbvtNode[] pns = new btDbvtNode[]{se.node.childs[0], se.node.childs[1]};
      sStkNPS[] nes = new sStkNPS[]{new sStkNPS(pns[0], se.mask, pns[0].volume.projectMinimum(
       sortaxis, srtsgns)),
       new sStkNPS(pns[1], se.mask, pns[1].volume.projectMinimum(sortaxis, srtsgns))};
      int q = nes[0].value < nes[1].value ? 1 : 0;
      int j = stack.size();
      if (fullsort && (j > 0)) {
       /* Insert 0	*/
       j = nearest(stack.toBackedArray(), stock, nes[q].value, 0, stack.size());
       stack.add(0);
       //void * memmove ( void * destination,   void * source, size_t num );
       for (int k = stack.size() - 1; k > j; --k) {
        stack.set(k, stack.get(k - 1));
       }
       stack.set(j, allocate(ifree, stock, nes[q]));
       /* Insert 1	*/
       j = nearest(stack.toBackedArray(), stock, nes[1 - q].value, j, stack.size());
       stack.add(0);
       for (int k = stack.size() - 1; k > j; --k) {
        stack.set(k, stack.get(k - 1));
       }
       stack.set(j, allocate(ifree, stock, nes[1 - q]));
      } else {
       stack.add(allocate(ifree, stock, nes[q]));
       stack.add(allocate(ifree, stock, nes[1 - q]));
      }
     } else {
      policy.process(se.node, se.value);
     }
    }
   } while (stack.size() > 0);
  }
 }

 static void collideOCL(btDbvtNode root,
  btVector3[] normals,
  float[] offsets, final btVector3 sortaxis,
  int count,
  ICollide policy) {
  collideOCL(root, normals, offsets, sortaxis, count, policy, true);
 }

 static void collideTU(btDbvtNode root,
  ICollide policy) {
  if (root != null) {
   final ArrayList<  btDbvtNode> stack = new ArrayList<>(SIMPLE_STACKSIZE);
   stack.add(root);
   do {
    btDbvtNode n = stack.get(stack.size() - 1);
    stack.remove(stack.size() - 1);
    if (policy.descent(n)) {
     if (n.isinternal()) {
      stack.add(n.childs[0]);
      stack.add(n.childs[1]);
     } else {
      policy.process(n);
     }
    }
   } while (stack.size() > 0);
  }
 }

 // Helpers	
 static int nearest(int[] i, ArrayList<sStkNPS> a, float v, int l, int h) {
  int m;
  int ll = l;
  int hh = h;
  while (ll < hh) {
   m = (ll + hh) >>> 1;
   if (a.get(i[m]).value >= v) {
    ll = m + 1;
   } else {
    hh = m;
   }
  }
  return (hh);
 }

 static int allocate(ArrayIntList ifree,
  ArrayList<sStkNPS> stock,
  sStkNPS value) {
  int i;
  if (ifree.size() > 0) {
   i = ifree.get(ifree.size() - 1);
   ifree.removeElement(ifree.size() - 1);
   stock.set(i, value);
  } else {
   i = stock.size();
   stock.add(value);
  }
  return (i);
 }
//

 static void recursedeletenode(btDbvt pdbvt,
  btDbvtNode node) {
  if (!node.isleaf()) {
   recursedeletenode(pdbvt, node.childs[0]);
   recursedeletenode(pdbvt, node.childs[1]);
  }
  if (node == pdbvt.m_root) {
   pdbvt.m_root = null;
  }
  deletenode(pdbvt, node);
 }
//

 static void deletenode(btDbvt pdbvt,
  btDbvtNode node) {
 }

 static void fetchleaves(btDbvt pdbvt,
  btDbvtNode root,
  ArrayList<btDbvtNode> leaves) {
  fetchleaves(pdbvt, root, leaves, -1);
 }
// 

 static void fetchleaves(btDbvt pdbvt,
  btDbvtNode root,
  ArrayList<btDbvtNode> leaves,
  int depth) {
  if (root.isinternal() && depth > 0) {
   fetchleaves(pdbvt, root.childs[0], leaves, depth - 1);
   fetchleaves(pdbvt, root.childs[1], leaves, depth - 1);
   deletenode(pdbvt, root);
  } else {
   leaves.add(root);
  }
 }
//

 static void bottomup(btDbvt pdbvt,
  ArrayList<btDbvtNode> leaves) {
  while (leaves.size() > 1) {
   float minsize = SIMD_INFINITY;
   int[] minidx = new int[]{-1, -1};
   for (int i = 0; i < leaves.size(); ++i) {
    for (int j = i + 1; j < leaves.size(); ++j) {
     float sz = size(merge(leaves.get(i).volume, leaves.get(j).volume));
     if (sz < minsize) {
      minsize = sz;
      minidx[0] = i;
      minidx[1] = j;
     }
    }
   }
   btDbvtNode[] n = new btDbvtNode[]{leaves.get(minidx[0]), leaves.get(minidx[1])};
   btDbvtNode p = createnode(pdbvt, null, n[0].volume, n[1].volume, 0, null);
   p.childs[0] = n[0];
   p.childs[1] = n[1];
   n[0].parent = p;
   n[1].parent = p;
   leaves.set(minidx[0], p);
   swap(leaves, minidx[1], leaves.size() - 1);
   leaves.remove(leaves.size() - 1);
  }
 }
 static btVector3[] axis = new btVector3[]{new btVector3(1, 0, 0),
  new btVector3(0, 1, 0),
  new btVector3(0, 0, 1)};
//

 static btDbvtNode topdown(btDbvt pdbvt,
  ArrayList<btDbvtNode> leaves,
  int bu_treshold) {
  if (leaves.size() > 1) {
   if (leaves.size() > bu_treshold) {
    btDbvtAabbMm vol = bounds(leaves);
    final btVector3 org = vol.center();
    ArrayList<btDbvtNode>[] sets = new ArrayList[]{new ArrayList<>(0), new ArrayList<>(0)};
    int bestaxis = -1;
    int bestmidp = leaves.size();
    int[][] splitcount = new int[][]{{0, 0}, {0, 0}, {0, 0}};
    //int i;
    for (int i = 0; i < leaves.size(); ++i) {
     final btVector3 x = new btVector3(leaves.get(i).volume.center()).sub(org);
     for (int j = 0; j < 3; ++j) {
      ++splitcount[j][btDot(x, axis[j]) > 0 ? 1 : 0];
     }
    }
    for (int i = 0; i < 3; ++i) {
     if ((splitcount[i][0] > 0) && (splitcount[i][1] > 0)) {
      int midp = (int) btFabs((float) (splitcount[i][0] - splitcount[i][1]));
      if (midp < bestmidp) {
       bestaxis = i;
       bestmidp = midp;
      }
     }
    }
    if (bestaxis >= 0) {
     sets[0].ensureCapacity(splitcount[bestaxis][0]);
     sets[1].ensureCapacity(splitcount[bestaxis][1]);
     split(leaves, sets[0], sets[1], org, axis[bestaxis]);
    } else {
     sets[0].ensureCapacity(leaves.size() / 2 + 1);
     sets[1].ensureCapacity(leaves.size() / 2);
     for (int i = 0, ni = leaves.size(); i < ni; ++i) {
      sets[i & 1].add(leaves.get(i));
     }
    }
    btDbvtNode node = createnode(pdbvt, null, vol, 0, null);
    node.childs[0] = topdown(pdbvt, sets[0], bu_treshold);
    node.childs[1] = topdown(pdbvt, sets[1], bu_treshold);
    node.childs[0].parent = node;
    node.childs[1].parent = node;
    return (node);
   } else {
    bottomup(pdbvt, leaves);
    return (leaves.get(0));
   }
  }
  return (leaves.get(0));
 }

//
 static btDbvtNode sort(btDbvtNode n, btDbvtNode[] r) {
 // btDbvtNode p = n.parent;
  //assert(n.isinternal());
  // C++ code compared pointers if (p>n), maybe it optimizes by moving data close together, not really possible in java.
//  if (p!=null && p  > n ) {
//   int i = indexof(n);
//   int j = 1 - i;
//   btDbvtNode s = p.childs[j];
//   btDbvtNode q = p.parent;
//   assert(n == p.childs[i]);
//   if (q != null) {
//    q.childs[indexof(p)] = n;
//   } else {
//    r[0] = n;
//   }
//   s.parent = n;
//   p.parent = n;
//   n.parent = q;
//   p.childs[0] = n.childs[0];
//   p.childs[1] = n.childs[1];
//   n.childs[0].parent = p;
//   n.childs[1].parent = p;
//   n.childs[i] = p;
//   n.childs[j] = s;
//   btDbvtAabbMm swapper = p.volume;
//   p.volume = n.volume;
//   n.volume = swapper;
//   return (p);
//  }
  return (n);
 }

 static btDbvtNode createnode(btDbvt pdbvt,
  btDbvtNode parent,
  int dataAsInt,
  Object data) {
  btDbvtNode node;
  node = new btDbvtNode();
  node.parent = parent;
  node.dataAsInt = dataAsInt;
  node.data = data;
  node.childs[1] = null;
  return (node);
 }
//

 static btDbvtNode createnode(btDbvt pdbvt,
  btDbvtNode parent,
  btDbvtAabbMm volume,
  int dataAsInt,
  Object data
 ) {
  btDbvtNode node = createnode(pdbvt, parent, dataAsInt, data);
  node.volume.set(volume);
  return (node);
 }

//
 static btDbvtNode createnode(btDbvt pdbvt,
  btDbvtNode parent,
  btDbvtAabbMm volume0,
  btDbvtAabbMm volume1,
  int dataAsInt,
  Object data) {
  btDbvtNode node = createnode(pdbvt, parent, dataAsInt, data);
  Merge(volume0, volume1, node.volume);
  return (node);
 }

//
 static void insertleaf(btDbvt pdbvt,
  btDbvtNode root,
  btDbvtNode leaf) {
  btDbvtNode do_root = root;
  if (pdbvt.m_root == null) {
   pdbvt.m_root = leaf;
   leaf.parent = null;
  } else {
   if (!do_root.isleaf()) {
    do {
     do_root = do_root.childs[Select(leaf.volume,
      do_root.childs[0].volume,
      do_root.childs[1].volume)];
    } while (!do_root.isleaf());
   }
   btDbvtNode prev = do_root.parent;
   btDbvtNode node = createnode(pdbvt, prev, leaf.volume, do_root.volume, 0, null);
   if (prev != null) {
    prev.childs[indexof(do_root)] = node;
    node.childs[0] = do_root;
    do_root.parent = node;
    node.childs[1] = leaf;
    leaf.parent = node;
    do {
     if (!prev.volume.contain(node.volume)) {
      Merge(prev.childs[0].volume, prev.childs[1].volume, prev.volume);
     } else {
      break;
     }
     node = prev;
    } while (null != (prev = node.parent));
   } else {
    node.childs[0] = do_root;
    do_root.parent = node;
    node.childs[1] = leaf;
    leaf.parent = node;
    pdbvt.m_root = node;
   }
  }
 }
//

 static btDbvtNode removeleaf(btDbvt pdbvt,
  btDbvtNode leaf) {
  if (leaf == pdbvt.m_root) {
   pdbvt.m_root = null;
   return (null);
  } else {
   btDbvtNode parent = leaf.parent;
   btDbvtNode prev = parent.parent;
   btDbvtNode sibling = parent.childs[1 - indexof(leaf)];
   if (prev != null) {
    prev.childs[indexof(parent)] = sibling;
    sibling.parent = prev;
    deletenode(pdbvt, parent);
    while (prev != null) {
     btDbvtAabbMm pb = prev.volume;
     Merge(prev.childs[0].volume, prev.childs[1].volume, prev.volume);
     if (NotEqual(pb, prev.volume)) {
      prev = prev.parent;
     } else {
      break;
     }
    }
    return (prev != null ? prev : pdbvt.m_root);
   } else {
    pdbvt.m_root = sibling;
    sibling.parent = null;
    deletenode(pdbvt, parent);
    return (pdbvt.m_root);
   }
  }
 }
//

 static void getmaxdepth(btDbvtNode node, int depth, int[] maxdepth) {
  if (node.isinternal()) {
   getmaxdepth(node.childs[0], depth + 1, maxdepth);
   getmaxdepth(node.childs[1], depth + 1, maxdepth);
  } else {
   maxdepth[0] = btMax(maxdepth[0], depth);
  }
 }

//
 static boolean Intersect(btDbvtAabbMm a,
  btDbvtAabbMm b) {
  return ((a.mi.x() <= b.mx.x()) &&
   (a.mx.x() >= b.mi.x()) &&
   (a.mi.y() <= b.mx.y()) &&
   (a.mx.y() >= b.mi.y()) &&
   (a.mi.z() <= b.mx.z()) &&
   (a.mx.z() >= b.mi.z()));
 }
//

 static boolean Intersect(btDbvtAabbMm a, final btVector3 b) {
  return ((b.x() >= a.mi.x()) &&
   (b.y() >= a.mi.y()) &&
   (b.z() >= a.mi.z()) &&
   (b.x() <= a.mx.x()) &&
   (b.y() <= a.mx.y()) &&
   (b.z() <= a.mx.z()));
 }

//
 static void Merge(btDbvtAabbMm a,
  btDbvtAabbMm b,
  btDbvtAabbMm r) {
  if (a.mi.x < b.mi.x) {
   r.mi.x = a.mi.x;
  } else {
   r.mi.x = b.mi.x;
  }
  if (a.mx.x > b.mx.x) {
   r.mx.x = a.mx.x;
  } else {
   r.mx.x = b.mx.x;
  }
  if (a.mi.y < b.mi.y) {
   r.mi.y = a.mi.y;
  } else {
   r.mi.y = b.mi.y;
  }
  if (a.mx.y > b.mx.y) {
   r.mx.y = a.mx.y;
  } else {
   r.mx.y = b.mx.y;
  }
  if (a.mi.z < b.mi.z) {
   r.mi.z = a.mi.z;
  } else {
   r.mi.z = b.mi.z;
  }
  if (a.mx.z > b.mx.z) {
   r.mx.z = a.mx.z;
  } else {
   r.mx.z = b.mx.z;
  }
 }

 static btDbvtAabbMm merge(btDbvtAabbMm a,
  btDbvtAabbMm b) {
  btDbvtAabbMm res = new btDbvtAabbMm();
  Merge(a, b, res);
  return (res);
 }

// volume+edge lengths
 static float size(btDbvtAabbMm a) {
  final btVector3 edges = a.lengths();
  return (edges.x() * edges.y() * edges.z() +
   edges.x() + edges.y() + edges.z());
 }
//

 static btDbvtAabbMm bounds(ArrayList<btDbvtNode> leaves) {
  btDbvtAabbMm volume = leaves.get(0).volume;
  for (int i = 1, ni = leaves.size(); i < ni; ++i) {
   Merge(volume, leaves.get(i).volume, volume);
  }
  return (volume);
 }
//

 static void split(ArrayList<btDbvtNode> leaves,
  ArrayList<btDbvtNode> left,
  ArrayList<btDbvtNode> right, final btVector3 org, final btVector3 axis) {
  left.clear();
  right.clear();
  for (int i = 0, ni = leaves.size(); i < ni; ++i) {
   if (btDot(axis, leaves.get(i).volume.center().sub(org)) < 0) {
    left.add(leaves.get(i));
   } else {
    right.add(leaves.get(i));
   }
  }
 }
//

 static int indexof(btDbvtNode node) {
  return (node.parent.childs[1] == node ? 1 : 0);
 }
//

 static int Select(btDbvtAabbMm o,
  btDbvtAabbMm a,
  btDbvtAabbMm b) {
  return (Proximity(o, a) < Proximity(o, b) ? 0 : 1);
 }
//

 static float Proximity(btDbvtAabbMm a,
  btDbvtAabbMm b) {
  final btVector3 d = new btVector3(a.mi).add(a.mx).sub(b.mi).sub(b.mx);
  return (btFabs(d.x()) + btFabs(d.y()) + btFabs(d.z()));
 }
//

 static boolean NotEqual(btDbvtAabbMm a,
  btDbvtAabbMm b) {
  return ((a.mi.x() != b.mi.x()) ||
   (a.mi.y() != b.mi.y()) ||
   (a.mi.z() != b.mi.z()) ||
   (a.mx.x() != b.mx.x()) ||
   (a.mx.y() != b.mx.y()) ||
   (a.mx.z() != b.mx.z()));
 }
}
