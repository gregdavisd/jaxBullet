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
import java.util.ArrayList;

/**
 * UnionFind calculates connected subsets Implements weighted Quick Union with path compression
 * optimization: could use short ints instead of ints (halving memory, would limit the number of
 * rigid bodies to 64k, sounds reasonable)
 *
 * @author Gregery Barton
 */
public class btUnionFind  implements Serializable {

 final ArrayList<btElement> m_elements = new ArrayList<>(0);

 public btUnionFind() {
 }

 //this is a special operation, destroying the content of btUnionFind.
 //it sorts the elements, based on island id, in order to make it easy to iterate over islands
 public void sortIslands() {
  //first store the original body index, and islandId
  int numElements = m_elements.size();
  for (int i = 0; i < numElements; i++) {
   m_elements.get(i).m_id = find(i);
  }
  // Sort the vector using predicate and std.sort
  m_elements.sort(null);
 }

 public void reset(int N) {
  allocate(N);
  for (int i = 0; i < N; i++) {
   m_elements.get(i).m_id = i;
   m_elements.get(i).m_sz = 1;
  }
 }

 public int getNumElements() {
  return m_elements.size();
 }

 public boolean isRoot(int x) {
  return (x == m_elements.get(x).m_id);
 }

 public btElement getElement(int index) {
  return m_elements.get(index);
 }

 void allocate(int N) {
  while (m_elements.size() < N) {
   m_elements.add(new btElement());
  }
  while (m_elements.size() > N) {
   m_elements.remove(m_elements.size() - 1);
  }
 }

 public void Free() {
  m_elements.clear();
 }

 public boolean find(int p, int q) {
  return (find(p) == find(q));
 }

 public void unite(int p, int q) {
  int i = find(p), j = find(q);
  if (i == j) {
   return;
  }
  m_elements.get(i).m_id = j;
  m_elements.get(j).m_sz += m_elements.get(i).m_sz;
 }

 public int find(int x) {
  int do_x = x;
  //assert(x < m_N);
  //assert(x >= 0);
  while (do_x != m_elements.get(do_x).m_id) {
   //not really a reason not to use path compression, and it flattens the trees/improves find performance dramatically
   btElement elementPtr = m_elements.get(m_elements.get(do_x).m_id);
   m_elements.get(do_x).m_id = elementPtr.m_id;
   do_x = elementPtr.m_id;
   //assert(x < m_N);
   //assert(x >= 0);
  }
  return do_x;
 }
}
