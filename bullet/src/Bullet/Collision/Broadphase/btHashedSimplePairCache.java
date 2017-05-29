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
package Bullet.Collision.Broadphase;

import Bullet.Collision.btSimplePair;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import javax.vecmath.Tuple2i;

/**
 *
 * @author Gregery Barton
 */
public class btHashedSimplePairCache implements Serializable {

 static int gOverlappingSimplePairs = 0;
 static int gRemoveSimplePairs = 0;
 static int gAddedSimplePairs = 0;
 static int gFindSimplePairs = 0;
 final ArrayList<btSimplePair> m_simplePairArray = new ArrayList<>(0);
 final HashMap<SimplePairKey, btSimplePair> m_simplePairMap = new HashMap<>();

 public btHashedSimplePairCache() {
 }

 public void removeAllPairs() {
  m_simplePairArray.clear();
  m_simplePairMap.clear();
 }

 public Object removeOverlappingPair(int indexA, int indexB) {
  gRemoveSimplePairs++;
  SimplePairKey key = new SimplePairKey(indexA, indexB);
  btSimplePair pair = m_simplePairMap.remove(key);
  if (pair == null) {
   return null;
  }
  Object userData = pair.m_userPointer != null ? pair.m_userPointer : pair.m_userValue;
  assert (pair.m_indexA == indexA);
  assert (pair.m_indexB == indexB);
  int find_pair = m_simplePairArray.indexOf(pair);
  assert (find_pair >= 0);
  m_simplePairArray.set(find_pair, m_simplePairArray.get(m_simplePairArray.size() - 1));
  m_simplePairArray.remove(m_simplePairArray.size() - 1);
  return userData;
 }

 // Add a pair and return the new pair. If the pair already exists,
 // no new pair is created and the old one is returned.
 public btSimplePair addOverlappingPair(int indexA, int indexB) {
  gAddedSimplePairs++;
  SimplePairKey key = new SimplePairKey(indexA, indexB);
  btSimplePair pair = new btSimplePair(indexA, indexB);
  m_simplePairMap.put(key, pair);
  m_simplePairArray.add(pair);
  return pair;
 }

 Collection<btSimplePair> getOverlappingPairArrayPtr() {
  return m_simplePairArray;
 }

 public Collection<btSimplePair> getOverlappingPairArray() {
  return m_simplePairArray;
 }

 public btSimplePair findPair(int indexA, int indexB) {
  gFindSimplePairs++;
  btSimplePair pair = m_simplePairMap.get(new SimplePairKey(indexA, indexB));
  return pair;
 }

 int getCount() {
  return m_simplePairArray.size();
 }

 int getNumOverlappingPairs() {
  return m_simplePairArray.size();
 }

 static class SimplePairKey extends Tuple2i {

  static final long serialVersionUID = 1;

  SimplePairKey(int x, int y) {
   super(x, y);
  }

  @Override
  public int hashCode() {
   /**
    * Thomas Wang's hash, see: http://www.concentric.net/~Ttwang/tech/inthash.htm This assumes
    * proxyId1 and proxyId2 are 16-bit.
    */
   int key = ((x) | ((y) << 16));
   // Thomas Wang's hash
   key += ~(key << 15);
   key ^= (key >>> 10);
   key += (key << 3);
   key ^= (key >>> 6);
   key += ~(key << 11);
   key ^= (key >>> 16);
   return (int) Math.abs(key);
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
   final SimplePairKey other = (SimplePairKey) obj;
   return super.equals(other);
  }
 }
};
