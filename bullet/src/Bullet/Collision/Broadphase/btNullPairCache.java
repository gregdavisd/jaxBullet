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
///btNullPairCache skips add/removal of overlapping pairs. Userful for benchmarking and unit testing.

import java.io.Serializable;
import java.util.ArrayList;

/**
 *
 * @author Gregery Barton
 */
public class btNullPairCache implements btOverlappingPairCache , Serializable {

 final ArrayList<btBroadphasePair> m_overlappingPairArray = new ArrayList<>(0);

 @Override
 public ArrayList<btBroadphasePair> getOverlappingPairArrayPtr() {
  return m_overlappingPairArray;
 }

 @Override
 public ArrayList<btBroadphasePair> getOverlappingPairArray() {
  return m_overlappingPairArray;
 }

 @Override
 public void cleanOverlappingPair(btBroadphasePair pair, btDispatcher dispatcher) {
 }

 @Override
 public int getNumOverlappingPairs() {
  return 0;
 }

 @Override
 public void cleanProxyFromPairs(btBroadphaseProxy proxy, btDispatcher dispatcher) {
 }

 @Override
 public void setOverlapFilterCallback(btOverlapFilterCallback callback) {
 }

 @Override
 public void processAllOverlappingPairs(btOverlapCallback callback, btDispatcher dispatcher) {
 }

 @Override
 public btBroadphasePair findPair(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1) {
  return null;
 }

 @Override
 public boolean hasDeferredRemoval() {
  return true;
 }

 @Override
 public void setInternalGhostPairCallback(btOverlappingPairCallback ghostPairCallback) {
 }

 /**
  *
  * @param proxy0
  * @param proxy1
  * @return
  */
 @Override
 public btBroadphasePair
  addOverlappingPair(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1) {
  return null;
 }

 /**
  *
  * @param proxy0
  * @param proxy1
  * @param dispatcher
  * @return
  */
 @Override
 public Object removeOverlappingPair(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1,
  btDispatcher dispatcher) {
  return 0;
 }

 /**
  *
  * @param proxy0
  * @param dispatcher
  */
 @Override
 public void removeOverlappingPairsContainingProxy(btBroadphaseProxy proxy0, btDispatcher dispatcher) {
 }

 @Override
 public void sortOverlappingPairs(btDispatcher dispatcher) {
  //(void) dispatcher;
 }

 @Override
 public void incrementalCleanup(int ni, btDispatcher dispatcher) {
 }
}
