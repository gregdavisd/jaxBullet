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
package Bullet.Collision.Algorithm;

import Bullet.Collision.Broadphase.btBroadphasePair;
import Bullet.Collision.Broadphase.btDispatcherInfo;
import Bullet.Collision.Broadphase.btOverlapCallback;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btCollisionPairCallback implements btOverlapCallback, Serializable {

 final btDispatcherInfo m_dispatchInfo;
 final btCollisionDispatcher m_dispatcher;

 btCollisionPairCallback(btDispatcherInfo dispatchInfo, btCollisionDispatcher dispatcher) {
  m_dispatchInfo = dispatchInfo;
  m_dispatcher = dispatcher;
 }

 @Override
 public boolean processOverlap(btBroadphasePair pair) {
  (m_dispatcher.getNearCallback()).callback(pair, m_dispatcher, m_dispatchInfo);
  return false;
 }
}
