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

import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class DefaultColors  implements Serializable {

 final btVector3 m_activeObject;
 final btVector3 m_deactivatedObject;
 final btVector3 m_wantsDeactivationObject;
 final btVector3 m_disabledDeactivationObject;
 final btVector3 m_disabledSimulationObject;
 final btVector3 m_aabb;
 final btVector3 m_contactPoint;

 DefaultColors() {
  m_activeObject = new btVector3(1, 1, 1);
  m_deactivatedObject = new btVector3(0, 1, 0);
  m_wantsDeactivationObject = new btVector3(0, 1, 1);
  m_disabledDeactivationObject = new btVector3(1, 0, 0);
  m_disabledSimulationObject = new btVector3(1, 1, 0);
  m_aabb = new btVector3(1, 0, 0);
  m_contactPoint = new btVector3(1, 1, 0);
 }
}
