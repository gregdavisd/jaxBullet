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

import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public interface ContactAddedCallback extends Serializable {

 /**
  *
  * @param cp
  * @param colObj0Wrap
  * @param partId0
  * @param index0
  * @param colObj1Wrap
  * @param partId1
  * @param index1
  * @return
  */
 boolean callback(btManifoldPoint cp, btCollisionObjectWrapper colObj0Wrap, int partId0, int index0,
  btCollisionObjectWrapper colObj1Wrap, int partId1, int index1);
}
