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

package Bullet.Dynamics;

import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btRigidBodyFlags implements Serializable  {

 public static final int BT_DISABLE_WORLD_GRAVITY = 1;
 ///BT_ENABLE_GYROPSCOPIC_FORCE flags is enabled by default in Bullet 2.83 and onwards.
 ///and it BT_ENABLE_GYROPSCOPIC_FORCE becomes equivalent to BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY
 ///See Demos/GyroscopicDemo and computeGyroscopicImpulseImplicit
 public static final int BT_ENABLE_GYROSCOPIC_FORCE_EXPLICIT = 2;
 public static final int BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD = 4;
 public static final int BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY = 8;
 public static final int BT_ENABLE_GYROPSCOPIC_FORCE = BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY;
}
