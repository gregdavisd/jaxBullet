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

import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class CollisionObjectTypes  implements Serializable {

 public static final int CO_COLLISION_OBJECT = 1;
 public static final int CO_RIGID_BODY = 2;
 ///CO_GHOST_OBJECT keeps track of all objects overlapping its AABB and that pass its collision filter
 ///It is useful for collision sensors, explosion objects, character controller etc.
 public static final int CO_GHOST_OBJECT = 4;
 public static final int CO_SOFT_BODY = 8;
 public static final int CO_HF_FLUID = 16;
 public static final int CO_USER_TYPE = 32;
 public static final int CO_FEATHERSTONE_LINK = 64;
}
