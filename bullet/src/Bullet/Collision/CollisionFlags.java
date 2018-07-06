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

import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class CollisionFlags implements Serializable {

 public static final int CF_STATIC_OBJECT = 1;
 public static final int CF_KINEMATIC_OBJECT = 2;
 public static final int CF_NO_CONTACT_RESPONSE = 4;
 public static final int CF_CUSTOM_MATERIAL_CALLBACK = 8;//this allows per-triangle material (friction/restitution)
 public static final int CF_CHARACTER_OBJECT = 16;
 public static final int CF_DISABLE_VISUALIZE_OBJECT = 32; //disable debug drawing
 public static final int CF_DISABLE_SPU_COLLISION_PROCESSING = 64;//disable parallel/SPU processing
 public static final int CF_HAS_CONTACT_STIFFNESS_DAMPING = 128;
 public static final int CF_HAS_CUSTOM_DEBUG_RENDERING_COLOR = 256;
}
