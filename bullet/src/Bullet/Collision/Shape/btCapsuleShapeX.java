/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.Collision.Shape;

import java.io.Serializable;

/**
 * btCapsuleShapeX represents a capsule around the Z axis the total height is height+2*radius, so
 * the height is just the height between the center of each 'sphere' of the capsule caps.
 *
 * @author Gregery Barton
 */
public class btCapsuleShapeX extends btCapsuleShape implements Serializable {

 public btCapsuleShapeX(float radius, float height) {
  m_collisionMargin = radius;
  m_upAxis = 0;
  m_implicitShapeDimensions.set(0.5f * height, radius, radius);
 }

 //debugging
 @Override
 public String getName() {
  return "CapsuleX";
 }
}
