/*
 * Copyright (c) 2017  Gregery Barton
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
package bullet_examples;

import static org.lwjgl.opengl.GL11.GL_TRIANGLES;

/**
 *
 * @author Gregery Barton
 */
public class TriangleElement extends GLDrawElements {

 public TriangleElement(int[] buffer, int[] indices) {
  super(buffer, indices);
 }

 public TriangleElement(float[] buffer, int[] indices) {
  super(buffer, indices);
 }

 @Override
 protected int get_mode() {
  return GL_TRIANGLES;
 }

}
