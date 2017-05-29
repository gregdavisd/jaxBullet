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

import java.nio.ByteBuffer;
import org.lwjgl.BufferUtils;
import org.lwjgl.opengl.GL11;
import static org.lwjgl.opengl.GL11.GL_FLOAT;
import static org.lwjgl.opengl.GL11.glDrawElements;
import static org.lwjgl.opengl.GL11.glFlush;
import static org.lwjgl.opengl.GL11.glNormalPointer;
import static org.lwjgl.opengl.GL11.glVertexPointer;
import org.lwjgl.opengl.GL15;
import static org.lwjgl.opengl.GL15.GL_ARRAY_BUFFER;
import static org.lwjgl.opengl.GL15.GL_ELEMENT_ARRAY_BUFFER;
import static org.lwjgl.opengl.GL15.glBindBuffer;
import static org.lwjgl.opengl.GL15.glBufferData;
import static org.lwjgl.opengl.GL15.glDeleteBuffers;

/**
 *
 * @author Gregery Barton
 */
abstract public class GLDrawElements {

 int vaa_name;
 int eab_name;
 int vaa_size;
 int eab_size;

 public GLDrawElements(int[] buffer, int[] indices) {
  upload_vaa(buffer);
  upload_eab(indices);
 }

 public GLDrawElements(float[] buffer, int[] indices) {
  upload_vaa(buffer);
  upload_eab(indices);
 }

 private void upload_eab(int[] buffer) {
  eab_name = GL15.glGenBuffers();
  GL15.glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eab_name);
  int capacity = buffer.length * 4;
  eab_size = capacity;
  ByteBuffer data = BufferUtils.createByteBuffer(capacity);
  data.asIntBuffer().put(buffer);
  data.rewind();
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, data, GL15.GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
 }

 private void upload_vaa(int[] buffer) {
  vaa_name = GL15.glGenBuffers();
  GL15.glBindBuffer(GL_ARRAY_BUFFER, vaa_name);
  int capacity = buffer.length * 4;
  vaa_size = capacity;
  ByteBuffer data = BufferUtils.createByteBuffer(capacity);
  data.asIntBuffer().put(buffer);
  data.rewind();
  glBufferData(GL_ARRAY_BUFFER, data, GL15.GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
 }

 private void upload_vaa(float[] buffer) {
  vaa_name = GL15.glGenBuffers();
  GL15.glBindBuffer(GL_ARRAY_BUFFER, vaa_name);
  int capacity = buffer.length * 4;
  vaa_size = capacity;
  ByteBuffer data = BufferUtils.createByteBuffer(capacity);
  data.asFloatBuffer().put(buffer);
  data.rewind();
  glBufferData(GL_ARRAY_BUFFER, data, GL15.GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
 }

 public void bind() {
  glBindBuffer(GL_ARRAY_BUFFER, vaa_name);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eab_name);
  glVertexPointer(3, GL_FLOAT, 6 * 4, 0);
  glNormalPointer(GL_FLOAT, 6 * 4, 3 * 4);
 }

 abstract protected int get_mode();

 public void destroy() {
  glDeleteBuffers(eab_name);
  glDeleteBuffers(vaa_name);
  eab_name = 0;
  vaa_name = 0;
 }

 public void draw() {
  glDrawElements(get_mode(), eab_size / 4, GL11.GL_UNSIGNED_INT, 0);
 }
}
