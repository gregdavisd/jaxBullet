/*
Copyright (c) 2017 Gregery Barton

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

import Bullet.Collision.btTriangleCallback;
import Bullet.LinearMath.btVector3;
import java.nio.FloatBuffer;

/**
 * Striding mesh interface adapted for java.Assumes that X,Y,Z are packed together in that order,
 * however there can be a stride between vertices. Note that the original bullet would
 * process_all_triangles a specified number of faces with numFaces, this interface processes a
 * specified number of vertices with numIndices. Return from overridden
 * getLockedReadOnlyVertexIndexBase(). Callbacks can return false to stop processing any further
 * triangles.
 *
 * @author Gregery Barton
 */
public class btStridingMeshLock {

 /**
  *
  */
 public static final int TRIANGLES = 1;
 /**
  *
  */
 public static final int TRIANGLE_STRIP = 0;
 /**
  * Store the Java array containing vertex points
  */
 public final Object vertexBuffer;
 /**
  * distance between vertices in the vertexBuffer (in Java style array indices not bytes)
  */
 public final int stride;
 /**
  * Store Java array of indices
  */
 public final Object indexBuffer;
 /**
  * arrangement of indices (TRIANGLES or TRIANGLE_STRIP)
  */
 public final int indexBufferType;
 /**
  * base index into indexBuffer (in Java style array indices not bytes)
  */
 public final int indexBase;
 /**
  * distance between subsequent indices (in Java style array indices not bytes)
  */
 public final int indexStride;
 /**
  * number of indices to process_all_triangles in the indexBuffer
  */
 public final int numIndices;
 private final GenericVertexDataBuffer gen_buffer;
 private final GenericIndexBuffer gen_indices;
 private final TriangleIndicesCallback triangle_internal;
 private int part;
 private btTriangleCallback vertex_callback;

 /**
  *
  * @param vertexBuffer Store the Java array containing vertex points
  * @param stride distance between vertices in the vertexBuffer (in Java style array indices of
  * elements not bytes)
  * @param indexBuffer Store Java array of indices
  * @param indexBufferType arrangement of indices (TRIANGLES or TRIANGLE_STRIP)
  * @param indexBase base index into indexBuffer (in Java style array indices not bytes)
  * @param indexStride distance between subsequent indices (in Java style array indices not bytes)
  * @param numIndices number of indices to process_all_triangles in the indexBuffer
  */
 public btStridingMeshLock(Object vertexBuffer, int stride, Object indexBuffer, int indexBufferType,
  int indexBase, int indexStride, int numIndices) {
  this.vertexBuffer = vertexBuffer;
  this.stride = stride;
  this.indexBuffer = indexBuffer;
  this.indexBufferType = indexBufferType;
  this.indexBase = indexBase;
  this.indexStride = indexStride;
  this.numIndices = numIndices;
  if (vertexBuffer instanceof float[]) {
   gen_buffer = new FloatVertexDataBuffer((float[]) vertexBuffer);
  } else if (vertexBuffer instanceof FloatBuffer) {
   gen_buffer = new FloatBufferVertexDataBuffer((FloatBuffer) vertexBuffer);
  } else if (vertexBuffer instanceof double[]) {
   gen_buffer = new DoubleVertexDataBuffer((double[]) vertexBuffer);
  } else if (vertexBuffer instanceof short[]) {
   gen_buffer = new ShortVertexDataBuffer((short[]) vertexBuffer);
  } else {
   gen_buffer = new InvalidVertexDataBuffer();
  }
  if (indexBuffer instanceof int[]) {
   gen_indices = new IntArrayIndexBuffer((int[]) indexBuffer);
  } else if (indexBuffer instanceof short[]) {
   gen_indices = new ShortArrayIndexBuffer((short[]) indexBuffer);
  } else if (indexBuffer instanceof byte[]) {
   gen_indices = new ByteArrayIndexBuffer((byte[]) indexBuffer);
  } else {
   gen_indices = new InvalidIndexBuffer();
  }
  triangle_internal = new TriangleIndicesBTVectorCallback();
 }

 private void stream_triangle_indices(GenericIndexBuffer indices, int start_triangle_index) {
  int end = indexBase + (numIndices * indexStride);
  int[] triangle = new int[3];
  int triangle_stride = (3 * indexStride);
  int triangle_index = start_triangle_index;
  for (int i = indexBase + (start_triangle_index * 3); i < end; i += triangle_stride) {
   triangle[0] = indices.get(i);
   triangle[1] = indices.get(i + indexStride);
   triangle[2] = indices.get(i + (2 * indexStride));
   if (!triangle_internal.triangle(triangle, triangle_index)) {
    return;
   }
   ++triangle_index;
  }
 }

 private void stream_strip_indices(GenericIndexBuffer indices, int start_triangle_index) {
  if (numIndices < 3) {
   return;
  }
  int[] triangle = new int[3];
  int[] flipped_triangle = new int[3];
  int triangle_index = start_triangle_index;
  int i = indexBase + (start_triangle_index * indexStride);
  triangle[0] = indices.get(i);
  i += indexStride;
  triangle[1] = indices.get(i);
  i += indexStride;
  boolean ccw = (triangle_index % 2) == 0;
  int end = indexBase + (numIndices * indexStride);
  for (; i < end; i += indexStride) {
   triangle[2] = indices.get(i);
   if (!((triangle[0] == triangle[1]) || (triangle[0] == triangle[2]) ||
    (triangle[1] == triangle[2]))) {
    if (ccw) {
     if (!triangle_internal.triangle(triangle, triangle_index)) {
      return;
     }
    } else {
     flipped_triangle[0] = triangle[2];
     flipped_triangle[1] = triangle[1];
     flipped_triangle[2] = triangle[0];
     if (!triangle_internal.triangle(flipped_triangle, triangle_index)) {
      return;
     }
    }
   }
   triangle[0] = triangle[1];
   triangle[1] = triangle[2];
   ccw = !ccw;
   ++triangle_index;
  }
 }

 private void stream_indices(int start_triangle_index) {
  switch (indexBufferType) {
   case TRIANGLES:
    stream_triangle_indices(gen_indices, start_triangle_index);
    break;
   case TRIANGLE_STRIP:
    stream_strip_indices(gen_indices, start_triangle_index);
    break;
   default:
    break;
  }
 }

 /**
  * Go through all the triangles or until the callback returns false.
  *
  * @param callback
  * @param part
  */
 public void process_all_triangles(btTriangleCallback callback, int part) {
  this.part = part;
  vertex_callback = callback;
  stream_indices(0);
 }

 public void process_all_triangles(btTriangleCallback callback, int part,
  int base_triangle_index) {
  this.part = part;
  vertex_callback = callback;
  stream_indices(base_triangle_index);
 }

 static interface GenericIndexBuffer {

  int get(int i);
 }

 static class InvalidIndexBuffer implements GenericIndexBuffer {

  @Override
  public int get(int i) {
   return 0;
  }
 }

 static class IntArrayIndexBuffer implements GenericIndexBuffer {

  final int[] buffer;

  IntArrayIndexBuffer(int[] buffer) {
   this.buffer = buffer;
  }

  @Override
  public int get(int i) {
   return buffer[i];
  }
 }

 static class ShortArrayIndexBuffer implements GenericIndexBuffer {

  final short[] buffer;

  ShortArrayIndexBuffer(short[] buffer) {
   this.buffer = buffer;
  }

  @Override
  public int get(int i) {
   return buffer[i];
  }
 }

 static class ByteArrayIndexBuffer implements GenericIndexBuffer {

  final byte[] buffer;

  ByteArrayIndexBuffer(byte[] buffer) {
   this.buffer = buffer;
  }

  @Override
  public int get(int i) {
   return buffer[i];
  }
 }

 static interface GenericVertexDataBuffer {

  float get(int i);
 }

 static class InvalidVertexDataBuffer implements GenericVertexDataBuffer {

  @Override
  public float get(int i) {
   return 0.0f;
  }
 }

 static class DoubleVertexDataBuffer implements GenericVertexDataBuffer {

  final double[] buffer;

  DoubleVertexDataBuffer(double[] buffer) {
   this.buffer = buffer;
  }

  @Override
  public float get(int i) {
   return (float) buffer[i];
  }
 }

 static class FloatVertexDataBuffer implements GenericVertexDataBuffer {

  final float[] buffer;

  FloatVertexDataBuffer(float[] buffer) {
   this.buffer = buffer;
  }

  @Override
  public float get(int i) {
   return (float) buffer[i];
  }
 }

 static class FloatBufferVertexDataBuffer implements GenericVertexDataBuffer {

  final FloatBuffer buffer;

  FloatBufferVertexDataBuffer(FloatBuffer buffer) {
   this.buffer = buffer;
  }

  @Override
  public float get(int i) {
   return buffer.get(i);
  }
 }

 static class ShortVertexDataBuffer implements GenericVertexDataBuffer {

  final short[] buffer;

  public ShortVertexDataBuffer(short[] buffer) {
   this.buffer = buffer;
  }

  @Override
  public float get(int i) {
   return buffer[i];
  }
 }

 private interface TriangleIndicesCallback {

  boolean triangle(int[] indices, int index);
 }

 private final class TriangleIndicesBTVectorCallback implements TriangleIndicesCallback {

  final btVector3[] v = new btVector3[]{new btVector3(), new btVector3(), new btVector3()};

  public TriangleIndicesBTVectorCallback() {
  }

  @Override
  public boolean triangle(int[] indices, int index) {
   for (int i = 0; i < 3; ++i) {
    set(v[i], indices[i]);
   }
   return vertex_callback.processTriangle(v, part, index);
  }

  void set(final btVector3 vec, int index) {
   int offset = index * stride;
   vec.set(gen_buffer.get(offset), gen_buffer.get(offset + 1), gen_buffer.get(offset + 2));
  }
 }
}
