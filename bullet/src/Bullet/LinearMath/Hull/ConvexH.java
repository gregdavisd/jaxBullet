/*
Stan Melax Convex Hull Computation
Copyright (c) 2008 Stan Melax http://www.melax.com/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.LinearMath.Hull;

import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import java.util.ArrayList;

/**
 *
 * @author Gregery Barton
 */
public class ConvexH implements Serializable {

 public static class HalfEdge {

  public short ea;         // the other half of the edge (index into edges list)
  public short v;  // the vertex at the start of this edge (index into vertices list)
  public short p;  // the facet on which this edge lies (index into facets list)

  public HalfEdge() {
  }

  public HalfEdge(short _ea, short _v, short _p) {
   ea = _ea;
   v = _v;
   p = _p;
  }
 };

 public ConvexH() {
 }
 public final ArrayList<btVector3> vertices = new ArrayList<>(0);
 public final ArrayList<HalfEdge> edges = new ArrayList<>(0);
 public final ArrayList<btPlane> facets = new ArrayList<>(0);

 ConvexH(int vertices_size, int edges_size, int facets_size) {
  vertices.ensureCapacity(vertices_size);
  while (vertices.size() < vertices_size) {
   vertices.add(new btVector3());
  }
  edges.ensureCapacity(edges_size);
  while (edges.size() < edges_size) {
   edges.add(new HalfEdge());
  }
  facets.ensureCapacity(facets_size);
  while (facets.size() < facets_size) {
   facets.add(new btPlane());
  }
 }
}
