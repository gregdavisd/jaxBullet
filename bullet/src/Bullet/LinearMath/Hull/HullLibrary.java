
/*
 * Stan Melax Convex Hull Computation
 * Copyright (c) 2008 Stan Melax http://www.melax.com/
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
package Bullet.LinearMath.Hull;

import static Bullet.LinearMath.Hull.HullError.QE_FAIL;
import static Bullet.LinearMath.Hull.HullError.QE_OK;
import static Bullet.LinearMath.Hull.HullFlag.QF_REVERSE_ORDER;
import static Bullet.LinearMath.Hull.HullFlag.QF_TRIANGLES;
import static Bullet.LinearMath.btScalar.FLT_MAX;
import static Bullet.LinearMath.btScalar.SIMD_RADS_PER_DEG;
import static Bullet.LinearMath.btScalar.btCos;
import static Bullet.LinearMath.btScalar.btFabs;
import static Bullet.LinearMath.btScalar.btSin;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.init;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Arrays;
import org.apache.commons.collections.primitives.ArrayIntList;

/**
 *
 * The HullLibrary class can create a convex hull from a collection of vertices,
 * using the ComputeHull method. The btShapeHull class uses this HullLibrary to
 * create a approximate convex mesh given a general (non-polyhedral) convex
 * shape.
 */
public class HullLibrary implements Serializable {

 static final float EPSILON = 0.000001f;

 static void addPoint(int[] vcount, btVector3[] p, float x, float y, float z) {
  // XXX, might be broken
  final btVector3 dest = p[vcount[0]];
  dest.set(x, y, z);
  vcount[0]++;
 }

 final ArrayList<btHullTriangle> m_tris = new ArrayList<>(0);
 public final ArrayIntList m_vertexIndexMapping = new ArrayIntList();

 public int CreateConvexHull(HullDesc desc, // describes the input request
  HullResult result) // contains the resulst
 {
  int ret = QE_FAIL;
  PHullResult hr = new PHullResult();
  int vcount = desc.mVcount;
  if (vcount < 8) {
   vcount = 8;
  }
  btVector3[] vertexSource = new btVector3[vcount];
  init(vertexSource);
  final btVector3 scale = new btVector3();
  int[] ovcount = new int[1];
  boolean ok = CleanupVertices(desc.mVcount, desc.mVertices, desc.mVertexStride,
   ovcount,
   vertexSource, desc.mNormalEpsilon, scale); // normalize point cloud, remove duplicates!
  if (ok) {
//		if ( 1 ) // scale vertices back to their original size.
   {
    for (int i = 0; i < ovcount[0]; i++) {
     final btVector3 v = vertexSource[i];
     v.mul(scale);
    }
   }
   ok = ComputeHull(ovcount[0], vertexSource, hr, desc.mMaxVertices);
   if (ok) {
    // re-index triangle mesh so it refers to only used vertices, rebuild a new vertex table.
    btVector3[] vertexScratch = new btVector3[hr.mVcount];
    init(vertexScratch);
    BringOutYourDead(hr.mVertices, hr.mVcount, vertexScratch, ovcount,
     hr.m_Indices.toBackedArray(),
     hr.mIndexCount);
    ret = QE_OK;
    if (desc.HasHullFlag(QF_TRIANGLES)) // if he wants the results as triangle!
    {
     result.mPolygons = false;
     result.mNumOutputVertices = ovcount[0];
     result.m_OutputVertices = Arrays.copyOfRange(vertexScratch, 0, ovcount[0]);
     result.mNumFaces = hr.mFaceCount;
     result.mNumIndices = hr.mIndexCount;
     hr_indices_to_result_triangles(result, desc, hr);
    } else {
     result.mPolygons = true;
     result.mNumOutputVertices = ovcount[0];
     result.m_OutputVertices = Arrays.copyOfRange(vertexScratch, 0, ovcount[0]);
     result.mNumFaces = hr.mFaceCount;
     result.mNumIndices = hr.mIndexCount + hr.mFaceCount;
     hr_indices_to_result_counted_triangles(result, desc, hr);
    }
    ReleaseHull(hr);
   }
  }
  return ret;
 }

 private void hr_indices_to_result_triangles(HullResult result, HullDesc desc,
  PHullResult hr) {
  if (desc.HasHullFlag(QF_REVERSE_ORDER)) {
   ArrayIntList Indices = new ArrayIntList();
   Indices.ensureCapacity(hr.mFaceCount * 3);
   for (int i = 0; i < hr.mFaceCount; i++) {
    Indices.add(hr.m_Indices.get((i * 3) + 2));
    Indices.add(hr.m_Indices.get((i * 3) + 1));
    Indices.add(hr.m_Indices.get(i * 3));
   }
   result.m_Indices = Indices.toBackedArray();
  } else {
   result.m_Indices = Arrays.copyOfRange(hr.m_Indices.toBackedArray(), 0,
    hr.mIndexCount);
  }
 }

 private void hr_indices_to_result_counted_triangles(HullResult result,
  HullDesc desc,
  PHullResult hr) {
  ArrayIntList dest = new ArrayIntList();
  dest.ensureCapacity(hr.mFaceCount * 4);
  for (int i = 0; i < hr.mFaceCount; i++) {
   dest.add(3);
   if (desc.HasHullFlag(QF_REVERSE_ORDER)) {
    dest.add(hr.m_Indices.get((i * 3) + 2));
    dest.add(hr.m_Indices.get((i * 3) + 1));
    dest.add(hr.m_Indices.get((i * 3)));
   } else {
    dest.add(hr.m_Indices.get((i * 3)));
    dest.add(hr.m_Indices.get((i * 3) + 1));
    dest.add(hr.m_Indices.get((i * 3) + 2));
   }
  }
  result.m_Indices = dest.toBackedArray();
 }

 public int ReleaseResult(HullResult result) // release memory allocated for this result, we are done with it.
 {
  result.mNumOutputVertices = 0;
  result.m_OutputVertices = null;
  result.mNumIndices = 0;
  result.m_Indices = null;
  return QE_OK;
 }

 boolean ComputeHull(int vcount, btVector3[] vertices, PHullResult result,
  int vlimit) {
  int[] tris_count = new int[1];
  int ret = calchull(vertices, vcount, result.m_Indices, tris_count, vlimit);
  if (ret == 0) {
   return false;
  }
  result.mIndexCount = (tris_count[0] * 3);
  result.mFaceCount = tris_count[0];
  result.mVertices = vertices;
  result.mVcount = vcount;
  return true;
 }

 btHullTriangle allocateTriangle(int a, int b, int c) {
  btHullTriangle tr = new btHullTriangle(a, b, c);
  tr.id = m_tris.size();
  m_tris.add(tr);
  return tr;
 }

 void deAllocateTriangle(btHullTriangle tri) {
  assert (m_tris.get(tri.id) == tri);
  m_tris.set(tri.id, null);
 }

 void b2bfix(btHullTriangle s, btHullTriangle t) {
  int i;
  for (i = 0; i < 3; i++) {
   int i1 = (i + 1) % 3;
   int i2 = (i + 2) % 3;
   int a = s.getElement(i1);
   int b = s.getElement(i2);
   assert (m_tris.get(s.neib(a, b).get()).neib(b, a).get() == s.id);
   assert (m_tris.get(t.neib(a, b).get()).neib(b, a).get() == t.id);
   m_tris.get(s.neib(a, b).get()).neib(b, a).set(t.neib(b, a).get());
   m_tris.get(t.neib(b, a).get()).neib(a, b).set(s.neib(a, b).get());
  }
 }

 void removeb2b(btHullTriangle s, btHullTriangle t) {
  b2bfix(s, t);
  deAllocateTriangle(s);
  deAllocateTriangle(t);
 }

 void checkit(btHullTriangle t) {
  int i;
  assert (m_tris.get(t.id) == t);
  for (i = 0; i < 3; i++) {
   int i1 = (i + 1) % 3;
   int i2 = (i + 2) % 3;
   int a = t.getElement(i1);
   int b = t.getElement(i2);
   assert (a != b);
   assert (m_tris.get(t.n.getElement(i)).neib(b, a).get() == t.id);
  }
 }

 btHullTriangle extrudable(float epsilon) {
  int i;
  btHullTriangle t = null;
  for (i = 0; i < m_tris.size(); i++) {
   btHullTriangle triangle = m_tris.get(i);
   if ((t == null) || ((triangle != null) && t.rise < triangle.rise)) {
    t = triangle;
   }
  }
  return (t != null && t.rise > epsilon) ? t : null;
 }

 int calchullgen(btVector3[] verts, int verts_count, int vlimit) {
  int do_vlimit = vlimit;
  if (verts_count < 4) {
   return 0;
  }
  if (do_vlimit == 0) {
   do_vlimit = 1000000000;
  }
  int j;
  final btVector3 bmin = new btVector3(verts[0]);
  final btVector3 bmax = new btVector3(verts[0]);
  ArrayIntList isextreme = new ArrayIntList(verts_count);
  ArrayIntList allow = new ArrayIntList(verts_count);
  float epsilon;
  for (j = 0; j < verts_count; j++) {
   allow.add(1);
   isextreme.add(0);
   bmin.setMin(verts[j]);
   bmax.setMax(verts[j]);
  }
  epsilon = new btVector3(bmax).sub(bmin).length() * (0.001f);
  assert (epsilon != 0.0);
  Int4 p = FindSimplex(verts, verts_count, allow.toBackedArray());
  if (p.x == -1) {
   return 0; // simplex failed
  }
  final btVector3 center = new btVector3(verts[p.x]).add(verts[p.y]).add(
   verts[p.z]).add(verts[p.w])
   .scale(1.0f / 4.0f);  // a valid interior point
  btHullTriangle t0 = allocateTriangle(p.z, p.w, p.y);
  t0.n.set(2, 3, 1);
  btHullTriangle t1 = allocateTriangle(p.w, p.z, p.x);
  t1.n.set(3, 2, 0);
  btHullTriangle t2 = allocateTriangle(p.x, p.y, p.w);
  t2.n.set(0, 1, 3);
  btHullTriangle t3 = allocateTriangle(p.y, p.x, p.z);
  t3.n.set(1, 0, 2);
  isextreme.set(p.x, 1);
  isextreme.set(p.y, 1);
  isextreme.set(p.z, 1);
  isextreme.set(p.w, 1);
  checkit(t0);
  checkit(t1);
  checkit(t2);
  checkit(t3);
  for (j = 0; j < m_tris.size(); j++) {
   btHullTriangle t = m_tris.get(j);
   assert (t != null);
   assert (t.vmax < 0);
   final btVector3 n = TriNormal(verts[t.x], verts[t.y], verts[t.z]);
   t.vmax = maxdirsterid(verts, verts_count, n, allow.toBackedArray());
   t.rise = n.dot(new btVector3(verts[t.vmax]).sub(verts[t.x]));
  }
  btHullTriangle te;
  do_vlimit -= 4;
  while (do_vlimit > 0 && ((te = extrudable(epsilon)) != null)) {
   //int3 ti=*te;
   int v = te.vmax;
   assert (v != -1);
   assert (isextreme.get(v) == 0);  // wtf we've already done this vertex
   isextreme.set(v, 1);
   //if(v==p0 || v==p1 || v==p2 || v==p3) continue; // done these already
   j = m_tris.size();
   while (j-- > 0) {
    btHullTriangle triangle = m_tris.get(j);
    if (triangle == null) {
     continue;
    }
    Int3 t = triangle;
    if (above(verts, t, verts[v], (0.01f) * epsilon) != 0) {
     extrude(m_tris.get(j), v);
    }
   }
   // now check for those degenerate cases where we have a flipped triangle or a really skinny triangle
   j = m_tris.size();
   while (j-- > 0) {
    btHullTriangle nt = m_tris.get(j);
    if (nt == null) {
     continue;
    }
    if (!hasvert(nt, v)) {
     break;
    }
    if (above(verts, nt, center, (0.01f) * epsilon) != 0 || (new btVector3(
     verts[nt.y]).sub(
     verts[nt.x])).cross(new btVector3(verts[nt.z]).sub(verts[nt.y])).length()
     < epsilon * epsilon * (0.1f)) {
     btHullTriangle nb = m_tris.get(m_tris.get(j).n.x);
     assert (nb != null);
     assert (!hasvert(nb, v));
     assert (nb.id < j);
     extrude(nb, v);
     j = m_tris.size();
    }
   }
   j = m_tris.size();
   while (j-- > 0) {
    btHullTriangle t = m_tris.get(j);
    if (t == null) {
     continue;
    }
    if (t.vmax >= 0) {
     break;
    }
    final btVector3 n = TriNormal(verts[t.x], verts[t.y], verts[t.z]);
    t.vmax = maxdirsterid(verts, verts_count, n, allow.toBackedArray());
    if (isextreme.get(t.vmax) != 0) {
     t.vmax = -1; // already done that vertex - algorithm needs to be able to terminate.
    } else {
     t.rise = (n.dot(new btVector3(verts[t.vmax]).sub(verts[t.x])));
    }
   }
   do_vlimit--;
  }
  return 1;
 }

 Int4 FindSimplex(btVector3[] verts, int verts_count, int[] allow) {
  btVector3[] basis = new btVector3[3];
  init(basis);
  basis[0].set(0.01f, 0.02f, 1.0f);
  int p0 = maxdirsterid(verts, verts_count, basis[0], allow);
  int p1 = maxdirsterid(verts, verts_count, new btVector3(basis[0]).negate(),
   allow);
  basis[0].set(verts[p0]).sub(verts[p1]);
  if (p0 == p1 || basis[0].equals(new btVector3(0, 0, 0))) {
   return new Int4(-1, -1, -1, -1);
  }
  basis[1].set(1f, 0.02f, 0f).cross(basis[0]);
  basis[2].set(-0.02f, 1f, 0f).cross(basis[0]);
  if (basis[1].length() > basis[2].length()) {
   basis[1].normalize();
  } else {
   basis[1].set(basis[2]);
   basis[1].normalize();
  }
  int p2 = maxdirsterid(verts, verts_count, basis[1], allow);
  if (p2 == p0 || p2 == p1) {
   p2 = maxdirsterid(verts, verts_count, new btVector3(basis[1]).negate(), allow);
  }
  if (p2 == p0 || p2 == p1) {
   return new Int4(-1, -1, -1, -1);
  }
  basis[1].set(verts[p2]).sub(verts[p0]);
  basis[2].set(basis[1]).cross(basis[0]).normalize();
  int p3 = maxdirsterid(verts, verts_count, basis[2], allow);
  if (p3 == p0 || p3 == p1 || p3 == p2) {
   p3 = maxdirsterid(verts, verts_count, new btVector3(basis[2]).negate(), allow);
  }
  if (p3 == p0 || p3 == p1 || p3 == p2) {
   return new Int4(-1, -1, -1, -1);
  }
  assert (!(p0 == p1 || p0 == p2 || p0 == p3 || p1 == p2 || p1 == p3 || p2 == p3));
  if (new btVector3(verts[p3])
   .sub(verts[p0])
   .dot(new btVector3(verts[p1])
    .sub(verts[p0])
    .cross(new btVector3(verts[p2])
     .sub(verts[p0]))) < 0) {
   int swapper = p2;
   p2 = p3;
   p3 = swapper;
  }
  return new Int4(p0, p1, p2, p3);
 }

 void extrude(btHullTriangle t0, int v) {
  Int3 t = t0;
  int n = m_tris.size();
  btHullTriangle ta = allocateTriangle(v, t.y, t.z);
  ta.n.set(t0.n.x, n + 1, n + 2);
  m_tris.get(t0.n.x).neib(t.y, t.z).set(n + 0);
  btHullTriangle tb = allocateTriangle(v, t.z, t.x);
  tb.n.set(t0.n.y, n + 2, n + 0);
  m_tris.get(t0.n.y).neib(t.z, t.x).set(n + 1);
  btHullTriangle tc = allocateTriangle(v, t.x, t.y);
  tc.n.set(t0.n.z, n + 0, n + 1);
  m_tris.get(t0.n.z).neib(t.x, t.y).set(n + 2);
  checkit(ta);
  checkit(tb);
  checkit(tc);
  if (hasvert(m_tris.get(ta.n.x), v)) {
   removeb2b(ta, m_tris.get(ta.n.x));
  }
  if (hasvert(m_tris.get(tb.n.x), v)) {
   removeb2b(tb, m_tris.get(tb.n.x));
  }
  if (hasvert(m_tris.get(tc.n.x), v)) {
   removeb2b(tc, m_tris.get(tc.n.x));
  }
  deAllocateTriangle(t0);
 }

 //BringOutYourDead (John Ratcliff): When you create a convex hull you hand it a large input set of vertices forming a 'point cloud'. 
 //After the hull is generated it give you back a set of polygon faces which index the *original* point cloud.
 //The thing is, often times, there are many 'dead vertices' in the point cloud that are on longer referenced by the hull.
 //The routine 'BringOutYourDead' find only the referenced vertices, copies them to an new buffer, and re-indexes the hull so that it is a minimal representation.
 void BringOutYourDead(btVector3[] verts, int vcount, btVector3[] overts,
  int[] ocount,
  int[] indices, int indexcount) {
  int[] tmpIndices = m_vertexIndexMapping.toArray();
  int[] usedIndices = new int[vcount];
  ocount[0] = 0;
  for (int i = 0; i < (indexcount); i++) {
   int v = indices[i]; // original array index
   assert (v >= 0 && v < vcount);
   if (usedIndices[v] != 0) // if already remapped
   {
    indices[i] = usedIndices[v] - 1; // index to new array
   } else {
    indices[i] = ocount[0];      // new index mapping
    overts[ocount[0]].set(verts[v]);
    for (int k = 0; k < m_vertexIndexMapping.size(); k++) {
     if (tmpIndices[k] == (v)) {
      m_vertexIndexMapping.set(k, ocount[0]);
     }
    }
    ocount[0]++; // increment output vert count
    assert (ocount[0] >= 0 && ocount[0] <= vcount);
    usedIndices[v] = ocount[0]; // assign new index remapping
   }
  }
 }

 boolean CleanupVertices(int svcount,
  btVector3[] svertices,
  int stride,
  int[] vcount, // output number of vertices
  btVector3[] vertices, // location to store the results.
  float normalepsilon, final btVector3 scale) {
  if (svcount == 0) {
   return false;
  }
  m_vertexIndexMapping.clear();
  vcount[0] = 0;
  float[] recip = new float[]{0.f, 0.f, 0.f};
  if (scale != null) {
   scale.set(1f, 1f, 1f);
  }
  float[] bmin = new float[]{FLT_MAX, FLT_MAX, FLT_MAX};
  float[] bmax = new float[]{-FLT_MAX, -FLT_MAX, -FLT_MAX};
  int vtx = 0;
//	if ( 1 )
  {
   for (int i = 0; i < svcount; i++) {
    final btVector3 p = svertices[vtx];
    vtx += stride;
    if (p.x < bmin[0]) {
     bmin[0] = p.x;
    }
    if (p.x > bmax[0]) {
     bmax[0] = p.x;
    }
    if (p.y < bmin[1]) {
     bmin[1] = p.y;
    }
    if (p.y > bmax[1]) {
     bmax[1] = p.y;
    }
    if (p.z < bmin[2]) {
     bmin[2] = p.z;
    }
    if (p.z > bmax[2]) {
     bmax[2] = p.z;
    }
   }
  }
  float dx = bmax[0] - bmin[0];
  float dy = bmax[1] - bmin[1];
  float dz = bmax[2] - bmin[2];
  final btVector3 center = new btVector3();
  center.x = dx * (0.5f) + bmin[0];
  center.y = dy * (0.5f) + bmin[1];
  center.z = dz * (0.5f) + bmin[2];
  if (dx < EPSILON || dy < EPSILON || dz < EPSILON || svcount < 3) {
   float len = FLT_MAX;
   if (dx > EPSILON && dx < len) {
    len = dx;
   }
   if (dy > EPSILON && dy < len) {
    len = dy;
   }
   if (dz > EPSILON && dz < len) {
    len = dz;
   }
   if (len == FLT_MAX) {
    dx = dy = dz = (0.01f); // one centimeter
   } else {
    if (dx < EPSILON) {
     dx = len * (0.05f); // 1/5th the shortest non-zero edge.
    }
    if (dy < EPSILON) {
     dy = len * (0.05f);
    }
    if (dz < EPSILON) {
     dz = len * (0.05f);
    }
   }
   float x1 = center.x - dx;
   float x2 = center.x + dx;
   float y1 = center.y - dy;
   float y2 = center.y + dy;
   float z1 = center.z - dz;
   float z2 = center.z + dz;
   addPoint(vcount, vertices, x1, y1, z1);
   addPoint(vcount, vertices, x2, y1, z1);
   addPoint(vcount, vertices, x2, y2, z1);
   addPoint(vcount, vertices, x1, y2, z1);
   addPoint(vcount, vertices, x1, y1, z2);
   addPoint(vcount, vertices, x2, y1, z2);
   addPoint(vcount, vertices, x2, y2, z2);
   addPoint(vcount, vertices, x1, y2, z2);
   return true; // return cube
  } else if (scale != null) {
   scale.set(dx, dy, dz);
   recip[0] = 1 / dx;
   recip[1] = 1 / dy;
   recip[2] = 1 / dz;
   center.x *= recip[0];
   center.y *= recip[1];
   center.z *= recip[2];
  }
  vtx = 0;
  for (int i = 0; i < svcount; i++) {
   final btVector3 p = svertices[vtx];
   vtx += stride;
   float px = p.getX();
   float py = p.getY();
   float pz = p.getZ();
   if (scale != null) {
    px *= recip[0]; // normalize
    py *= recip[1]; // normalize
    pz *= recip[2]; // normalize
   }
//		if ( 1 )
   {
    int j;
    for (j = 0; j < vcount[0]; j++) {
     /// XXX might be broken
     final btVector3 v = vertices[j];
     float x = v.x;
     float y = v.y;
     float z = v.z;
     float _dx = btFabs(x - px);
     float _dy = btFabs(y - py);
     float _dz = btFabs(z - pz);
     if (_dx < normalepsilon && _dy < normalepsilon && _dz < normalepsilon) {
      // ok, it is close enough to the old one
      // now let us see if it is further from the center of the point cloud than the one we already recorded.
      // in which case we keep this one instead.
      float dist1 = GetDist(px, py, pz, center);
      float dist2 = GetDist(v.x, v.y, v.z, center);
      if (dist1 > dist2) {
       v.x = px;
       v.y = py;
       v.z = pz;
      }
      break;
     }
    }
    if (j == vcount[0]) {
     final btVector3 dest = vertices[vcount[0]];
     dest.set(px, py, pz);
     vcount[0]++;
    }
    m_vertexIndexMapping.add(j);
   }
  }
  // ok..now make sure we didn't prune so many vertices it is now invalid.
  {
   bmin = new float[]{FLT_MAX, FLT_MAX, FLT_MAX};
   bmax = new float[]{-FLT_MAX, -FLT_MAX, -FLT_MAX};
   for (int i = 0; i < vcount[0]; i++) {
    final btVector3 p = vertices[i];
    if (p.x < bmin[0]) {
     bmin[0] = p.x;
    }
    if (p.x > bmax[0]) {
     bmax[0] = p.x;
    }
    if (p.y < bmin[1]) {
     bmin[1] = p.y;
    }
    if (p.y > bmax[1]) {
     bmax[1] = p.y;
    }
    if (p.z < bmin[2]) {
     bmin[2] = p.z;
    }
    if (p.z > bmax[2]) {
     bmax[2] = p.z;
    }
   }
   float _dx = bmax[0] - bmin[0];
   float _dy = bmax[1] - bmin[1];
   float _dz = bmax[2] - bmin[2];
   if (_dx < EPSILON || _dy < EPSILON || _dz < EPSILON || vcount[0] < 3) {
    float cx = dx * (0.5f) + bmin[0];
    float cy = dy * (0.5f) + bmin[1];
    float cz = dz * (0.5f) + bmin[2];
    float len = FLT_MAX;
    if (dx >= EPSILON && dx < len) {
     len = dx;
    }
    if (dy >= EPSILON && dy < len) {
     len = dy;
    }
    if (dz >= EPSILON && dz < len) {
     len = dz;
    }
    if (len == FLT_MAX) {
     dx = dy = dz = (0.01f); // one centimeter
    } else {
     if (dx < EPSILON) {
      dx = len * (0.05f); // 1/5th the shortest non-zero edge.
     }
     if (dy < EPSILON) {
      dy = len * (0.05f);
     }
     if (dz < EPSILON) {
      dz = len * (0.05f);
     }
    }
    float x1 = cx - dx;
    float x2 = cx + dx;
    float y1 = cy - dy;
    float y2 = cy + dy;
    float z1 = cz - dz;
    float z2 = cz + dz;
    vcount[0] = 0; // add box
    addPoint(vcount, vertices, x1, y1, z1);
    addPoint(vcount, vertices, x2, y1, z1);
    addPoint(vcount, vertices, x2, y2, z1);
    addPoint(vcount, vertices, x1, y2, z1);
    addPoint(vcount, vertices, x1, y1, z2);
    addPoint(vcount, vertices, x2, y1, z2);
    addPoint(vcount, vertices, x2, y2, z2);
    addPoint(vcount, vertices, x1, y2, z2);
    return true;
   }
  }
  return true;
 }

 float GetDist(float px, float py, float pz, final btVector3 p2) {
  float dx = px - p2.x;
  float dy = py - p2.y;
  float dz = pz - p2.z;
  return dx * dx + dy * dy + dz * dz;
 }

 int calchull(btVector3[] verts, int verts_count, ArrayIntList tris_out,
  int[] tris_count,
  int vlimit) {
  int rc = calchullgen(verts, verts_count, vlimit);
  if (rc == 0) {
   return 0;
  }
  ArrayIntList ts = new ArrayIntList();
  int i;
  for (i = 0; i < m_tris.size(); i++) {
   btHullTriangle triangle = m_tris.get(i);
   if (triangle != null) {
    for (int j = 0; j < 3; j++) {
     ts.add(triangle.getElement(j));
    }
    deAllocateTriangle(triangle);
   }
  }
  tris_count[0] = ts.size() / 3;
  tris_out.addAll(ts);
  m_tris.clear();
  m_tris.trimToSize();
  return 1;
 }

 void ReleaseHull(PHullResult result) {
  if (!result.m_Indices.isEmpty()) {
   result.m_Indices.clear();
  }
  result.mVcount = 0;
  result.mIndexCount = 0;
  result.mVertices = null;
 }

 int maxdirsterid(btVector3[] p, int count, final btVector3 dir, int[] allow) {
  int m = -1;
  while (m == -1) {
   m = maxdirfiltered(p, count, dir, allow);
   if (allow[m] == 3) {
    return m;
   }
   final btVector3 u = orth(dir);
   final btVector3 v = new btVector3(u).cross(dir);
   int ma = -1;
   for (float x = (0.0f); x <= (360.0f); x += (45.0f)) {
    float s = btSin(SIMD_RADS_PER_DEG * (x));
    float c = btCos(SIMD_RADS_PER_DEG * (x));
    int mb = maxdirfiltered(p, count,
     new btVector3(dir)
      .add((new btVector3(u)
       .scale(s))
       .add(new btVector3(v)
        .scale(c))
       .scale(0.025f)),
     allow);
    if (ma == m && mb == m) {
     allow[m] = 3;
     return m;
    }
    if (ma != -1 && ma != mb) // Yuck - this is really ugly
    {
     int mc = ma;
     for (float xx = x - (40.0f); xx <= x; xx += (5.0f)) {
      s = btSin(SIMD_RADS_PER_DEG * (xx));
      c = btCos(SIMD_RADS_PER_DEG * (xx));
      int md = maxdirfiltered(p, count,
       new btVector3(dir)
        .add((new btVector3(u)
         .scale(s))
         .add(new btVector3(v)
          .scale(c))
         .scale(0.025f)),
       allow);
      if (mc == m && md == m) {
       allow[m] = 3;
       return m;
      }
      mc = md;
     }
    }
    ma = mb;
   }
   allow[m] = 0;
   m = -1;
  }
  assert (false);
  return m;
 }

 btVector3 orth(final btVector3 v) {
  final btVector3 a = new btVector3(v).cross(new btVector3(0, 0, 1));
  final btVector3 b = new btVector3(v).cross(new btVector3(0, 1, 0));
  if (a.length() > b.length()) {
   return a.normalize();
  } else {
   return b.normalize();
  }
 }

 int maxdirfiltered(btVector3[] p, int count, final btVector3 dir, int[] allow) {
  assert (count != 0);
  int m = -1;
  for (int i = 0; i < count; i++) {
   if (allow[i] != 0) {
    if (m == -1 || (p[i].dot(dir)) > (p[m].dot(dir))) {
     m = i;
    }
   }
  }
  assert (m != -1);
  return m;
 }

 int above(btVector3[] vertices, Int3 t, final btVector3 p, float epsilon) {
  final btVector3 n = TriNormal(vertices[t.x], vertices[t.y], vertices[t.z]);
  return ((n.dot(new btVector3(p).sub(vertices[t.x]))) > epsilon) ? 1 : 0; // EPSILON???
 }

 btVector3 TriNormal(final btVector3 v0, final btVector3 v1, final btVector3 v2) {
  // return the normal of the triangle
  // inscribed by v0, v1, and v2
  final btVector3 cp = (new btVector3(v1).sub(v0)).cross(new btVector3(v2).sub(
   v1));
  float m = cp.length();
  if (m == 0) {
   return new btVector3(1, 0, 0);
  }
  return cp.scale(1.0f / m);
 }

 boolean hasvert(Int3 t, int v) {
  return (t.x == v || t.y == v || t.z == v);
 }

}
