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

import Bullet.Collision.GJK.sSV;
import static Bullet.Extras.btMinMax.btMax;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.btDot;
import static Bullet.LinearMath.btVector3.det;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class EPA implements Serializable {

 static final int EPA_MAX_VERTICES = 128;
 static final int EPA_MAX_ITERATIONS = 255;
 static final float EPA_ACCURACY = 0.0001f;
 static final float EPA_PLANE_EPS = 0.00001f;
 static final float EPA_INSIDE_EPS = 0.01f;
 static final float EPA_FALLBACK = (10f * EPA_ACCURACY);
 static final int EPA_MAX_FACES = (EPA_MAX_VERTICES * 2);

 /*
  * Types
  */
 static class sFace {

  final btVector3 n = new btVector3();
  float d;
  final GJK.sSV[] c;
  final sFace[] f;
  final sFace[] l;
  final int[] e;
  int pass;

  sFace() {
   this.e = new int[3];
   this.l = new sFace[2];
   this.f = new sFace[3];
   this.c = new GJK.sSV[3];
  }

  sFace(sFace o) {
   this();
   assert (e.length == o.e.length);
   assert (l.length == o.l.length);
   assert (f.length == o.f.length);
   assert (c.length == o.c.length);
   System.arraycopy(o.e, 0, e, 0, e.length);
   System.arraycopy(o.l, 0, l, 0, l.length);
   System.arraycopy(o.f, 0, f, 0, f.length);
   System.arraycopy(o.c, 0, c, 0, c.length);
   n.set(o.n);
   d = o.d;
   pass = o.pass;
  }

 };

 static class sList {

  sFace root;
  int count;

  sList() {
   root = null;
   count = 0;
  }

 };

 static class sHorizon {

  sFace cf;
  sFace ff;
  int nf;

  sHorizon() {
   cf = null;
   ff = null;
   nf = 0;
  }

 };

 static enum eStatus {
  Valid,
  Touching,
  Degenerated,
  NonConvex,
  InvalidHull,
  OutOfFaces,
  OutOfVertices,
  AccuraryReached,
  FallBack,
  Failed
 }

 /*
  * Fields
  */
 eStatus m_status;
 final GJK.sSimplex m_result = new GJK.sSimplex();
 final btVector3 m_normal = new btVector3();
 float m_depth;
 final GJK.sSV[] m_sv_store = new GJK.sSV[EPA_MAX_VERTICES];
 final sFace[] m_fc_store = new sFace[EPA_MAX_FACES];
 int m_nextsv;
 final sList m_hull = new sList();
 final sList m_stock = new sList();

 /*
  * Methods
  */
 EPA() {
  initialize();
 }

 static void bind(sFace fa, int ea, sFace fb, int eb) {
  fa.e[ea] = eb;
  fa.f[ea] = fb;
  fb.e[eb] = ea;
  fb.f[eb] = fa;
 }

 static void append(sList list, sFace face) {
  face.l[0] = null;
  face.l[1] = list.root;
  if (list.root != null) {
   list.root.l[0] = face;
  }
  list.root = face;
  ++list.count;
 }

 static void remove(sList list, sFace face) {
  if (face.l[1] != null) {
   face.l[1].l[0] = face.l[0];
  }
  if (face.l[0] != null) {
   face.l[0].l[1] = face.l[1];
  }
  if (face == list.root) {
   list.root = face.l[1];
  }
  --list.count;
 }

 final void initialize() {
  m_status = eStatus.Failed;
  m_depth = 0;
  m_nextsv = 0;
  for (int i = 0; i < m_sv_store.length; ++i) {
   m_sv_store[i] = new GJK.sSV();
  }
  for (int i = 0; i < m_fc_store.length; ++i) {
   m_fc_store[i] = new sFace();
  }
  for (int i = 0; i < EPA_MAX_FACES; ++i) {
   append(m_stock, m_fc_store[EPA_MAX_FACES - i - 1]);
  }
 }

 eStatus evaluate(GJK gjk, final btVector3 guess) {
  GJK.sSimplex simplex = gjk.m_simplex;
  if ((simplex.rank > 1) && gjk.encloseOrigin()) {

   /*
    * Clean up
    */
   while (m_hull.root != null) {
    sFace f = m_hull.root;
    remove(m_hull, f);
    append(m_stock, f);
   }
   m_status = eStatus.Valid;
   m_nextsv = 0;
   /*
    * Orient simplex
    */
   if (det(new btVector3(simplex.c[0].w).sub(simplex.c[3].w),
    new btVector3(simplex.c[1].w).sub(simplex.c[3].w),
    new btVector3(simplex.c[2].w).sub(simplex.c[3].w)) < 0) {
    {
     sSV swapper = simplex.c[0];
     simplex.c[0] = simplex.c[1];
     simplex.c[1] = swapper;
    }
    {
     float swapper = simplex.p[0];
     simplex.p[0] = simplex.p[1];
     simplex.p[1] = swapper;
    }
   }
   /*
    * Build initial hull
    */
   sFace[] tetra = new sFace[]{newface(simplex.c[0], simplex.c[1], simplex.c[2],
    true),
    newface(simplex.c[1], simplex.c[0], simplex.c[3], true),
    newface(simplex.c[2], simplex.c[1], simplex.c[3], true),
    newface(simplex.c[0], simplex.c[2], simplex.c[3], true)};
   if (m_hull.count == 4) {
    sFace best = findbest();
    sFace outer = new sFace(best);
    int pass = 0;
    int iterations = 0;
    bind(tetra[0], 0, tetra[1], 0);
    bind(tetra[0], 1, tetra[2], 0);
    bind(tetra[0], 2, tetra[3], 0);
    bind(tetra[1], 1, tetra[3], 2);
    bind(tetra[1], 2, tetra[2], 1);
    bind(tetra[2], 2, tetra[3], 1);
    m_status = eStatus.Valid;
    for (; iterations < EPA_MAX_ITERATIONS; ++iterations) {
     if (m_nextsv < EPA_MAX_VERTICES) {
      sHorizon horizon = new sHorizon();
      sSV w = m_sv_store[m_nextsv++];
      boolean valid = true;
      best.pass = (++pass);
      gjk.getsupport(best.n, w);
      float wdist = btDot(best.n, w.w) - best.d;
      if (wdist > EPA_ACCURACY) {
       for (int j = 0; (j < 3) && valid; ++j) {
        valid = valid && expand(pass, w,
         best.f[j], best.e[j],
         horizon);
       }
       if (valid && (horizon.nf >= 3)) {
        bind(horizon.cf, 1, horizon.ff, 2);
        remove(m_hull, best);
        append(m_stock, best);
        best = findbest();
        outer = new sFace(best);
       } else {
        m_status = eStatus.InvalidHull;
        break;
       }
      } else {
       m_status = eStatus.AccuraryReached;
       break;
      }
     } else {
      m_status = eStatus.OutOfVertices;
      break;
     }
    }
    final btVector3 projection = new btVector3(outer.n).scale(outer.d);
    m_normal.set(outer.n);
    m_depth = outer.d;
    m_result.rank = 3;
    m_result.c[0] = outer.c[0];
    m_result.c[1] = outer.c[1];
    m_result.c[2] = outer.c[2];
    m_result.p[0] = new btVector3(outer.c[1].w).sub(projection).cross(
     new btVector3(outer.c[2].w).sub(projection)).length();
    m_result.p[1] = new btVector3(outer.c[2].w).sub(projection).cross(
     new btVector3(outer.c[0].w).sub(projection)).length();
    m_result.p[2] = new btVector3(outer.c[0].w).sub(projection).cross(
     new btVector3(outer.c[1].w).sub(projection)).length();
    float sum = m_result.p[0] + m_result.p[1] + m_result.p[2];
    m_result.p[0] /= sum;
    m_result.p[1] /= sum;
    m_result.p[2] /= sum;
    return (m_status);
   }
  }
  /*
   * Fallback
   */
  m_status = eStatus.FallBack;
  m_normal.set(guess).negate();
  float nl = m_normal.length();
  if (nl > 0) {
   m_normal.scale(1.0f / nl);
  } else {
   m_normal.set(1, 0, 0);
  }
  m_depth = 0;
  m_result.rank = 1;
  m_result.c[0] = simplex.c[0];
  m_result.p[0] = 1;
  return (m_status);
 }

 boolean getedgedist(sFace face, sSV a, sSV b, float[] dist) {
  final btVector3 ba = new btVector3(b.w).sub(a.w);
  final btVector3 n_ab = new btVector3(ba).cross(face.n); // Outward facing edge normal direction, on triangle plane
  float a_dot_nab = btDot(a.w, n_ab); // Only care about the sign to determine inside/outside, so not normalization required
  if (a_dot_nab < 0) {
   // Outside of edge a.b
   float ba_l2 = ba.lengthSquared();
   float a_dot_ba = btDot(a.w, ba);
   float b_dot_ba = btDot(b.w, ba);
   if (a_dot_ba > 0) {
    // Pick distance vertex a
    dist[0] = a.w.length();
   } else if (b_dot_ba < 0) {
    // Pick distance vertex b
    dist[0] = b.w.length();
   } else {
    // Pick distance to edge a.b
    float a_dot_b = btDot(a.w, b.w);
    dist[0] = btSqrt(btMax((a.w.lengthSquared() * b.w.lengthSquared() - a_dot_b
     * a_dot_b) / ba_l2,
     0f));
   }
   return true;
  }
  return false;
 }

 sFace newface(sSV a, sSV b, sSV c, boolean forced) {
  if (m_stock.root != null) {
   sFace face = m_stock.root;
   remove(m_stock, face);
   append(m_hull, face);
   face.pass = 0;
   face.c[0] = a;
   face.c[1] = b;
   face.c[2] = c;
   face.n.set(new btVector3(b.w).sub(a.w).cross(new btVector3(c.w).sub(a.w)));
   float l = face.n.length();
   boolean v = l > EPA_ACCURACY;
   if (v) {
    {
     float[] face_d = {face.d};
     if (!(getedgedist(face, a, b, face_d) || getedgedist(face, b, c, face_d)
      || getedgedist(face, c, a, face_d))) {
      // Origin projects to the interior of the triangle
      // Use distance to triangle plane
      face.d = btDot(a.w, face.n) / l;
     } else {
      face.d = face_d[0];
     }
    }
    face.n.scale(1.0f / l);
    if (forced || (face.d >= -EPA_PLANE_EPS)) {
     return face;
    } else {
     m_status = eStatus.NonConvex;
    }
   } else {
    m_status = eStatus.Degenerated;
   }
   remove(m_hull, face);
   append(m_stock, face);
   return null;
  }
  m_status = eStatus.OutOfFaces;
  return null;
 }

 sFace findbest() {
  sFace minf = m_hull.root;
  float mind = minf.d * minf.d;
  for (sFace f = minf.l[1]; f != null; f = f.l[1]) {
   float sqd = f.d * f.d;
   if (sqd < mind) {
    minf = f;
    mind = sqd;
   }
  }
  return (minf);
 }

 static final int[] i1m3 = new int[]{1, 2, 0};
 static final int[] i2m3 = new int[]{2, 0, 1};

 boolean expand(int pass, sSV w, sFace f, int e, sHorizon horizon) {
  if (f.pass != pass) {
   int e1 = i1m3[e];
   if ((btDot(f.n, w.w) - f.d) < -EPA_PLANE_EPS) {
    sFace nf = newface(f.c[e1], f.c[e], w, false);
    if (nf != null) {
     bind(nf, 0, f, e);
     if (horizon.cf != null) {
      bind(horizon.cf, 1, nf, 2);
     } else {
      horizon.ff = nf;
     }
     horizon.cf = nf;
     ++horizon.nf;
     return (true);
    }
   } else {
    int e2 = i2m3[e];
    f.pass = pass;
    if (expand(pass, w, f.f[e1], f.e[e1], horizon) && expand(pass, w, f.f[e2],
     f.e[e2], horizon)) {
     remove(m_hull, f);
     append(m_stock, f);
     return (true);
    }
   }
  }
  return (false);
 }

};
