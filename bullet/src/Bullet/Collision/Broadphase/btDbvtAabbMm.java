/*
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/
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
///btDbvt implementation by Nathanael Presson
package Bullet.Collision.Broadphase;

import static Bullet.LinearMath.btScalar.btFabs;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.btDot;
import java.io.Serializable;
import javax.vecmath.Tuple2f;

/**
 *
 * @author Gregery Barton
 */
public class btDbvtAabbMm implements Serializable {

 static btDbvtAabbMm fromCE(final btVector3 c, final btVector3 e) {
  btDbvtAabbMm box = new btDbvtAabbMm();
  box.mi.set(new btVector3(c).sub(e));
  box.mx.set(new btVector3(c).sub(e));
  return (box);
 }

 public static btDbvtAabbMm fromCR(final btVector3 c, float r) {
  return (fromCE(c, new btVector3(r, r, r)));
 }

 public static btDbvtAabbMm fromMM(final btVector3 mi, final btVector3 mx) {
  btDbvtAabbMm box = new btDbvtAabbMm();
  box.mi.set(mi);
  box.mx.set(mx);
  return (box);
 }

 public static btDbvtAabbMm fromPoints(btVector3[] pts, int n) {
  btDbvtAabbMm box = new btDbvtAabbMm();
  box.mi.set(pts[0]);
  box.mx.set(pts[0]);
  for (int i = 1; i < n; ++i) {
   box.mi.setMin(pts[i]);
   box.mx.setMax(pts[i]);
  }
  return (box);
 }

 public static boolean intersect(btDbvtAabbMm a,
  btDbvtAabbMm b) {
  return ((a.mi.x() <= b.mx.x()) && (a.mx.x() >= b.mi.x()) && (a.mi.y() <= b.mx
   .y()) && (a.mx.y() >= b.mi.y()) && (a.mi.z() <= b.mx.z()) && (a.mx.z()
   >= b.mi.z()));
 }

 public static boolean intersect(btDbvtAabbMm a, final btVector3 b) {
  return ((b.x() >= a.mi.x()) && (b.y() >= a.mi.y()) && (b.z() >= a.mi.z())
   && (b.x() <= a.mx.x()) && (b.y() <= a.mx.y()) && (b.z() <= a.mx.z()));
 }

 public static float proximity(btDbvtAabbMm a,
  btDbvtAabbMm b) {
  final btVector3 d = new btVector3(a.mi).add(a.mx).sub(b.mi).sub(b.mx);
  return (btFabs(d.x()) + btFabs(d.y()) + btFabs(d.z()));
 }

 public static int select(btDbvtAabbMm o,
  btDbvtAabbMm a,
  btDbvtAabbMm b) {
  return (proximity(o, a) < proximity(o, b) ? 0 : 1);
 }

 public static void merge(btDbvtAabbMm a,
  btDbvtAabbMm b,
  btDbvtAabbMm r) {
  if (a.mi.x < b.mi.x) {
   r.mi.x = a.mi.x;
  } else {
   r.mi.x = b.mi.x;
  }
  if (a.mx.x > b.mx.x) {
   r.mx.x = a.mx.x;
  } else {
   r.mx.x = b.mx.x;
  }
  if (a.mi.y < b.mi.y) {
   r.mi.y = a.mi.y;
  } else {
   r.mi.y = b.mi.y;
  }
  if (a.mx.y > b.mx.y) {
   r.mx.y = a.mx.y;
  } else {
   r.mx.y = b.mx.y;
  }
  if (a.mi.z < b.mi.z) {
   r.mi.z = a.mi.z;
  } else {
   r.mi.z = b.mi.z;
  }
  if (a.mx.z > b.mx.z) {
   r.mx.z = a.mx.z;
  } else {
   r.mx.z = b.mx.z;
  }
 }

 public static boolean notEqual(btDbvtAabbMm a,
  btDbvtAabbMm b) {
  return ((a.mi.x() != b.mi.x()) || (a.mi.y() != b.mi.y()) || (a.mi.z() != b.mi
   .z()) || (a.mx.x() != b.mx.x()) || (a.mx.y() != b.mx.y()) || (a.mx.z()
   != b.mx.z()));
 }

 final btVector3 mi = new btVector3();
 final btVector3 mx = new btVector3();

 public btVector3 center() {
  return new btVector3(mi).add(mx).scale(0.5f);
 }

 public btVector3 lengths() {
  return new btVector3(mx).sub(mi);
 }

 btVector3 extents() {
  return new btVector3(mx).sub(mi).scale(0.5f);
 }

 public btVector3 mins() {
  return (new btVector3(mi));
 }

 public btVector3 maxs() {
  return (new btVector3(mx));
 }

 public void expand(final btVector3 e) {
  mi.sub(e);
  mx.add(e);
 }

 public void signedExpand(final btVector3 e) {
  if (e.x() > 0) {
   mx.setX(mx.x() + e.x);
  } else {
   mi.setX(mi.x() + e.x);
  }
  if (e.y() > 0) {
   mx.setY(mx.y() + e.y);
  } else {
   mi.setY(mi.y() + e.y);
  }
  if (e.z() > 0) {
   mx.setZ(mx.z() + e.z);
  } else {
   mi.setZ(mi.z() + e.z);
  }
 }

 public boolean contain(btDbvtAabbMm a) {
  return ((mi.x() <= a.mi.x()) && (mi.y() <= a.mi.y()) && (mi.z() <= a.mi.z())
   && (mx.x() >= a.mx.x()) && (mx.y() >= a.mx.y()) && (mx.z() >= a.mx.z()));
 }

 public int classify(final btVector3 n, float o, int s) {
  final btVector3 pi = new btVector3();
  final btVector3 px = new btVector3();
  switch (s) {
   case (0 + 0 + 0):
    px.set(mi.x(), mi.y(), mi.z());
    pi.set(mx.x(), mx.y(), mx.z());
    break;
   case (1 + 0 + 0):
    px.set(mx.x(), mi.y(), mi.z());
    pi.set(mi.x(), mx.y(), mx.z());
    break;
   case (0 + 2 + 0):
    px.set(mi.x(), mx.y(), mi.z());
    pi.set(mx.x(), mi.y(), mx.z());
    break;
   case (1 + 2 + 0):
    px.set(mx.x(), mx.y(), mi.z());
    pi.set(mi.x(), mi.y(), mx.z());
    break;
   case (0 + 0 + 4):
    px.set(mi.x(), mi.y(), mx.z());
    pi.set(mx.x(), mx.y(), mi.z());
    break;
   case (1 + 0 + 4):
    px.set(mx.x(), mi.y(), mx.z());
    pi.set(mi.x(), mx.y(), mi.z());
    break;
   case (0 + 2 + 4):
    px.set(mi.x(), mx.y(), mx.z());
    pi.set(mx.x(), mi.y(), mi.z());
    break;
   case (1 + 2 + 4):
    px.set(mx.x(), mx.y(), mx.z());
    pi.set(mi.x(), mi.y(), mi.z());
    break;
  }
  if ((btDot(n, px) + o) < 0) {
   return (-1);
  }
  if ((btDot(n, pi) + o) >= 0) {
   return (+1);
  }
  return (0);
 }

 public float projectMinimum(final btVector3 v, int signs) {
  btVector3[] b = new btVector3[]{mx, mi};
  final btVector3 p = new btVector3(b[(signs) & 1].x(),
   b[(signs >> 1) & 1].y(),
   b[(signs >> 2) & 1].z());
  return (btDot(p, v));
 }

 public btVector3 tMins() {
  return (new btVector3(mi));
 }

 public btVector3 tMaxs() {
  return (new btVector3(mx));
 }

 public void addSpan(final btVector3 d, Tuple2f smix) {
  if (d.x < 0) {
   smix.x += mx.x * d.x;
   smix.y += mi.x * d.x;
  } else {
   smix.x += mi.x * d.x;
   smix.y += mx.x * d.x;
  }
  if (d.y < 0) {
   smix.x += mx.y * d.y;
   smix.y += mi.y * d.y;
  } else {
   smix.x += mi.y * d.y;
   smix.y += mx.y * d.y;
  }
  if (d.z < 0) {
   smix.x += mx.z * d.z;
   smix.y += mi.z * d.z;
  } else {
   smix.x += mi.z * d.z;
   smix.y += mx.z * d.z;
  }
 }

 public void set(btDbvtAabbMm volume) {
  mi.set(volume.mi);
  mx.set(volume.mx);
 }

};
