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

package Bullet.Collision;

import static Bullet.LinearMath.btScalar.SIMD_2_PI;
import static Bullet.LinearMath.btScalar.SIMD_HALF_PI;
import static Bullet.LinearMath.btScalar.SIMD_PI;
import static Bullet.LinearMath.btScalar.SIMD_RADS_PER_DEG;
import static Bullet.LinearMath.btScalar.btCos;
import static Bullet.LinearMath.btScalar.btFabs;
import static Bullet.LinearMath.btScalar.btSin;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.btPlaneSpace1;
import static Bullet.LinearMath.btVector3.init;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public abstract class btIDebugDraw implements Serializable {

 /**
  *
  */
 public static final int DBG_NoDebug = 0;
 /**
  *
  */
 public static final int DBG_DrawWireframe = 1;
 /**
  *
  */
 public static final int DBG_DrawAabb = 2;
 /**
  *
  */
 public static final int DBG_DrawFeaturesText = 4;
 /**
  *
  */
 public static final int DBG_DrawContactPoints = 8;
 /**
  *
  */
 public static final int DBG_NoDeactivation = 16;
 /**
  *
  */
 public static final int DBG_NoHelpText = 32;
 /**
  *
  */
 public static final int DBG_DrawText = 64;
 /**
  *
  */
 public static final int DBG_ProfileTimings = 128;
 /**
  *
  */
 public static final int DBG_EnableSatComparison = 256;
 /**
  *
  */
 public static final int DBG_DisableBulletLCP = 512;
 /**
  *
  */
 public static final int DBG_EnableCCD = 1024;
 /**
  *
  */
 public static final int DBG_DrawConstraints = (1 << 11);
 /**
  *
  */
 public static final int DBG_DrawConstraintLimits = (1 << 12);
 /**
  *
  */
 public static final int DBG_FastWireframe = (1 << 13);
 /**
  *
  */
 public static final int DBG_DrawNormals = (1 << 14);
 /**
  *
  */
 public static final int DBG_DrawFrames = (1 << 15);
 /**
  *
  */
 public static final int DBG_MAX_DEBUG_DRAW_MODE = DBG_DrawFrames + 1;

 public DefaultColors getDefaultColors() {
  return new DefaultColors();
 }

 ///the default implementation for setDefaultColors has no effect. A derived class can implement it and store the colors.
 public void setDefaultColors(DefaultColors colors) {
 }

 public abstract void drawLine(final btVector3 from, final btVector3 to, final btVector3 color);

 public void drawLine(final btVector3 from, final btVector3 to, final btVector3 fromColor,
  final btVector3 toColor) {
  drawLine(from, to, fromColor);
 }

 public void drawSphere(float radius, final btTransform transform, final btVector3 color) {
  final btVector3 center = transform.getOrigin();
  final btVector3 up = transform.getBasisColumn(1);
  final btVector3 axis = transform.getBasisColumn(0);
  float minTh = -SIMD_HALF_PI;
  float maxTh = SIMD_HALF_PI;
  float minPs = -SIMD_HALF_PI;
  float maxPs = SIMD_HALF_PI;
  float stepDegrees = 30.f;
  drawSpherePatch(center, up, axis, radius, minTh, maxTh, minPs, maxPs, color, stepDegrees, false);
  drawSpherePatch(center, up, axis.negate(), radius, minTh, maxTh, minPs, maxPs, color, stepDegrees,
   false);
 }

 public void drawSphere(final btVector3 p, float radius, final btVector3 color) {
  final btTransform tr = new btTransform();
  tr.setIdentity();
  tr.setOrigin(p);
  drawSphere(radius, tr, color);
 }

 public void drawTriangle(final btVector3 v0, final btVector3 v1, final btVector3 v2,
  final btVector3 n0, final btVector3 n1, final btVector3 n2, final btVector3 color, float alpha) {
  drawTriangle(v0, v1, v2, color, alpha);
 }

 public void drawTriangle(final btVector3 v0, final btVector3 v1, final btVector3 v2,
  final btVector3 color, float alpha) {
  drawLine(v0, v1, color);
  drawLine(v1, v2, color);
  drawLine(v2, v0, color);
 }

 public abstract void drawContactPoint(final btVector3 PointOnB, final btVector3 normalOnB,
  float distance,
  int lifeTime, final btVector3 color);

 public abstract void reportErrorWarning(String warningString);

 public abstract void draw3dText(final btVector3 location, String textString);

 public abstract void setDebugMode(int debugMode);

 public abstract int getDebugMode();

 public void drawAabb(final btVector3 from, final btVector3 to, final btVector3 color) {
  final btVector3 halfExtents = new btVector3(to).sub(from).scale(0.5f);
  final btVector3 center = new btVector3(to).add(from).scale(0.5f);
  int i, j;
  final btVector3 edgecoord = new btVector3(1.f, 1.f, 1.f);
  final btVector3 pa = new btVector3();
  final btVector3 pb = new btVector3();
  for (i = 0; i < 4; i++) {
   for (j = 0; j < 3; j++) {
    pa.set(edgecoord.x * halfExtents.x, edgecoord.y * halfExtents.y,
     edgecoord.z * halfExtents.z);
    pa.add(center);
    int othercoord = j % 3;
    edgecoord.setElement(othercoord, edgecoord.getElement(othercoord) * -1.f);
    pb.set(edgecoord.x * halfExtents.x, edgecoord.y * halfExtents.y,
     edgecoord.z * halfExtents.z);
    pb.add(center);
    drawLine(pa, pb, color);
   }
   edgecoord.set(-1.f, -1.f, -1.f);
   if (i < 3) {
    edgecoord.setElement(i, edgecoord.getElement(i) * -1.f);
   }
  }
 }

 public void drawTransform(final btTransform transform, float orthoLen
 ) {
  final btVector3 start = transform.getOrigin();
  drawLine(start, new btVector3(start).add(transform.transform3x3(new btVector3(orthoLen, 0, 0))),
   new btVector3(1.f, 0.3f, 0.3f));
  drawLine(start, new btVector3(start).add(transform.transform3x3(new btVector3(0, orthoLen, 0))),
   new btVector3(0.3f, 1.f, 0.3f));
  drawLine(start, new btVector3(start).add(transform.transform3x3(new btVector3(0, 0, orthoLen))),
   new btVector3(0.3f, 0.3f, 1.f));
 }

 public void drawArc(final btVector3 center, final btVector3 normal, final btVector3 axis,
  float radiusA,
  float radiusB, float minAngle, float maxAngle, final btVector3 color, boolean drawSect) {
  drawArc(center, normal, axis, radiusA, radiusB, minAngle, maxAngle, color, drawSect, 1.0f);
 }

 public void drawArc(final btVector3 center, final btVector3 normal, final btVector3 axis,
  float radiusA,
  float radiusB, float minAngle, float maxAngle, final btVector3 color, boolean drawSect,
  float stepDegrees) {
  final btVector3 vx = axis;
  final btVector3 vy = new btVector3().cross(normal, axis);
  float step = stepDegrees * SIMD_RADS_PER_DEG;
  int nSteps = (int) btFabs((maxAngle - minAngle) / step);
  if (nSteps == 0) {
   nSteps = 1;
  }
  final btVector3 prev = new btVector3(center).add(new btVector3(vx)
   .scale(radiusA * btCos(minAngle)))
   .add(new btVector3(vy).scale(radiusB * btSin(minAngle)));
  final btVector3 next = new btVector3();
  if (drawSect) {
   drawLine(center, prev, color);
  }
  for (int i = 1; i <= nSteps; i++) {
   float angle = minAngle + (maxAngle - minAngle) * (float) (i) / (float) (nSteps);
   next.set(new btVector3(center).add(new btVector3(vx).scale(radiusA * btCos(angle))).add(
    new btVector3(vy).scale(radiusB * btSin(angle))));
   drawLine(prev, next, color);
   prev.set(next);
  }
  if (drawSect) {
   drawLine(center, prev, color);
  }
 }

 public void drawSpherePatch(final btVector3 center, final btVector3 up, final btVector3 axis,
  float radius,
  float minTh, float maxTh, float minPs, float maxPs, final btVector3 color) {
  drawSpherePatch(center, up, axis, radius, minTh, maxTh, minPs, maxPs, color, 10.0f);
 }

 public void drawSpherePatch(final btVector3 center, final btVector3 up, final btVector3 axis,
  float radius,
  float minTh, float maxTh, float minPs, float maxPs, final btVector3 color, float stepDegrees) {
  drawSpherePatch(center, up, axis, radius, minTh, maxTh, minPs, maxPs, color, stepDegrees,
   true);
 }

 public void drawSpherePatch(final btVector3 center, final btVector3 up, final btVector3 axis,
  float radius,
  float minTh, float maxTh, float minPs, float maxPs, final btVector3 color, float stepDegrees,
  boolean drawCenter) {
  float _minTh = minTh;
  float _maxTh = maxTh;
  float _minPs = minPs;
  float _maxPs = maxPs;
  btVector3[] vA = new btVector3[74];init(vA);
  btVector3[] vB = new btVector3[74];init(vB);
  btVector3[] pT;
  final btVector3 npole = new btVector3(center).add(new btVector3(up).scale(radius));
  final btVector3 spole = new btVector3(center).sub(new btVector3(up).scale(radius));
  final btVector3 arcStart = new btVector3();
  float step = stepDegrees * SIMD_RADS_PER_DEG;
  final btVector3 kv = up;
  final btVector3 iv = axis;
  final btVector3 jv = new btVector3().cross(kv, iv);
  boolean drawN = false;
  boolean drawS = false;
  if (_minTh <= -SIMD_HALF_PI) {
   _minTh = -SIMD_HALF_PI + step;
   drawN = true;
  }
  if (_maxTh >= SIMD_HALF_PI) {
   _maxTh = SIMD_HALF_PI - step;
   drawS = true;
  }
  if (_minTh > _maxTh) {
   _minTh = -SIMD_HALF_PI + step;
   _maxTh = SIMD_HALF_PI - step;
   drawN = drawS = true;
  }
  int n_hor = (int) ((_maxTh - _minTh) / step) + 1;
  if (n_hor < 2) {
   n_hor = 2;
  }
  float step_h = (_maxTh - _minTh) / (n_hor - 1);
  boolean isClosed;
  if (_minPs > _maxPs) {
   _minPs = -SIMD_PI + step;
   _maxPs = SIMD_PI;
   isClosed = true;
  } else {
   isClosed = (_maxPs - _minPs) >= SIMD_PI * (2.f);
  }
  int n_vert = (int) ((_maxPs - _minPs) / step) + 1;
  if (n_vert < 2) {
   n_vert = 2;
  }
  float step_v = (_maxPs - _minPs) / (n_vert - 1);
  for (int i = 0; i < n_hor; i++) {
   float th = _minTh + (float) (i) * step_h;
   float sth = radius * btSin(th);
   float cth = radius * btCos(th);
   for (int j = 0; j < n_vert; j++) {
    float psi = _minPs + (float) (j) * step_v;
    float sps = btSin(psi);
    float cps = btCos(psi);
    vB[j] .set(center).add(new btVector3(iv).scale(cth * cps)).add(new btVector3(jv)
     .scale(cth * sps)).add(new btVector3(kv).scale(sth));
    if (i != 0) {
     drawLine(vA[j], vB[j], color);
    } else if (drawS) {
     drawLine(spole, vB[j], color);
    }
    if (j != 0) {
     drawLine(vB[j - 1], vB[j], color);
    } else {
     arcStart.set(vB[j]);
    }
    if ((i == (n_hor - 1)) && drawN) {
     drawLine(npole, vB[j], color);
    }
    if (drawCenter) {
     if (isClosed) {
      if (j == (n_vert - 1)) {
       drawLine(arcStart, vB[j], color);
      }
     } else if (((i == 0) || (i == (n_hor - 1))) && ((j == 0) || (j == (n_vert - 1)))) {
      drawLine(center, vB[j], color);
     }
    }
   }
   pT = vA;
   vA = vB;
   vB = pT;
  }
 }

 public void drawBox(final btVector3 bbMin, final btVector3 bbMax, final btVector3 color) {
  drawLine(new btVector3(bbMin.x, bbMin.y, bbMin.z), new btVector3(bbMax.x, bbMin.y, bbMin.z), color);
  drawLine(new btVector3(bbMax.x, bbMin.y, bbMin.z), new btVector3(bbMax.x, bbMax.y, bbMin.z), color);
  drawLine(new btVector3(bbMax.x, bbMax.y, bbMin.z), new btVector3(bbMin.x, bbMax.y, bbMin.z), color);
  drawLine(new btVector3(bbMin.x, bbMax.y, bbMin.z), new btVector3(bbMin.x, bbMin.y, bbMin.z), color);
  drawLine(new btVector3(bbMin.x, bbMin.y, bbMin.z), new btVector3(bbMin.x, bbMin.y, bbMax.z), color);
  drawLine(new btVector3(bbMax.x, bbMin.y, bbMin.z), new btVector3(bbMax.x, bbMin.y, bbMax.z), color);
  drawLine(new btVector3(bbMax.x, bbMax.y, bbMin.z), new btVector3(bbMax.x, bbMax.y, bbMax.z), color);
  drawLine(new btVector3(bbMin.x, bbMax.y, bbMin.z), new btVector3(bbMin.x, bbMax.y, bbMax.z), color);
  drawLine(new btVector3(bbMin.x, bbMin.y, bbMax.z), new btVector3(bbMax.x, bbMin.y, bbMax.z), color);
  drawLine(new btVector3(bbMax.x, bbMin.y, bbMax.z), new btVector3(bbMax.x, bbMax.y, bbMax.z), color);
  drawLine(new btVector3(bbMax.x, bbMax.y, bbMax.z), new btVector3(bbMin.x, bbMax.y, bbMax.z), color);
  drawLine(new btVector3(bbMin.x, bbMax.y, bbMax.z), new btVector3(bbMin.x, bbMin.y, bbMax.z), color);
 }

 public void drawBox(final btVector3 bbMin, final btVector3 bbMax, final btTransform trans,
  final btVector3 color) {
  drawLine(trans.transform(new btVector3(bbMin.x, bbMin.y, bbMin.z)), trans.transform(new btVector3(
   bbMax.x, bbMin.y, bbMin.z)), color);
  drawLine(trans.transform(new btVector3(bbMax.x, bbMin.y, bbMin.z)), trans.transform(new btVector3(
   bbMax.x, bbMax.y, bbMin.z)), color);
  drawLine(trans.transform(new btVector3(bbMax.x, bbMax.y, bbMin.z)), trans.transform(new btVector3(
   bbMin.x, bbMax.y, bbMin.z)), color);
  drawLine(trans.transform(new btVector3(bbMin.x, bbMax.y, bbMin.z)), trans.transform(new btVector3(
   bbMin.x, bbMin.y, bbMin.z)), color);
  drawLine(trans.transform(new btVector3(bbMin.x, bbMin.y, bbMin.z)), trans.transform(new btVector3(
   bbMin.x, bbMin.y, bbMax.z)), color);
  drawLine(trans.transform(new btVector3(bbMax.x, bbMin.y, bbMin.z)), trans.transform(new btVector3(
   bbMax.x, bbMin.y, bbMax.z)), color);
  drawLine(trans.transform(new btVector3(bbMax.x, bbMax.y, bbMin.z)), trans.transform(new btVector3(
   bbMax.x, bbMax.y, bbMax.z)), color);
  drawLine(trans.transform(new btVector3(bbMin.x, bbMax.y, bbMin.z)), trans.transform(new btVector3(
   bbMin.x, bbMax.y, bbMax.z)), color);
  drawLine(trans.transform(new btVector3(bbMin.x, bbMin.y, bbMax.z)), trans.transform(new btVector3(
   bbMax.x, bbMin.y, bbMax.z)), color);
  drawLine(trans.transform(new btVector3(bbMax.x, bbMin.y, bbMax.z)), trans.transform(new btVector3(
   bbMax.x, bbMax.y, bbMax.z)), color);
  drawLine(trans.transform(new btVector3(bbMax.x, bbMax.y, bbMax.z)), trans.transform(new btVector3(
   bbMin.x, bbMax.y, bbMax.z)), color);
  drawLine(trans.transform(new btVector3(bbMin.x, bbMax.y, bbMax.z)), trans.transform(new btVector3(
   bbMin.x, bbMin.y, bbMax.z)), color);
 }
                                   
 public void drawCapsule(float radius, float halfHeight, int upAxis, final btTransform transform,
  final btVector3 color) {
  int stepDegrees = 30;
  final btVector3 capStart = new btVector3(0.f, 0.f, 0.f);
  capStart.setElement(upAxis, -halfHeight);
  final btVector3 capEnd = new btVector3(0.f, 0.f, 0.f);
  capEnd.setElement(upAxis, halfHeight);
  // Draw the ends
  {
   final btTransform childTransform = new btTransform(transform);
   childTransform.setOrigin(transform.transform(new btVector3(capStart)));
   {
    final btVector3 center = childTransform.getOrigin();
    final btVector3 up = childTransform.getBasisColumn((upAxis + 1) % 3);
    final btVector3 axis = childTransform.getBasisColumn(upAxis).negate();
    float minTh = -SIMD_HALF_PI;
    float maxTh = SIMD_HALF_PI;
    float minPs = -SIMD_HALF_PI;
    float maxPs = SIMD_HALF_PI;
    drawSpherePatch(center, up, axis, radius, minTh, maxTh, minPs, maxPs, color, (stepDegrees),
     false);
   }
  }
  {
   final btTransform childTransform = new btTransform(transform);
   childTransform.setOrigin(transform.transform(new btVector3(capEnd)));
   {
    final btVector3 center = childTransform.getOrigin();
    final btVector3 up = childTransform.getBasisColumn((upAxis + 1) % 3);
    final btVector3 axis = childTransform.getBasisColumn(upAxis);
    float minTh = -SIMD_HALF_PI;
    float maxTh = SIMD_HALF_PI;
    float minPs = -SIMD_HALF_PI;
    float maxPs = SIMD_HALF_PI;
    drawSpherePatch(center, up, axis, radius, minTh, maxTh, minPs, maxPs, color, (stepDegrees),
     false);
   }
  }
  // Draw some additional lines
  final btVector3 start = transform.getOrigin();
  for (int i = 0; i < 360; i += stepDegrees) {
   float s = btSin((i) * SIMD_RADS_PER_DEG) * radius;
   capEnd.setElement((upAxis + 1) % 3, s);
   capStart.setElement((upAxis + 1) % 3, s);
   float c = btCos((i) * SIMD_RADS_PER_DEG) * radius;
   capEnd.setElement((upAxis + 2) % 3, c);
   capStart.setElement((upAxis + 2) % 3, c);
   drawLine(
    new btVector3(start)
    .add(transform.transform3x3(new btVector3(capStart))),
    new btVector3(start)
    .add(transform.transform3x3(new btVector3(capEnd))),
    color);
  }
 }

 public void drawCylinder(float radius, float halfHeight, int upAxis, final btTransform transform,
  final btVector3 color) {
  final btVector3 start = transform.getOrigin();
  final btVector3 offsetHeight = new btVector3();
  offsetHeight.setElement(upAxis, halfHeight);
  int stepDegrees = 30;
  final btVector3 capStart = new btVector3(0.f, 0.f, 0.f);
  capStart.setElement(upAxis, -halfHeight);
  final btVector3 capEnd = new btVector3(0.f, 0.f, 0.f);
  capEnd.setElement(upAxis, halfHeight);
  for (int i = 0; i < 360; i += stepDegrees) {
   float s = btSin((i) * SIMD_RADS_PER_DEG) * radius;
   capEnd.setElement((upAxis + 1) % 3, s);
   capStart.setElement((upAxis + 1) % 3, s);
   float c = btCos((i) * SIMD_RADS_PER_DEG) * radius;
   capEnd.setElement((upAxis + 2) % 3, c);
   capStart.setElement((upAxis + 2) % 3, c);
   drawLine(new btVector3(start).add(transform.transform3x3(new btVector3(capStart))),
    new btVector3(start).add(transform.transform3x3(new btVector3(capEnd))), color);
  }
  // Drawing top and bottom caps of the cylinder
  final btVector3 yaxis = new btVector3();
  yaxis.setElement(upAxis, (1.0f));
  final btVector3 xaxis = new btVector3();
  xaxis.setElement((upAxis + 1) % 3, (1.0f));
  drawArc(
   new btVector3(start).sub(transform.transform3x3(new btVector3(offsetHeight))),
   transform.transform3x3(new btVector3(yaxis)),
   transform.transform3x3(new btVector3(xaxis)),
   radius,
   radius,
   0,
   SIMD_2_PI, color, false, 10.0f);
  drawArc(
   new btVector3(start).add(transform.transform3x3(new btVector3(offsetHeight))),
   transform.transform3x3(new btVector3(yaxis)),
   transform.transform3x3(new btVector3(xaxis)),
   radius,
   radius,
   0,
   SIMD_2_PI,
   color,
   false,
   10.0f);
 }

 public void drawCone(float radius, float height, int upAxis, final btTransform transform,
  final btVector3 color) {
  int stepDegrees = 30;
  final btVector3 start = transform.getOrigin();
  final btVector3 offsetHeight = new btVector3(0, 0, 0);
  float halfHeight = height * 0.5f;
  offsetHeight.setElement(upAxis, halfHeight);
  final btVector3 offsetRadius = new btVector3(0, 0, 0);
  offsetRadius.setElement((upAxis + 1) % 3, radius);
  final btVector3 offset2Radius = new btVector3(0, 0, 0);
  offset2Radius.setElement((upAxis + 2) % 3, radius);
  final btVector3 capEnd = new btVector3(0.f, 0.f, 0.f);
  capEnd.setElement(upAxis, -halfHeight);
  for (int i = 0; i < 360; i += stepDegrees) {
   float s = btSin((i) * SIMD_RADS_PER_DEG) * radius;
   capEnd.setElement((upAxis + 1) % 3, s);
   float c = btCos((i) * SIMD_RADS_PER_DEG) * radius;
   capEnd.setElement((upAxis + 2) % 3, c);
   drawLine(
    new btVector3(start).add(transform.transform3x3(new btVector3(offsetHeight))),
    new btVector3(start).add(transform.transform3x3(new btVector3(capEnd))),
    color);
  }
  drawLine(
   new btVector3(start).add(transform.transform3x3(new btVector3(offsetHeight))),
   new btVector3(start).add(transform.transform3x3(new btVector3(offsetHeight).negate().add(
    offsetRadius))),
   color);
  drawLine(
   new btVector3(start).add(transform.transform3x3(new btVector3(offsetHeight))),
   new btVector3(start).add(transform.transform3x3(new btVector3(offsetHeight).negate().sub(
    offsetRadius))),
   color);
  drawLine(
   new btVector3(start).add(transform.transform3x3(new btVector3(offsetHeight))),
   new btVector3(start).add(transform.transform3x3(new btVector3(offsetHeight).negate().add(
    offset2Radius))),
   color);
  drawLine(
   new btVector3(start).add(transform.transform3x3(new btVector3(offsetHeight))),
   new btVector3(start).add(transform.transform3x3(new btVector3(offsetHeight).negate().sub(
    offset2Radius))),
   color);
  // Drawing the base of the cone
  final btVector3 yaxis = new btVector3(0, 0, 0);
  yaxis.setElement(upAxis, (1.0f));
  final btVector3 xaxis = new btVector3(0, 0, 0);
  xaxis.setElement((upAxis + 1) % 3, (1.0f));
  drawArc(
   new btVector3(start).sub(transform.transform3x3(new btVector3(offsetHeight))),
   transform.transform3x3(new btVector3(yaxis)),
   transform.transform3x3(new btVector3(xaxis)),
   radius,
   radius,
   0,
   SIMD_2_PI,
   color,
   false,
   10.0f);
 }

 public void drawPlane(final btVector3 planeNormal, float planeConst, final btTransform transform,
  final btVector3 color) {
  final btVector3 planeOrigin = new btVector3(planeNormal).scale(planeConst);
  final btVector3 vec0 = new btVector3();
  final btVector3 vec1 = new btVector3();
  btPlaneSpace1(planeNormal, vec0, vec1);
  float vecLen = 100.f;
  final btVector3 pt0 = new btVector3(vec0).scaleAdd(vecLen, planeOrigin);
  final btVector3 pt1 = new btVector3(vec0).negate().scaleAdd(vecLen, planeOrigin);
  final btVector3 pt2 = new btVector3(vec1).scaleAdd(vecLen, planeOrigin);
  final btVector3 pt3 = new btVector3(vec1).negate().scaleAdd(vecLen, planeOrigin);
  drawLine(transform.transform(pt0), transform.transform(pt1), color);
  drawLine(transform.transform(pt2), transform.transform(pt3), color);
 }

 public void flushLines() {
 }
};
