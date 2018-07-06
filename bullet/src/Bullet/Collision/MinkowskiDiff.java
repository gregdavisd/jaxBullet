/*
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://continuousphysics.com/Bullet/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the
 * use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely,
 * subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software in a
 * product, an acknowledgment in the product documentation would be appreciated
 * but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

 /*
 * GJK-EPA collision solver by Nathanael Presson, 2008
 */
package Bullet.Collision;

import Bullet.Collision.Shape.btConvexShape;
import Bullet.LinearMath.btMatrix3x3;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class MinkowskiDiff implements Serializable {

 final btConvexShape[] m_shapes = new btConvexShape[2];
 final btMatrix3x3 m_toshape1 = new btMatrix3x3();
 final btTransform m_toshape0 = new btTransform();
 boolean m_enableMargin;

 MinkowskiDiff() {
 }

 //btVector3				(btConvexShape::*Ls)(  btVector3&)  ;
 btVector3 Ls(btConvexShape shape, final btVector3 v) {
  if (m_enableMargin) {
   return shape.localGetSupportVertexNonVirtual(v);
  } else {
   return shape.localGetSupportVertexWithoutMarginNonVirtual(v);
  }
 }

 void enableMargin(boolean enable) {
  m_enableMargin = enable;
 }

 btVector3 support0(final btVector3 d) {
  return Ls(m_shapes[0], d);
  //return(((m_shapes[0]).*(Ls))(d));
 }

 btVector3 support1(final btVector3 d) {
  return m_toshape0.transform(Ls(m_shapes[1], m_toshape1.transform(
   new btVector3(d))));
  //return(m_toshape0*Ls(m_shapes[1]).*(Ls))(m_toshape1*d));
 }

 btVector3 support(final btVector3 d) {
  return (support0(d).sub(support1(new btVector3(d).negate())));
 }

 btVector3 support(final btVector3 d, int index) {
  if (index != 0) {
   return (support1(d));
  } else {
   return (support0(d));
  }
 }

 public void set(MinkowskiDiff copy) {
  m_enableMargin = copy.m_enableMargin;
  System.arraycopy(copy.m_shapes, 0, m_shapes, 0, m_shapes.length);
  m_toshape0.set(copy.m_toshape0);
  m_toshape1.set(copy.m_toshape1);
 }

};
