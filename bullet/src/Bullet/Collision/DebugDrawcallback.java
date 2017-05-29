/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

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

import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class DebugDrawcallback implements btTriangleCallback, Serializable {

 final btIDebugDraw m_debugDrawer;
 final btVector3 m_color = new btVector3();
 final btTransform m_worldTrans = new btTransform();

 DebugDrawcallback(btIDebugDraw debugDrawer, final btTransform worldTrans, final btVector3 color) {
  m_debugDrawer = debugDrawer;
  m_color.set(color);
  m_worldTrans.set(worldTrans);
 }

 /**
  *
  * @param triangle
  * @param partId
  * @param triangleIndex
  */
 @Override
 public boolean processTriangle(btVector3[] triangle, int partId, int triangleIndex) {
  final btVector3 wv0;
  final btVector3 wv1;
  final btVector3 wv2;
  wv0 = m_worldTrans.transform(new btVector3(triangle[0]));
  wv1 = m_worldTrans.transform(new btVector3(triangle[1]));
  wv2 = m_worldTrans.transform(new btVector3(triangle[2]));
  final btVector3 center = new btVector3(wv0).add(wv1).add(wv2).scale(1.f / 3.f);
  if ((m_debugDrawer.getDebugMode() & btIDebugDraw.DBG_DrawNormals) != 0) {
   final btVector3 normal = ((new btVector3(wv1).sub(wv0)).cross(new btVector3(wv2).sub(wv0)));
   normal.normalize();
   final btVector3 normalColor = new btVector3(1, 1, 0);
   m_debugDrawer.drawLine(center, normal.add(center), normalColor);
  }
  m_debugDrawer.drawLine(wv0, wv1, m_color);
  m_debugDrawer.drawLine(wv1, wv2, m_color);
  m_debugDrawer.drawLine(wv2, wv0, m_color);
  return true;
 }
};
