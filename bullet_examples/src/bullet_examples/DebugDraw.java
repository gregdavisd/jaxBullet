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
package bullet_examples;

import Bullet.Collision.btIDebugDraw;
import Bullet.LinearMath.btVector3;
import static org.lwjgl.opengl.GL11.GL_LINES;
import static org.lwjgl.opengl.GL11.glBegin;
import static org.lwjgl.opengl.GL11.glColor3f;
import static org.lwjgl.opengl.GL11.glEnd;
import static org.lwjgl.opengl.GL11.glVertex3f;

/**
 *
 * @author Gregery Barton
 */
public class DebugDraw extends btIDebugDraw {

 int debug_mode = 0;

 @Override
 public void drawLine(final btVector3 from, final btVector3 to, final btVector3 color) {
  glBegin(GL_LINES);
  glColor3f(color.x, color.y, color.z);
  glVertex3f(from.x, from.y, from.z);
  glVertex3f(to.x, to.y, to.z);
  glEnd();
 }

 @Override
 public void drawContactPoint(final btVector3 PointOnB, final btVector3 normalOnB, float distance,
  int lifeTime, final btVector3 color) {
  drawLine(PointOnB, new btVector3().scaleAdd(distance*10.0f, normalOnB, PointOnB), color);
  drawLine(PointOnB, new btVector3().scaleAdd(0.1f, normalOnB, PointOnB), new btVector3(1,0,0));
 }

 @Override
 public void reportErrorWarning(String warningString) {
  throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
 }

 @Override
 public void draw3dText(final btVector3 location, String textString) {
  throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
 }

 @Override
 public void setDebugMode(int debugMode) {
  debug_mode = debugMode;
 }

 @Override
 public int getDebugMode() {
  return debug_mode;
 }

 public DebugDraw() {
 }
}
