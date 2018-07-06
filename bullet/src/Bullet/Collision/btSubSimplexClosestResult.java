/*
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
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

import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public final class btSubSimplexClosestResult implements Serializable {

 final btVector3 m_closestPointOnSimplex = new btVector3();
 //MASK for m_usedVertices
 //stores the simplex vertex-usage, using the MASK, 
 // if m_usedVertices & MASK then the related vertex is used
 final btUsageBitfield m_usedVertices = new btUsageBitfield();
 final float[] m_barycentricCoords = new float[4];
 boolean m_degenerate;

 void reset() {
  m_degenerate = false;
  setBarycentricCoordinates();
  m_usedVertices.reset();
 }

 boolean isValid() {
  boolean valid = (m_barycentricCoords[0] >= (0.f)) && (m_barycentricCoords[1]
   >= (0.f)) && (m_barycentricCoords[2] >= (0.f)) && (m_barycentricCoords[3]
   >= (0.f));
  return valid;
 }

 void setBarycentricCoordinates(float a, float b, float c) {
  setBarycentricCoordinates(a, b, c, 0);
 }

 void setBarycentricCoordinates(float a, float b) {
  setBarycentricCoordinates(a, b, 0, 0);
 }

 void setBarycentricCoordinates(float a) {
  setBarycentricCoordinates(a, 0, 0, 0);
 }

 void setBarycentricCoordinates() {
  setBarycentricCoordinates(0, 0, 0, 0);
 }

 void setBarycentricCoordinates(float a, float b, float c, float d) {
  m_barycentricCoords[0] = a;
  m_barycentricCoords[1] = b;
  m_barycentricCoords[2] = c;
  m_barycentricCoords[3] = d;
 }

};
