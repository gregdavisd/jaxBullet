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

/*
 *
 * @author Gregery Barton
 */
/**
 *
 * @author Gregery Barton
 */
public interface btSimplexSolverInterface extends Serializable {

 /**
  * reset
  */
 void reset();

 /**
  *
  * @param w
  * @param p
  * @param q
  */
 void addVertex(final btVector3 w, final btVector3 p, final btVector3 q);

 /**
  *
  * @param v
  * @return
  */
 boolean closest(final btVector3 v);

 /**
  *
  * @return
  */
 float maxVertex();

 /**
  *
  * @return
  */
 boolean fullSimplex();

 /**
  *
  * @param pBuf
  * @param qBuf
  * @param yBuf
  * @return
  */
 int getSimplex(btVector3[] pBuf, btVector3[] qBuf, btVector3[] yBuf);

 /**
  *
  * @param w
  * @return
  */
 boolean inSimplex(final btVector3 w);

 /**
  *
  * @param v
  */
 void backup_closest(final btVector3 v);

 /**
  *
  * @return
  */
 boolean emptySimplex();

 /**
  *
  * @param p1
  * @param p2
  */
 void compute_points(final btVector3 p1, final btVector3 p2);

 /**
  *
  * @return
  */
 int numVertices();

};
