/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
///btDbvt implementation by Nathanael Presson
package Bullet.Collision.Broadphase;

import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public final class btDbvtNode implements Serializable {

 private final btDbvtAabbMm volume = new btDbvtAabbMm();
 private btDbvtNode parent;
 /* convert array of children to fields, because checking the node leaf status with 'childs[1]!=null' was a  
 cache miss and therefore slower than performing the actual intersection.
  */
 private btDbvtNode child0;
 private btDbvtNode child1;
 private int dataAsInt;
 private Object data;
 private boolean isinternal;

 /**
  * equivalent to child[1]==null
  * @return
  */
 public boolean isleaf() {
  return !isinternal;
 }

 /**
  * equivalent to childs[1]!=null
  * @return
  */
 public boolean isinternal() {
  return isinternal;
 }

 /**
  * getter instead of accessing volume directly
  * @return
  */
 public btDbvtAabbMm volume() {
  return volume;
 }

 /**
  * getter instead of accessing parent directly
  * @return
  */
 public btDbvtNode parent() {
  return parent;
 }

 /**
  * setter instead of accessing parent directly
  * @param p
  * @return
  */
 public btDbvtNode parent(btDbvtNode p) {
  return parent = p;
 }

 /**
  * getter instead of childs[0]
  * @return
  */
 public btDbvtNode child0() {
  return child0;
 }

 /**
  *getter instead of childs[1]
  * @return
  */
 public btDbvtNode child1() {
  return child1;
 }

 /**
  * setter instead of childs[0]=child
  * @param c
  * @return
  */
 public btDbvtNode child0(btDbvtNode c) {
  return child0 = c;
 }

 /**
  * setter instead of childs[1]=child
  * @param c
  * @return
  */
 public btDbvtNode child1(btDbvtNode c) {
  isinternal = c != null;
  return child1 = c;
 }

 /**
  * getter instead of childs[i]
  * @param i
  * @return
  */
 public btDbvtNode childs(int i) {
  switch (i) {
   case 0:
    return child0;
   case 1:
    return child1;
   default:
    assert (false);
  }
  return null;
 }

 /**
  * setter instead of childs[i]=child
  * @param i
  * @param c
  * @return
  */
 public btDbvtNode childs(int i, btDbvtNode c) {
  switch (i) {
   case 0:
    return child0 = c;
   case 1:
    isinternal = c != null;
    return child1 = c;
   default:
    assert (false);
  }
  return null;
 }

 /**
  * getter for dataAsInt
  * @return
  */
 public int dataAsInt() {
  return dataAsInt;
 }

 /**
  *setter for dataAsInt
  * @param d
  * @return
  */
 public int dataAsInt(int d) {
  return dataAsInt = d;
 }

 /**
  * getter for data
  * @return
  */
 public Object data() {
  return data;
 }

 /**
  * setter instead of accessing data directly
  * @param d
  * @return
  */
 public Object data(Object d) {
  return data = d;
 }
}
