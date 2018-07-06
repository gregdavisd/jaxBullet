/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package Bullet.LinearMath;

import static Bullet.LinearMath.btTransform.getIdentity;
import java.io.Serializable;

/**
 * The btDefaultMotionState provides a common implementation to synchronize
 * world transforms with offsets.
 *
 * @author Gregery Barton
 */
public class btDefaultMotionState implements btMotionState, Serializable {

 public final btTransform m_graphicsWorldTrans = new btTransform();
 public final btTransform m_centerOfMassOffset = new btTransform();
 public final btTransform m_startWorldTrans = new btTransform();
 final public Object m_userPointer;

 public btDefaultMotionState() {
  this(getIdentity(), getIdentity());
 }

 public btDefaultMotionState(final btTransform startTrans) {
  this(startTrans, getIdentity());
 }

 public btDefaultMotionState(final btTransform startTrans,
  final btTransform centerOfMassOffset) {
  m_graphicsWorldTrans.set(startTrans);
  m_centerOfMassOffset.set(centerOfMassOffset);
  m_startWorldTrans.set(startTrans);
  m_userPointer = null;
 }

 ///synchronizes world transform from user to physics
 @Override
 public void getWorldTransform(final btTransform centerOfMassWorldTrans) {
  centerOfMassWorldTrans.set(m_graphicsWorldTrans).mul(new btTransform(
   m_centerOfMassOffset)
   .invert());
 }

 ///synchronizes world transform from physics to user
 ///Bullet only calls the update of worldtransform for active objects
 @Override
 public void setWorldTransform(final btTransform centerOfMassWorldTrans) {
  m_graphicsWorldTrans.set(centerOfMassWorldTrans).mul(m_centerOfMassOffset);
 }

}
