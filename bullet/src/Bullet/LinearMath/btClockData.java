/*
 *
 ***************************************************************************************************
 **
 ** profile.cpp
 **
 ** Real-Time Hierarchical Profiling for Game Programming Gems 3
 **
 ** by Greg Hjelstrom & Byon Garrabrant
 **
 ************************************************************************************************** */
// Credits: The Clock class was inspired by the Timer classes in
// Ogre (www.ogre3d.org).
package Bullet.LinearMath;

import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
class btClockData implements Serializable {

 final long mClockFrequency;
 long mStartTick;
 long mStartTime;

 public btClockData() {
  mClockFrequency = 1_000_000_000;
 }

}
