/*

***************************************************************************************************
**
** profile.cpp
**
** Real-Time Hierarchical Profiling for Game Programming Gems 3
**
** by Greg Hjelstrom & Byon Garrabrant
**
***************************************************************************************************/
// Credits: The Clock class was inspired by the Timer classes in
// Ogre (www.ogre3d.org).
package Bullet.LinearMath;

import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public final class btClock implements Serializable {

 public btClock() {
  m_data = new btClockData();
  reset();
 }

 public btClock(btClock other) {
  m_data = (btClockData) other.m_data.clone();
 }

 public void set(btClock other) {
  m_data = (btClockData) other.m_data.clone();
 }

 /// Resets the initial reference time.
 public void reset() {
  m_data.mStartTick = m_data.mStartTime = System.nanoTime();
 }

 /// Returns the time in ms since the last call to reset or since 
 /// the btClock was created.
 public long getTimeMilliseconds() {
  long currentTime = System.nanoTime();
  long elapsedTime = currentTime - m_data.mStartTime;
  // Compute the number of millisecond ticks elapsed.
  long msecTicks = (long) (1_000l * elapsedTime /
   m_data.mClockFrequency);
  assert (msecTicks > 0);
  return msecTicks;
 }

 /// Returns the time in us since the last call to reset or since 
 /// the Clock was created.
 public long getTimeMicroseconds() {
  long currentTime = System.nanoTime();
  long elapsedTime = currentTime - m_data.mStartTime;
  // Compute the number of millisecond ticks elapsed.
  long msecTicks = (long) (1_000_000l * elapsedTime /
   m_data.mClockFrequency);
  assert (msecTicks >= 0);
  return msecTicks;
 }

 public long getTimeNanoseconds() {
  long currentTime = System.nanoTime();
  long elapsedTime = currentTime - m_data.mStartTime;
  // Compute the number of millisecond ticks elapsed.
  long msecTicks = (1_000_000_000l * elapsedTime /
   m_data.mClockFrequency);
  assert (msecTicks > 0);
  return msecTicks;
 }

 /// Returns the time in s since the last call to reset or since 
 /// the Clock was created.
 public float getTimeSeconds() {
  long currentTime = System.nanoTime();
  long elapsedTime = currentTime - m_data.mStartTime;
  // Compute the number of millisecond ticks elapsed.
  float msecTicks = ((float) elapsedTime /
   (float) m_data.mClockFrequency);
  assert (msecTicks >= 0);
  return msecTicks;
 }
 btClockData m_data;
}
