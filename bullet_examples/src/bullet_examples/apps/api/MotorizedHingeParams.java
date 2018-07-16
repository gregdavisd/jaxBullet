/*
 * Copyright (c) 2017  Gregery Barton
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
package bullet_examples.apps.api;

import com.leeheegee.throttle.Throttle;

/**
 *
 * @author Gregery Barton
 */
public class MotorizedHingeParams extends javax.swing.JPanel {

 /**
  * Creates new form MotorizedHingeParams
  */
 public MotorizedHingeParams() {
  initComponents();
  target_vel_slider.setValue(0);
  max_impulse_slider.setValue(10000);
  actual_vel_slider.setValue(0);
  angle_slider.setValue(0);
 }

 private float target_vel;
 private float max_impulse;
 private float actual_vel;
 private float angle;
 private final Throttle throttle = new Throttle(12, 2);

 /**
  * This method is called from within the constructor to initialize the form.
  * WARNING: Do NOT modify this code. The content of this method is always
  * regenerated by the Form Editor.
  */
 @SuppressWarnings("unchecked")
 // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
 private void initComponents() {

  target_vel_slider = new javax.swing.JSlider();
  max_impulse_slider = new javax.swing.JSlider();
  actual_vel_slider = new javax.swing.JSlider();
  jScrollPane1 = new javax.swing.JScrollPane();
  target_vel_text = new javax.swing.JTextArea();
  jScrollPane2 = new javax.swing.JScrollPane();
  max_impulse_text = new javax.swing.JTextArea();
  jScrollPane3 = new javax.swing.JScrollPane();
  actual_vel_text = new javax.swing.JTextArea();
  angle_slider = new javax.swing.JSlider();
  jScrollPane4 = new javax.swing.JScrollPane();
  angle_text = new javax.swing.JTextArea();

  target_vel_slider.setMajorTickSpacing(5);
  target_vel_slider.setMaximum(40);
  target_vel_slider.setMinimum(-40);
  target_vel_slider.setMinorTickSpacing(5);
  target_vel_slider.setSnapToTicks(true);
  target_vel_slider.setValue(-40);
  target_vel_slider.addChangeListener(new javax.swing.event.ChangeListener() {
   public void stateChanged(javax.swing.event.ChangeEvent evt) {
    target_vel_sliderStateChanged(evt);
   }
  });

  max_impulse_slider.setMajorTickSpacing(625);
  max_impulse_slider.setMaximum(10000);
  max_impulse_slider.setMinorTickSpacing(625);
  max_impulse_slider.setSnapToTicks(true);
  max_impulse_slider.setValue(0);
  max_impulse_slider.addChangeListener(new javax.swing.event.ChangeListener() {
   public void stateChanged(javax.swing.event.ChangeEvent evt) {
    max_impulse_sliderStateChanged(evt);
   }
  });

  actual_vel_slider.setMajorTickSpacing(10);
  actual_vel_slider.setMaximum(40);
  actual_vel_slider.setMinimum(-40);
  actual_vel_slider.setMinorTickSpacing(5);
  actual_vel_slider.setSnapToTicks(true);

  jScrollPane1.setHorizontalScrollBarPolicy(javax.swing.ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
  jScrollPane1.setVerticalScrollBarPolicy(javax.swing.ScrollPaneConstants.VERTICAL_SCROLLBAR_NEVER);

  target_vel_text.setEditable(false);
  target_vel_text.setColumns(20);
  target_vel_text.setRows(1);
  jScrollPane1.setViewportView(target_vel_text);

  jScrollPane2.setHorizontalScrollBarPolicy(javax.swing.ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
  jScrollPane2.setVerticalScrollBarPolicy(javax.swing.ScrollPaneConstants.VERTICAL_SCROLLBAR_NEVER);

  max_impulse_text.setEditable(false);
  max_impulse_text.setColumns(20);
  max_impulse_text.setRows(1);
  jScrollPane2.setViewportView(max_impulse_text);

  jScrollPane3.setHorizontalScrollBarPolicy(javax.swing.ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
  jScrollPane3.setVerticalScrollBarPolicy(javax.swing.ScrollPaneConstants.VERTICAL_SCROLLBAR_NEVER);

  actual_vel_text.setEditable(false);
  actual_vel_text.setColumns(20);
  actual_vel_text.setRows(1);
  jScrollPane3.setViewportView(actual_vel_text);

  angle_slider.setMaximum(7200);
  angle_slider.setMinimum(-7200);
  angle_slider.setValue(-7200);

  jScrollPane4.setHorizontalScrollBarPolicy(javax.swing.ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
  jScrollPane4.setVerticalScrollBarPolicy(javax.swing.ScrollPaneConstants.VERTICAL_SCROLLBAR_NEVER);

  angle_text.setEditable(false);
  angle_text.setColumns(20);
  angle_text.setRows(1);
  jScrollPane4.setViewportView(angle_text);

  javax.swing.GroupLayout layout = new javax.swing.GroupLayout(this);
  this.setLayout(layout);
  layout.setHorizontalGroup(
   layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
   .addGroup(layout.createSequentialGroup()
    .addContainerGap()
    .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
     .addComponent(target_vel_slider, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
     .addComponent(jScrollPane1)
     .addComponent(max_impulse_slider, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
     .addComponent(actual_vel_slider, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
     .addComponent(jScrollPane2, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.DEFAULT_SIZE, 257, Short.MAX_VALUE)
     .addComponent(jScrollPane3, javax.swing.GroupLayout.DEFAULT_SIZE, 257, Short.MAX_VALUE)
     .addComponent(angle_slider, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
     .addComponent(jScrollPane4, javax.swing.GroupLayout.DEFAULT_SIZE, 257, Short.MAX_VALUE))
    .addContainerGap())
  );
  layout.setVerticalGroup(
   layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
   .addGroup(layout.createSequentialGroup()
    .addContainerGap()
    .addComponent(jScrollPane1, javax.swing.GroupLayout.PREFERRED_SIZE, 20, javax.swing.GroupLayout.PREFERRED_SIZE)
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
    .addComponent(target_vel_slider, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
    .addComponent(jScrollPane2, javax.swing.GroupLayout.PREFERRED_SIZE, 20, javax.swing.GroupLayout.PREFERRED_SIZE)
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
    .addComponent(max_impulse_slider, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
    .addComponent(jScrollPane3, javax.swing.GroupLayout.PREFERRED_SIZE, 20, javax.swing.GroupLayout.PREFERRED_SIZE)
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
    .addComponent(actual_vel_slider, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
    .addComponent(jScrollPane4, javax.swing.GroupLayout.PREFERRED_SIZE, 20, javax.swing.GroupLayout.PREFERRED_SIZE)
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
    .addComponent(angle_slider, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
    .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
  );
 }// </editor-fold>//GEN-END:initComponents

 private void target_vel_sliderStateChanged(javax.swing.event.ChangeEvent evt) {//GEN-FIRST:event_target_vel_sliderStateChanged
  int value = round_int(target_vel_slider.getValue(), target_vel_slider
   .getMinorTickSpacing());
  target_vel = value / 10.f;
  target_vel_text.setText("target vel : " + target_vel);
 }//GEN-LAST:event_target_vel_sliderStateChanged

 private void max_impulse_sliderStateChanged(javax.swing.event.ChangeEvent evt) {//GEN-FIRST:event_max_impulse_sliderStateChanged
  int value = round_int(max_impulse_slider.getValue(), max_impulse_slider
   .getMinorTickSpacing());
  max_impulse = value / 10.f;
  max_impulse_text.setText("max impulse : " + max_impulse);
 }//GEN-LAST:event_max_impulse_sliderStateChanged

 private int round_int(int value, int nearest) {
  int remainder = value % nearest;
  int round = value - remainder;
  return round;
 }

 public float get_target_vel() {
  return target_vel;
 }

 public float get_max_impulse() {
  return max_impulse;
 }

 public float get_actual_vel() {
  return actual_vel;
 }

 public float get_angle() {
  return angle;
 }

 public void set_actual_vel(float value) {
  actual_vel = value;
  if (throttle.update_now()) {
   java.awt.EventQueue.invokeLater(new Runnable() {
    @Override
    public void run() {
     actual_vel_slider.setValue((int) (actual_vel * 10f));
     actual_vel_text.setText(String.format("actual vel : %.4f", actual_vel));
    }

   });
  }
 }

 public void set_angle(float value) {
  angle = value;
  if (throttle.update_now()) {
   java.awt.EventQueue.invokeLater(new Runnable() {
    @Override
    public void run() {
     angle_slider.setValue((int) (angle % 720f * 10f));
     angle_text.setText("angle : " + angle);
    }

   });
  }
 }
 // Variables declaration - do not modify//GEN-BEGIN:variables
 private javax.swing.JSlider actual_vel_slider;
 private javax.swing.JTextArea actual_vel_text;
 private javax.swing.JSlider angle_slider;
 private javax.swing.JTextArea angle_text;
 private javax.swing.JScrollPane jScrollPane1;
 private javax.swing.JScrollPane jScrollPane2;
 private javax.swing.JScrollPane jScrollPane3;
 private javax.swing.JScrollPane jScrollPane4;
 private javax.swing.JSlider max_impulse_slider;
 private javax.swing.JTextArea max_impulse_text;
 private javax.swing.JSlider target_vel_slider;
 private javax.swing.JTextArea target_vel_text;
 // End of variables declaration//GEN-END:variables
}
