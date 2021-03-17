/*
 * QEncoder.h
 *
 *  Created on: Feb 15, 2021
 *      Author: billy
 */

#ifndef SRC_QENCODER_H_
#define SRC_QENCODER_H_
#pragma once
#ifdef __cplusplus

extern "C"
{
#endif
#include "FFBMain.h"
  /**TIM3 GPIO Configuration
   PB4     ------> TIM3_CH1
   PB5     ------> TIM3_CH2
   */
#define ENCODER_X		TIM3
  /**TIM4 GPIO Configuration
   PB6     ------> TIM4_CH1
   PB7     ------> TIM4_CH2
   */
#define ENCODER_Y		TIM4

  class QEncoder
  {
  public:

    QEncoder ();
    virtual
    ~QEncoder ();
    void
    Begin ();

    TDF_AXIS axis[NUM_OF_ENC_AXIS];
    void initVariables (void);
    int32_t getPos (uint8_t axis);
    void setPos (uint8_t axis, int32_t pos);
    void setPeriod (uint8_t axis, uint32_t period);
    void overflowCallback_X ();
    void overflowCallback_Y ();
    void timerElapsed (TIM_HandleTypeDef *htim);
    void updatePosition (uint8_t axis);
  private:
    void Check_Axis_XY_Invert_Change ();

  };
#ifdef __cplusplus
}
#endif
#endif /* SRC_QENCODER_H_ */
