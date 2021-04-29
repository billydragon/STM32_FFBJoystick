/*
 * QEncoder.cpp
 *
 *  Created on: Feb 15, 2021
 *      Author: billy
 */

#include "QEncoder.h"
#include "FFBConfig.h"
#include "FFBMain.h"

extern FFBConfig config;

TIM_HandleTypeDef *htim_X;
TIM_HandleTypeDef *htim_Y;
volatile int32_t pos_X __attribute__((section("ccmram"))); // Extra position counter for overflows
volatile int32_t pos_Y __attribute__((section("ccmram"))); // Extra position counter for overflows
volatile int32_t offset_X  __attribute__((section("ccmram")));
volatile int32_t offset_Y __attribute__((section("ccmram")));

QEncoder::QEncoder ()
{
  htim_X = &htim3;
  htim_Y = &htim4;

}

void QEncoder::Begin ()
{

  /**TIM3 GPIO Configuration
   PB4     ------> TIM3_CH1
   PB5     ------> TIM3_CH2
   */
  HAL_TIM_Encoder_Start (&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT (&htim3);
  /**TIM4 GPIO Configuration
   PB6     ------> TIM4_CH1
   PB7     ------> TIM4_CH2
   */
  HAL_TIM_Encoder_Start (&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT (&htim4);

  initVariables ();

}

QEncoder::~QEncoder ()
{
  // TODO Auto-generated destructor stub
  HAL_TIM_Encoder_Stop (&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  HAL_TIM_Encoder_Stop (&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
}

void QEncoder::initVariables ()
{
  for (int i = 0; i < NUM_OF_ENC_AXIS; i++)
    {
      axis[i].current_Position = 0;
      axis[i].last_Position = 0;
      axis[i].correct_Position = 0;
      axis[i].max_Acceleration = 0;
      axis[i].max_Speed = 0;
      axis[i].last_EncoderTime = HAL_GetTick ();
      axis[i].last_Speed = 0;
      axis[i].minValue = DIGITAL_AXIS_MIN;
      axis[i].maxValue = DIGITAL_AXIS_MAX;

      Check_Axis_XY_Invert_Change ();

      setPos (i, 0);
    }
}

void QEncoder::Check_Axis_XY_Invert_Change ()
{
  axis[X_AXIS].inverted = (bool) config.SysConfig.AppConfig.Axis_Inv_X;
  axis[Y_AXIS].inverted = (bool) config.SysConfig.AppConfig.Axis_Inv_Y;

}

int32_t QEncoder::getPos (uint8_t axis)
{
  int32_t timpos = 0;
  int32_t result = 0;
  if (axis == X_AXIS)
    {
      timpos = TIM3->CNT - (int32_t) 0x7FFF;
      result = timpos + pos_X;

    }
  else if (axis == Y_AXIS)
    {
      timpos = TIM4->CNT - (int32_t) 0x7FFF;
      result = timpos + pos_Y;
    }
  return result;
}

void QEncoder::setPos (uint8_t axis, int32_t pos)
{
  if (axis == X_AXIS)
    {
      pos_X = pos;
      TIM3->CNT = pos + 0x7fff;
    }
  else if (axis == Y_AXIS)
    {
      pos_Y = pos;
      TIM4->CNT = pos + 0x7fff;
    }
}

void QEncoder::setPeriod (uint8_t axis, uint32_t period)
{
  if (axis == X_AXIS)
    {
      TIM3->ARR = period - 1;
    }
  else if (axis == Y_AXIS)
    {
      TIM4->ARR = period - 1;
    }
}

void QEncoder::timerElapsed (TIM_HandleTypeDef *htim)
{

  if (htim->Instance == TIM3)
    {
      overflowCallback_X ();
    }
  else if (htim->Instance == TIM4)
    {
      overflowCallback_Y ();
    }

}

void QEncoder::overflowCallback_X ()
{

  if ((TIM3->CR1 >> 4 & 0x01) == 0) 			//DIR==0
  //if(TIM3->CNT > TIM3->ARR/2)
    {
      pos_X += TIM3->ARR / 2;
    }
  else if ((TIM3->CR1 >> 4 & 0x01) == 1)		//DIR==1
    {
      pos_X -= TIM3->ARR / 2;
    }

  TIM3->CNT = 0x7FFF;
  __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);

}

void QEncoder::overflowCallback_Y ()
{
  if ((TIM4->CR1 >> 4 & 0x01) == 0) 			//DIR==0
  //if(TIM4->CNT > TIM4->ARR/2)
    {
      pos_Y += TIM4->ARR / 2;
    }
  else if ((TIM4->CR1 >> 4 & 0x01) == 1)  		//DIR == 1
    {
      pos_Y -= TIM4->ARR / 2;
    }

  TIM4->CNT = 0x7FFF;
  __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);

}

void QEncoder::updatePosition (uint8_t idx)
{

	int32_t read_Position = getPos (idx);
    QEncoder::Check_Axis_XY_Invert_Change ();
   if (axis[idx].inverted == true)
       read_Position = ~read_Position;
   axis[idx].current_Position = read_Position;



}

void QEncoder::Update_Metric(uint32_t idx) //Test
{
			axis[idx].position_Changed = axis[idx].current_Position - axis[idx].last_Position;
			axis[idx].current_Speed = (axis[idx].current_Position - axis[idx].last_Position);
			axis[idx].current_Acceleration = (axis[idx].current_Speed - axis[idx].last_Speed);
			axis[idx].last_Speed = axis[idx].current_Speed;
			axis[idx].last_Position = axis[idx].current_Position;

}

void QEncoder::Update_Metric_By_Time(uint32_t ax)
{

			uint32_t currentEncoderTime = HAL_GetTick();
		   axis[ax].position_Changed = axis[ax].current_Position - axis[ax].last_Position;

		  int32_t diffTime = (int16_t)(currentEncoderTime - axis[ax].last_EncoderTime) ;
		  if (diffTime > 0) {
			 axis[ax].current_Speed = axis[ax].position_Changed / diffTime;
			 axis[ax].current_Acceleration = (abs(axis[ax].current_Speed) - abs(axis[ax].last_Speed))/diffTime;
			 axis[ax].last_EncoderTime = currentEncoderTime;
			 axis[ax].last_Speed = axis[ax].current_Speed;
		  }
		  axis[ax].last_Position = axis[ax].last_Position;

		  if (axis[ax].max_Speed < abs(axis[ax].current_Speed))
		    {
		      axis[ax].max_Speed = abs(axis[ax].current_Speed);
		    }

		  if (axis[ax].max_Acceleration < abs(axis[ax].current_Acceleration))
		    {
		      axis[ax].max_Acceleration = abs(axis[ax].current_Acceleration);
		    }
}



