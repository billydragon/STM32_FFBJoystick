/* USER CODE BEGIN Header */

#include "cppmain.h"
#include "FFBMain.h"
//#include "stm32f4xx_hal.h"
#include "QEncoder.h"
#include "MotorDriver.h"

long map (long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t * JoystickHIDReportDescr;
uint16_t  JoystickHIDReportDescr_Size;

extern QEncoder encoder;
extern MotorDriver Motors;
extern int32_t xy_forces[2];
extern volatile bool RunFirstTime;

void LimitSwitch_trig(uint16_t GPIO_Pin);

uint32_t micros ()
{ return htim10.Instance->CNT; }

void delay_us (uint16_t us)
{
  uint32_t time = micros ();
  while ((micros () - time) < us)
    {;}
}



void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{

	//JBUTTON0
	if (GPIO_Pin == Buttons[0].pinNumber)
		{
		  uint8_t bState = HAL_GPIO_ReadPin (Buttons[0].Port, Buttons[0].pinNumber);
		  if (Buttons[0].CurrentState != bState)
		{
		  if ((HAL_GetTick () - Buttons[0].millis_time) > DEBOUNCE_TIME)
			{

			  Buttons[0].CurrentState = bState;
			  Buttons[0].millis_time = HAL_GetTick ();
			}
		  else
			{
			  Buttons[0].CurrentState = HAL_GPIO_ReadPin (Buttons[0].Port, Buttons[0].pinNumber);
			}
		}

		  EXTI->PR |= Buttons[0].pinNumber;

		}

  //JBUTTON1 - JBUTTON9
	for (int i = 1; i < NUM_OF_BUTTONS; i++)
	{
		if (GPIO_Pin == Buttons[i].pinNumber)
		    {
		      uint8_t bState = !HAL_GPIO_ReadPin (Buttons[i].Port, Buttons[i].pinNumber);
		      if (Buttons[i].CurrentState != bState)
			{
			  if ((HAL_GetTick () - Buttons[i].millis_time) > DEBOUNCE_TIME)
			    {

			      Buttons[i].CurrentState = bState;
			      Buttons[i].millis_time = HAL_GetTick ();
			    }
			  else
			    {
			      Buttons[i].CurrentState = !HAL_GPIO_ReadPin (Buttons[i].Port, Buttons[i].pinNumber);
			    }
			}

		      EXTI->PR |= Buttons[i].pinNumber;

		    }

	}


  if(RunFirstTime == true)
  		{
	  LimitSwitch_trig(GPIO_Pin);
  		}

}

void LimitSwitch_trig(uint16_t GPIO_Pin)
{
	 uint8_t limitswitch_state = 0;

		  if(GPIO_Pin == Limit_Switch[X_LIMIT_MAX].pinNumber)
		  {
			  if ((HAL_GetTick () - Limit_Switch[X_LIMIT_MAX].millis_time) > 5)
			  {
				  limitswitch_state = !HAL_GPIO_ReadPin (Limit_Switch[X_LIMIT_MAX].Port, Limit_Switch[X_LIMIT_MAX].pinNumber);
				  Limit_Switch[X_LIMIT_MAX].CurrentState = limitswitch_state;
				  Limit_Switch[X_LIMIT_MAX].millis_time = HAL_GetTick ();
			  }
			  else
			  {
				  Limit_Switch[X_LIMIT_MAX].CurrentState = !HAL_GPIO_ReadPin (Limit_Switch[X_LIMIT_MAX].Port, Limit_Switch[X_LIMIT_MAX].pinNumber);
			  }
				  EXTI->PR |= Limit_Switch[X_LIMIT_MAX].pinNumber;
		  }
		  else if(GPIO_Pin == Limit_Switch[X_LIMIT_MIN].pinNumber)
		  {
			  if ((HAL_GetTick () - Limit_Switch[X_LIMIT_MIN].millis_time) > 5)
			  {
				  limitswitch_state = !HAL_GPIO_ReadPin (Limit_Switch[X_LIMIT_MIN].Port, Limit_Switch[X_LIMIT_MIN].pinNumber);
				  Limit_Switch[X_LIMIT_MIN].CurrentState = limitswitch_state;
				  Limit_Switch[X_LIMIT_MIN].millis_time = HAL_GetTick ();
			  }
			  else
			  {
				  Limit_Switch[X_LIMIT_MIN].CurrentState = !HAL_GPIO_ReadPin (Limit_Switch[X_LIMIT_MIN].Port, Limit_Switch[X_LIMIT_MIN].pinNumber);
			  }
				  EXTI->PR |= Limit_Switch[X_LIMIT_MIN].pinNumber;
		  }
		  else if(GPIO_Pin == Limit_Switch[Y_LIMIT_MAX].pinNumber)
		  {
			  if ((HAL_GetTick () - Limit_Switch[Y_LIMIT_MAX].millis_time) > 5)
			  {
				  limitswitch_state = !HAL_GPIO_ReadPin (Limit_Switch[Y_LIMIT_MAX].Port, Limit_Switch[Y_LIMIT_MAX].pinNumber);
				  Limit_Switch[Y_LIMIT_MAX].CurrentState = limitswitch_state;
				  Limit_Switch[Y_LIMIT_MAX].millis_time = HAL_GetTick ();
			  }
			  else
			  {
				  Limit_Switch[Y_LIMIT_MAX].CurrentState = !HAL_GPIO_ReadPin (Limit_Switch[Y_LIMIT_MAX].Port, Limit_Switch[Y_LIMIT_MAX].pinNumber);
			  }
				  EXTI->PR |= Limit_Switch[Y_LIMIT_MAX].pinNumber;
		  }
		  else if (GPIO_Pin == Limit_Switch[Y_LIMIT_MIN].pinNumber)
		  {
			  if ((HAL_GetTick () - Limit_Switch[Y_LIMIT_MIN].millis_time) > 5)
			  {
				  limitswitch_state = !HAL_GPIO_ReadPin (Limit_Switch[Y_LIMIT_MIN].Port, Limit_Switch[Y_LIMIT_MIN].pinNumber);
				  Limit_Switch[Y_LIMIT_MIN].CurrentState = limitswitch_state;
				  Limit_Switch[Y_LIMIT_MIN].millis_time = HAL_GetTick ();
			  }
			  else
			  {
				  Limit_Switch[Y_LIMIT_MIN].CurrentState = !HAL_GPIO_ReadPin (Limit_Switch[Y_LIMIT_MIN].Port, Limit_Switch[Y_LIMIT_MIN].pinNumber);
			  }
				  EXTI->PR |= Limit_Switch[Y_LIMIT_MIN].pinNumber;
		  }

}


void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *hadc)
{
  analog_axis[RX_AXIS].currentPosition = map(adc_buff[RX_AXIS], 0, 4095,
											  analog_axis[RX_AXIS].minValue, analog_axis[RX_AXIS].maxValue);
  analog_axis[RY_AXIS].currentPosition = map(adc_buff[RY_AXIS], 0, 4095,
											  analog_axis[RY_AXIS].minValue, analog_axis[RY_AXIS].maxValue);
}




void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
  if ((htim->Instance == TIM3) || (htim->Instance == TIM4))
    {
      encoder.timerElapsed (htim);
    }

}

void cppmain (void)
{
	//printf("STM32 FFBJoystick v100.100.\n Starting initial device...\n");

  init_Joystick ();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_buff, NUM_OF_ADC_CHANNELS);

  while (1)
    {

      start_joystick ();

      //HAL_Delay (1);

    }

}

