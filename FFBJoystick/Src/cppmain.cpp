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

	uint8_t bState = 0;
  //JBUTTON0 - JBUTTON11
	for (int i = 0; i < NUM_OF_EXTI; i++)
	{

		if (GPIO_Pin == Buttons[i].pinNumber)
		    {
				  if ((HAL_GetTick () - Buttons[i].millis_time) < DEBOUNCE_TIME)
					 {
					  for(long d = 0; d < 25000; d++) { ; }

					 }

				  bState = !HAL_GPIO_ReadPin (Buttons[i].Port, Buttons[i].pinNumber);

				  if(i > 5 && i < 10)	// Left stick buttons
				  {

					  if(bState & !HAL_GPIO_ReadPin (Buttons[10].Port, Buttons[10].pinNumber))
					  {
						  Buttons[i].CurrentState = 0;
						  Buttons[10].CurrentState = 0;
						  HatButtons[i - 6].CurrentState = bState;
					  }
					  else
					  {
						  HatButtons[i - 6].CurrentState = 0;
						  Buttons[i].CurrentState = bState;
					  }

				  }
				  else if (i == 10)
				  {
					  Buttons[10].CurrentState = bState;
					  for (int h = 0; h <4 ; h++)
					  {
						  if (HatButtons[h].CurrentState)
						  {
							  Buttons[10].CurrentState = 0;
						  }
					  }
				  }
				  else
				  {
					  Buttons[i].CurrentState = bState;
				  }

				  Buttons[i].millis_time = HAL_GetTick ();
				  EXTI->PR |= Buttons[i].pinNumber;

		    }

	}


  if(RunFirstTime == true)
  		{
	  LimitSwitch_trig(GPIO_Pin);
  		}

  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
}

void LimitSwitch_trig(uint16_t GPIO_Pin)
{

		  if(GPIO_Pin == Limit_Switch[X_LIMIT_MAX].pinNumber)
		  {
			  if ((HAL_GetTick () - Limit_Switch[X_LIMIT_MAX].millis_time) > 5)
			  {

				  Limit_Switch[X_LIMIT_MAX].CurrentState = !HAL_GPIO_ReadPin (Limit_Switch[X_LIMIT_MAX].Port, Limit_Switch[X_LIMIT_MAX].pinNumber);

			  }
			  else
			  {
				  for(int d = 0; d < 5000; d++) { __ASM volatile ("NOP"); }
				  Limit_Switch[X_LIMIT_MAX].CurrentState = !HAL_GPIO_ReadPin (Limit_Switch[X_LIMIT_MAX].Port, Limit_Switch[X_LIMIT_MAX].pinNumber);

			  }
			     Limit_Switch[X_LIMIT_MAX].millis_time = HAL_GetTick ();
				  EXTI->PR |= Limit_Switch[X_LIMIT_MAX].pinNumber;
		  }
		  else if(GPIO_Pin == Limit_Switch[X_LIMIT_MIN].pinNumber)
		  {
			  if ((HAL_GetTick () - Limit_Switch[X_LIMIT_MIN].millis_time) > 5)
			  {

				  Limit_Switch[X_LIMIT_MIN].CurrentState = !HAL_GPIO_ReadPin (Limit_Switch[X_LIMIT_MIN].Port, Limit_Switch[X_LIMIT_MIN].pinNumber);

			  }
			  else
			  {
				  for(int d = 0; d < 5000; d++) { __ASM volatile ("NOP"); }
				  Limit_Switch[X_LIMIT_MIN].CurrentState = !HAL_GPIO_ReadPin (Limit_Switch[X_LIMIT_MIN].Port, Limit_Switch[X_LIMIT_MIN].pinNumber);
			  }
			     Limit_Switch[X_LIMIT_MIN].millis_time = HAL_GetTick ();
				  EXTI->PR |= Limit_Switch[X_LIMIT_MIN].pinNumber;
		  }
		  else if(GPIO_Pin == Limit_Switch[Y_LIMIT_MAX].pinNumber)
		  {
			  if ((HAL_GetTick () - Limit_Switch[Y_LIMIT_MAX].millis_time) > 5)
			  {

				  Limit_Switch[Y_LIMIT_MAX].CurrentState = !HAL_GPIO_ReadPin (Limit_Switch[Y_LIMIT_MAX].Port, Limit_Switch[Y_LIMIT_MAX].pinNumber);

			  }
			  else
			  {
				  for(int d = 0; d < 5000; d++) { __ASM volatile ("NOP"); }
				  Limit_Switch[Y_LIMIT_MAX].CurrentState = !HAL_GPIO_ReadPin (Limit_Switch[Y_LIMIT_MAX].Port, Limit_Switch[Y_LIMIT_MAX].pinNumber);
			  }
			     Limit_Switch[Y_LIMIT_MAX].millis_time = HAL_GetTick ();
				  EXTI->PR |= Limit_Switch[Y_LIMIT_MAX].pinNumber;
		  }
		  else if (GPIO_Pin == Limit_Switch[Y_LIMIT_MIN].pinNumber)
		  {
			  if ((HAL_GetTick () - Limit_Switch[Y_LIMIT_MIN].millis_time) > 5)
			  {

				  Limit_Switch[Y_LIMIT_MIN].CurrentState = !HAL_GPIO_ReadPin (Limit_Switch[Y_LIMIT_MIN].Port, Limit_Switch[Y_LIMIT_MIN].pinNumber);

			  }
			  else
			  {
				  for(int d = 0; d < 5000; d++) { __ASM volatile ("NOP"); }
				  Limit_Switch[Y_LIMIT_MIN].CurrentState = !HAL_GPIO_ReadPin (Limit_Switch[Y_LIMIT_MIN].Port, Limit_Switch[Y_LIMIT_MIN].pinNumber);
			  }
			     Limit_Switch[Y_LIMIT_MIN].millis_time = HAL_GetTick ();
				  EXTI->PR |= Limit_Switch[Y_LIMIT_MIN].pinNumber;
		  }

}


void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *hadc)
{
  analog_axis[RX_AXIS].current_Position = map(adc_buff[RX_AXIS], 0, 4095,
											  analog_axis[RX_AXIS].minValue, analog_axis[RX_AXIS].maxValue);
  analog_axis[RY_AXIS].current_Position = map(adc_buff[RY_AXIS], 0, 4095,
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

