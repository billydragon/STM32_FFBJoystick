/* USER CODE BEGIN Header */

#include "cppmain.h"
#include "FFBMain.h"
#include "stm32f4xx_hal.h"
#include "QEncoder.h"

long map (long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t * JoystickHIDReportDescr;
uint16_t  JoystickHIDReportDescr_Size;


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
  /* Prevent unused argument(s) compilation warning */
  if (GPIO_Pin == Buttons[0].pinNumber)
    {
      uint8_t bState = !HAL_GPIO_ReadPin (Buttons[0].Port,
					  Buttons[0].pinNumber);
      if (Buttons[0].CurrentState != bState)
	{
	  if ((HAL_GetTick () - Buttons[0].millis_time) > DEBOUNCE_TIME)
	    {

	      Buttons[0].CurrentState = bState;
	      Buttons[0].millis_time = HAL_GetTick ();
	    }
	  else
	    {
	      Buttons[0].CurrentState = !HAL_GPIO_ReadPin (
		  Buttons[0].Port, Buttons[0].pinNumber);
	    }
	}

      EXTI->PR |= Buttons[0].pinNumber;

    }

}

void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *hadc)
{
  analog_axis[RX_AXIS].currentPosition = map(adc_buff[RX_AXIS], 0, 4095,
											  analog_axis[RX_AXIS].minValue, analog_axis[RX_AXIS].maxValue);
  analog_axis[RY_AXIS].currentPosition = map(adc_buff[RY_AXIS], 0, 4095,
											  analog_axis[RY_AXIS].minValue, analog_axis[RY_AXIS].maxValue);
}

extern QEncoder encoder;


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

      HAL_Delay (1);

    }

}

