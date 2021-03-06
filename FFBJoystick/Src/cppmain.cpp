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
//extern MotorDriver Motors;
//extern int32_t xy_forces[2];
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

uint32_t multiplier;

void TM_Delay_Init(void) {

    /* Get system clocks */

    uint32_t clkval = HAL_RCC_GetSysClockFreq();		//sysclk

    /* While loop takes 4 cycles */
    /* For 1 us delay, we need to divide with 4M */
    multiplier = clkval / 4000000;
}

void TM_DelayMicros(uint32_t micros) {
    /* Multiply micros with multipler */
    /* Substract 10 */
    micros = micros * multiplier - 10;
    /* 4 cycles for one loop */
    while (micros--);
}

void TM_DelayMillis(uint32_t millis) {
    /* Multiply millis with multipler */
    /* Substract 10 */
    millis = 1000 * millis * multiplier - 10;
    /* 4 cycles for one loop */
    while (millis--);
}

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
	__disable_irq ();
	uint8_t bState = 0;
  //JBUTTON0 - JBUTTON11
	for (int i = 0; i < NUM_OF_EXTI_BUTTONS; i++)
	{

		if (GPIO_Pin == Buttons[i].pinNumber)
		    {

					bState = !HAL_GPIO_ReadPin (Buttons[i].Port, Buttons[i].pinNumber);
				  if ((HAL_GetTick () - Buttons[i].millis_time) < DEBOUNCE_TIME)
					 {

					  TM_DelayMillis(DEBOUNCE_TIME);
					  bState = !HAL_GPIO_ReadPin (Buttons[i].Port, Buttons[i].pinNumber);
					 }

				  if(i > HAT_START_PIN - 1 && i <= HAT_END_PIN)	// Left stick buttons
				  {
					  //TM_DelayMillis(100);
					  if(bState & !HAL_GPIO_ReadPin (Buttons[JOYSTICK_PUSHBUTTON_A].Port, Buttons[JOYSTICK_PUSHBUTTON_A].pinNumber))
					  {
						  Buttons[i].CurrentState = 0;
						  Buttons[JOYSTICK_PUSHBUTTON_A].CurrentState = 0;
						  HatButtons[i - HAT_START_PIN].CurrentState = bState;
					  }
					  else
					  {
						  HatButtons[i - HAT_START_PIN].CurrentState = 0;
						  Buttons[i].CurrentState = bState;
					  }

				  }
				  else if (i == JOYSTICK_PUSHBUTTON_A)
				  {
					  Buttons[JOYSTICK_PUSHBUTTON_A].CurrentState = bState;
					  for (int h = 0; h < 4 ; h++)
					  {
						  if ((HatButtons[h].CurrentState) || (Buttons[h + HAT_START_PIN].CurrentState))
						  {
							  Buttons[JOYSTICK_PUSHBUTTON_A].CurrentState = 0;
						  }
					  }
				  }
				  else
				  {
					  Buttons[i].CurrentState = bState;
				  }


				  Buttons[i].millis_time = HAL_GetTick ();
				  //EXTI->PR |= Buttons[i].pinNumber;

		    }

	}

  if(RunFirstTime == true)
  		{
		  LimitSwitch_trig(GPIO_Pin);
  		}

  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

  __enable_irq();
}


void LimitSwitch_trig(uint16_t GPIO_Pin)
{

		  if(GPIO_Pin == Limit_Switch[X_LIMIT_MAX].pinNumber)
		  {
			  if ((HAL_GetTick () - Limit_Switch[X_LIMIT_MAX].millis_time) < 5)
			  {
				  TM_DelayMillis(5);
			  }

				  Limit_Switch[X_LIMIT_MAX].CurrentState = !HAL_GPIO_ReadPin (Limit_Switch[X_LIMIT_MAX].Port, Limit_Switch[X_LIMIT_MAX].pinNumber);

			     Limit_Switch[X_LIMIT_MAX].millis_time = HAL_GetTick ();

		  }
		  else if(GPIO_Pin == Limit_Switch[X_LIMIT_MIN].pinNumber)
		  {
			  if ((HAL_GetTick () - Limit_Switch[X_LIMIT_MIN].millis_time) < 5)
			  {
				  TM_DelayMillis(5);

			  }

				  Limit_Switch[X_LIMIT_MIN].CurrentState = !HAL_GPIO_ReadPin (Limit_Switch[X_LIMIT_MIN].Port, Limit_Switch[X_LIMIT_MIN].pinNumber);
			     Limit_Switch[X_LIMIT_MIN].millis_time = HAL_GetTick ();

		  }
		  else if(GPIO_Pin == Limit_Switch[Y_LIMIT_MAX].pinNumber)
		  {
			  if ((HAL_GetTick () - Limit_Switch[Y_LIMIT_MAX].millis_time) < 5)
			  {
				  TM_DelayMillis(5);

			  }

				  Limit_Switch[Y_LIMIT_MAX].CurrentState = !HAL_GPIO_ReadPin (Limit_Switch[Y_LIMIT_MAX].Port, Limit_Switch[Y_LIMIT_MAX].pinNumber);
			     Limit_Switch[Y_LIMIT_MAX].millis_time = HAL_GetTick ();

		  }
		  else if (GPIO_Pin == Limit_Switch[Y_LIMIT_MIN].pinNumber)
		  {
			  if ((HAL_GetTick () - Limit_Switch[Y_LIMIT_MIN].millis_time) > 5)
			  {

				  TM_DelayMillis(5);
			  }

				  Limit_Switch[Y_LIMIT_MIN].CurrentState = !HAL_GPIO_ReadPin (Limit_Switch[Y_LIMIT_MIN].Port, Limit_Switch[Y_LIMIT_MIN].pinNumber);
			     Limit_Switch[Y_LIMIT_MIN].millis_time = HAL_GetTick ();

		  }

}


#if(NUM_OF_ADC_CHANNELS)

void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *hadc)
{
  analog_axis[RX_AXIS].current_Position = map(adc_buff[RX_AXIS], 0, 4095,
											  analog_axis[RX_AXIS].minValue, analog_axis[RX_AXIS].maxValue);
  analog_axis[RY_AXIS].current_Position = map(adc_buff[RY_AXIS], 0, 4095,
											  analog_axis[RY_AXIS].minValue, analog_axis[RY_AXIS].maxValue);
}

#endif


int32_t getEncoder_TIM2 ()
{
  int32_t timpos = 0;
  int32_t result = 0;

      timpos = TIM2->CNT - (int32_t) 0x7FFFFFFF;
      result = timpos + Encoder_TIM2_Counter;

  return result;
}

void setEncoder_TIM2 (int32_t val)
{

	Encoder_TIM2_Counter = val;
   TIM2->CNT = val + 0x7FFFFFFF;

}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
  if ((htim->Instance == TIM3) || (htim->Instance == TIM4))
    {
      encoder.timerElapsed (htim);
    }


  if (htim->Instance == TIM2)
	  {
	  if ((TIM2->CR1 >> 4 & 0x01) == 0) 			//DIR==0
	    //if(TIM3->CNT > TIM3->ARR/2)
	      {
		  Encoder_TIM2_Counter += TIM2->ARR / 2;
	      }
	    else if ((TIM2->CR1 >> 4 & 0x01) == 1)		//DIR==1
	      {
	   	 Encoder_TIM2_Counter -= TIM2->ARR / 2;
	      }

	    TIM2->CNT = 0x7FFFFFFF;
	    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
	  }

}

void cppmain (void)
{
	//printf("STM32 FFBJoystick v100.100.\n Starting initial device...\n");
	 /**TIM2 GPIO Configuration
	    PA1     ------> TIM2_CH2
	    PA15     ------> TIM2_CH1
	    */
	HAL_TIM_Encoder_Start (&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT (&htim2);

	setEncoder_TIM2(0);
  init_Joystick ();

#if(NUM_OF_ADC_CHANNELS)
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_buff, NUM_OF_ADC_CHANNELS);
#endif
  TM_Delay_Init();

  while (1)
    {

      start_joystick ();

      //HAL_Delay (1);

    }

}

