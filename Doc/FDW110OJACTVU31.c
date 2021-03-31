
//https://www.instructables.com/Speed-Control-of-DC-Motor-Using-PID-Algorithm-STM3/

#include <stm32f4xx.h>

//ALL the variables have been defined here

static int i = 0;int bit =0;int speed =0;int res =0;

// refernce and pid related constants
int ref_speed = 120;float duty =0;int iteration_time =0;

//errors variables
float previous_error = 0;float current_error = 0;


// variables for the pid function that is actually not a function 
float KP = 0.1;int KI = 10;int KD = 1;

// some extra variables working as temporary storage
int input = 0;int integration_sum = 0;

// To Get Current Count
int GETVAL(void){
	 return SysTick->VAL;
}

// To account for Multiple Cycles of Timer
// for more than the period of the timer
void SysTick_Handler(void)
{
	i++;
}


// Timer Start Function
// when called the timer starts counting
void Timer_start_func(void){
	  SysTick->LOAD  = 168000 - 1;                                  /* set reload register */
	  NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority for Systick Interrupt */
	  SysTick->VAL   = 0;                                          /* Load the SysTick Counter Value */
	  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
	                   SysTick_CTRL_TICKINT_Msk   |
	                   SysTick_CTRL_ENABLE_Msk;
}

// Timer Count End Function
// when called the timer stops counting
void Timer_end_func(void){
	 SysTick->CTRL  = 0;
	 i=0;
}

// Timer configurations done here
// initialization of timer related gpios also done here
static void Timer_configuration()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  uint16_t TimerPeriod = 0;
  uint16_t Channel1Pulse = 0;

  /* GPIOA Configuration*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);

  /* TIM1 Configuration
   * TIM1 input clock (TIM1CLK) is set to APB2 clock (PCLK2)
   * => TIM1CLK = PCLK2 = SystemCoreClock
   * TIM1CLK = SystemCoreClock, Prescaler = 0, TIM1 counter clock = SystemCoreClock
   * SystemCoreClock is set to 168 MHz for STM32F30x devices
   * TIM1_Period = (SystemCoreClock / 17570) - 1

   * The Timer pulse is calculated as follows:
     - ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
  */

  // Compute the value to be set in ARR register to generate signal frequency at 50 Hz
  // was changed to 117 
  TimerPeriod = (SystemCoreClock / 117 ) - 1;
  // Compute CCR1 value to generate a duty cycle at 100% for channel 1 and 1N
  Channel1Pulse = (uint16_t) (((uint32_t)  1* (TimerPeriod - 1)) / 1);

  /* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);


  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);

  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);

  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

int main()
{
    // Initializations of GPIO Modules
	// GPIOE8 for LED
	// GPIOA1 for Input Pin
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE );

												// Setting Properties of Pins
												// PinE8 is for Identification of Completion of Revolution
												//    GPIO_StructInit( &gpio );
												//    gpio.GPIO_Mode = GPIO_Mode_OUT;
												//    gpio.GPIO_Pin  = GPIO_Pin_8;
												//    GPIO_Init( GPIOE, &gpio );
    // PinA1 is for Input speed
    GPIO_StructInit( &gpio );
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_Pin  = GPIO_Pin_1;
    GPIO_Init( GPIOA, &gpio );
    GPIO_StructInit( &gpio );

    Timer_configuration();


    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //************************************The main code starts here****************************************//
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    Timer_start_func();  //

    while(1)
    {
    	// reads the current state of PA1
    	// default function used
    	if ( GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == Bit_RESET){
    		while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == Bit_RESET){}
    		
    		if (res==0){
    			// res here is a flag to see whether to start counter or to stop it
    			Timer_start_func();
    			res=1;
    		} else if (res==1){
    			
    			iteration_time = (1 - GETVAL()/168000) + i; //DOWN Counter 168000 to 0 Thats why we subtract 
    			
    			// this formula comes as
    			// time between on slit = iteration_time
    			// time for 20 slits  = 20 * iteration_time
    			// for 1 time = 1/20*iteration time
    			// for one minute = 60000/20*iteration_time 
    			
    			speed = 3000/iteration_time;
    			
    			// timer end function as we have seen the second high pulse
    			Timer_end_func();

    			// to remove certain high level debouncing values
    			if (speed < 3000) {
    				input = speed;
    			}

    			//******************************************///
    			//PId has been implemented here
    			//PID constants are
    			// KP =0.1 Kd = 1 KI =10
    			// input to the pid setup is the current_error
    			current_error = ref_speed - input;
    			integration_sum += (current_error * iteration_time);
    			duty = KP * current_error + KI * integration_sum + KD * 1000 * (current_error -previous_error)/iteration_time;
    			
    			// directly loaded to the current compare register value instead of using the funtion for 
    			// PWM to speed up the iteration tim intervals
    			TIM1->CCR1 = duty;
    			
    			// to keep a track of the previous error
    			previous_error = current_error;

    			//*******************************************///
    			res=0;

    		}
    	}
    }
}

