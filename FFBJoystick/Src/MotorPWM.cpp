/*
 * MotorPWM.cpp
 *
 *  Created on: Mar 29, 2020
 *      Author: Yannick
 */

#include <MotorPWM.h>
#include "cppmain.h"
#include "FFBMain.h"
#include "FFBConfig.h"


extern FFBConfig config;

/**TIM1 GPIO Configuration
    PE9      ------> TIM1_CH1
    PE11     ------> TIM1_CH2
    PE13     ------> TIM1_CH3
    PE14     ------> TIM1_CH4
    */

MotorPWM::MotorPWM() {

}

void MotorPWM::Init()
{

		restoreFlash();
		//printf("MotorPWM: Init done\n");
		//setPwmSpeed(pwmspeed);
}


void MotorPWM::set_motor_pwm(int32_t * xy_forces)
{

	if(this->mode != ModePWM_DRV(config.SysConfig.PWM_Settings.PWM_Mode))
	{
		this->mode = ModePWM_DRV(config.SysConfig.PWM_Settings.PWM_Mode);
		this->setMode(this->mode);
		//printf("MotorPWM: Mode Changed\n");
	}

	if(this->pwmspeed != SpeedPWM_DRV(config.SysConfig.PWM_Settings.PWM_Speed))
	{
		this->pwmspeed = SpeedPWM_DRV(config.SysConfig.PWM_Settings.PWM_Speed);
		this->setPwmSpeed(this->pwmspeed);
		//printf("MotorPWM: PWM speed Changed\n");
	}

	this->turn(xy_forces);
}

void MotorPWM::turn(int32_t * xy_forces){
	if(!active)
		return;

	int32_t x_force = xy_forces[0];
	int32_t y_force = xy_forces[1];
	/*
	 * Generates a classic RC 20ms 1000-2000µs signal
	 * Centered at 1500µs for bidirectional RC ESCs and similiar stuff
	 */
	if(mode == ModePWM_DRV::RC_PWM){
		//int32_t pval = x_force;

		float x_val = ((x_force * 1000)/0x7fff)*tFreq;
		x_val = clip((1500*tFreq)-x_val,1000*tFreq, 2000*tFreq);
		setPWM(x_val,ccr_1);

		float y_val = ((y_force * 1000)/0x7fff)*tFreq;
		y_val = clip((1500*tFreq)-y_val,1000*tFreq, 2000*tFreq);
		setPWM(y_val,ccr_2);

	/*
	 * Generates a 0-100% PWM signal
	 * and outputs complementary direction signals.
	 * Can be used with cheap halfbridge modules and DC motors
	 */
	}else if(mode == ModePWM_DRV::PWM_DIR){
		if(x_force < 0){			//Dir X
			setPWM(0,ccr_3);
			//setPWM(0xffff,ccr_4);
		}else{
			//setPWM(0,ccr_4);
			setPWM(0xffff,ccr_3);
		}

		if(y_force < 0){			//Dir Y
			setPWM(0,ccr_4);
			//setPWM(0xffff,ccr_3);
		}else{
			//setPWM(0,ccr_3);
			setPWM(0xffff,ccr_4);
		}

		int32_t x_val = (uint32_t)((abs(x_force) * period)/0x7fff);
		setPWM(x_val,ccr_1);

		int32_t y_val = (uint32_t)((abs(y_force) * period)/0x7fff);
		setPWM(y_val,ccr_2);

	}else if(mode == ModePWM_DRV::CENTERED_PWM){
		int32_t x_pval = 0x7fff + x_force;
		int32_t x_val = (x_pval * period)/0xffff;
		setPWM(x_val,ccr_1);

		int32_t y_pval = 0x7fff + y_force;
		int32_t y_val = (y_pval * period)/0xffff;
		setPWM(y_val,ccr_2);


	}else if(mode == ModePWM_DRV::PWM_DUAL){
		int32_t x_val = (uint32_t)((abs(x_force) * period)/0x7fff);
		if(x_force < 0){
			setPWM(0,ccr_1);
			setPWM(x_val,ccr_2);
		}else{
			setPWM(0,ccr_2);
			setPWM(x_val,ccr_1);
		}

		int32_t y_val = (uint32_t)((abs(y_force) * period)/0x7fff);
				if(y_force < 0){
					setPWM(0,ccr_3);
					setPWM(y_val,ccr_4);
				}else{
					setPWM(0,ccr_4);
					setPWM(y_val,ccr_3);
				}
	}
}

/*
 * Setup the timer for different frequency presets.
 */
void MotorPWM::setPwmSpeed(SpeedPWM_DRV spd){
	bool ok = true;
	switch(spd){

	case SpeedPWM_DRV::LOW:
		if(mode == ModePWM_DRV::RC_PWM){
			period =  40000;  //20ms (40000/Sysclock)
			prescaler = TIM_PWM_FREQ/2000000;
		}else{
			period = TIM_PWM_FREQ/3000; // Check if timer can count high enough for very high clock speeds!
			prescaler = 0;
		}

	break;
	case SpeedPWM_DRV::MID:
		if(mode == ModePWM_DRV::RC_PWM){
			period = 30000;//15ms(30000/47)
			prescaler = TIM_PWM_FREQ/2000000;
		}else{
			period = TIM_PWM_FREQ/9000;
			prescaler = 0;
		}

	break;
	case SpeedPWM_DRV::HIGH:
		if(mode == ModePWM_DRV::RC_PWM){
			period = 20000; //10ms (20000/47)
			prescaler = TIM_PWM_FREQ/2000000;
		}else{
			period = TIM_PWM_FREQ/17000;
			prescaler = 0;
		}
	break;
	case SpeedPWM_DRV::VERYHIGH:
		if(mode == ModePWM_DRV::RC_PWM){
			period = 10000; //5ms (20000/23)
			prescaler = TIM_PWM_FREQ/2000000;
		}else{
			period = TIM_PWM_FREQ/24000;
			prescaler = 0;
		}
	break;
	default:
		ok = false;
	}

	if(ok){
		this->pwmspeed = spd;
		tFreq = (float)(TIM_PWM_FREQ/1000000)/(float)(prescaler+1);

		pwmInitTimer(timer, channel_1,period,prescaler);
		pwmInitTimer(timer, channel_2,period,prescaler);
		pwmInitTimer(timer, channel_3,period,prescaler);
		pwmInitTimer(timer, channel_4,period,prescaler);

//		setPWM_HAL(0, timer, channel_1, period);
//		pwmInitTimer(timer, channel_2,period,prescaler);
//		setPWM_HAL(0, timer, channel_2, period);


	}
}



/*
 * Updates pwm pulse length
 */
void MotorPWM::setPWM(uint32_t value,uint8_t ccr){
	if(ccr == 1){
		timer->Instance->CCR1 = value; // Set next CCR for channel 1
	}else if(ccr == 2){
		timer->Instance->CCR2 = value; // Set next CCR for channel 2
	}else if(ccr == 3){
		timer->Instance->CCR3 = value; // Set next CCR for channel 3
	}else if(ccr == 4){
		timer->Instance->CCR4 = value; // Set next CCR for channel 4
	}

}

SpeedPWM_DRV MotorPWM::getPwmSpeed(){
	return this->pwmspeed;
}



void MotorPWM::saveFlash(){
	// 0-3: mode
	// 4-6: speed
	//uint16_t var = (uint8_t)this->mode & 0xf;
	//var |= ((uint8_t)this->pwmspeed & 0x7) << 4;
	//Flash_Write(ADR_PWM_MODE, var);
}
void MotorPWM::restoreFlash(){
	//uint16_t var = 0;
	//if(Flash_Read(ADR_PWM_MODE, &var)){
		uint8_t m = config.SysConfig.PWM_Settings.PWM_Mode;
		this->setMode(ModePWM_DRV(m));
		uint8_t s = config.SysConfig.PWM_Settings.PWM_Speed;
		this->setPwmSpeed(SpeedPWM_DRV(s));
	//}
}


MotorPWM::~MotorPWM() {
	HAL_TIM_PWM_Stop(timer, channel_1);
	HAL_TIM_PWM_Stop(timer, channel_2);
	HAL_TIM_PWM_Stop(timer, channel_3);
	HAL_TIM_PWM_Stop(timer, channel_4);
	HAL_TIM_Base_Stop_IT(timer);
}


void MotorPWM::start(){
	active = true;
}

void MotorPWM::stop(){
	turn(0);
	active = false;
}

bool MotorPWM::GetState()
{

	return active;
}

void MotorPWM::setMode(ModePWM_DRV mode){
	this->mode = mode;
	setPwmSpeed(pwmspeed); // Reinit timer
}

ModePWM_DRV MotorPWM::getMode(){
	return this->mode;
}




void pwmInitTimer(TIM_HandleTypeDef* timer,uint32_t channel,uint32_t period,uint32_t prescaler){
	timer->Instance->ARR = period;
	timer->Instance->PSC = prescaler;
	TIM_OC_InitTypeDef sConfigOC = {0};
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	HAL_TIM_PWM_ConfigChannel(timer, &sConfigOC, channel);

	//setPWM_HAL(0,timer,channel,period);
	HAL_TIM_PWM_Start(timer, channel);

}


/*
 * Changes the pwm value of the timer via HAL
 */
void setPWM_HAL(uint32_t value,TIM_HandleTypeDef* timer,uint32_t channel,uint32_t period){
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(timer, &sConfigOC, channel);
    HAL_TIM_PWM_Start(timer, channel);
}

