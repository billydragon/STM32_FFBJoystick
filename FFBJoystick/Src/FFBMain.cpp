/*
 * FFBMain.cpp
 *
 *  Created on: Jan 26, 2021
 *      Author: duongnt
 */
#include "FFBMain.h"
#include "Joystick.h"
#include "FFB_Calculator.h"
#include "PIDReportHandler.h"
#include "usbd_joystick_hid_if.h"
#include "QEncoder.h"
#include "FFBConfig.h"
#include "MotorDriver.h"
#include "PID_v2.h"
#include "types.h"

extern PIDReportHandler pidReportHandler;
Gains gain[2] __attribute__((section("ccmram")));
EffectParams effects[2]  __attribute__((section("ccmram")));
int32_t xy_forces[2]  __attribute__((section("ccmram"))) = { 0 };
TDF_AXIS analog_axis[NUM_OF_ANALOG_AXIS]  __attribute__((section("ccmram")));
FFBConfig config  __attribute__((section("ccmram")));
QEncoder encoder  __attribute__((section("ccmram")));
MotorDriver Motors  __attribute__((section("ccmram")));

TDF_BUTTON Buttons[NUM_OF_BUTTONS];
TDF_BUTTON HatButtons[4];
TDF_BUTTON Limit_Switch[NUM_OF_LIMITSWITCH];

TDF_BUTTON Estop_Sw;

uint16_t adc_buff[NUM_OF_ADC_CHANNELS];
USB_LoggerReport_t USBLog;

uint32_t USBLog_timer =0;
volatile bool RunFirstTime = true;


#define GOBACK_KP		3.00f
#define GOBACK_KI		0.1f
#define GOBACK_KD		0.00f
#define GOBACK_SAMPLETIME	0.01f


double Setpoint[2], Input[2], Output[2];
double Kp[2], Ki[2], Kd[2];

PID  myPID[2] = {PID(&Input[X_AXIS], &Output[X_AXIS], &Setpoint[X_AXIS], Kp[X_AXIS], Ki[X_AXIS], Kd[X_AXIS], DIRECT),
                 PID(&Input[Y_AXIS], &Output[Y_AXIS], &Setpoint[Y_AXIS], Kp[Y_AXIS], Ki[Y_AXIS], Kd[Y_AXIS], DIRECT)};

Joystick_ Joystick (JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK, NUM_OF_BUTTONS, NUM_OF_HATSWITCH,
						true, true, false, // X and Y, Z Axis
						false, false, false, // Rx, Ry, or Rz
						false, false, // rudder or throttle
						false, false, false) ; // accelerator, brake, steering


void Set_PID_Turnings()
{
	float temp_outputlimit = 0;

  for(int ax = 0; ax < 2 ; ax++)
  {
  Kp[ax]= config.SysConfig.Pid[ax].Kp;
  Ki[ax] = config.SysConfig.Pid[ax].Ki;
  Kd[ax] = config.SysConfig.Pid[ax].Kd;
  myPID[ax].SetTunings(Kp[ax],Ki[ax],Kd[ax]);
  myPID[ax].SetSampleTime(config.SysConfig.Pid[ax].SampleTime);
  temp_outputlimit = map(config.SysConfig.Pid[ax].MaxOutput, MIN_DAC_OUT_VOLT, MAX_DAC_OUT_VOLT, 0, 32767);
  myPID[ax].SetOutputLimits(-temp_outputlimit, temp_outputlimit);
  myPID[ax].SetMode(AUTOMATIC);
  myPID[ax].SetControllerDirection(DIRECT);
  }

}


void init_Joystick ()
{

  config.begin ();
  InitBiquadLp();
  Set_PID_Turnings();

  for (int i = 0; i < NUM_OF_ANALOG_AXIS; i++)
    {
      analog_axis[i].current_Position = 0;
      analog_axis[i].last_Position = 0;
      analog_axis[i].correct_Position = 0;
      analog_axis[i].max_Acceleration = 0;
      analog_axis[i].max_Speed = 0;
      analog_axis[i].last_EncoderTime = HAL_GetTick ();
      analog_axis[i].last_Speed = 0;
	  analog_axis[i].minValue = ADC_AXIS_MIN;
	  analog_axis[i].maxValue = ADC_AXIS_MAX;

    }

	Buttons[0].pinNumber = JBUTTON0_Pin;
	Buttons[0].Port = JBUTTON0_GPIO_Port;

	Buttons[1].pinNumber = JBUTTON1_Pin;
	Buttons[1].Port = JBUTTON1_GPIO_Port;

	Buttons[2].pinNumber = JBUTTON2_Pin;
	Buttons[2].Port = JBUTTON2_GPIO_Port;

	Buttons[3].pinNumber = JBUTTON3_Pin;
	Buttons[3].Port = JBUTTON3_GPIO_Port;

	Buttons[4].pinNumber = JBUTTON4_Pin;
	Buttons[4].Port = JBUTTON4_GPIO_Port;

	Buttons[5].pinNumber = JBUTTON5_Pin;
	Buttons[5].Port = JBUTTON5_GPIO_Port;

	Buttons[6].pinNumber = JBUTTON6_Pin;
	Buttons[6].Port = JBUTTON6_GPIO_Port;

	Buttons[7].pinNumber = JBUTTON7_Pin;
	Buttons[7].Port = JBUTTON7_GPIO_Port;

	Buttons[8].pinNumber = JBUTTON8_Pin;
	Buttons[8].Port = JBUTTON8_GPIO_Port;

	Buttons[9].pinNumber = JBUTTON9_Pin;
	Buttons[9].Port = JBUTTON9_GPIO_Port;

	Buttons[10].pinNumber = JBUTTON10_Pin;
	Buttons[10].Port = JBUTTON10_GPIO_Port;




  Limit_Switch[X_LIMIT_MAX].pinNumber = X_LIMIT_MAX_Pin;
  Limit_Switch[X_LIMIT_MAX].Port = X_LIMIT_MAX_GPIO_Port;
  Limit_Switch[X_LIMIT_MIN].pinNumber = X_LIMIT_MIN_Pin;
  Limit_Switch[X_LIMIT_MIN].Port = X_LIMIT_MIN_GPIO_Port;

  Limit_Switch[Y_LIMIT_MAX].pinNumber = Y_LIMIT_MAX_Pin;
  Limit_Switch[Y_LIMIT_MAX].Port = Y_LIMIT_MAX_GPIO_Port;
  Limit_Switch[Y_LIMIT_MIN].pinNumber = Y_LIMIT_MIN_Pin;
  Limit_Switch[Y_LIMIT_MIN].Port = Y_LIMIT_MIN_GPIO_Port;


  encoder.Begin ();

  Joystick.setXAxisRange(encoder.axis[X_AXIS].minValue, encoder.axis[X_AXIS].maxValue);
  Joystick.setYAxisRange(encoder.axis[Y_AXIS].minValue, encoder.axis[Y_AXIS].maxValue);
  Joystick.setRxAxisRange(analog_axis[RX_AXIS].minValue, analog_axis[RX_AXIS].maxValue);
  Joystick.setRyAxisRange(analog_axis[RY_AXIS].minValue, analog_axis[RY_AXIS].maxValue);

  Joystick.begin (true);
  Motors.Init();

}


void Update_Analog_Axis()
{
			if (analog_axis[RX_AXIS].current_Position > analog_axis[RX_AXIS].maxValue)
		    {
		      Joystick.setRxAxis (analog_axis[RX_AXIS].maxValue);
		    }
		  else if (analog_axis[RX_AXIS].current_Position < analog_axis[RX_AXIS].minValue)
		    {
		      Joystick.setRxAxis (analog_axis[RX_AXIS].minValue);
		    }
		  else
		    {
		      Joystick.setRxAxis (analog_axis[RX_AXIS].current_Position);

		    }

		  if (analog_axis[RY_AXIS].current_Position > analog_axis[RY_AXIS].maxValue)
		    {
		      Joystick.setRyAxis (analog_axis[RY_AXIS].maxValue);
		    }
		  else if (analog_axis[RY_AXIS].current_Position < analog_axis[RY_AXIS].minValue)
		    {
		      Joystick.setRyAxis (analog_axis[RY_AXIS].minValue);
		    }
		  else
		    {
		      Joystick.setRyAxis (analog_axis[RY_AXIS].current_Position);

		    }

}


void Update_Encoder_Axis(int32_t * tempForce)
{

		for(int i =0; i <2;i++)
		{
					myPID[i].SetTunings(GOBACK_KP, GOBACK_KI, GOBACK_KD);
					myPID[i].SetSampleTime(GOBACK_SAMPLETIME);
					myPID[i].SetOutputLimits(-32000, 32000);
					myPID[i].SetMode(AUTOMATIC);
					//Setpoint[axis_num] = Target;
		}

	  encoder.updatePosition (X_AXIS);

	  if (encoder.axis[X_AXIS].current_Position >= encoder.axis[X_AXIS].maxValue )
	    {
			  Setpoint[X_AXIS] = encoder.axis[X_AXIS].maxValue;
			  myPID[X_AXIS].Compute();
			  tempForce[X_AXIS] = -Output[X_AXIS];
	        Joystick.setXAxis (encoder.axis[X_AXIS].maxValue);
	    }
	  else if (encoder.axis[X_AXIS].current_Position <= encoder.axis[X_AXIS].minValue)
	    {
			  Setpoint[X_AXIS] = encoder.axis[X_AXIS].minValue;
			  myPID[X_AXIS].Compute();
			  tempForce[X_AXIS] = -Output[X_AXIS];
			  Joystick.setXAxis (encoder.axis[X_AXIS].minValue);
	    }
	  else
	    {
	       Joystick.setXAxis (encoder.axis[X_AXIS].current_Position);
	    }

	      encoder.updatePosition (Y_AXIS);

	  if (encoder.axis[Y_AXIS].current_Position >= encoder.axis[Y_AXIS].maxValue)
	    {
			   Setpoint[Y_AXIS] = encoder.axis[Y_AXIS].maxValue;
			   myPID[Y_AXIS].Compute();
			   tempForce[Y_AXIS] = -Output[Y_AXIS];
				Joystick.setYAxis (encoder.axis[Y_AXIS].maxValue);
	    }
	  else if (encoder.axis[Y_AXIS].current_Position <= encoder.axis[Y_AXIS].minValue)
	    {
				Setpoint[Y_AXIS] = encoder.axis[Y_AXIS].minValue;
		 		myPID[Y_AXIS].Compute();
		 		tempForce[Y_AXIS] = -Output[Y_AXIS];
		 		Joystick.setYAxis (encoder.axis[Y_AXIS].minValue);
	    }
	  else
	    {
	      Joystick.setYAxis (encoder.axis[Y_AXIS].current_Position);
	    }

}


void Update_Buttons()
{
	int HatButtonState = -1;

			if(HatButtons[0].CurrentState)
			{
				HatButtonState = 0;

			}
			else if(HatButtons[1].CurrentState)
			{
				HatButtonState = 90;

			}
			else if(HatButtons[2].CurrentState)
			{
				HatButtonState = 270;

			}
			else if(HatButtons[3].CurrentState)
			{
				HatButtonState = 180;

			}

			for(int b = 6 ; b < ENC_PUSH_BUTTON ; b++)
				 {
					 if(HatButtonState != -1)
					 {
						 Buttons[ENC_PUSH_BUTTON].CurrentState = 0;
						 Buttons[b].CurrentState = 0;
					 }
				 }

				Joystick.setHatSwitch(0,HatButtonState);

			//Send buttons report to host
			for (int i = 0; i < NUM_OF_BUTTONS; i++)
				 {

					if (Buttons[i].LastState != Buttons[i].CurrentState)
					{
						Buttons[i].LastState = Buttons[i].CurrentState;
					  Joystick.setButton (i, Buttons[i].CurrentState);
					}
				 }

}


void start_joystick ()
{

	int32_t _tempforce[2] = { 0, 0};

	if(RunFirstTime == true)
	{
			if(config.SysConfig.AppConfig.Auto_Calibration ==1)
			{
				findCenter_Auto();
			}
			else
			{
				findCenter_Manual(X_AXIS);
				HAL_Delay(500);
				findCenter_Manual(Y_AXIS);
				HAL_Delay(500);
			}

			gotoPosition(X_AXIS, 0);
			gotoPosition(Y_AXIS, 0);

			RunFirstTime = false;
	}


	   Update_Buttons();

		Update_Analog_Axis();

	   Update_Encoder_Axis(_tempforce);

	   SetEffects();

		Set_Gains();

		getForce (xy_forces);

 if (config.SysConfig.AppConfig.AutoCenter == true)
    {
		 if(pidReportHandler.FFB_Active == false)
		 {
			 xy_forces[X_AXIS] += (int32_t) AutoCenter_spring(X_AXIS);
			 xy_forces[Y_AXIS] += (int32_t) AutoCenter_spring(Y_AXIS);

		 }
    }

	 xy_forces[X_AXIS] += _tempforce[X_AXIS];
	 xy_forces[Y_AXIS] += _tempforce[Y_AXIS];

	 xy_forces[X_AXIS] = constrain(xy_forces[X_AXIS], -32767,32767);
	 xy_forces[Y_AXIS] = constrain(xy_forces[Y_AXIS], -32767,32767);

	 Motors.SetMotorOutput(xy_forces);

	 Send_Debug_Report();

}

void Send_Debug_Report()
{
	 if(HAL_GetTick()- USBLog_timer >= USBLOG_INTERVAL)
		{
			  USBLog.xy_forces[X_AXIS] = xy_forces[X_AXIS];
			  USBLog.xy_forces[Y_AXIS] = xy_forces[Y_AXIS];
			  USBLog.current_pos[X_AXIS] = constrain(encoder.axis[X_AXIS].current_Position, encoder.axis[X_AXIS].minValue, encoder.axis[X_AXIS].maxValue);
			  USBLog.current_pos[Y_AXIS] = constrain(encoder.axis[Y_AXIS].current_Position, encoder.axis[Y_AXIS].minValue, encoder.axis[Y_AXIS].maxValue);
			  USBLog.axis_min[X_AXIS] = encoder.axis[X_AXIS].minValue;
			  USBLog.axis_max[X_AXIS] = encoder.axis[X_AXIS].maxValue;
			  USBLog.axis_min[Y_AXIS] = encoder.axis[Y_AXIS].minValue;
			  USBLog.axis_max[Y_AXIS] = encoder.axis[Y_AXIS].maxValue;
			  uint8_t fReport[USBD_CUSTOMHID_INREPORT_BUF_SIZE] = { 0 };
			  uint8_t offset = 0;
			  memcpy (&fReport[offset], (uint8_t*)&USBLog, sizeof(USB_LoggerReport_t));

			  uint8_t result =  USBD_CUSTOM_HID_SendReport_FS (fReport, sizeof(fReport));
			  if (result == USBD_OK)
				{
					  USBLog_timer = HAL_GetTick();
				}

		}

}


void SetEffects() //Test
{

	for (int ax = 0; ax <2; ax++)
	{
#if(0)	//Activate for calculate FFB by Encoder Position
				 effects[ax].springPosition = encoder.axis[ax].current_Position;
				 effects[ax].springMaxPosition = encoder.axis[ax].maxValue;
				 effects[ax].frictionPositionChange = encoder.axis[ax].position_Changed; //lastX - posX;
				 effects[ax].frictionMaxPositionChange = encoder.axis[ax].maxValue;
				 effects[ax].inertiaAcceleration = encoder.axis[ax].current_Acceleration;
				 effects[ax].inertiaMaxAcceleration = encoder.axis[ax].maxValue;
				 effects[ax].damperVelocity = encoder.axis[ax].current_Speed;
				 effects[ax].damperMaxVelocity = encoder.axis[ax].maxValue;
#else		//Active for calculate FFB by Time
			    effects[ax].springPosition = encoder.axis[ax].current_Position;
		 		 effects[ax].springMaxPosition = encoder.axis[ax].maxValue;
		 		 effects[ax].frictionPositionChange = encoder.axis[ax].position_Changed; //lastX - posX;
		 		 effects[ax].frictionMaxPositionChange = encoder.axis[ax].maxValue;
		 		 effects[ax].inertiaAcceleration = encoder.axis[ax].current_Acceleration;
		 		 effects[ax].inertiaMaxAcceleration = encoder.axis[ax].max_Acceleration;
		 		 effects[ax].damperVelocity = encoder.axis[ax].current_Speed;
		 		 effects[ax].damperMaxVelocity = encoder.axis[ax].max_Speed;
#endif

	}

	setEffectParams (effects);
}



void Set_Gains ()
{

  memcpy (gain, config.SysConfig.Gain, sizeof(gain));

  setGains (gain);
}

void Set_RunFirstTime_state(bool state)
{
	RunFirstTime = state;

}


void AutoCalibration(uint8_t idx)
{

    if (encoder.axis[idx].current_Position < encoder.axis[idx].minValue)
    {
      encoder.axis[idx].minValue = encoder.axis[idx].current_Position;
    }
    if (encoder.axis[idx].current_Position > encoder.axis[idx].maxValue)
    {
      encoder.axis[idx].maxValue = encoder.axis[idx].current_Position;
    }

}



float AutoCenter_spring(uint8_t ax)
{

	float tempforce = 0;
	int32_t deadband = config.SysConfig.AC_MotorSettings[ax].Dead_Zone;
	//float Coefficient = 10000;
	encoder.Update_Metric_By_Time(ax);
	if(encoder.axis[ax].current_Position < -deadband)
	{
		tempforce = (encoder.axis[ax].current_Position + deadband)  * 3.0f * gain[ax].springGain/255;

	}
	else if(encoder.axis[ax].current_Position > deadband)
	{
		tempforce = (encoder.axis[ax].current_Position - deadband)  * 3.0f * gain[ax].springGain/255;
	}
	tempforce += encoder.axis[ax].current_Speed  * 2.0f * gain[ax].damperGain /255;
	tempforce += encoder.axis[ax].position_Changed  * 4.0f * gain[ax].frictionGain /255;

	tempforce = -constrain(tempforce, -32767, 32767);
	return tempforce;
}

void findCenter_Manual(int axis_num)
{

  int32_t LastPos=0, Axis_Center=0 ,Axis_Range=0;
  //int32_t MotorOut[2] = {0,0};
  xy_forces[X_AXIS] = 0;
  xy_forces[Y_AXIS] = 0;
  uint32_t LedBlinkTime = HAL_GetTick();

  Motors.SetMotorOutput(xy_forces);
  Motors.MotorDriverOff(X_AXIS);
  Motors.MotorDriverOff(Y_AXIS);

  encoder.axis[axis_num].minValue =0;
  encoder.axis[axis_num].maxValue =0;
  encoder.setPos(axis_num, 0);

  Motors.MotorDriverOn(X_AXIS);
  Motors.MotorDriverOn(Y_AXIS);

  while (Buttons[9].CurrentState == 0)
  {
	if(( HAL_GetTick() - LedBlinkTime) > 500)
	{
		if(axis_num == X_AXIS)
		 HAL_GPIO_TogglePin(GPIOD, LED2_Pin);
		else if(axis_num == Y_AXIS)
		 HAL_GPIO_TogglePin(GPIOD, LED4_Pin);

		LedBlinkTime = HAL_GetTick();
	}

    encoder.updatePosition(axis_num);
    if(LastPos != encoder.axis[axis_num].current_Position)
    {
		AutoCalibration(axis_num);
		LastPos = encoder.axis[axis_num].current_Position;

    }
    Send_Debug_Report();
  }

  HAL_GPIO_WritePin(GPIOD, LED2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, LED4_Pin, GPIO_PIN_RESET);

    Axis_Center= (encoder.axis[axis_num].minValue + encoder.axis[axis_num].maxValue)/2 ;
    Axis_Range =  abs(encoder.axis[axis_num].minValue) + abs(encoder.axis[axis_num].maxValue);

    switch (axis_num)
		{
			case X_AXIS:
				gotoPosition(X_AXIS, Axis_Center);    //goto center X
				//SetZero_Encoder(X_AXIS);
				encoder.setPos(X_AXIS, 0);
				encoder.axis[X_AXIS].maxValue =(Axis_Range - AXIS_BACKWARD_X)/2;
				encoder.axis[X_AXIS].minValue = -encoder.axis[X_AXIS].maxValue;
				Joystick.setXAxisRange(encoder.axis[X_AXIS].minValue, encoder.axis[X_AXIS].maxValue);
				Joystick.setXAxis(encoder.axis[X_AXIS].current_Position);
				break;
			case Y_AXIS:
				gotoPosition(Y_AXIS, Axis_Center);    //goto center X
				encoder.setPos(Y_AXIS, 0);
				encoder.axis[Y_AXIS].maxValue = (Axis_Range - AXIS_BACKWARD_Y)/2;
				encoder.axis[Y_AXIS].minValue = -encoder.axis[Y_AXIS].maxValue;
				Joystick.setYAxisRange(encoder.axis[Y_AXIS].minValue, encoder.axis[Y_AXIS].maxValue);
				Joystick.setYAxis(encoder.axis[Y_AXIS].current_Position);
				break;
			default:
				break;
		}
    HAL_GPIO_WritePin(GPIOD, LED3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, LED1_Pin, GPIO_PIN_RESET);
}



void findCenter_Auto()
{

	int32_t Axis_Center=0 ,Axis_Range=0;
	  //int32_t MotorOut[2] = {0,0};
	  xy_forces[X_AXIS] = 0;
	  xy_forces[Y_AXIS] = 0;

	  Motors.SetMotorOutput(xy_forces);
	  Motors.MotorDriverOff(X_AXIS);
	  Motors.MotorDriverOff(Y_AXIS);

	  encoder.axis[X_AXIS].minValue =0;
	  encoder.axis[X_AXIS].maxValue =0;
	  encoder.setPos(X_AXIS, 0);
	  Motors.MotorDriverOn(X_AXIS);
	  Motors.MotorDriverOn(Y_AXIS);
	  HAL_Delay(2000);
	   Limit_Switch[X_LIMIT_MAX].CurrentState = 0;

		do //X MAX
		{
		xy_forces[X_AXIS] = map(config.SysConfig.AppConfig.Home_Speed, MIN_DAC_OUT_VOLT, MAX_DAC_OUT_VOLT,0,32767);
		Motors.SetMotorOutput(xy_forces);
		encoder.updatePosition(X_AXIS);

		AutoCalibration(X_AXIS);
		Send_Debug_Report();
		}while (Limit_Switch[X_LIMIT_MAX].CurrentState == 0);
		xy_forces[X_AXIS] = 0;
		Motors.SetMotorOutput(xy_forces);
		HAL_Delay(500);
		Limit_Switch[X_LIMIT_MIN].CurrentState = 0;

		do //X MIN
		{
		xy_forces[X_AXIS] = -map(config.SysConfig.AppConfig.Home_Speed, MIN_DAC_OUT_VOLT, MAX_DAC_OUT_VOLT,0,32767);
		Motors.SetMotorOutput(xy_forces);
		encoder.updatePosition(X_AXIS);
		AutoCalibration(X_AXIS);
		Send_Debug_Report();
		}while (Limit_Switch[X_LIMIT_MIN].CurrentState == 0);
		xy_forces[X_AXIS] = 0;
		Motors.SetMotorOutput(xy_forces);
		HAL_Delay(500);

		Axis_Center= (encoder.axis[X_AXIS].minValue + encoder.axis[X_AXIS].maxValue)/2 ;
		Axis_Range =  abs(encoder.axis[X_AXIS].minValue) + abs(encoder.axis[X_AXIS].maxValue);

		gotoPosition(X_AXIS, Axis_Center);    //goto center X
		encoder.setPos(X_AXIS, 0);
		encoder.axis[X_AXIS].maxValue =(Axis_Range - AXIS_BACKWARD_X)/2;
		encoder.axis[X_AXIS].minValue = -encoder.axis[X_AXIS].maxValue;
		Joystick.setXAxisRange(encoder.axis[X_AXIS].minValue, encoder.axis[X_AXIS].maxValue);
		Joystick.setXAxis(encoder.axis[X_AXIS].current_Position);

		HAL_Delay(500);
		//***************************************************************************************
		encoder.axis[Y_AXIS].minValue =0;
		encoder.axis[Y_AXIS].maxValue =0;
		encoder.setPos(Y_AXIS, 0);
		Motors.MotorDriverOn(Y_AXIS);
		Limit_Switch[Y_LIMIT_MAX].CurrentState = 0;

		do //Y MAX
		{
		xy_forces[Y_AXIS] = map(config.SysConfig.AppConfig.Home_Speed, MIN_DAC_OUT_VOLT,MAX_DAC_OUT_VOLT,0,32767);
		Motors.SetMotorOutput(xy_forces);
		encoder.updatePosition(Y_AXIS);

		AutoCalibration(Y_AXIS);
		Send_Debug_Report();
		}while (Limit_Switch[Y_LIMIT_MAX].CurrentState == 0);
		xy_forces[Y_AXIS] = 0;
		Motors.SetMotorOutput(xy_forces);
		HAL_Delay(500);
		Limit_Switch[Y_LIMIT_MIN].CurrentState = 0;

		do //Y MIN
		{
		xy_forces[Y_AXIS] = -map(config.SysConfig.AppConfig.Home_Speed, MIN_DAC_OUT_VOLT,MAX_DAC_OUT_VOLT,0,32767);
		Motors.SetMotorOutput(xy_forces);
		encoder.updatePosition(Y_AXIS);
		AutoCalibration(Y_AXIS);
		Send_Debug_Report();
		}while (Limit_Switch[Y_LIMIT_MIN].CurrentState == 0);
		xy_forces[Y_AXIS] = 0;
		Motors.SetMotorOutput(xy_forces);
		HAL_Delay(500);

		Axis_Center= (encoder.axis[Y_AXIS].minValue + encoder.axis[Y_AXIS].maxValue)/2 ;
		Axis_Range =  abs(encoder.axis[Y_AXIS].minValue) + abs(encoder.axis[Y_AXIS].maxValue);
		gotoPosition(Y_AXIS, Axis_Center);    //goto center X
		encoder.setPos(Y_AXIS, 0);
		encoder.axis[Y_AXIS].maxValue = (Axis_Range - AXIS_BACKWARD_Y)/2;
		encoder.axis[Y_AXIS].minValue = -encoder.axis[Y_AXIS].maxValue;
		Joystick.setYAxisRange(encoder.axis[Y_AXIS].minValue, encoder.axis[Y_AXIS].maxValue);
		Joystick.setYAxis(encoder.axis[Y_AXIS].current_Position);
		HAL_Delay(500);

}


void gotoPosition(int axis_num, int32_t targetPosition) {

  //int32_t MotorOut[2] = {0,0};
  xy_forces[X_AXIS] = 0;
  xy_forces[Y_AXIS] = 0;


  Setpoint[axis_num] = targetPosition;
  Set_PID_Turnings();
  while (encoder.axis[axis_num].current_Position != targetPosition)
  {

    encoder.updatePosition(axis_num);
    Input[axis_num] = encoder.axis[axis_num].current_Position ;
    myPID[axis_num].Compute();
    if(encoder.axis[axis_num].current_Position < targetPosition)
    {
    	HAL_GPIO_WritePin(GPIOD, LED3_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(GPIOD, LED1_Pin, GPIO_PIN_RESET);

    }
    else if (encoder.axis[axis_num].current_Position > targetPosition)
    {
    	HAL_GPIO_WritePin(GPIOD, LED3_Pin, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOD, LED1_Pin, GPIO_PIN_SET);
    }


    xy_forces[axis_num] = Output[axis_num];
    Motors.SetMotorOutput(xy_forces);
    Send_Debug_Report();
  }
  xy_forces[axis_num] = 0;
  Motors.SetMotorOutput(xy_forces);
}

void Correct_Joystick_Positions(int axis_num, int32_t targetPosition)
{

	int32_t Target = targetPosition;
	myPID[axis_num].SetTunings(GOBACK_KP, GOBACK_KI, GOBACK_KD);
	myPID[axis_num].SetSampleTime(GOBACK_SAMPLETIME);
	myPID[axis_num].SetOutputLimits(-32000, 32000);
	myPID[axis_num].SetMode(AUTOMATIC);
	Setpoint[axis_num] = Target;

	if(Target > 0)
	{
			do
			{

				myPID[axis_num].Compute();
				xy_forces[axis_num] = -Output[axis_num];
				Motors.SetMotorOutput(xy_forces);
				Send_Debug_Report();
				encoder.updatePosition(axis_num);
			}while(encoder.axis[axis_num].current_Position >= Target);

	}

	if(Target < 0)
	{
			do
			{
				myPID[axis_num].Compute();
				xy_forces[axis_num] = -Output[axis_num];
				Motors.SetMotorOutput(xy_forces);
				Send_Debug_Report();
				encoder.updatePosition(axis_num);
			}while(encoder.axis[axis_num].current_Position <= Target);

	}

	    xy_forces[axis_num] = 0;
		Motors.SetMotorOutput(xy_forces);

}



