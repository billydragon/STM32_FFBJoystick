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



Gains gain[2] __attribute__((section("ccmram")));
EffectParams effects[2]  __attribute__((section("ccmram")));
int32_t xy_forces[2]  __attribute__((section("ccmram"))) = { 0 };
TDF_AXIS analog_axis[NUM_OF_ANALOG_AXIS]  __attribute__((section("ccmram")));
FFBConfig config  __attribute__((section("ccmram")));
QEncoder encoder  __attribute__((section("ccmram")));
MotorDriver Motors  __attribute__((section("ccmram")));

TDF_BUTTON Buttons[NUM_OF_BUTTONS];
TDF_BUTTON Limit_Switch[NUM_OF_LIMITSWITCH];

TDF_BUTTON Estop_Sw;

uint16_t adc_buff[NUM_OF_ADC_CHANNELS];
USB_LoggerReport_t USBLog;
uint32_t USBLog_timer =0;
volatile bool RunFirstTime = true;

double Setpoint[2], Input[2], Output[2];
double Kp[2], Ki[2], Kd[2];

PID  myPID[2] = {PID(&Input[X_AXIS], &Output[X_AXIS], &Setpoint[X_AXIS], Kp[X_AXIS], Ki[X_AXIS], Kd[X_AXIS], DIRECT),
                 PID(&Input[Y_AXIS], &Output[Y_AXIS], &Setpoint[Y_AXIS], Kp[Y_AXIS], Ki[Y_AXIS], Kd[Y_AXIS], DIRECT)};

Joystick_ Joystick (JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK, NUM_OF_BUTTONS, NUM_OF_HATSWITCH,
						true, true, false, // X and Y, Z Axis
						true, true, false, // Rx, Ry, or Rz
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
  temp_outputlimit = map(config.SysConfig.Pid[ax].MaxOutput, 0, 3000, 0, 32767);
  myPID[ax].SetOutputLimits(-temp_outputlimit, temp_outputlimit);
  myPID[ax].SetMode(AUTOMATIC);
  }

}


void init_Joystick ()
{

  config.begin ();
  Set_PID_Turnings();

  for (int i = 0; i < NUM_OF_ANALOG_AXIS; i++)
    {
      analog_axis[i].currentPosition = 0;
      analog_axis[i].lastPosition = 0;
      analog_axis[i].correctPosition = 0;
      analog_axis[i].maxAcceleration = 0;
      analog_axis[i].maxVelocity = 0;
      analog_axis[i].lastEncoderTime = HAL_GetTick ();
      analog_axis[i].lastVelocity = 0;
	  analog_axis[i].minValue = ADC_AXIS_MIN;
	  analog_axis[i].maxValue = ADC_AXIS_MAX;

    }

	Buttons[0].pinNumber = JBUTTON1_Pin;
	Buttons[0].Port = JBUTTON1_GPIO_Port;

	Buttons[1].pinNumber = JBUTTON2_Pin;
	Buttons[1].Port = JBUTTON2_GPIO_Port;

	Buttons[2].pinNumber = JBUTTON3_Pin;
	Buttons[2].Port = JBUTTON3_GPIO_Port;

	Buttons[3].pinNumber = JBUTTON4_Pin;
	Buttons[3].Port = JBUTTON4_GPIO_Port;

	Buttons[4].pinNumber = JBUTTON5_Pin;
	Buttons[4].Port = JBUTTON5_GPIO_Port;

	Estop_Sw.pinNumber = E_STOP_SW_Pin;
	Estop_Sw.Port = E_STOP_SW_GPIO_Port;

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

void start_joystick ()
{

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

		RunFirstTime = false;
	}

  for (int i = 0; i < NUM_OF_BUTTONS; i++)
    {
      if (Buttons[i].LastState != Buttons[i].CurrentState)
	{
	  Buttons[i].LastState = Buttons[i].CurrentState;
	  Joystick.setButton (i, Buttons[i].CurrentState);
	}
    }

  encoder.updatePosition (X_AXIS);

  if (encoder.axis[X_AXIS].currentPosition > encoder.axis[X_AXIS].maxValue)
    {
      Joystick.setXAxis (encoder.axis[X_AXIS].maxValue);
    }
  else if (encoder.axis[X_AXIS].currentPosition < encoder.axis[X_AXIS].minValue)
    {
      Joystick.setXAxis (encoder.axis[X_AXIS].minValue);
    }
  else
    {
      Joystick.setXAxis (encoder.axis[X_AXIS].currentPosition);
    }

  encoder.updatePosition (Y_AXIS);
  if (encoder.axis[Y_AXIS].currentPosition > encoder.axis[Y_AXIS].maxValue)
    {
      Joystick.setYAxis (encoder.axis[Y_AXIS].maxValue);
    }
  else if (encoder.axis[Y_AXIS].currentPosition < encoder.axis[Y_AXIS].minValue)
    {
      Joystick.setYAxis (encoder.axis[Y_AXIS].minValue);
    }
  else
    {
      Joystick.setYAxis (encoder.axis[Y_AXIS].currentPosition);

    }

  if (analog_axis[RX_AXIS].currentPosition > analog_axis[RX_AXIS].maxValue)
    {
      Joystick.setRxAxis (analog_axis[RX_AXIS].maxValue);
    }
  else if (analog_axis[RX_AXIS].currentPosition < analog_axis[RX_AXIS].minValue)
    {
      Joystick.setRxAxis (analog_axis[RX_AXIS].minValue);
    }
  else
    {
      Joystick.setRxAxis (analog_axis[RX_AXIS].currentPosition);

    }

  if (analog_axis[RY_AXIS].currentPosition > analog_axis[RY_AXIS].maxValue)
    {
      Joystick.setRyAxis (analog_axis[RY_AXIS].maxValue);
    }
  else if (analog_axis[RY_AXIS].currentPosition < analog_axis[RY_AXIS].minValue)
    {
      Joystick.setRyAxis (analog_axis[RY_AXIS].minValue);
    }
  else
    {
      Joystick.setRyAxis (analog_axis[RY_AXIS].currentPosition);

    }

  SetEffects ();
  Set_Gains ();
  getForce (xy_forces);
  if (config.SysConfig.AppConfig.Swap_xy_forces == true)
    {
      int32_t temp_force = xy_forces[1];
      xy_forces[1] = xy_forces[0];
      xy_forces[0] = temp_force;

    }


  if (encoder.axis[X_AXIS].currentPosition >= encoder.axis[X_AXIS].maxValue)
    {
      xy_forces[X_AXIS] = XY_FORCE_MIN;
    }
  else if (encoder.axis[X_AXIS].currentPosition <= encoder.axis[X_AXIS].minValue)
    {
      xy_forces[X_AXIS] = XY_FORCE_MAX;
    }
  if (encoder.axis[Y_AXIS].currentPosition >= encoder.axis[Y_AXIS].maxValue)
    {
      xy_forces[Y_AXIS] = XY_FORCE_MIN;
    }
  else if (encoder.axis[Y_AXIS].currentPosition <= encoder.axis[Y_AXIS].minValue)
    {
      xy_forces[Y_AXIS] = XY_FORCE_MAX;
    }


  Motors.SetMotorOutput(xy_forces);

  Send_Debug_Report();

}

void Send_Debug_Report()
{
	if(micros()- USBLog_timer > USBLOG_INTERVAL)
		{

			  USBLog.xy_forces[0] = xy_forces[0];
			  USBLog.xy_forces[1] = xy_forces[1];
			  USBLog.current_pos[0] = constrain(encoder.axis[X_AXIS].currentPosition, encoder.axis[X_AXIS].minValue, encoder.axis[X_AXIS].maxValue);
			  USBLog.current_pos[1] = constrain(encoder.axis[Y_AXIS].currentPosition, encoder.axis[Y_AXIS].minValue, encoder.axis[Y_AXIS].maxValue);
			  USBLog.axis_min[0] = encoder.axis[X_AXIS].minValue;
			  USBLog.axis_max[0] = encoder.axis[X_AXIS].maxValue;
			  USBLog.axis_min[1] = encoder.axis[Y_AXIS].minValue;
			  USBLog.axis_max[1] = encoder.axis[Y_AXIS].maxValue;

			  uint8_t fReport[USBD_CUSTOMHID_INREPORT_BUF_SIZE] = { 0 };
			  uint8_t offset = 0;
			  memcpy (&fReport[offset], (uint8_t*)&USBLog, sizeof(USB_LoggerReport_t));

			  USBD_CUSTOM_HID_SendReport_FS (fReport, sizeof(fReport));
			  USBLog_timer = micros();
		}


}

void SetEffects ()
{
	CalculateMaxSpeedAndMaxAcceleration ();

  effects[X_AXIS].springPosition = encoder.axis[X_AXIS].currentPosition;
  effects[Y_AXIS].springPosition = encoder.axis[Y_AXIS].currentPosition;

  effects[X_AXIS].springMaxPosition = encoder.axis[X_AXIS].maxValue / 2;
  effects[Y_AXIS].springMaxPosition = encoder.axis[Y_AXIS].maxValue / 2;

  effects[X_AXIS].frictionPositionChange = encoder.axis[X_AXIS].positionChange; //lastX - posX;
  effects[Y_AXIS].frictionPositionChange = encoder.axis[Y_AXIS].positionChange; //lastY - posY;

  effects[X_AXIS].frictionMaxPositionChange = encoder.axis[X_AXIS].maxPositionChange;
  effects[Y_AXIS].frictionMaxPositionChange = encoder.axis[Y_AXIS].maxPositionChange;

  effects[X_AXIS].inertiaAcceleration = encoder.axis[X_AXIS].currentPosition;
  effects[Y_AXIS].inertiaAcceleration = encoder.axis[Y_AXIS].currentPosition;

  effects[X_AXIS].inertiaMaxAcceleration = encoder.axis[X_AXIS].maxAcceleration;
  effects[Y_AXIS].inertiaMaxAcceleration = encoder.axis[Y_AXIS].maxAcceleration;

  effects[X_AXIS].damperVelocity = encoder.axis[X_AXIS].currentVelocity;
  effects[Y_AXIS].damperVelocity = encoder.axis[Y_AXIS].currentVelocity;

  effects[X_AXIS].damperMaxVelocity = encoder.axis[X_AXIS].maxVelocity;
  effects[Y_AXIS].damperMaxVelocity = encoder.axis[Y_AXIS].maxVelocity;

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

    if (encoder.axis[idx].currentPosition < encoder.axis[idx].minValue)
    {
      encoder.axis[idx].minValue = encoder.axis[idx].currentPosition;
    }
    if (encoder.axis[idx].currentPosition > encoder.axis[idx].maxValue)
    {
      encoder.axis[idx].maxValue = encoder.axis[idx].currentPosition;
    }

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

  while (Buttons[0].CurrentState == 0)
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
    if(LastPos != encoder.axis[axis_num].currentPosition)
    {
		AutoCalibration(axis_num);
		LastPos = encoder.axis[axis_num].currentPosition;

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
				encoder.axis[X_AXIS].maxValue =(Axis_Range - AXIS_BACKWARD)/2;
				encoder.axis[X_AXIS].minValue = -encoder.axis[X_AXIS].maxValue;
				Joystick.setXAxisRange(encoder.axis[X_AXIS].minValue, encoder.axis[X_AXIS].maxValue);
				Joystick.setXAxis(encoder.axis[X_AXIS].currentPosition);
				Motors.SetMotorOutput(xy_forces);
				break;
			case Y_AXIS:
				gotoPosition(Y_AXIS, Axis_Center);    //goto center X
				encoder.setPos(Y_AXIS, 0);
				encoder.axis[Y_AXIS].maxValue = (Axis_Range - AXIS_BACKWARD)/2;
				encoder.axis[Y_AXIS].minValue = -encoder.axis[Y_AXIS].maxValue;
				Joystick.setYAxisRange(encoder.axis[Y_AXIS].minValue, encoder.axis[Y_AXIS].maxValue);
				Joystick.setYAxis(encoder.axis[Y_AXIS].currentPosition);
				Motors.SetMotorOutput(xy_forces);
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
	  //Motors.MotorDriverOn(Y_AXIS);

	  do //X MAX
	  {
		xy_forces[X_AXIS] = map(config.SysConfig.AppConfig.Home_Speed, 0,3000,0,32767);
		Motors.SetMotorOutput(xy_forces);
	    encoder.updatePosition(X_AXIS);
		AutoCalibration(X_AXIS);
	    Send_Debug_Report();
	  }while (Limit_Switch[X_LIMIT_MAX].CurrentState == 0);

	  xy_forces[X_AXIS] = 0;
	  Motors.SetMotorOutput(xy_forces);

	  do //X MIN
	 	  {
	 		xy_forces[X_AXIS] = -map(config.SysConfig.AppConfig.Home_Speed, 0,3000,0,32767);
	 		Motors.SetMotorOutput(xy_forces);
	 	    encoder.updatePosition(X_AXIS);
	 		AutoCalibration(X_AXIS);
	 	    Send_Debug_Report();
	 	  }while (Limit_Switch[X_LIMIT_MIN].CurrentState == 0);

	     xy_forces[X_AXIS] = 0;
	     Motors.SetMotorOutput(xy_forces);

	    Axis_Center= (encoder.axis[X_AXIS].minValue + encoder.axis[X_AXIS].maxValue)/2 ;
	    Axis_Range =  abs(encoder.axis[X_AXIS].minValue) + abs(encoder.axis[X_AXIS].maxValue);

					gotoPosition(X_AXIS, Axis_Center);    //goto center X
					//SetZero_Encoder(X_AXIS);
					encoder.setPos(X_AXIS, 0);
					encoder.axis[X_AXIS].maxValue =(Axis_Range - AXIS_BACKWARD)/2;
					encoder.axis[X_AXIS].minValue = -encoder.axis[X_AXIS].maxValue;
					Joystick.setXAxisRange(encoder.axis[X_AXIS].minValue, encoder.axis[X_AXIS].maxValue);
					Joystick.setXAxis(encoder.axis[X_AXIS].currentPosition);
					Motors.SetMotorOutput(xy_forces);

	//***************************************************************************************
				encoder.axis[Y_AXIS].minValue =0;
				encoder.axis[Y_AXIS].maxValue =0;
				encoder.setPos(Y_AXIS, 0);
				 Motors.MotorDriverOn(Y_AXIS);

				  do //Y MAX
				  {
					xy_forces[Y_AXIS] = map(config.SysConfig.AppConfig.Home_Speed, 0,3000,0,32767);
					Motors.SetMotorOutput(xy_forces);
					encoder.updatePosition(Y_AXIS);
					AutoCalibration(Y_AXIS);
					Send_Debug_Report();

				  }while (Limit_Switch[Y_LIMIT_MAX].CurrentState == 0);
				  xy_forces[Y_AXIS] = 0;
				  Motors.SetMotorOutput(xy_forces);
				  do //Y MIN
					  {
						xy_forces[Y_AXIS] = -map(config.SysConfig.AppConfig.Home_Speed, 0,3000,0,32767);
						Motors.SetMotorOutput(xy_forces);
						encoder.updatePosition(Y_AXIS);
						AutoCalibration(Y_AXIS);
						Send_Debug_Report();
					  }while (Limit_Switch[Y_LIMIT_MIN].CurrentState == 0);

					 xy_forces[Y_AXIS] = 0;
					 Motors.SetMotorOutput(xy_forces);
					Axis_Center= (encoder.axis[Y_AXIS].minValue + encoder.axis[Y_AXIS].maxValue)/2 ;
					Axis_Range =  abs(encoder.axis[Y_AXIS].minValue) + abs(encoder.axis[Y_AXIS].maxValue);
					gotoPosition(Y_AXIS, Axis_Center);    //goto center X
					encoder.setPos(Y_AXIS, 0);
					encoder.axis[Y_AXIS].maxValue = (Axis_Range - AXIS_BACKWARD)/2;
					encoder.axis[Y_AXIS].minValue = -encoder.axis[Y_AXIS].maxValue;
					Joystick.setYAxisRange(encoder.axis[Y_AXIS].minValue, encoder.axis[Y_AXIS].maxValue);
					Joystick.setYAxis(encoder.axis[Y_AXIS].currentPosition);
					Motors.SetMotorOutput(xy_forces);

}


void gotoPosition(int axis_num, int32_t targetPosition) {

  //int32_t MotorOut[2] = {0,0};
  xy_forces[X_AXIS] = 0;
  xy_forces[Y_AXIS] = 0;


  Setpoint[axis_num] = targetPosition;
  while (encoder.axis[axis_num].currentPosition != targetPosition)
  {

	Set_PID_Turnings();
    Setpoint[axis_num] = targetPosition;
    encoder.updatePosition(axis_num);
    Input[axis_num] = encoder.axis[axis_num].currentPosition ;
    myPID[axis_num].Compute();
    if(encoder.axis[axis_num].currentPosition < targetPosition)
    {
    	HAL_GPIO_WritePin(GPIOD, LED3_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(GPIOD, LED1_Pin, GPIO_PIN_RESET);

    }
    else if (encoder.axis[axis_num].currentPosition > targetPosition)
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


void CalculateMaxSpeedAndMaxAcceleration()
{

  for (int i = 0; i < 2; i++)
    {
      if (encoder.axis[i].maxVelocity < abs(encoder.axis[i].currentVelocity))
	{
	  encoder.axis[i].maxVelocity = abs(encoder.axis[i].currentVelocity);
	}
      if(encoder.axis[i].maxAcceleration
	  < abs(encoder.axis[i].currentAcceleration))
	{
	  encoder.axis[i].maxAcceleration = abs(encoder.axis[i].currentAcceleration);
	}
      if(encoder.axis[i].maxPositionChange < abs(encoder.axis[i].positionChange))
	{
	  encoder.axis[i].maxPositionChange = abs(encoder.axis[i].positionChange);
	}
    }
}



