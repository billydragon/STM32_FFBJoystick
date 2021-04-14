/*
 Joystick.h

 Copyright (c) 2015-2017, Matthew Heironimus

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef JOYSTICK_h
#define JOYSTICK_h
#include "main.h"
#include "FFBMain.h"
//#include "PIDReportHandler.h"
//#include "PIDReportType.h"
#include "ffb.h"

//================================================================================
//  Joystick (Gamepad)

#define JOYSTICK_DEFAULT_REPORT_ID         0x01
#define JOYSTICK_DEFAULT_BUTTON_COUNT        32
#define JOYSTICK_DEFAULT_AXIS_MINIMUM         0
#define JOYSTICK_DEFAULT_AXIS_MAXIMUM      1023
#define JOYSTICK_DEFAULT_SIMULATOR_MINIMUM    0
#define JOYSTICK_DEFAULT_SIMULATOR_MAXIMUM 1023
#define JOYSTICK_DEFAULT_HATSWITCH_COUNT      2
#define JOYSTICK_HATSWITCH_COUNT_MAXIMUM      2
#define JOYSTICK_HATSWITCH_RELEASE           -1
#define JOYSTICK_TYPE_JOYSTICK             0x04
#define JOYSTICK_TYPE_GAMEPAD              0x05
#define JOYSTICK_TYPE_MULTI_AXIS           0x08

#define DIRECTION_ENABLE                   0x04
#define X_AXIS_ENABLE                      0x01
#define Y_AXIS_ENABLE                      0x02

#ifndef FFB_AXIS_COUNT
	#define FFB_AXIS_COUNT                 0x02
#endif

#define FORCE_FEEDBACK_MAXGAIN              100
#ifndef DEG_TO_RAD
#define DEG_TO_RAD              ((float)((float)3.14159265359 / 180.0))
#endif

// HID Descriptor definitions - Axes
#define HID_USAGE_X		0x30
#define HID_USAGE_Y		0x31
#define HID_USAGE_Z		0x32
#define HID_USAGE_RX	0x33
#define HID_USAGE_RY	0x34
#define HID_USAGE_RZ	0x35
#define HID_USAGE_SL0	0x36
#define HID_USAGE_SL1	0x37
#define HID_USAGE_WHL	0x38
#define HID_USAGE_POV	0x39
#define HID_USAGE_RUDDER	0xBA
#define HID_USAGE_THROTTLER 0xBB
#define HID_USAGE_ACCELERATOR	0xC4
#define HID_USAGE_BRAKE			0xC5
#define HID_USAGE_STEERING	0xC8

// HID Descriptor definitions - FFB Effects
#define HID_USAGE_CONST 0x26    //    Usage ET Constant Force
#define HID_USAGE_RAMP  0x27    //    Usage ET Ramp
#define HID_USAGE_SQUR  0x30    //    Usage ET Square
#define HID_USAGE_SINE  0x31    //    Usage ET Sine
#define HID_USAGE_TRNG  0x32    //    Usage ET Triangle
#define HID_USAGE_STUP  0x33    //    Usage ET Sawtooth Up
#define HID_USAGE_STDN  0x34    //    Usage ET Sawtooth Down
#define HID_USAGE_SPRNG 0x40    //    Usage ET Spring
#define HID_USAGE_DMPR  0x41    //    Usage ET Damper
#define HID_USAGE_INRT  0x42    //    Usage ET Inertia
#define HID_USAGE_FRIC  0x43    //    Usage ET Friction

// HID Descriptor definitions - FFB Report IDs
#define HID_ID_STATE	0x02	// Usage PID State report

#define HID_ID_EFFREP	0x01	// Usage Set Effect Report
#define HID_ID_ENVREP	0x02	// Usage Set Envelope Report
#define HID_ID_CONDREP	0x03	// Usage Set Condition Report
#define HID_ID_PRIDREP	0x04	// Usage Set Periodic Report
#define HID_ID_CONSTREP	0x05	// Usage Set Constant Force Report
#define HID_ID_RAMPREP	0x06	// Usage Set Ramp Force Report
#define HID_ID_CSTMREP	0x07	// Usage Custom Force Data Report
#define HID_ID_SMPLREP	0x08	// Usage Download Force Sample
#define HID_ID_EFOPREP	0x0A	// Usage Effect Operation Report
#define HID_ID_BLKFRREP	0x0B	// Usage PID Block Free Report
#define HID_ID_CTRLREP	0x0C	// Usage PID Device Control
#define HID_ID_GAINREP	0x0D	// Usage Device Gain Report
#define HID_ID_SETCREP	0x0E	// Usage Set Custom Force Report
// Features
#define HID_ID_NEWEFREP	0x11	// Usage Create New Effect Report
#define HID_ID_BLKLDREP	0x12	// Usage Block Load Report
#define HID_ID_POOLREP	0x13	// Usage PID Pool Report

#define FFB_EFFECT_NONE			0x00
#define FFB_EFFECT_CONSTANT		0x01
#define FFB_EFFECT_RAMP			0x02
#define FFB_EFFECT_SQUARE 		0x03
#define FFB_EFFECT_SINE 		0x04
#define FFB_EFFECT_TRIANGLE		0x05
#define FFB_EFFECT_SAWTOOTHUP	0x06
#define FFB_EFFECT_SAWTOOTHDOWN	0x07
#define FFB_EFFECT_SPRING		0x08
#define FFB_EFFECT_DAMPER		0x09
#define FFB_EFFECT_INERTIA		0x0A
#define FFB_EFFECT_FRICTION		0x0B
#define FFB_EFFECT_CUSTOM		0x0C

#define HID_ACTUATOR_POWER 		0x08
#define HID_SAFETY_SWITCH 		0x04
#define HID_ENABLE_ACTUATORS 	0x02
#define HID_EFFECT_PAUSE		0x01
#define HID_ENABLE_ACTUATORS_MASK 0xFD
#define HID_EFFECT_PLAYING 		0x10

class Joystick_
{
private:

  // Joystick State
  int16_t _xAxis;
  int16_t _yAxis;
  int16_t _zAxis;
  int16_t _xAxisRotation;
  int16_t _yAxisRotation;
  int16_t _zAxisRotation;
  int16_t _throttle;
  int16_t _rudder;
  int16_t _accelerator;
  int16_t _brake;
  int16_t _steering;
  int16_t _hatSwitchValues[JOYSTICK_HATSWITCH_COUNT_MAXIMUM];
  uint8_t *_buttonValues = NULL;

  // Joystick Settings
  bool _autoSendState;
  uint8_t _buttonCount;
  uint8_t _buttonValuesArraySize = 0;
  uint8_t _hatSwitchCount;
  uint8_t _includeAxisFlags;
  uint8_t _includeSimulatorFlags;
  int16_t _xAxisMinimum = JOYSTICK_DEFAULT_AXIS_MINIMUM;
  int16_t _xAxisMaximum = JOYSTICK_DEFAULT_AXIS_MAXIMUM;
  int16_t _yAxisMinimum = JOYSTICK_DEFAULT_AXIS_MINIMUM;
  int16_t _yAxisMaximum = JOYSTICK_DEFAULT_AXIS_MAXIMUM;
  int16_t _zAxisMinimum = JOYSTICK_DEFAULT_AXIS_MINIMUM;
  int16_t _zAxisMaximum = JOYSTICK_DEFAULT_AXIS_MAXIMUM;
  int16_t _rxAxisMinimum = JOYSTICK_DEFAULT_AXIS_MINIMUM;
  int16_t _rxAxisMaximum = JOYSTICK_DEFAULT_AXIS_MAXIMUM;
  int16_t _ryAxisMinimum = JOYSTICK_DEFAULT_AXIS_MINIMUM;
  int16_t _ryAxisMaximum = JOYSTICK_DEFAULT_AXIS_MAXIMUM;
  int16_t _rzAxisMinimum = JOYSTICK_DEFAULT_AXIS_MINIMUM;
  int16_t _rzAxisMaximum = JOYSTICK_DEFAULT_AXIS_MAXIMUM;
  int16_t _rudderMinimum = JOYSTICK_DEFAULT_SIMULATOR_MINIMUM;
  int16_t _rudderMaximum = JOYSTICK_DEFAULT_SIMULATOR_MAXIMUM;
  int16_t _throttleMinimum = JOYSTICK_DEFAULT_SIMULATOR_MINIMUM;
  int16_t _throttleMaximum = JOYSTICK_DEFAULT_SIMULATOR_MAXIMUM;
  int16_t _acceleratorMinimum = JOYSTICK_DEFAULT_SIMULATOR_MINIMUM;
  int16_t _acceleratorMaximum = JOYSTICK_DEFAULT_SIMULATOR_MAXIMUM;
  int16_t _brakeMinimum = JOYSTICK_DEFAULT_SIMULATOR_MINIMUM;
  int16_t _brakeMaximum = JOYSTICK_DEFAULT_SIMULATOR_MAXIMUM;
  int16_t _steeringMinimum = JOYSTICK_DEFAULT_SIMULATOR_MINIMUM;
  int16_t _steeringMaximum = JOYSTICK_DEFAULT_SIMULATOR_MAXIMUM;

  uint8_t _hidReportId;
  uint8_t _hidReportSize;

protected:
  int buildAndSet16BitValue (bool includeValue, int16_t value, int16_t valueMinimum, int16_t valueMaximum, int16_t actualMinimum, int16_t actualMaximum, uint8_t dataLocation[]);
  int buildAndSetAxisValue (bool includeAxis, int16_t axisValue, int16_t axisMinimum, int16_t axisMaximum, uint8_t dataLocation[]);
  int buildAndSetSimulationValue (bool includeValue, int16_t value, int16_t valueMinimum, int16_t valueMaximum, uint8_t dataLocation[]);

public:
  Joystick_ (uint8_t hidReportId = JOYSTICK_DEFAULT_REPORT_ID,
	     uint8_t joystickType = JOYSTICK_TYPE_JOYSTICK,
	     uint8_t buttonCount = JOYSTICK_DEFAULT_BUTTON_COUNT,
	     uint8_t hatSwitchCount = JOYSTICK_DEFAULT_HATSWITCH_COUNT,
	     bool includeXAxis = true, bool includeYAxis = true,
	     bool includeZAxis = true, bool includeRxAxis = true,
	     bool includeRyAxis = true, bool includeRzAxis = true,
	     bool includeRudder = true, bool includeThrottle = true,
	     bool includeAccelerator = true, bool includeBrake = true,
	     bool includeSteering = true);

  void begin (bool initAutoSendState = true);
  void end ();

  // Set Range Functions
  inline void setXAxisRange (int16_t minimum, int16_t maximum)
  {
    _xAxisMinimum = minimum;
    _xAxisMaximum = maximum;
  }
  inline void setYAxisRange (int16_t minimum, int16_t maximum)
  {
    _yAxisMinimum = minimum;
    _yAxisMaximum = maximum;
  }
  inline void setZAxisRange (int16_t minimum, int16_t maximum)
  {
    _zAxisMinimum = minimum;
    _zAxisMaximum = maximum;
  }
  inline void setRxAxisRange (int16_t minimum, int16_t maximum)
  {
    _rxAxisMinimum = minimum;
    _rxAxisMaximum = maximum;
  }
  inline void setRyAxisRange (int16_t minimum, int16_t maximum)
  {
    _ryAxisMinimum = minimum;
    _ryAxisMaximum = maximum;
  }
  inline void setRzAxisRange (int16_t minimum, int16_t maximum)
  {
    _rzAxisMinimum = minimum;
    _rzAxisMaximum = maximum;
  }
  inline void setRudderRange (int16_t minimum, int16_t maximum)
  {
    _rudderMinimum = minimum;
    _rudderMaximum = maximum;
  }
  inline void setThrottleRange (int16_t minimum, int16_t maximum)
  {
    _throttleMinimum = minimum;
    _throttleMaximum = maximum;
  }
  inline void setAcceleratorRange (int16_t minimum, int16_t maximum)
  {
    _acceleratorMinimum = minimum;
    _acceleratorMaximum = maximum;
  }
  inline void setBrakeRange (int16_t minimum, int16_t maximum)
  {
    _brakeMinimum = minimum;
    _brakeMaximum = maximum;
  }
  inline void setSteeringRange (int16_t minimum, int16_t maximum)
  {
    _steeringMinimum = minimum;
    _steeringMaximum = maximum;
  }

  // Set Axis Values
  void setXAxis (int16_t value);
  void setYAxis (int16_t value);
  void setZAxis (int16_t value);
  void setRxAxis (int16_t value);
  void setRyAxis (int16_t value);
  void setRzAxis (int16_t value);

  // Set Simuation Values
  void setRudder (int16_t value);
  void setThrottle (int16_t value);
  void setAccelerator (int16_t value);
  void setBrake (int16_t value);
  void setSteering (int16_t value);

  void setButton (uint8_t button, uint8_t value);
  void pressButton (uint8_t button);
  void releaseButton (uint8_t button);
  void setHatSwitch (int8_t hatSwitch, int16_t value);
  void sendState ();

};

#endif // JOYSTICK_h
