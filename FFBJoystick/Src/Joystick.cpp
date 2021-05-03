/*
 * Joystick.cpp
 *
 *  Created on: Jan 24, 2021
 *      Author: billy
 */

#include "usbd_joystick_hid_if.h"
#include "Joystick.h"
#include "FFBMain.h"
#include "FFBDescriptor.h"

#define JOYSTICK_REPORT_ID_INDEX 7
#define JOYSTICK_AXIS_MINIMUM -32767
#define JOYSTICK_AXIS_MAXIMUM 32767
#define JOYSTICK_SIMULATOR_MINIMUM -32767
#define JOYSTICK_SIMULATOR_MAXIMUM 32767

#define JOYSTICK_INCLUDE_X_AXIS  		0b00000001
#define JOYSTICK_INCLUDE_Y_AXIS  		0b00000010
#define JOYSTICK_INCLUDE_Z_AXIS  		0b00000100
#define JOYSTICK_INCLUDE_RX_AXIS 		0b00001000
#define JOYSTICK_INCLUDE_RY_AXIS 		0b00010000
#define JOYSTICK_INCLUDE_RZ_AXIS 		0b00100000

#define JOYSTICK_INCLUDE_RUDDER      0b00000001
#define JOYSTICK_INCLUDE_THROTTLE    0b00000010
#define JOYSTICK_INCLUDE_ACCELERATOR 0b00000100
#define JOYSTICK_INCLUDE_BRAKE       0b00001000
#define JOYSTICK_INCLUDE_STEERING    0b00010000

extern USBD_HandleTypeDef hUsbDeviceFS;

uint32_t JoystickReportTime = 0;

Joystick_::Joystick_::Joystick_ (uint8_t hidReportId, uint8_t joystickType,
				 uint8_t buttonCount, uint8_t hatSwitchCount,
				 bool includeXAxis, bool includeYAxis,
				 bool includeZAxis, bool includeRxAxis,
				 bool includeRyAxis, bool includeRzAxis,
				 bool includeRudder, bool includeThrottle,
				 bool includeAccelerator, bool includeBrake,
				 bool includeSteering)
{
  // Set the USB HID Report ID
  _hidReportId = hidReportId;

  // Save Joystick Settings
  _buttonCount = buttonCount;
  _hatSwitchCount = hatSwitchCount;
  _includeAxisFlags = 0;
  _includeAxisFlags |= (includeXAxis ? JOYSTICK_INCLUDE_X_AXIS : 0);
  _includeAxisFlags |= (includeYAxis ? JOYSTICK_INCLUDE_Y_AXIS : 0);
  _includeAxisFlags |= (includeZAxis ? JOYSTICK_INCLUDE_Z_AXIS : 0);
  _includeAxisFlags |= (includeRxAxis ? JOYSTICK_INCLUDE_RX_AXIS : 0);
  _includeAxisFlags |= (includeRyAxis ? JOYSTICK_INCLUDE_RY_AXIS : 0);
  _includeAxisFlags |= (includeRzAxis ? JOYSTICK_INCLUDE_RZ_AXIS : 0);
  _includeSimulatorFlags = 0;
  _includeSimulatorFlags |= (includeRudder ? JOYSTICK_INCLUDE_RUDDER : 0);
  _includeSimulatorFlags |= (includeThrottle ? JOYSTICK_INCLUDE_THROTTLE : 0);
  _includeSimulatorFlags |= (
      includeAccelerator ? JOYSTICK_INCLUDE_ACCELERATOR : 0);
  _includeSimulatorFlags |= (includeBrake ? JOYSTICK_INCLUDE_BRAKE : 0);
  _includeSimulatorFlags |= (includeSteering ? JOYSTICK_INCLUDE_STEERING : 0);

  // Build Joystick HID Report Description

  // Button Calculations
  uint8_t buttonsInLastByte = _buttonCount % 8;
  uint8_t buttonPaddingBits = 0;
  if (buttonsInLastByte > 0)
    {
      buttonPaddingBits = 8 - buttonsInLastByte;
    }

  // Axis Calculations
  uint8_t axisCount = (includeXAxis == true) + (includeYAxis == true)
      + (includeZAxis == true) + (includeRxAxis == true)
      + (includeRyAxis == true) + (includeRzAxis == true);

  uint8_t simulationCount = (includeRudder == true) + (includeThrottle == true)
      + (includeAccelerator == true) + (includeBrake == true)
      + (includeSteering == true);

  static uint8_t tempHidReportDescriptor[150];
  int hidReportDescriptorSize = 0;

  //   // USAGE_PAGE (Generic Desktop)
  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x05;
  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x01;

  // USAGE (Joystick - 0x04; Gamepad - 0x05; Multi-axis Controller - 0x08)
  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x09;
  tempHidReportDescriptor[hidReportDescriptorSize++] = joystickType;

  // COLLECTION (Application)
  tempHidReportDescriptor[hidReportDescriptorSize++] = 0xa1;
  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x01;
  // USAGE (Pointer)
  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x09;
  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x01;
  // REPORT_ID (Default: 1)
  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x85;
  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x01;

  // COLLECTION (Physical)
  tempHidReportDescriptor[hidReportDescriptorSize++] = 0xa1;
  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x00;

  if (_buttonCount > 0)
    {

      // USAGE_PAGE (Button)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x05;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x09;

      // USAGE_MINIMUM (Button 1)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x19;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x01;

      // USAGE_MAXIMUM (Button 32)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x29;
      tempHidReportDescriptor[hidReportDescriptorSize++] = _buttonCount;

      // LOGICAL_MINIMUM (0)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x15;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x00;

      // LOGICAL_MAXIMUM (1)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x25;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x01;

      // REPORT_SIZE (1)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x75;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x01;

      // REPORT_COUNT (# of buttons)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x95;
      tempHidReportDescriptor[hidReportDescriptorSize++] = _buttonCount;

      // UNIT_EXPONENT (0)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x55;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x00;

      // UNIT (None)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x65;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x00;

      // INPUT (Data,Var,Abs)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x81;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x02;

      if (buttonPaddingBits > 0)
	{

	  // REPORT_SIZE (1)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x75;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x01;

	  // REPORT_COUNT (# of padding bits)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x95;
	  tempHidReportDescriptor[hidReportDescriptorSize++] =
	      buttonPaddingBits;

	  // INPUT (Const,Var,Abs)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x81;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x03;

	} // Padding Bits Needed
    } // Buttons

  if ((axisCount > 0) || (_hatSwitchCount > 0))
    {

      // USAGE_PAGE (Generic Desktop)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x05;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x01;

    }

  if (_hatSwitchCount > 0)
    {

      // USAGE (Hat Switch)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x09;
      tempHidReportDescriptor[hidReportDescriptorSize++] = HID_USAGE_POV;

      // LOGICAL_MINIMUM (0)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x15;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x00;

      // LOGICAL_MAXIMUM (7)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x25;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x07;

      // PHYSICAL_MINIMUM (0)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x35;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x00;

      // PHYSICAL_MAXIMUM (315)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x46;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x3B;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x01;

      // UNIT (Eng Rot:Angular Pos)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x65;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x14;

      // REPORT_SIZE (4)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x75;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x04;

      // REPORT_COUNT (1)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x95;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x01;

      // INPUT (Data,Var,Abs)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x81;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x02;

      if (_hatSwitchCount > 1)
	{

	  // USAGE (Hat Switch)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x09;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = HID_USAGE_POV;

	  // LOGICAL_MINIMUM (0)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x15;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x00;

	  // LOGICAL_MAXIMUM (7)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x25;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x07;

	  // PHYSICAL_MINIMUM (0)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x35;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x00;

	  // PHYSICAL_MAXIMUM (315)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x46;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x3B;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x01;

	  // UNIT (Eng Rot:Angular Pos)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x65;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x14;

	  // REPORT_SIZE (4)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x75;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x04;

	  // REPORT_COUNT (1)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x95;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x01;

	  // INPUT (Data,Var,Abs)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x81;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x02;

	}
      else
	{

	  // Use Padding Bits

	  // REPORT_SIZE (1)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x75;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x01;

	  // REPORT_COUNT (4)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x95;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x04;

	  // INPUT (Const,Var,Abs)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x81;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x03;

	} // One or Two Hat Switches?

    } // Hat Switches

  if (axisCount > 0)
    {

      // USAGE (Pointer)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x09;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x01;

      // LOGICAL_MINIMUM (-32767)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x16;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x01;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x80;

      // LOGICAL_MAXIMUM (+32767)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x26;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0xFF;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x7F;

      // REPORT_SIZE (16)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x75;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x10;

      // REPORT_COUNT (axisCount)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x95;
      tempHidReportDescriptor[hidReportDescriptorSize++] = axisCount;

      // COLLECTION (Physical)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0xA1;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x00;

      if (includeXAxis == true)
	{
	  // USAGE (X)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x09;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = HID_USAGE_X;
	}

      if (includeYAxis == true)
	{
	  // USAGE (Y)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x09;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = HID_USAGE_Y;
	}

      if (includeZAxis == true)
	{
	  // USAGE (Z)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x09;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = HID_USAGE_Z;
	}

      if (includeRxAxis == true)
	{
	  // USAGE (Rx)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x09;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = HID_USAGE_RX;
	}

      if (includeRyAxis == true)
	{
	  // USAGE (Ry)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x09;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = HID_USAGE_RY;
	}

      if (includeRzAxis == true)
	{
	  // USAGE (Rz)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x09;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = HID_USAGE_RZ;
	}

      // INPUT (Data,Var,Abs)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x81;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x02;

      // END_COLLECTION (Physical)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0xc0;

    } // X, Y, Z, Rx, Ry, and Rz Axis

  if (simulationCount > 0)
    {

      // USAGE_PAGE (Simulation Controls)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x05;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x02;

      // LOGICAL_MINIMUM (-32767)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x16;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x01;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x80;

      // LOGICAL_MAXIMUM (+32767)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x26;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0xFF;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x7F;

      // REPORT_SIZE (16)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x75;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x10;

      // REPORT_COUNT (simulationCount)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x95;
      tempHidReportDescriptor[hidReportDescriptorSize++] = simulationCount;

      // COLLECTION (Physical)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0xA1;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x00;

      if (includeRudder == true)
	{
	  // USAGE (Rudder)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x09;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = HID_USAGE_RUDDER;
	}

      if (includeThrottle == true)
	{
	  // USAGE (Throttle)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x09;
	  tempHidReportDescriptor[hidReportDescriptorSize++] =
	  HID_USAGE_THROTTLER;
	}

      if (includeAccelerator == true)
	{
	  // USAGE (Accelerator)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x09;
	  tempHidReportDescriptor[hidReportDescriptorSize++] =
	  HID_USAGE_ACCELERATOR;
	}

      if (includeBrake == true)
	{
	  // USAGE (Brake)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x09;
	  tempHidReportDescriptor[hidReportDescriptorSize++] = HID_USAGE_BRAKE;
	}

      if (includeSteering == true)
	{
	  // USAGE (Steering)
	  tempHidReportDescriptor[hidReportDescriptorSize++] = 0x09;
	  tempHidReportDescriptor[hidReportDescriptorSize++] =
	  HID_USAGE_STEERING;
	}

      // INPUT (Data,Var,Abs)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x81;
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0x02;

      //END_COLLECTION (Physical)
      tempHidReportDescriptor[hidReportDescriptorSize++] = 0xc0;
    } // Simulation Controls

  // END_COLLECTION
  tempHidReportDescriptor[hidReportDescriptorSize++] = 0xc0;

  // Create a copy of the HID Report Descriptor template that is just the right size
  JoystickHIDReportDescr_Size = hidReportDescriptorSize + pidReportDescriptorSize;
  JoystickHIDReportDescr = new uint8_t[JoystickHIDReportDescr_Size];
  memcpy (JoystickHIDReportDescr, tempHidReportDescriptor, hidReportDescriptorSize);
  memcpy (JoystickHIDReportDescr + hidReportDescriptorSize, pidReportDescriptor, pidReportDescriptorSize);
  //printf("Joystick: Created HID Report Descriptor done.\n");


  // Setup Joystick State
  if (buttonCount > 0)
    {
      _buttonValuesArraySize = _buttonCount / 8;
      if ((_buttonCount % 8) > 0)
	{
	  _buttonValuesArraySize++;
	}
      _buttonValues = new uint8_t[_buttonValuesArraySize];
    }

  // Calculate HID Report Size
  _hidReportSize = _buttonValuesArraySize;
  _hidReportSize += (_hatSwitchCount > 0);
  _hidReportSize += (axisCount * 2);
  _hidReportSize += (simulationCount * 2);
  Last_HidReport = new uint8_t[_hidReportSize];

  // Initalize Joystick State
  _xAxis = 0;
  _yAxis = 0;
  _zAxis = 0;
  _xAxisRotation = 0;
  _yAxisRotation = 0;
  _zAxisRotation = 0;
  _throttle = 0;
  _rudder = 0;
  _accelerator = 0;
  _brake = 0;
  _steering = 0;
  for (int index = 0; index < JOYSTICK_HATSWITCH_COUNT_MAXIMUM; index++)
    {
      _hatSwitchValues[index] = JOYSTICK_HATSWITCH_RELEASE;
    }
  for (int index = 0; index < _buttonValuesArraySize; index++)
    {
      _buttonValues[index] = 0;
    }
}

void Joystick_::begin (bool initAutoSendState)
{
  _autoSendState = initAutoSendState;
  //printf("Joystick: Init done.\n");
  sendState ();
}

void Joystick_::setButton (uint8_t button, uint8_t value)
{
  if (value == 0)
    {
      releaseButton (button);
    }
  else
    {
      pressButton (button);
    }
}

void Joystick_::pressButton (uint8_t button)
{
  if (button >= _buttonCount)
    return;

  int index = button / 8;
  int bit = button % 8;

  _buttonValues[index] |= (1UL << bit);		//((value) |= (1UL << (bit)))
  if (_autoSendState)
    sendState ();
}

void Joystick_::releaseButton (uint8_t button)
{
  if (button >= _buttonCount)
    return;

  int index = button / 8;
  int bit = button % 8;

  _buttonValues[index] &= ~(1UL << bit);	//((value) &= ~(1UL << (bit)))
  if (_autoSendState)
    sendState ();
}

void Joystick_::setXAxis (int16_t value)
{
  _xAxis = value;
  if (_autoSendState)
    sendState ();
}

void Joystick_::setYAxis (int16_t value)
{
  _yAxis = value;
  if (_autoSendState)
    sendState ();
}

void Joystick_::setZAxis (int16_t value)
{
  _zAxis = value;
  if (_autoSendState)
    sendState ();
}

void Joystick_::setRxAxis (int16_t value)
{
  _xAxisRotation = value;
  if (_autoSendState)
    sendState ();
}

void Joystick_::setRyAxis (int16_t value)
{
  _yAxisRotation = value;
  if (_autoSendState)
    sendState ();
}

void Joystick_::setRzAxis (int16_t value)
{
  _zAxisRotation = value;
  if (_autoSendState)
    sendState ();
}

void Joystick_::setRudder (int16_t value)
{
  _rudder = value;
  if (_autoSendState)
    sendState ();
}

void Joystick_::setThrottle (int16_t value)
{
  _throttle = value;
  if (_autoSendState)
    sendState ();
}

void Joystick_::setAccelerator (int16_t value)
{
  _accelerator = value;
  if (_autoSendState)
    sendState ();
}

void Joystick_::setBrake (int16_t value)
{
  _brake = value;
  if (_autoSendState)
    sendState ();
}

void
Joystick_::setSteering (int16_t value)
{
  _steering = value;
  if (_autoSendState)
    sendState ();
}

void Joystick_::setHatSwitch (int8_t hatSwitchIndex, int16_t value)
{
  if (hatSwitchIndex >= _hatSwitchCount)
    return;

  _hatSwitchValues[hatSwitchIndex] = value;
  if (_autoSendState)
    sendState ();
}

int Joystick_::buildAndSet16BitValue (bool includeValue, int16_t value,
				  int16_t valueMinimum, int16_t valueMaximum,
				  int16_t actualMinimum, int16_t actualMaximum,
				  uint8_t dataLocation[])
{
  int16_t convertedValue;
  uint8_t highByte;
  uint8_t lowByte;
  int16_t realMinimum =
      (valueMinimum < valueMaximum) ? valueMinimum : valueMaximum;//min(valueMinimum, valueMaximum);
  int16_t realMaximum =
      (valueMinimum > valueMaximum) ? valueMinimum : valueMaximum;//max(valueMinimum, valueMaximum);

  if (includeValue == false)
    return 0;

  if (value < realMinimum)
    {
      value = realMinimum;
    }
  if (value > realMaximum)
    {
      value = realMaximum;
    }

  if (valueMinimum > valueMaximum)
    {
      // Values go from a larger number to a smaller number (e.g. 1024 to 0)
      value = realMaximum - value + realMinimum;
    }

  convertedValue = map (value, realMinimum, realMaximum, actualMinimum,
			actualMaximum);

  highByte = (uint8_t) (convertedValue >> 8);
  lowByte = (uint8_t) (convertedValue & 0x00FF);

  dataLocation[0] = lowByte;
  dataLocation[1] = highByte;

  return 2;
}

int Joystick_::buildAndSetAxisValue (bool includeAxis, int16_t axisValue,
				 int16_t axisMinimum, int16_t axisMaximum,
				 uint8_t dataLocation[])
{
  return buildAndSet16BitValue (includeAxis, axisValue, axisMinimum,
				axisMaximum, JOYSTICK_AXIS_MINIMUM,
				JOYSTICK_AXIS_MAXIMUM,
				dataLocation);
}

int Joystick_::buildAndSetSimulationValue (bool includeValue, int16_t value,
				       int16_t valueMinimum,
				       int16_t valueMaximum,
				       uint8_t dataLocation[])
{
  return buildAndSet16BitValue (includeValue, value, valueMinimum, valueMaximum,
  JOYSTICK_SIMULATOR_MINIMUM,
				JOYSTICK_SIMULATOR_MAXIMUM, dataLocation);
}

void Joystick_::sendState ()
{
  uint8_t data[_hidReportSize] = { 0 };
  int index = 0;

  // Load Button State
  for (; index < _buttonValuesArraySize; index++)
    {
      data[index] = _buttonValues[index];
    }

  // Set Hat Switch Values
  if (_hatSwitchCount > 0)
    {

      // Calculate hat-switch values
      uint8_t convertedHatSwitch[JOYSTICK_HATSWITCH_COUNT_MAXIMUM];
      for (int hatSwitchIndex = 0;
	  hatSwitchIndex < JOYSTICK_HATSWITCH_COUNT_MAXIMUM; hatSwitchIndex++)
	{
	  if (_hatSwitchValues[hatSwitchIndex] < 0)
	    {
	      convertedHatSwitch[hatSwitchIndex] = 8;
	    }
	  else
	    {
	      convertedHatSwitch[hatSwitchIndex] =
		  (_hatSwitchValues[hatSwitchIndex] % 360) / 45;
	    }
	}

      // Pack hat-switch states into a single byte
      data[index++] = (convertedHatSwitch[1] << 4)
	  | (0b00001111 & convertedHatSwitch[0]);

    } // Hat Switches

  // Set Axis Values
  index += buildAndSetAxisValue (_includeAxisFlags & JOYSTICK_INCLUDE_X_AXIS, _xAxis, _xAxisMinimum, _xAxisMaximum, &(data[index]));
  index += buildAndSetAxisValue (_includeAxisFlags & JOYSTICK_INCLUDE_Y_AXIS, _yAxis, _yAxisMinimum, _yAxisMaximum, &(data[index]));
  index += buildAndSetAxisValue (_includeAxisFlags & JOYSTICK_INCLUDE_Z_AXIS, _zAxis, _zAxisMinimum, _zAxisMaximum, &(data[index]));
  index += buildAndSetAxisValue (_includeAxisFlags & JOYSTICK_INCLUDE_RX_AXIS, _xAxisRotation, _rxAxisMinimum, _rxAxisMaximum, &(data[index]));
  index += buildAndSetAxisValue (_includeAxisFlags & JOYSTICK_INCLUDE_RY_AXIS, _yAxisRotation, _ryAxisMinimum, _ryAxisMaximum, &(data[index]));
  index += buildAndSetAxisValue (_includeAxisFlags & JOYSTICK_INCLUDE_RZ_AXIS, _zAxisRotation, _rzAxisMinimum, _rzAxisMaximum, &(data[index]));

  // Set Simulation Values
  index += buildAndSetSimulationValue (_includeSimulatorFlags & JOYSTICK_INCLUDE_RUDDER, _rudder, _rudderMinimum, _rudderMaximum, &(data[index]));
  index += buildAndSetSimulationValue (_includeSimulatorFlags & JOYSTICK_INCLUDE_THROTTLE, _throttle, _throttleMinimum, _throttleMaximum, &(data[index]));
  index += buildAndSetSimulationValue (_includeSimulatorFlags & JOYSTICK_INCLUDE_ACCELERATOR, _accelerator, _acceleratorMinimum, _acceleratorMaximum, &(data[index]));
  index += buildAndSetSimulationValue (_includeSimulatorFlags & JOYSTICK_INCLUDE_BRAKE, _brake, _brakeMinimum, _brakeMaximum, &(data[index]));
  index += buildAndSetSimulationValue (_includeSimulatorFlags & JOYSTICK_INCLUDE_STEERING, _steering, _steeringMinimum, _steeringMaximum, &(data[index]));

  uint8_t _report[_hidReportSize + 1];
  _report[0] = _hidReportId;
  memcpy (&_report[1], data, _hidReportSize);
  uint8_t n = memcmp(data, Last_HidReport,_hidReportSize);
  uint32_t time = HAL_GetTick();
  if(( time - JoystickReportTime >= 5) && (n != 0))
  {
	  int8_t result = USBD_JOYSTICK_HID_SendReport_FS ( _report, (uint16_t) _hidReportSize + 1);
	    if (result == USBD_OK)
	      {
	   			 JoystickReportTime = HAL_GetTick();
	      }

  }
  else if( time - JoystickReportTime >= 10)
  {

	  int8_t result = USBD_JOYSTICK_HID_SendReport_FS ( _report, (uint16_t) _hidReportSize + 1);
	  	    if (result == USBD_OK)
	  	      {
	  	   			 JoystickReportTime = HAL_GetTick();
	  	      }
  }
  memcpy (Last_HidReport, data, _hidReportSize);

}

