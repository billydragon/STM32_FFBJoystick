/*
 * PIDReportType.h
 *
 *  Created on: Jan 24, 2021
 *      Author: billy
 */

#ifndef INC_PIDREPORTTYPE_H_
#define INC_PIDREPORTTYPE_H_
#include "cppmain.h"
#include <stdbool.h>


#define MAX_EFFECTS 40
#define FFB_ID_OFFSET 0x00

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

#define FFB_AXIS_COUNT 2
#define SIZE_EFFECT sizeof(TEffectState)
#define MEMORY_SIZE (uint16_t)(MAX_EFFECTS*SIZE_EFFECT)
#define TO_LT_END_16(x) ((x<<8)&0xFF00)|((x>>8)&0x00FF)

////refer to FFBDescriptor.h

///Device-->Host

typedef struct //PID state
{
  uint8_t reportId; //2
  uint8_t status; // Bits: 0=Device Paused,1=Actuators Enabled,2=Safety Switch,3=Actuator Override Switch,4=Actuator Power
  uint8_t effectBlockIndex; // Bit7=Effect Playing, Bit0..7=EffectId (1..40)
} USB_FFBReport_PIDStatus_Input_Data_t;

///Host-->Device

typedef struct //FFB: Set Effect Output Report
{
  uint8_t reportId;	// =1
  uint8_t effectBlockIndex;	// 1..40
  uint8_t effectType;// 1..12 (effect usages: 26,27,30,31,32,33,34,40,41,42,43,28)
  uint16_t duration; // 0..32767 ms
  uint16_t triggerRepeatInterval; // 0..32767 ms
  uint16_t samplePeriod;	// 0..32767 ms
  uint16_t gain;	// 0..255	 (physical 0..10000)
  uint8_t triggerButton;	// button ID (0..8)
  uint8_t enableAxis; // bits: 0=X, 1=Y, 2=DirectionEnable
  uint16_t directionX;	// angle (0=0 .. 35999=360deg)
  uint16_t directionY;	// angle (0=0 .. 35999=360deg)
  //	uint16_t	startDelay;	// 0..32767 ms
} __attribute__((packed)) USB_FFBReport_SetEffect_Output_Data_t;

typedef struct	//FFB: Set Envelope Output Report
{
  uint8_t reportId;	// =2
  uint8_t effectBlockIndex;	// 1..40
  uint16_t attackLevel;
  uint16_t fadeLevel;
  uint16_t attackTime;	// ms
  uint16_t fadeTime;	// ms
} __attribute__((packed)) USB_FFBReport_SetEnvelope_Output_Data_t;

typedef struct	// FFB: Set Condition Output Report
{
  uint8_t reportId;	// =3
  uint8_t effectBlockIndex;	// 1..40
  uint8_t parameterBlockOffset;	// bits: 0..3=parameterBlockOffset, 4..5=instance1, 6..7=instance2
  int16_t cpOffset;	// -10000..10000
  int16_t positiveCoefficient;	// 0..10000
  int16_t negativeCoefficient;	// 0..10000
  uint16_t positiveSaturation;	// 0..10000
  uint16_t negativeSaturation;	// 0..10000
  uint16_t deadBand;	// 0..10000
} __attribute__((packed)) USB_FFBReport_SetCondition_Output_Data_t;

typedef struct	//FFB: Set Periodic Output Report
{
  uint8_t reportId;	// =4
  uint8_t effectBlockIndex;	// 1..40
  uint16_t magnitude;
  int16_t offset;
  uint16_t phase;	// 0..35999 (=0..359, exp-2)
  uint32_t period;	// 0..32767 ms
} __attribute__((packed)) USB_FFBReport_SetPeriodic_Output_Data_t;

typedef struct	//FFB: Set ConstantForce Output Report
{
  uint8_t reportId;	// =5
  uint8_t effectBlockIndex;	// 1..40
  int16_t magnitude;	// -10000..10000
} __attribute__((packed)) USB_FFBReport_SetConstantForce_Output_Data_t;

typedef struct	//FFB: Set RampForce Output Report
{
  uint8_t reportId;	// =6
  uint8_t effectBlockIndex;	// 1..40
  int16_t startMagnitude;
  int16_t endMagnitude;
} __attribute__((packed)) USB_FFBReport_SetRampForce_Output_Data_t;

typedef struct	//FFB: Set CustomForceData Output Report
{
  uint8_t reportId;	// =7
  uint8_t effectBlockIndex;	// 1..40
  uint16_t dataOffset;
  int8_t data[12];
} __attribute__((packed)) USB_FFBReport_SetCustomForceData_Output_Data_t;

typedef struct	//FFB: Set DownloadForceSample Output Report
{
  uint8_t reportId;	// =8
  int8_t x;
  int8_t y;
} __attribute__((packed)) USB_FFBReport_SetDownloadForceSample_Output_Data_t;

typedef struct	//FFB: Set EffectOperation Output Report
{
  uint8_t reportId;	// =10
  uint8_t effectBlockIndex;	// 1..40
  uint8_t operation; // 1=Start, 2=StartSolo, 3=Stop
  uint8_t loopCount;
} __attribute__((packed)) USB_FFBReport_EffectOperation_Output_Data_t;

typedef struct //FFB: Block Free Output Report
{
  uint8_t reportId;	// =11
  uint8_t effectBlockIndex;	// 1..40
} __attribute__((packed)) USB_FFBReport_BlockFree_Output_Data_t;

typedef struct	//FFB: Device Control Output Report
{
  uint8_t reportId;	// =12
  uint8_t control;// 1=Enable Actuators, 2=Disable Actuators, 4=Stop All Effects, 8=Reset, 16=Pause, 32=Continue
} __attribute__((packed)) USB_FFBReport_DeviceControl_Output_Data_t;

typedef struct	//FFB: DeviceGain Output Report
{
  uint8_t reportId;	// =13
  uint16_t gain;
} __attribute__((packed)) USB_FFBReport_DeviceGain_Output_Data_t;

typedef struct	// FFB: Set Custom Force Output Report
{
  uint8_t reportId;	// =14
  uint8_t effectBlockIndex;	// 1..40
  uint8_t sampleCount;
  uint16_t samplePeriod;	// 0..32767 ms
} __attribute__((packed)) USB_FFBReport_SetCustomForce_Output_Data_t;

///Feature
typedef struct //FFB: Create New Effect Feature Report
{
  uint8_t reportId;	//5
  uint8_t effectType;	// Enum (1..12): ET 26,27,30,31,32,33,34,40,41,42,43,28
  uint16_t byteCount;	// 0..511
} __attribute__((packed)) USB_FFBReport_CreateNewEffect_Feature_Data_t;

typedef struct	// FFB: PID Block Load Feature Report
{
  uint8_t reportId;	// =6
  uint8_t effectBlockIndex;	// 1..40
  uint8_t loadStatus;	// 1=Success,2=Full,3=Error
  uint16_t ramPoolAvailable;	// =0 or 0xFFFF?
} __attribute__((packed)) USB_FFBReport_PIDBlockLoad_Feature_Data_t;

typedef struct	// FFB: PID Pool Feature Report
{
  uint8_t reportId;	// =7
  uint16_t ramPoolSize;	// ?
  uint8_t maxSimultaneousEffects;	// ?? 40?
  uint8_t memoryManagement;// Bits: 0=DeviceManagedPool, 1=SharedParameterBlocks
} __attribute__((packed)) USB_FFBReport_PIDPool_Feature_Data_t;

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

///effect
#define USB_DURATION_INFINITE		0xFFFF
#define USB_EFFECT_CONSTANT	  		0x01
#define USB_EFFECT_RAMP				0x02
#define USB_EFFECT_SQUARE 			0x03
#define USB_EFFECT_SINE 			0x04
#define USB_EFFECT_TRIANGLE			0x05
#define USB_EFFECT_SAWTOOTHDOWN		0x06
#define USB_EFFECT_SAWTOOTHUP		0x07
#define USB_EFFECT_SPRING			0x08
#define USB_EFFECT_DAMPER			0x09
#define USB_EFFECT_INERTIA			0x0A
#define USB_EFFECT_FRICTION			0x0B
#define USB_EFFECT_CUSTOM			0x0C
// Bit-masks for effect states
#define MEFFECTSTATE_FREE			0x00
#define MEFFECTSTATE_ALLOCATED		0x01
#define MEFFECTSTATE_PLAYING		0x02

#define X_AXIS_ENABLE				0x01
#define Y_AXIS_ENABLE				0x02
#define DIRECTION_ENABLE			0x04
//these were needed for testing
#define INERTIA_FORCE 				0xFF
#define FRICTION_FORCE				0xFF
#define INERTIA_DEADBAND			0x30
#define FRICTION_DEADBAND			0x30
#define EFFECT_STATE_INACTIVE 		0

// Only include these for cpp
#ifdef __cplusplus


typedef struct
{
  int16_t cpOffset; // -128..127
  int16_t positiveCoefficient; // -128..127
  int16_t negativeCoefficient; // -128..127
  uint16_t positiveSaturation;  // -128..127
  uint16_t negativeSaturation;  // -128..127
  uint16_t deadBand;  // 0..255
} TEffectCondition;

typedef struct
{
  volatile uint8_t state;  // see constants <MEffectState_*>
  uint8_t effectType; //
  int16_t offset;
  uint16_t gain;
  int16_t attackLevel;
  int16_t fadeLevel;
  int16_t magnitude;
  uint8_t enableAxis; // bits: 0=X, 1=Y, 2=DirectionEnable
  uint16_t directionX; // angle (0=0 .. 35999=360deg)
  uint16_t directionY; // angle (0=0 .. 35999=360deg)
  uint8_t axesIdx;
  uint8_t conditionBlocksCount;
  TEffectCondition conditions[FFB_AXIS_COUNT];

  uint16_t phase;  // 0..255 (=0..359, exp-2)
  int16_t startMagnitude;
  int16_t endMagnitude;
  uint16_t period; // 0..32767 ms
  uint32_t duration, fadeTime, attackTime, elapsedTime;
  uint64_t startTime;
  bool useEnvelope;
} TEffectState;


#endif
#endif /* INC_PIDREPORTTYPE_H_ */
