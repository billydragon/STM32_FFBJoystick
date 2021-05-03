#include <usbd_joystick_hid_if.h>
#include "PIDReportHandler.h"
#include "main.h"
#include "FFBMain.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

PIDReportHandler::PIDReportHandler ()
{
  nextEID = 1;
  devicePaused = 0;
}

PIDReportHandler::~PIDReportHandler ()
{
  FreeAllEffects ();
}

uint8_t PIDReportHandler::GetNextFreeEffect (void)
{
  if (nextEID == MAX_EFFECTS)
    return 0;

  uint8_t id = nextEID++;

  while (g_EffectStates[nextEID].state != 0)
    {
      if (nextEID >= MAX_EFFECTS)
	break;  // the last spot was taken
      nextEID++;
    }

  g_EffectStates[id].state = MEFFECTSTATE_ALLOCATED;

  return id;
}


void PIDReportHandler::StopAllEffects (void)
{
  for (uint8_t id = 0; id <= MAX_EFFECTS; id++)
  {
	  StopEffect (id);
  }
}

void PIDReportHandler::StartEffect (uint8_t id)
{
  if (id > MAX_EFFECTS)
    return;
  g_EffectStates[id].state = MEFFECTSTATE_PLAYING;
  g_EffectStates[id].elapsedTime = 0;
  g_EffectStates[id].startTime = (uint64_t) HAL_GetTick();

}

void PIDReportHandler::StopEffect (uint8_t id)
{
  if ((id > MAX_EFFECTS) || (g_EffectStates[id].state == 0))
    return;
  g_EffectStates[id].state &= ~MEFFECTSTATE_PLAYING;
  pidBlockLoad.ramPoolAvailable += SIZE_EFFECT;
}

void PIDReportHandler::FreeEffect (uint8_t id)
{
  if (id ==0  || id > MAX_EFFECTS)
    return;

  g_EffectStates[id].state = FFB_EFFECT_NONE;
  if (id < nextEID)
    nextEID = id;
}

void PIDReportHandler::FreeAllEffects (void)
{
  nextEID = 1;
  memset ((void*) &g_EffectStates, 0, sizeof(g_EffectStates));
  pidBlockLoad.ramPoolAvailable = MEMORY_SIZE;
  pidState.effectBlockIndex = 0;
  pidState.status = (ACTUATOR_POWER) | (ACTUATORS_ENABLED);
}

void PIDReportHandler::EffectOperation (USB_FFBReport_EffectOperation_Output_Data_t *data)
{
	uint8_t effectId = data->effectBlockIndex;
	uint8_t op = data->operation;

	switch(op)
	{
		case 1:
				if (data->loopCount > 0)
			    	g_EffectStates[effectId].duration *= data->loopCount;
				   Bset(pidState.status,EFFECT_PLAYING);
			      StartEffect (effectId);

			break;
		case 2:
					StopAllEffects ();
					Bset(pidState.status,EFFECT_PLAYING);
			      StartEffect (effectId);
			break;
		case 3:
					Bclr(pidState.status,EFFECT_PLAYING);
					StopEffect (effectId);
			break;
		default:
			break;

	}

}

void PIDReportHandler::BlockFree (USB_FFBReport_BlockFree_Output_Data_t *data)
{
  uint8_t eid = data->effectBlockIndex;

  if (eid == 0xFF)
    { // all effects
      FreeAllEffects ();
    }
  else
    {
      FreeEffect (eid);
    }
}

void PIDReportHandler::sendStatusReport(uint8_t effectID){
		  pidState.reportId = 2;

		  Bset(pidState.status,ACTUATOR_POWER);
		  Bset(pidState.status, SAFETY_SWITCH);
		  pidState.effectBlockIndex = effectID;

		if(effectID > 0 && g_EffectStates[effectID].state == MEFFECTSTATE_PLAYING)
			Bset(pidState.status,EFFECT_PLAYING);


	 USBD_JOYSTICK_HID_SendReport_FS ((uint8_t *)&pidState, sizeof(USB_FFBReport_PIDStatus_Input_Data_t));

}

void PIDReportHandler::DeviceControl (
    USB_FFBReport_DeviceControl_Output_Data_t *data)
{

	  uint8_t cmd = data->control;
		// Control: 1=Enable Actuators, 2=Disable Actuators, 4=Stop All Effects, 8=Reset, 16=Pause, 32=Continue
		// Status Bits: Status Bits: 0=Device Paused,1=Actuators Enabled,2=Safety Switch,3=Actuator Power, 4=Play, 5=Actuator Override Switch,
	  if(cmd & 0x01) //1=Enable Actuators
	  {
				 Bset(pidState.status, ACTUATORS_ENABLED);
				 FFB_Active = true;
	  }

	  if(cmd & 0x02) //2=Disable Actuators
	  {

				 Bclr(pidState.status, ACTUATORS_ENABLED);
	  }

	  if(cmd & 0x03) //Stop All Effects Some app send
	  {
				 StopAllEffects ();
				 FFB_Active = false;

	  }

	  if(cmd & 0x04) //4=Stop All Effects Some app send as device reset
	  {
				 StopAllEffects ();
				 FreeAllEffects ();
				 FFB_Active = false;
	  }

	  if(cmd & 0x08) //8=Reset
	  {
				 StopAllEffects ();
				 FreeAllEffects ();
				 FFB_Active = false;
	  }

	  if(cmd & 0x10) //16=Pause
	  {
				 Bset(pidState.status, DEVICE_PAUSED);
				 devicePaused = 1;
	  }

	  if(cmd & 0x20) //32=Continue
	  {
				 Bclr(pidState.status, DEVICE_PAUSED);
				 devicePaused = 0;
	  }

}

void PIDReportHandler::DeviceGain (USB_FFBReport_DeviceGain_Output_Data_t *data)
{
  deviceGain.gain = data->gain;
}

void PIDReportHandler::SetCustomForce (USB_FFBReport_SetCustomForce_Output_Data_t *data)
{
}

void PIDReportHandler::SetCustomForceData (USB_FFBReport_SetCustomForceData_Output_Data_t *data)
{
}

void PIDReportHandler::SetDownloadForceSample (USB_FFBReport_SetDownloadForceSample_Output_Data_t *data)
{
}

void PIDReportHandler::SetEffect (USB_FFBReport_SetEffect_Output_Data_t *data)
{
	uint8_t index = data->effectBlockIndex;
		if(index > MAX_EFFECTS || index == 0)
			return;

  volatile TEffectState *effect = &g_EffectStates[data->effectBlockIndex];

  effect->duration = data->duration;
  effect->directionX = data->directionX;
  effect->directionY = data->directionY;
  effect->effectType = data->effectType;
  effect->gain = data->gain;
  effect->enableAxis = data->enableAxis;

}

void PIDReportHandler::SetEnvelope (USB_FFBReport_SetEnvelope_Output_Data_t *data, volatile TEffectState *effect)
{
  effect->attackLevel = data->attackLevel;
  effect->fadeLevel = data->fadeLevel;
  effect->attackTime = data->attackTime;
  effect->fadeTime = data->fadeTime;
  effect->useEnvelope = true;
}

void PIDReportHandler::SetCondition (USB_FFBReport_SetCondition_Output_Data_t *data, volatile TEffectState *effect)
{
  uint8_t axis = data->parameterBlockOffset;
  effect->axesIdx = (data->parameterBlockOffset + 1 > effect->axesIdx) ? (data->parameterBlockOffset + 1) : (effect->axesIdx);
  effect->conditions[axis].cpOffset = data->cpOffset;
  effect->conditions[axis].positiveCoefficient = data->positiveCoefficient;
  effect->conditions[axis].negativeCoefficient = data->negativeCoefficient;
  effect->conditions[axis].positiveSaturation = data->positiveSaturation;
  effect->conditions[axis].negativeSaturation = data->negativeSaturation;
  effect->conditions[axis].deadBand = data->deadBand;
  //effect->axesIdx = (axis + 1 > effect->axesIdx) ? (axis + 1) : (effect->axesIdx);

  effect->conditionBlocksCount++;

}

void PIDReportHandler::SetPeriodic (USB_FFBReport_SetPeriodic_Output_Data_t *data, volatile TEffectState *effect)
{
  effect->magnitude = data->magnitude;
  effect->offset = data->offset;
  effect->phase = data->phase;
  effect->period = constrain(data->period,1,32767); //prevent priod = 0
}

void PIDReportHandler::SetConstantForce (USB_FFBReport_SetConstantForce_Output_Data_t *data,volatile TEffectState *effect)
{
  //  ReportPrint(*effect);
  effect->magnitude = data->magnitude;
}

void PIDReportHandler::SetRampForce (USB_FFBReport_SetRampForce_Output_Data_t *data,
				volatile TEffectState *effect)
{
  effect->startMagnitude = data->startMagnitude;
  effect->endMagnitude = data->endMagnitude;
  // Full magnitude for envelope calculation. This effect does not have a periodic report
  effect->magnitude = 10000;
}

void PIDReportHandler::CreateNewEffect (USB_FFBReport_CreateNewEffect_Feature_Data_t *inData)
{
  pidBlockLoad.reportId = 6;
  pidBlockLoad.effectBlockIndex = GetNextFreeEffect ();

  if (pidBlockLoad.effectBlockIndex == 0)
    {
      pidBlockLoad.loadStatus = 2;    // 1=Success,2=Full,3=Error
    }
  else
    {
      pidBlockLoad.loadStatus = 1;    // 1=Success,2=Full,3=Error

      volatile TEffectState *effect = &g_EffectStates[pidBlockLoad.effectBlockIndex];

      memset ((void*) effect, 0, sizeof(TEffectState));
      effect->state = MEFFECTSTATE_ALLOCATED;
      pidBlockLoad.ramPoolAvailable -= SIZE_EFFECT;
      pidState.effectBlockIndex = pidBlockLoad.effectBlockIndex;
    }

}

void PIDReportHandler::UppackUsbData (uint8_t *data, uint16_t len)
{

  uint8_t effectId = data[1]; // effectBlockIndex is always the second byte.
  switch (data[0]) // reportID
    {
    case 1:
      SetEffect ((USB_FFBReport_SetEffect_Output_Data_t*) data);
      break;
    case 2: //My Device don't work on ReportID(2) -> Move to ReportID(9)
      SetEnvelope ((USB_FFBReport_SetEnvelope_Output_Data_t*) data, &g_EffectStates[effectId]);
      break;
    case 3:
      SetCondition ((USB_FFBReport_SetCondition_Output_Data_t*) data, &g_EffectStates[effectId]);
      break;
    case 4:
      SetPeriodic ((USB_FFBReport_SetPeriodic_Output_Data_t*) data, &g_EffectStates[effectId]);
      break;
    case 5:
      SetConstantForce ((USB_FFBReport_SetConstantForce_Output_Data_t*) data, &g_EffectStates[effectId]);
      break;
    case 6:
      SetRampForce ((USB_FFBReport_SetRampForce_Output_Data_t*) data, &g_EffectStates[effectId]);
      break;
    case 7:
      SetCustomForceData ((USB_FFBReport_SetCustomForceData_Output_Data_t*) data);
      break;
    case 8:
      SetDownloadForceSample ((USB_FFBReport_SetDownloadForceSample_Output_Data_t*) data);
      break;
    case 9:
   	 SetEnvelope ((USB_FFBReport_SetEnvelope_Output_Data_t*) data, &g_EffectStates[effectId]);
      break;
    case 10:
      EffectOperation ((USB_FFBReport_EffectOperation_Output_Data_t*) data);
      sendStatusReport(effectId);
      break;
    case 11:
      BlockFree ((USB_FFBReport_BlockFree_Output_Data_t*) data);
      break;
    case 12:
      DeviceControl ((USB_FFBReport_DeviceControl_Output_Data_t*) data);
      sendStatusReport(0);
      break;
    case 13:
      DeviceGain ((USB_FFBReport_DeviceGain_Output_Data_t*) data);
      break;
    case 14:
      SetCustomForce ((USB_FFBReport_SetCustomForce_Output_Data_t*) data);
      break;
    case 0x11:
      CreateNewEffect ((USB_FFBReport_CreateNewEffect_Feature_Data_t*) data);
      break;
    default:
      break;
    }
}

uint8_t* PIDReportHandler::getPIDPool ()
{
  FreeAllEffects ();

  pidPoolReport.reportId = 7;
  pidPoolReport.ramPoolSize = MEMORY_SIZE;
  pidPoolReport.maxSimultaneousEffects = MAX_EFFECTS;
  pidPoolReport.memoryManagement = 3;
  return (uint8_t*) &pidPoolReport;
}

uint8_t* PIDReportHandler::getPIDBlockLoad ()
{
  return (uint8_t*) &pidBlockLoad;
}

uint8_t* PIDReportHandler::getPIDStatus ()
{
  return (uint8_t*) &pidState;
}

