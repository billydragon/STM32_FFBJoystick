#pragma once
#ifndef _FFB_H
#define _FFB_H

static const uint8_t pidReportDescriptor[]
  {

		  // PID State Report
		  0x05, 0x0F,        //   Usage Page (PID Page)
		  0x09, 0x92,        //   Usage (PID State Report)
		  0xA1, 0x02,        //   Collection (Logical)
			  0x85, 0x02,        //     Report ID (2)
			  0x09, 0x9F,        //     Usage (Device Paused)
			  0x09, 0xA0,        //     Usage (Actuators Enabled)
			  0x09, 0xA4,        //     Usage (Safety Switch)
			  0x09, 0xA5,        //     Usage (Actuator Override Switch)
			  0x09, 0xA6,        //     Usage (Actuator Power)
			  0x15, 0x00,        //     Logical Minimum (0)
			  0x25, 0x01,        //     Logical Maximum (1)
			  0x35, 0x00,        //     Physical Minimum (0)
			  0x45, 0x01,        //     Physical Maximum (1)
			  0x75, 0x01,        //     Report Size (1)
			  0x95, 0x05,        //     Report Count (5)
			  0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
			  0x95, 0x03,        //     Report Count (3)
			  0x81, 0x03,        //     Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
			  0x09, 0x94,        //     Usage (Effect Playing)
			  0x15, 0x00,        //     Logical Minimum (0)
			  0x25, 0x01,        //     Logical Maximum (1)
			  0x35, 0x00,        //     Physical Minimum (0)
			  0x45, 0x01,        //     Physical Maximum (1)
			  0x75, 0x01,        //     Report Size (1)
			  0x95, 0x01,        //     Report Count (1)
			  0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
			  0x09, 0x22,        //     Usage (Effect Block Index)
			  0x15, 0x01,        //     Logical Minimum (1)
			  0x25, 0x28,        //     Logical Maximum (40)
			  0x35, 0x01,        //     Physical Minimum (1)
			  0x45, 0x28,        //     Physical Maximum (40)
			  0x75, 0x07,        //     Report Size (7)
			  0x95, 0x01,        //     Report Count (1)
			  0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
		  0xC0,              //   End Collection
		  0x09, 0x21,        //   Usage (Set Effect Report)
		  0xA1, 0x02,        //   Collection (Logical)
		  0x85, 0x01,        //     Report ID (1)
		  0x09, 0x22,        //     Usage (Effect Block Index)
		  0x15, 0x01,        //     Logical Minimum (1)
		  0x25, 0x28,        //     Logical Maximum (40)
		  0x35, 0x01,        //     Physical Minimum (1)
		  0x45, 0x28,        //     Physical Maximum (40)
		  0x75, 0x08,        //     Report Size (8)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x25,        //     Usage (Effect Type)
		  0xA1, 0x02,        //     Collection (Logical)
		  0x09, 0x26,        //       Usage (ET Constant Force)
		  0x09, 0x27,        //       Usage (ET Ramp)
		  0x09, 0x30,        //       Usage (ET Square)
		  0x09, 0x31,        //       Usage (ET Sine)
		  0x09, 0x32,        //       Usage (ET Triangle)
		  0x09, 0x33,        //       Usage (ET Sawtooth Up)
		  0x09, 0x34,        //       Usage (ET Sawtooth Down)
		  0x09, 0x40,        //       Usage (ET Spring)
		  0x09, 0x41,        //       Usage (ET Damper)
		  0x09, 0x42,        //       Usage (ET Inertia)
		  0x09, 0x43,        //       Usage (ET Friction)
		  0x09, 0x28,        //       Usage (ET Custom Force Data)
		  0x15, 0x01,        //       Logical Minimum (1)
		  0x25, 0x0C,        //       Logical Maximum (12)
		  0x45, 0x0C,        //       Physical Maximum (12)
		  0x35, 0x01,        //       Physical Minimum (1)
		  0x75, 0x08,        //       Report Size (8)
		  0x95, 0x01,        //       Report Count (1)
		  0x91, 0x00,        //       Output (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0xC0,              //     End Collection
		  0x09, 0x50,        //     Usage (Duration)
		  0x09, 0x54,        //     Usage (Trigger Repeat Interval)
		  0x09, 0x51,        //     Usage (Sample Period)
		  0x16, 0x00, 0x00,  //     Logical Minimum (0)
		  0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
		  0x36, 0x00, 0x00,  //     Physical Minimum (0)
		  0x46, 0xFF, 0x7F,  //     Physical Maximum (32767)
		  0x66, 0x03, 0x10,  //     Unit (System: English Linear, Time: Seconds)
		  0x55, 0xFD,        //     Unit Exponent
		  0x75, 0x10,        //     Report Size (16)
		  0x95, 0x03,        //     Report Count (3)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x55, 0x00,        //     Unit Exponent (0)
		  0x66, 0x00, 0x00,  //     Unit (None)
		  0x09, 0x52,        //     Usage (Gain)
		  0x16, 0x00, 0x00,  //     Logical Minimum (0)
		  0x26, 0x10, 0x27,  //     Logical Maximum (10000)
		  0x36, 0x00, 0x00,  //     Physical Minimum (0)
		  0x46, 0x10, 0x27,  //     Physical Maximum (10000)
		  0x75, 0x10,        //     Report Size (16)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x53,        //     Usage (Trigger Button)
		  0x15, 0x01,        //     Logical Minimum (1)
		  0x25, 0x08,        //     Logical Maximum (8)
		  0x35, 0x01,        //     Physical Minimum (1)
		  0x45, 0x08,        //     Physical Maximum (8)
		  0x75, 0x08,        //     Report Size (8)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x55,        //     Usage (Axes Enable)
		  0xA1, 0x02,        //     Collection (Logical)
		  0x05, 0x01,        //       Usage Page (Generic Desktop Ctrls)
		  0x09, 0x30,        //       Usage (X)
		  0x09, 0x31,        //       Usage (Y)
		  0x15, 0x00,        //       Logical Minimum (0)
		  0x25, 0x01,        //       Logical Maximum (1)
		  0x75, 0x01,        //       Report Size (1)
		  0x95, 0x02,        //       Report Count (2)
		  0x91, 0x02,        //       Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0xC0,              //     End Collection
		  0x05, 0x0F,        //     Usage Page (PID Page)
		  0x09, 0x56,        //     Usage (Direction Enable)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x95, 0x05,        //     Report Count (5)
		  0x91, 0x03,        //     Output (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x57,        //     Usage (Direction)
		  0xA1, 0x02,        //     Collection (Logical)
		  0x0B, 0x01, 0x00, 0x0A, 0x00,  //       Usage (Ordinal:Ordinal 1)
		  0x0B, 0x02, 0x00, 0x0A, 0x00,  //       Usage (Ordinal:Ordinal 2)
		  0x66, 0x14, 0x00,  //       Unit (System: English Rotation, Length: Centimeter)
		  0x55, 0xFE,        //       Unit Exponent
		  0x17, 0x00, 0x00, 0x00, 0x00,  //       Logical Minimum (0)
		  0x27, 0x9F, 0x8C, 0x00, 0x00,  //       Logical Maximum (35999)
		  0x37, 0x00, 0x00, 0x00, 0x00,  //       Physical Minimum (0)
		  0x47, 0x9F, 0x8C, 0x00, 0x00,  //       Physical Maximum (35999)
		  0x66, 0x00, 0x00,  //       Unit (None)
		  0x75, 0x10,        //       Report Size (16)
		  0x95, 0x02,        //       Report Count (2)
		  0x91, 0x02,        //       Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x55, 0x00,        //       Unit Exponent (0)
		  0x66, 0x00, 0x00,  //       Unit (None)
		  0xC0,              //     End Collection
		  0x05, 0x0F,        //     Usage Page (PID Page)
		  0x66, 0x03, 0x10,  //     Unit (System: English Linear, Time: Seconds)
		  0x55, 0xFD,        //     Unit Exponent
		  0x16, 0x00, 0x00,  //     Logical Minimum (0)
		  0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
		  0x36, 0x00, 0x00,  //     Physical Minimum (0)
		  0x46, 0xFF, 0x7F,  //     Physical Maximum (32767)
		  0x75, 0x10,        //     Report Size (16)
		  0x95, 0x01,        //     Report Count (1)
		  0x66, 0x00, 0x00,  //     Unit (None)
		  0x55, 0x00,        //     Unit Exponent (0)
		  0xC0,              //   End Collection
		  0x05, 0x0F,        //   Usage Page (PID Page)
		  0x09, 0x5A,        //    Usage (Set Envelope Report)
		  0xA1, 0x02,        //   Collection (Logical)
		  0x85, 0x02,        //     Report ID (2)
		  0x09, 0x22,        //     Usage (Effect Block Index)
		  0x15, 0x01,        //     Logical Minimum (1)
		  0x25, 0x28,        //     Logical Maximum (40)
		  0x35, 0x01,        //     Physical Minimum (1)
		  0x45, 0x28,        //     Physical Maximum (40)
		  0x75, 0x08,        //     Report Size (8)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x5B,        //     Usage (Attack Level)
		  0x09, 0x5D,        //     Usage (Fade Level)
		  0x16, 0x00, 0x00,  //     Logical Minimum (0)
		  0x26, 0x10, 0x27,  //     Logical Maximum (10000)
		  0x36, 0x00, 0x00,  //     Physical Minimum (0)
		  0x46, 0x10, 0x27,  //     Physical Maximum (10000)
		  0x75, 0x10,        //     Report Size (16)
		  0x95, 0x02,        //     Report Count (2)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x5C,        //     Usage (Attack Time)
		  0x09, 0x5E,        //      Usage (Fade Time)
		  0x66, 0x03, 0x10,  //     Unit (System: English Linear, Time: Seconds)
		  0x55, 0xFD,        //     Unit Exponent
		  0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
		  0x46, 0xFF, 0x7F,  //     Physical Maximum (32767)
		  0x75, 0x10,        //     Report Size (16)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x45, 0x00,        //     Physical Maximum (0)
		  0x66, 0x00, 0x00,  //     Unit (None)
		  0x55, 0x00,        //     Unit Exponent (0)
		  0xC0,              //   End Collection
		  0x09, 0x5F,        //   Usage (0x5F)
		  0xA1, 0x02,        //   Collection (Logical)
		  0x85, 0x03,        //     Report ID (3)
		  0x09, 0x22,        //     Usage (Effect Block Index)
		  0x15, 0x01,        //     Logical Minimum (1)
		  0x25, 0x28,        //     Logical Maximum (40)
		  0x35, 0x01,        //     Physical Minimum (1)
		  0x45, 0x28,        //     Physical Maximum (40)
		  0x75, 0x08,        //     Report Size (8)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x23,        //     Usage (Parameter Block Offset)
		  0x15, 0x00,        //     Logical Minimum (0)
		  0x25, 0x03,        //     Logical Maximum (3)
		  0x35, 0x00,        //     Physical Minimum (0)
		  0x45, 0x03,        //     Physical Maximum (3)
		  0x75, 0x04,        //     Report Size (4)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x58,        //     Usage (Type Specific Block Offset)
		  0xA1, 0x02,        //     Collection (Logical)
		  0x0B, 0x01, 0x00, 0x0A, 0x00,  //       Usage (Ordinal:Ordinal 1)
		  0x0B, 0x02, 0x00, 0x0A, 0x00,  //       Usage (Ordinal:Ordinal 2)
		  0x75, 0x02,        //       Report Size (2)
		  0x95, 0x02,        //       Report Count (2)
		  0x91, 0x02,        //       Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0xC0,              //     End Collection
		  0x16, 0xF0, 0xD8,  //     Logical Minimum (-10000)
		  0x26, 0x10, 0x27,  //     Logical Maximum (10000)
		  0x36, 0xF0, 0xD8,  //     Physical Minimum (-10000)
		  0x46, 0x10, 0x27,  //     Physical Maximum (10000)
		  0x09, 0x60,        //     Usage (CP Offset)
		  0x75, 0x10,        //     Report Size (16)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x16, 0xF0, 0xD8,  //     Logical Minimum (-10000)
		  0x26, 0x10, 0x27,  //     Logical Maximum (10000)
		  0x36, 0xF0, 0xD8,  //     Physical Minimum (-10000)
		  0x46, 0x10, 0x27,  //     Physical Maximum (10000)
		  0x09, 0x61,        //     Usage (Positive Coefficient)
		  0x09, 0x62,        //     Usage (Negative Coefficient)
		  0x95, 0x02,        //     Report Count (2)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x16, 0x00, 0x00,  //     Logical Minimum (0)
		  0x26, 0x10, 0x27,  //     Logical Maximum (10000)
		  0x36, 0x00, 0x00,  //     Physical Minimum (0)
		  0x46, 0x10, 0x27,  //     Physical Maximum (10000)
		  0x09, 0x63,        //     Usage (Positive Saturation)
		  0x09, 0x64,        //     Usage (Negative Saturation)
		  0x75, 0x10,        //     Report Size (16)
		  0x95, 0x02,        //     Report Count (2)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x65,        //     Usage (Dead Band)
		  0x46, 0x10, 0x27,  //     Physical Maximum (10000)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0xC0,              //   End Collection
		  0x09, 0x6E,        //   Usage (Set Periodic Report)
		  0xA1, 0x02,        //   Collection (Logical)
		  0x85, 0x04,        //     Report ID (4)
		  0x09, 0x22,        //     Usage (Effect Block Index)
		  0x15, 0x01,        //     Logical Minimum (1)
		  0x25, 0x28,        //     Logical Maximum (40)
		  0x35, 0x01,        //     Physical Minimum (1)
		  0x45, 0x28,        //     Physical Maximum (40)
		  0x75, 0x08,        //     Report Size (8)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x70,        //     Usage (Magnitude)
		  0x16, 0x00, 0x00,  //     Logical Minimum (0)
		  0x26, 0x10, 0x27,  //     Logical Maximum (10000)
		  0x36, 0x00, 0x00,  //     Physical Minimum (0)
		  0x46, 0x10, 0x27,  //     Physical Maximum (10000)
		  0x75, 0x10,        //     Report Size (16)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x6F,        //     Usage (Offset)
		  0x16, 0xF0, 0xD8,  //     Logical Minimum (-10000)
		  0x26, 0x10, 0x27,  //     Logical Maximum (10000)
		  0x36, 0xF0, 0xD8,  //     Physical Minimum (-10000)
		  0x46, 0x10, 0x27,  //     Physical Maximum (10000)
		  0x75, 0x10,        //     Report Size (16)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x71,        //     Usage (Phase)
		  0x66, 0x14, 0x00,  //     Unit (System: English Rotation, Length: Centimeter)
		  0x55, 0xFE,        //     Unit Exponent
		  0x17, 0x00, 0x00, 0x00, 0x00,   //     Logical Minimum (0)
		  0x27, 0x9F, 0x8C, 0x00, 0x00,  //     Logical Maximum (35999)
		  0x37, 0x00, 0x00, 0x00, 0x00,  //     Physical Minimum (0)
		  0x47, 0x9F, 0x8C, 0x00, 0x00,  //     Physical Maximum (35999)
		  0x75, 0x10,        //     Report Size (16)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x72,        //     Usage (Period)
		  0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
		  0x46, 0xFF, 0x7F,  //     Physical Maximum (32767)
		  0x66, 0x03, 0x10,  //     Unit (System: English Linear, Time: Seconds)
		  0x55, 0xFD,        //     Unit Exponent
		  0x75, 0x20,        //     Report Size (32)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x66, 0x00, 0x00,  //     Unit (None)
		  0x55, 0x00,        //     Unit Exponent (0)
		  0xC0,              //   End Collection
		  0x09, 0x73,        //   Usage (Set Constant Force Report)
		  0xA1, 0x02,        //   Collection (Logical)
		  0x85, 0x05,        //     Report ID (5)
		  0x09, 0x22,        //     Usage (Effect Block Index)
		  0x15, 0x01,        //     Logical Minimum (1)
		  0x25, 0x28,        //     Logical Maximum (40)
		  0x35, 0x01,        //     Physical Minimum (1)
		  0x45, 0x28,        //     Physical Maximum (40)
		  0x75, 0x08,        //     Report Size (8)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x70,        //     Usage (Magnitude)
		  0x16, 0xF0, 0xD8,  //     Logical Minimum (-10000)
		  0x26, 0x10, 0x27,  //     Logical Maximum (10000)
		  0x36, 0xF0, 0xD8,  //     Physical Minimum (-10000)
		  0x46, 0x10, 0x27,  //     Physical Maximum (10000)
		  0x75, 0x10,        //     Report Size (16)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0xC0,              //   End Collection
		  0x09, 0x74,        //   Usage (Set Ramp Force Report)
		  0xA1, 0x02,        //   Collection (Logical)
		  0x85, 0x06,        //     Report ID (6)
		  0x09, 0x22,        //     Usage (Effect Block Index)
		  0x15, 0x01,        //     Logical Minimum (1)
		  0x25, 0x28,        //     Logical Maximum (40)
		  0x35, 0x01,        //     Physical Minimum (1)
		  0x45, 0x28,        //     Physical Maximum (40)
		  0x75, 0x08,        //     Report Size (8)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x75,        //     Usage (Ramp Start)
		  0x09, 0x76,        //     Usage (Ramp End)
		  0x16, 0xF0, 0xD8,  //     Logical Minimum (-10000)
		  0x26, 0x10, 0x27,  //     Logical Maximum (10000)
		  0x36, 0xF0, 0xD8,  //     Physical Minimum (-10000)
		  0x46, 0x10, 0x27,  //     Physical Maximum (10000)
		  0x75, 0x10,        //     Report Size (16)
		  0x95, 0x02,        //     Report Count (2)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0xC0,              //   End Collection
		  0x09, 0x68,        //   Usage (Custom Force Data Report)
		  0xA1, 0x02,        //   Collection (Logical)
		  0x85, 0x07,        //     Report ID (7)
		  0x09, 0x22,        //     Usage (Effect Block Index)
		  0x15, 0x01,        //     Logical Minimum (1)
		  0x25, 0x28,        //     Logical Maximum (40)
		  0x35, 0x01,        //     Physical Minimum (1)
		  0x45, 0x28,        //     Physical Maximum (40)
		  0x75, 0x08,        //     Report Size (8)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x6C,        //     Usage (Custom Force Data Offset)
		  0x16, 0x00, 0x00,  //     Logical Minimum (0)
		  0x26, 0x10, 0x27,  //     Logical Maximum (10000)
		  0x36, 0x00, 0x00,  //     Physical Minimum (0)
		  0x46, 0x10, 0x27,  //     Physical Maximum (10000)
		  0x75, 0x10,        //     Report Size (16)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x69,        //     Usage (Custom Force Data)
		  0x15, 0x81,        //     Logical Minimum (-127)
		  0x25, 0x7F,        //     Logical Maximum (127)
		  0x36, 0x00, 0x00,  //     Physical Minimum (0)
		  0x46, 0xFF, 0x00,  //     Physical Maximum (255)
		  0x75, 0x08,        //     Report Size (8)
		  0x95, 0x0C,        //     Report Count (12)
		  0x92, 0x02, 0x01,  //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile,Buffered Bytes)
		  0xC0,              //   End Collection
		  0x09, 0x66,        //   Usage (Download Force Sample)
		  0xA1, 0x02,        //   Collection (Logical)
		  0x85, 0x08,        //     Report ID (8)
		  0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
		  0x09, 0x30,        //     Usage (X)
		  0x09, 0x31,        //     Usage (Y)
		  0x15, 0x81,        //     Logical Minimum (-127)
		  0x25, 0x7F,        //     Logical Maximum (127)
		  0x36, 0x00, 0x00,  //     Physical Minimum (0)
		  0x46, 0xFF, 0x00,  //     Physical Maximum (255)
		  0x75, 0x08,        //     Report Size (8)
		  0x95, 0x02,        //     Report Count (2)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0xC0,              //   End Collection
		  0x05, 0x0F,        //   Usage Page (PID Page)
		  0x09, 0x77,        //   Usage (Effect Operation Report)
		  0xA1, 0x02,        //   Collection (Logical)
		  0x85, 0x0A,        //     Report ID (10)
		  0x09, 0x22,        //     Usage (Effect Block Index)
		  0x15, 0x01,        //     Logical Minimum (1)
		  0x25, 0x28,        //     Logical Maximum (40)
		  0x35, 0x01,        //     Physical Minimum (1)
		  0x45, 0x28,        //     Physical Maximum (40)
		  0x75, 0x08,        //     Report Size (8)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x78,        //     Usage (Effect Operation)
		  0xA1, 0x02,        //     Collection (Logical)
		  0x09, 0x79,        //       Usage (Op Effect Start)
		  0x09, 0x7A,        //       Usage (Op Effect Start Solo)
		  0x09, 0x7B,        //       Usage (Op Effect Stop)
		  0x15, 0x01,        //       Logical Minimum (1)
		  0x25, 0x03,        //       Logical Maximum (3)
		  0x75, 0x08,        //       Report Size (8)
		  0x95, 0x01,        //       Report Count (1)
		  0x91, 0x00,        //       Output (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0xC0,              //     End Collection
		  0x09, 0x7C,        //     Usage (Loop Count)
		  0x16, 0x00, 0x00,  //     Logical Minimum (0)
		  0x26, 0xFF, 0x00,  //     Logical Maximum (255)
		  0x36, 0x00, 0x00,  //     Physical Minimum (0)
		  0x46, 0xFF, 0x00,  //     Physical Maximum (255)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0xC0,              //   End Collection
		  0x09, 0x90,        //   Usage (PID Block Free Report)
		  0xA1, 0x02,        //   Collection (Logical)
		  0x85, 0x0B,        //     Report ID (11)
		  0x09, 0x22,        //     Usage (Effect Block Index)
		  0x25, 0x28,        //     Logical Maximum (40)
		  0x15, 0x01,        //     Logical Minimum (1)
		  0x35, 0x01,        //     Physical Minimum (1)
		  0x45, 0x28,        //     Physical Maximum (40)
		  0x75, 0x08,        //     Report Size (8)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0xC0,              //   End Collection
		  0x09, 0x96,        //   Usage (PID Device Control)
		  0xA1, 0x02,        //   Collection (Logical)
		  0x85, 0x0C,        //     Report ID (12)
		  0x09, 0x97,        //     Usage (DC Enable Actuators)
		  0x09, 0x98,        //     Usage (DC Disable Actuators)
		  0x09, 0x99,        //     Usage (DC Stop All Effects)
		  0x09, 0x9A,        //     Usage (DC Device Reset)
		  0x09, 0x9B,        //     Usage (DC Device Pause)
		  0x09, 0x9C,        //     Usage (DC Device Continue)
		  0x15, 0x01,        //     Logical Minimum (1)
		  0x25, 0x06,        //     Logical Maximum (6)
		  0x75, 0x08,        //     Report Size (8)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x00,        //     Output (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0xC0,              //   End Collection
		  0x09, 0x7D,        //   Usage (Device Gain Report)
		  0xA1, 0x02,        //   Collection (Logical)
		  0x85, 0x0D,        //     Report ID (13)
		  0x09, 0x7E,        //      Usage (Device Gain)
		  0x16, 0x00, 0x00,  //     Logical Minimum (0)
		  0x26, 0x10, 0x27,  //     Logical Maximum (10000)
		  0x36, 0x00, 0x00,  //     Physical Minimum (0)
		  0x46, 0x10, 0x27,  //     Physical Maximum (10000)
		  0x75, 0x10,        //     Report Size (16)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0xC0,              //   End Collection
		  0x09, 0x6B,        //   Usage (Set Custom Force Report)
		  0xA1, 0x02,        //   Collection (Logical)
		  0x85, 0x0E,        //     Report ID (14)
		  0x09, 0x22,        //     Usage (Effect Block Index)
		  0x15, 0x01,        //     Logical Minimum (1)
		  0x25, 0x28,        //     Logical Maximum (40)
		  0x35, 0x01,        //     Physical Minimum (1)
		  0x45, 0x28,        //     Physical Maximum (40)
		  0x75, 0x08,        //     Report Size (8)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x6D,        //     Usage (0x6D)
		  0x16, 0x00, 0x00,  //     Logical Minimum (0)
		  0x26, 0xFF, 0x00,  //     Logical Maximum (255)
		  0x36, 0x00, 0x00,  //     Physical Minimum (0)
		  0x46, 0xFF, 0x00,  //     Physical Maximum (255)
		  0x75, 0x08,        //     Report Size (8)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x51,        //     Usage (Sample Period)
		  0x66, 0x03, 0x10,  //     Unit (System: English Linear, Time: Seconds)
		  0x55, 0xFD,        //     Unit Exponent
		  0x16, 0x00, 0x00,  //     Logical Minimum (0)
		  0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
		  0x36, 0x00, 0x00,  //     Physical Minimum (0)
		  0x46, 0xFF, 0x7F,  //     Physical Maximum (32767)
		  0x75, 0x10,        //     Report Size (16)
		  0x95, 0x01,        //     Report Count (1)
		  0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x55, 0x00,        //     Unit Exponent (0)
		  0x66, 0x00, 0x00,  //     Unit (None)
		  0xC0,              //   End Collection
		  0x09, 0xAB,        //   Usage (Create New Effect Report)
		  0xA1, 0x02,        //   Collection (Logical)
		  0x85, 0x05,        //     Report ID (5)
		  0x09, 0x25,        //     Usage (Effect Type)
		  0xA1, 0x02,        //     Collection (Logical)
		  0x09, 0x26,        //       Usage (ET Constant Force)
		  0x09, 0x27,        //       Usage (ET Ramp)
		  0x09, 0x30,        //       Usage (ET Square)
		  0x09, 0x31,        //       Usage (ET Sine)
		  0x09, 0x32,        //       Usage (ET Triangle)
		  0x09, 0x33,        //       Usage (ET Sawtooth Up)
		  0x09, 0x34,        //       Usage (ET Sawtooth Down)
		  0x09, 0x40,        //       Usage (ET Spring)
		  0x09, 0x41,        //       Usage (ET Damper)
		  0x09, 0x42,        //       Usage (ET Inertia)
		  0x09, 0x43,        //       Usage (ET Friction)
		  0x09, 0x28,        //       Usage (ET Custom Force Data)
		  0x25, 0x0C,        //       Logical Maximum (12)
		  0x15, 0x01,        //       Logical Minimum (1)
		  0x35, 0x01,        //       Physical Minimum (1)
		  0x45, 0x0C,        //       Physical Maximum (12)
		  0x75, 0x08,        //       Report Size (8)
		  0x95, 0x01,        //       Report Count (1)
		  0xB1, 0x00,        //       Feature (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0xC0,              //     End Collection
		  0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
		  0x09, 0x3B,        //     Usage (Byte Count)
		  0x16, 0x00, 0x00,  //     Logical Minimum (0)
		  0x26, 0xFF, 0x01,  //     Logical Maximum (511)
		  0x36, 0x00, 0x00,  //     Physical Minimum (0)
		  0x46, 0xFF, 0x01,  //     Physical Maximum (511)
		  0x75, 0x0A,        //     Report Size (10)
		  0x95, 0x01,        //     Report Count (1)
		  0xB1, 0x02,        //     Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x75, 0x06,        //     Report Size (6)
		  0xB1, 0x01,        //     Feature (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0xC0,              //   End Collection
		  0x05, 0x0F,        //   Usage Page (PID Page)
		  0x09, 0x89,        //   Usage (PID Block Load Report)
		  0xA1, 0x02,        //   Collection (Logical)
		  0x85, 0x06,        //     Report ID (6)
		  0x09, 0x22,        //     Usage (Effect Block Index)
		  0x25, 0x28,        //     Logical Maximum (40)
		  0x15, 0x01,        //     Logical Minimum (1)
		  0x35, 0x01,        //     Physical Minimum (1)
		  0x45, 0x28,        //     Physical Maximum (40)
		  0x75, 0x08,        //     Report Size (8)
		  0x95, 0x01,        //     Report Count (1)
		  0xB1, 0x02,        //     Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x8B,        //     Usage (0x8B)
		  0xA1, 0x02,        //     Collection (Logical)
		  0x09, 0x8C,        //       Usage (Block Load Success)
		  0x09, 0x8D,        //       Usage (Block Load Full)
		  0x09, 0x8E,        //       Usage (Block Load Error)
		  0x25, 0x03,        //       Logical Maximum (3)
		  0x15, 0x01,        //       Logical Minimum (1)
		  0x35, 0x01,        //       Physical Minimum (1)
		  0x45, 0x03,        //       Physical Maximum (3)
		  0x75, 0x08,        //       Report Size (8)
		  0x95, 0x01,        //       Report Count (1)
		  0xB1, 0x00,        //       Feature (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0xC0,              //     End Collection
		  0x09, 0xAC,        //     Usage (RAM Pool Available)
		  0x17, 0x00, 0x00, 0x00, 0x00,  //     Logical Minimum (0)
		  0x27, 0xFF, 0xFF, 0x00, 0x00,  //     Logical Maximum (65535)
		  0x37, 0x00, 0x00, 0x00, 0x00,  //     Physical Minimum (0)
		  0x47, 0xFF, 0xFF, 0x00, 0x00,  //     Physical Maximum (65535)
		  0x75, 0x10,        //     Report Size (16)
		  0x95, 0x01,        //     Report Count (1)
		  0xB1, 0x00,        //     Feature (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0xC0,              //   End Collection
		  0x09, 0x7F,        //   Usage (PID Pool Report)
		  0xA1, 0x02,        //   Collection (Logical)
		  0x85, 0x07,        //     Report ID (7)
		  0x09, 0x80,        //     Usage (RAM Pool Size)
		  0x75, 0x10,        //     Report Size (16)
		  0x95, 0x01,        //     Report Count (1)
		  0x17, 0x00, 0x00, 0x00, 0x00, //     Logical Minimum (0)
		  0x37, 0x00, 0x00, 0x00, 0x00, //     Physical Minimum (0)
		  0x27, 0xFF, 0xFF, 0x00, 0x00,  //     Logical Maximum (65535)
		  0x47, 0xFF, 0xFF, 0x00, 0x00,  //     Physical Maximum (65535)
		  0xB1, 0x02,        //     Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0x83,        //     Usage (Simultaneous Effects Max)
		  0x26, 0xFF, 0x00,  //     Logical Maximum (255)
		  0x46, 0xFF, 0x00,  //     Physical Maximum (255)
		  0x75, 0x08,        //     Report Size (8)
		  0x95, 0x01,        //     Report Count (1)
		  0xB1, 0x02,        //     Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x09, 0xA9,        //      Usage (Device Managed Pool)
		  0x09, 0xAA,        //     Usage (Shared Parameter Blocks)
		  0x75, 0x01,        //     Report Size (1)
		  0x95, 0x02,        //     Report Count (2)
		  0x15, 0x00,        //     Logical Minimum (0)
		  0x25, 0x01,        //     Logical Maximum (1)
		  0x35, 0x00,        //     Physical Minimum (0)
		  0x45, 0x01,        //     Physical Maximum (1)
		  0xB1, 0x02,        //     Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0x75, 0x06,        //     Report Size (6)
		  0x95, 0x01,        //     Report Count (1)
		  0xB1, 0x03,        //     Feature (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		  0xC0,              //   End Collection
      0xC0 // END COLLECTION ()
  };

int pidReportDescriptorSize = sizeof(pidReportDescriptor) / sizeof(pidReportDescriptor[0]);

#endif // !

