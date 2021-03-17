/*
 * ACServo.h
 *
 *  Created on: Mar 9, 2021
 *      Author: billy
 */

#ifndef SRC_ACSERVO_H_
#define SRC_ACSERVO_H_
#include "cppmain.h"



class ACServo
	{
	public:

		void Init_AcServo();
		void stop();
		void start();
		void set_motor_dac(int32_t * _xy_forces);
		bool GetState();
		ACServo();
		virtual ~ACServo();
	private:
		bool active = false;

	};

#endif /* SRC_ACSERVO_H_ */
