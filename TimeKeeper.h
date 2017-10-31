/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TimeKeeper.h
 * Author: yasufumi
 *
 * Created on 2017/05/31, 18:51
 */

#ifndef TIMEKEEPER_H
#define TIMEKEEPER_H

#include "TimeCount.h"

namespace nsTimeKeeper{
	const double SAMPLING_TIME = 0.015;	//sec (more than 0.008)

	class TimeKeeper : public nsTimeCount::TimeCount {
	public:
		TimeKeeper();
		TimeKeeper(const TimeKeeper& orig);
		virtual ~TimeKeeper();

		void TimeKeep();

	private:

	};

}

#endif /* TIMEKEEPER_H */

