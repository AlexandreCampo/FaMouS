/*----------------------------------------------------------------------------*/
/*    Copyright (C) 2011-2012 Alexandre Campo                                 */
/*                                                                            */
/*    This file is part of FaMouS  (a fast, modular and simple simulator).    */
/*                                                                            */
/*    FaMouS is free software: you can redistribute it and/or modify          */
/*    it under the terms of the GNU General Public License as published by    */
/*    the Free Software Foundation, either version 3 of the License, or       */
/*    (at your option) any later version.                                     */
/*                                                                            */
/*    FaMouS is distributed in the hope that it will be useful,               */
/*    but WITHOUT ANY WARRANTY; without even the implied warranty of          */
/*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           */
/*    GNU General Public License for more details.                            */
/*                                                                            */
/*    You should have received a copy of the GNU General Public License       */
/*    along with FaMouS.  If not, see <http://www.gnu.org/licenses/>.         */
/*----------------------------------------------------------------------------*/

#include "ControllerBaseStation.h"


#include <cmath>
#include <iostream>


#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
extern gsl_rng* rng;
extern long int rngSeed;


ControllerBaseStation::ControllerBaseStation (BaseStation* bs, int signal)
    : Controller (bs)
{
    this->bs = bs;
    this->signal = signal;

    Reset ();
}


ControllerBaseStation::~ControllerBaseStation ()
{
}

void ControllerBaseStation::Reset ()
{
    lastSignalingTime = 0.0;
}

void ControllerBaseStation::Step (float time, float timestep)
{
    // every 0.5 secs send an acoustic signal 
    if (time - lastSignalingTime >= 0.5) 
    {
	lastSignalingTime = time;
	
	bs->acoustic->Send (signal);
    }
}
