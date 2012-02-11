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

#include "ControllerRandom.h"


#include <cmath>
#include <iostream>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
extern gsl_rng* rng;
extern long int rngSeed;


ControllerRandom::ControllerRandom (BasicRobot* br)
    : Controller (br)
{
    this->robot = br;
}


ControllerRandom::~ControllerRandom ()
{

}

void ControllerRandom::Step (float time, float timestep)
{
    float rate = 2.0;
    if (gsl_ran_flat(rng, 0.0, 1.0) < rate * timestep)
    {
        // now select a random speed    
        float direction = gsl_ran_flat(rng, -M_PI, M_PI);
	float magnitude = 300.0 * gsl_ran_flat(rng, 0.1, 1.0);
        robot->magicForce->ApplyForce(btVector3(cos(direction) * magnitude, sin(direction) * magnitude, 0.0));
    }
}

void ControllerRandom::Reset()
{

}
