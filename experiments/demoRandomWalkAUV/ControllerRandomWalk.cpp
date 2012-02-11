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

#include "ControllerRandomWalk.h"


#include <cmath>
#include <iostream>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
extern gsl_rng* rng;
extern long int rngSeed;


ControllerRandomWalk::ControllerRandomWalk (RobotLily* l)
    : Controller (l)
{
    this->lily = l;

    Reset();
}


ControllerRandomWalk::~ControllerRandomWalk ()
{

}

void ControllerRandomWalk::SetParameters(ControllerRandomWalkParameters* p)
{
    params = *p;
}


void ControllerRandomWalk::Step (float time, float timestep)
{
    this->time = time;
    this->timestep = timestep;

    switch (state)
    {
    case EXPLORE : StateExplore(); break;
    case TURN : StateTurn(); break;
    }
}


void ControllerRandomWalk::StateExploreInit ()
{
    float rnd = 1.0 - gsl_ran_flat(rng, 0.0, 1.0);
    exploreDuration = - log (rnd) * params.exploreMeanDuration;

    exploreStartTime = time;
    state = EXPLORE;
}


void ControllerRandomWalk::StateExplore ()
{
    // if time to change direction -> turn
    if (time - exploreStartTime > exploreDuration)
    {
	// jump to turn state
	float angle = gsl_rng_uniform(rng) * 2.0 * M_PI - M_PI;    
	StateTurnInit(EXPLORE, angle);

	return;
    }

    if (ObstacleAvoidance ())
    {
	lily->SetColor(1.0,0.0,0.0); // red
	return;
    }
    else
    {
	lily->SetColor(0.5,0.5,1.0); // blue
	lily->propellers->SetSpeed(params.speed, params.speed);
    }
}


void ControllerRandomWalk::StateTurnInit(int previousState, float angle)
{
    turnPreviousState = previousState;

    if (angle < 0.0)
	turnSign = 1.0;
    else
	turnSign = -1.0;

    turnDuration = (fabs(angle) / M_PI) / 3.0 / params.speed;// /5.0
    turnStartTime = time;
    state = TURN;
}

void ControllerRandomWalk::StateTurn()
{
    if (time - turnStartTime > turnDuration)
    {
	switch (turnPreviousState)
	{
	case EXPLORE : StateExploreInit(); return;
	}
    }

    lily->propellers->SetSpeed(params.speed * turnSign, -params.speed * turnSign); // /5.0
}


bool ControllerRandomWalk::ObstacleAvoidance()
{    
    // check if an obstacle is perceived 
    int obstaclePerceived = 0;

    float pl = lily->rayFrontLU->GetValue() + lily->rayFrontLD->GetValue() + lily->rayLeft->GetValue();
    float pr = lily->rayFrontRU->GetValue() + lily->rayFrontRD->GetValue() + lily->rayRight->GetValue();
    pl /= 3.0;
    pr /= 3.0;   

    if (lily->rayFrontLU->hasHit()
	|| lily->rayFrontLD->hasHit()
	|| lily->rayFrontRU->hasHit()
	|| lily->rayFrontRD->hasHit()
	|| lily->rayLeft->hasHit()
	|| lily->rayRight->hasHit()
	)
	obstaclePerceived = 1;	    
    
    // no obstacles to avoid, return immediately
    if (obstaclePerceived == 0)
	return false;

    float leftSpeed = 0.0;
    float rightSpeed = 0.0;

    // turn left
    if (pr > 0.15 && pl > 0.15)
    {
	leftSpeed = -1.0 * pr / (pr + pl);
	rightSpeed = -1.0 * pl / (pr + pl);
    }
    else if (pr >= pl)
    {
	leftSpeed = -1.0 * pr;
	rightSpeed = 1.0 * pr;
    }
    // turn right
    else
    {
	leftSpeed = 1.0 * pl;
	rightSpeed = -1.0 * pl;
    }
    
    // rescale values
    leftSpeed *= params.speed; // /5 
    rightSpeed *= params.speed; // /5

    // change movement direction
    lily->propellers->SetSpeed(leftSpeed, rightSpeed);

    // advertise obstacle avoidance in progress
    return true;
}


/******************************************************************************/
/******************************************************************************/

void ControllerRandomWalk::Reset ()
{
    // reset time
    time = 0.0;
    timestep = 0.0;

    // state working variables
    exploreDuration = 0.0;
    exploreStartTime = 0.0;

    turnPreviousState = 0;
    turnDuration = 0.0;
    turnStartTime = 0.0;
    turnSign = 1.0;
    
    // start in explore state
    state = EXPLORE;
    StateExploreInit();
}
