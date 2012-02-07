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

#include "ControllerDiscrimination.h"


#include <cmath>
#include <iostream>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
extern gsl_rng* rng;
extern long int rngSeed;


ControllerDiscrimination::ControllerDiscrimination (RobotLily* l)
    : Controller (l)
{
    this->lily = l;

    Reset();
}


ControllerDiscrimination::~ControllerDiscrimination ()
{

}

void ControllerDiscrimination::SetParameters(ControllerDiscriminationParameters* p)
{
    params = *p;
}


void ControllerDiscrimination::Step (float time, float timestep)
{
    this->time = time;
    this->timestep = timestep;

    switch (state)
    {
    case EXPLORE : StateExplore(); break;
    case TURN : StateTurn(); break;
    case REST : StateRest(); break;
    }
}

bool ControllerDiscrimination::IsAtBase ()
{
    if (!lily->acoustic->messagesReceived.empty())
    {
	restLastSignalTime = time;
	lily->acoustic->messagesReceived.clear();
    }

    if (time - restLastSignalTime < 1.0) return true;
    else return false;
}


void ControllerDiscrimination::StateExploreInit ()
{
    float rnd = 1.0 - gsl_ran_flat(rng, 0.0, 1.0);
    exploreDuration = - log (rnd) * params.exploreMeanDuration;

    exploreStartTime = time;
    state = EXPLORE;

    // set robot's colour
    lily->SetColor(0.5,0.5,1.0); 
}


void ControllerDiscrimination::StateExplore ()
{
    // ==================
    // transitions to other states

    // if under shelter -> rest
    if (IsAtBase())
    {
	if (params.controlVariant == 0)
	{	    
	    StateRestInit();
	}
	else if (params.controlVariant == 1)
	{	    
	    // evaluate local density
	    float front = lily->rayFrontLU->GetValue();
	    float v = lily->rayFrontLD->GetValue();
	    if (v > front) front = v;
	    v = lily->rayFrontRU->GetValue();
	    if (v > front) front = v;
	    v = lily->rayFrontRD->GetValue();
	    if (v > front) front = v;
	    float density = front + lily->rayLeft->GetValue() + lily->rayRight->GetValue() + lily->rayTop->GetValue() + lily->rayBottom->GetValue() + lily->rayBack->GetValue();
	    density /= 6.0;
	    
	    // go in rest state only if detected density is high enough
	    float proba = params.theta / (1.0 + params.rho * density * density); 
	    float rand = gsl_rng_uniform(rng);
	    if (rand < proba * timestep)
	    {	    
		StateRestInit();
	    }
	}
	return;	   
    }
    else
    {
	lily->SetColor(0.5,0.5,1.0); 
    }

    // if time to change direction -> turn
    if (time - exploreStartTime > exploreDuration)
    {
	// jump to turn state
	float angle = gsl_rng_uniform(rng) * 2.0 * M_PI - M_PI;    
	StateTurnInit(EXPLORE, angle);

	return;
    }

    // ==================
    // inside the state
    
    if (ObstacleAvoidance ())
	return;

    lily->propellers->SetSpeed(params.exploreSpeed, params.exploreSpeed);
}

void ControllerDiscrimination::StateRestInit ()
{
    state = REST;    

    restLastSignalTime = time;
    collisionsDecisionLastTime = time;

    // set robot's colour
    lily->SetColor(1.0,0.0,0.0); 
}

void ControllerDiscrimination::StateRest ()
{
    // ==================
    // transitions to other states

    // if not anymore under shelter -> come back
    if (!IsAtBase())
    {
	restEscapeFromShelter = false;
	StateExploreInit ();
	return;
    }

    // ==================
    // inside the state

    // break ? 
    if (time - restStartTime > 2.5)
    {
	lily->propellers->SetSpeed(0.0, 0.0);
    }
    else if (time - restStartTime > 1.0)
    {
	lily->propellers->SetSpeed(-params.breakSpeed, -params.breakSpeed);
    }

    // check if robot wants to leave shelter
    if (time - collisionsDecisionLastTime> params.collisionsDecisionDelay)
    {       
	// evaluate local density
	float front = lily->rayFrontLU->GetValue();
	float v = lily->rayFrontLD->GetValue();
	if (v > front) front = v;
	v = lily->rayFrontRU->GetValue();
	if (v > front) front = v;
	v = lily->rayFrontRD->GetValue();
	if (v > front) front = v;
	float density = front + lily->rayLeft->GetValue() + lily->rayRight->GetValue() + lily->rayTop->GetValue() + lily->rayBottom->GetValue() + lily->rayBack->GetValue();
	density /= 6.0;
	
	float proba = params.theta / (1.0 + params.rho * density * density); 
	float rand = gsl_rng_uniform(rng);
	if (rand < proba)
	{
	    restEscapeFromShelter = true;
	}	

	collisionsDecisionLastTime = time;
    }

    if (restEscapeFromShelter)
    {
	if (!ObstacleAvoidance ())
	{
	    lily->propellers->SetSpeed(params.restSpeed, params.restSpeed);
	}
    }
}




void ControllerDiscrimination::StateTurnInit(int previousState, float angle)
{
    turnPreviousState = previousState;

    if (angle < 0.0)
	turnSign = 1.0;
    else
	turnSign = -1.0;

    turnDuration = (fabs(angle) / M_PI) / 3.0 / params.turnSpeed;
    turnStartTime = time;
    state = TURN;
}

void ControllerDiscrimination::StateTurn()
{
    // ==================
    // transitions to other states
    if (time - turnStartTime > turnDuration)
    {
	switch (turnPreviousState)
	{
	case EXPLORE : StateExploreInit(); return;
	case REST : StateRestInit(); return;
	}
    }

    // ==================
    // inside the state
    lily->propellers->SetSpeed(params.turnSpeed * turnSign, -params.turnSpeed * turnSign);
}


bool ControllerDiscrimination::ObstacleAvoidance()
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
    leftSpeed *= params.obstacleAvoidanceSpeed;
    rightSpeed *= params.obstacleAvoidanceSpeed;

    // change movement direction
    lily->propellers->SetSpeed(leftSpeed, rightSpeed);

    // advertise obstacle avoidance in progress
    return true;
}


/******************************************************************************/
/******************************************************************************/

void ControllerDiscrimination::Reset ()
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
    
    restDuration = 0.0;
    restStartTime = 0.0;
    restEscapeFromShelter = false;
    restLastSignalTime = -100.0;

    // start in explore state
    state = EXPLORE;
    StateExploreInit();
}


