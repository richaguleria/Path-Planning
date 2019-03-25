#include "Vehicle.h"

Vehicle::Vehicle(){}
Vehicle::Vehicle(int lane, double s, double d, double V, lanestate state)
{
	this->lane = lane;
	this->s = s;
	this->d = d;
	this->vel = V;
	this->state = state;
}

bool Vehicle::PrepLaneChange(lanestate state, double &s, double &d, double&v, double* cost)
{
	double cost = 0.0;
	bool found = false;

	switch(state)
	{
		case PREP_LEFT_LANE:
			if(0 < d < 4)
			{
				//if in left lane good gap exists
				if((s > this->s) && (s-(this->s) > 40))
				{
					*cost = (this->vel-v)/(this->vel);
					found = true;
					std::cout<<"Left Lane Changed ?";
				}

			}
		case PREP_RIGHT_LANE:
			if(8 < d < 12)
			{
				//if in right lane good gap exists
				if((s > this->s) && (s-(this->s) > 40))
				{
					*cost = (this->vel - v)/(this->vel);
					found = true;
					std::cout<<"Right Lane Changed ?";

				}

			}
		default:
			std::cout<<"Not a valid case";
	}
	return found;
}
