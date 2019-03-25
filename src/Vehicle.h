#ifndef VEHICLE_H
#define VEHICLE_H
#endif

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

enum lanestate{
  KEEP_LANE = 0,
  PREP_LEFT_LANE = 1,
  LEFT_LANE = 2,
  PREP_RIGHT_LANE = 3,
  RIGHT_LANE = 4
};

class Vehicle{
public:
	Vehicle();
	virtual ~Vehicle();

	Vehicle(int lane, double s, double d, double V, lanestate state);

	//bool PrepLaneChange(lanestate state, auto& data, int& path_size);
	bool PrepLaneChange(lanestate state, double &s, double &d,double&v, double* cost);

	lanestate state;
	int lane;
	double s;
	double d;
	double vel;
};