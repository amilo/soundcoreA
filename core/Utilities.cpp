/*
 * Utilities.cpp
 *
 *  Created on: Oct 27, 2014
 *      Author: parallels
 */

#include "../include/Utilities.h"

// map()
//
// Scale an input value from one range to another. Works like its Wiring language equivalent.
// x is the value to scale; in_min and in_max are the input range; out_min and out_max
// are the output range.

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// constrain()
//
// Clips an input value to be between two end points
// x is the value to constrain; min_val and max_val are the range

float constrain(float x, float min_val, float max_val)
{
	if(x < min_val) return min_val;
	if(x > max_val) return max_val;
	return x;
}
