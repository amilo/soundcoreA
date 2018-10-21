/*
 * Utilities.h
 *
 *  Created on: Oct 27, 2014
 *      Author: parallels
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

extern int gNumAudioChannels;	// How many audio channels are present
extern int gNumMatrixChannels;	// How many matrix channels are present

// Macros for accessing the matrix values: usable _only_ within render()

// Read an analog input from input pin p at frame f
#define analogRead(p, f) (matrixIn[(f)*gNumMatrixChannels + (p)])
// Write an analog output frame at output pin p, frame f, to value v
#define analogWrite(p, f, v) (matrixOut[(f)*gNumMatrixChannels + (p)] = (uint16_t)(v))

float map(float x, float in_min, float in_max, float out_min, float out_max);
float constrain(float x, float min_val, float max_val);

#endif /* UTILITIES_H_ */
