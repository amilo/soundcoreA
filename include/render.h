/*
 * render.h
 *
 *  Created on: May 28, 2014
 *      Author: Victor Zappi
 */

#ifndef RENDER_H_
#define RENDER_H_

// uint types
#include <stdint.h>

// Mappings from pin numbers on PCB to actual DAC channels
// This gives the DAC and ADC connectors the same effective pinout
#define DAC_PIN0	6
#define DAC_PIN1	4
#define DAC_PIN2	2
#define DAC_PIN3	0
#define DAC_PIN4	1
#define DAC_PIN5	3
#define DAC_PIN6	5
#define DAC_PIN7	7

#define ADC_PIN0	0
#define ADC_PIN1	1
#define ADC_PIN2	2
#define ADC_PIN3	3
#define ADC_PIN4	4
#define ADC_PIN5	5
#define ADC_PIN6	6
#define ADC_PIN7	7

#define MATRIX_MAX  65535.0

extern int gNumAudioChannels;	// How many audio channels are present
extern int gNumMatrixChannels;	// How many matrix channels are present

bool initialise_render(int numMatrixChannels, int numAudioChannels,
		  	   	   	   int numMatrixFramesPerPeriod,
					   int numAudioFramesPerPeriod,
					   float matrixSampleRate, float audioSampleRate,
					   void *userData);

void render(int numMatrixFrames, int numAudioFrames, float *audioIn, float *audioOut,
			float *matrixIn, float *matrixOut);

void render_medium_prio();
void render_low_prio();

void schedule_render_medium_prio();
void schedule_render_low_prio();


void cleanup_render();

#endif /* RENDER_H_ */
