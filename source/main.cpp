/*
 * main.cpp
 *
 *  Created on: Oct 24, 2014
 *      Author: parallels
 * -------------------------------------------
 *      Title: SoundCoreA
 *      Author: Alessia Milo MAT PhD Student
 *      Real-Time DSP - ECS732 - Final Project - Lecturer:  Andrew McPherson - TA - Giulio Moro
 *      April 2015
 *
 *
 *
 *
 */


#include	<cstring>
#include	<sndfile.hh> //for sound writing

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <libgen.h>
#include <signal.h>
#include <getopt.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <sndfile.h>				// to load audio files
#include "SimpleGPIO.h"
#include "SampleData.h"
#include "../include/RTAudio.h"

using namespace std;

// Global variables used by getCurrentTime()
unsigned long long gFirstSeconds, gFirstMicroseconds;

int gOutputPinLo = 66, gOutputPinHi = 67;	 // LED on P8, pin 7, 8
int gLEDLoOutput = LOW, gLEDHiOutput = LOW;

int gInputPin1 = 69;	 // Button on P8, pin 9
int gInputPin2 = 68;	 // Button on P8, pin 10

int gTriggerButton1 = 0;	/* Triggers that can be sent to audio code from buttons */
int gTriggerButton2 = 0;

int gButtonSwitch = 0; //value for the switch associated with the button 2

unsigned int latestValue1=0;			//Step 2
unsigned int latestValue2=1;			//Step 3 - depends on button

int gLastTriggerValue1 = 0;				//Step 2
int gLastTriggerValue2 = 1;				//Step 3


int gIsPlaying = 0;
//int gIsPlaying2 = 0;



// Load samples from file
int initFile(string file, SampleData *smp)//float *& smp)
{
	SNDFILE *sndfile ;
	SF_INFO sfinfo ;

	if (!(sndfile = sf_open (file.c_str(), SFM_READ, &sfinfo))) {
		cout << "Couldn't open file " << file << endl;
		return 1;
	}

	int numChan = sfinfo.channels;
	if(numChan != 1)
	{
		cout << "Error: " << file << " is not a mono file" << endl;
		return 1;
	}

	smp->sampleLen = sfinfo.frames * numChan;
	smp->samples = new float[smp->sampleLen];
	if(smp == NULL){
		cout << "Could not allocate buffer" << endl;
		return 1;
	}

	int subformat = sfinfo.format & SF_FORMAT_SUBMASK;
	int readcount = sf_read_float(sndfile, smp->samples, smp->sampleLen);

	// Pad with zeros in case we couldn't read whole file
	for(int k = readcount; k <smp->sampleLen; k++)
		smp->samples[k] = 0;

	if (subformat == SF_FORMAT_FLOAT || subformat == SF_FORMAT_DOUBLE) {
		double	scale ;
		int 	m ;

		sf_command (sndfile, SFC_CALC_SIGNAL_MAX, &scale, sizeof (scale)) ;
		if (scale < 1e-10)
			scale = 1.0 ;
		else
			scale = 32700.0 / scale ;
		cout << "File samples scale = " << scale << endl;

		for (m = 0; m < smp->sampleLen; m++)
			smp->samples[m] *= scale;
	}

	sf_close(sndfile);

	return 0;
}


// Handle Ctrl-C by requesting that the audio rendering stop
void interrupt_handler(int var)
{
	gShouldStop = true;
}

// Print usage information
void usage(const char * processName)
{
	cerr << "Usage: " << processName << " [options]" << endl;

	BeagleRT_usage();

	cerr << "   --help [-h]:                Print this menu\n";
}

/* Function which returns the time since start of the program
 * in (fractional) seconds.
 */
double getCurrentTime(void) {
	unsigned long long result;
	struct timeval tv;

	gettimeofday(&tv, NULL);
	result = (tv.tv_sec - gFirstSeconds) * 1000000ULL + (tv.tv_usec - gFirstMicroseconds);
	return (double)result / 1000000.0;
}

// Thread for checking button
void *triggerFunction(void *data)
{
	int lastOutputLo = -1, lastOutputHi = -1;
	float lastTime = 0;
	float triggerInterval = 0.5;

	while(!gShouldStop)
	{
		// Write output whenever it changes
		if(gLEDLoOutput != lastOutputLo) {
			lastOutputLo = gLEDLoOutput;
			gpio_set_value(gOutputPinLo, lastOutputLo);
		}
		if(gLEDHiOutput != lastOutputHi) {
			lastOutputHi = gLEDHiOutput;
			gpio_set_value(gOutputPinHi, lastOutputHi);
		}
		// Read GPIO pin 1 & 2 - buttons
		if(gpio_get_value(gInputPin1, &latestValue1))
		{
			printf("Unable to read input value\n");
		}
		if(gpio_get_value(gInputPin2, &latestValue2))
		{
			printf("Unable to read input value\n");
		}
		//button 1
		if (gLastTriggerValue1 == 1 && latestValue1 == 0)

		{
			gTriggerButton1 = 1;
			//printf("Button1Pressed;\n");
			gIsPlaying =!gIsPlaying; 						//step 4b - Start/Stop
		}

		else {
			gTriggerButton1 = 0;
			//printf("Button1Off;\n");
		}

		//button2
		if (gLastTriggerValue2 == 0 && latestValue2 == 1)

		{
			gTriggerButton2 = 1;

			gButtonSwitch ++;
			if (gButtonSwitch>4)
				gButtonSwitch =0;
			//printf("Button2On; value= %d\n", gTriggerButton2 );


		}
		else {
			gTriggerButton2 = 0;
			//printf("Button2Off;\n");
		}

		gLastTriggerValue1 = latestValue1;
		gLastTriggerValue2 = latestValue2;


		usleep(1000);
	}

	pthread_exit(NULL);
}



int main(int argc, char *argv[])
{
	RTAudioSettings settings;	// Standard audio settings
	pthread_t gpioThread;
	struct timeval tv;
	string fileName;			// Name of the sample to load


	struct option customOptions[] =
	{
		{"help", 0, NULL, 'h'},
		{"file", 1, NULL, 'f'},
		{NULL, 0, NULL, 0}
	};

	SampleData sampleData;		// User define structure to pass data retrieved from file to render function
	sampleData.samples = 0;
	sampleData.sampleLen = -1;

	// Set default settings
	BeagleRT_defaultSettings(&settings);

	settings.periodSize = 32; // Larger period size by default, for testing

	// Parse command-line arguments
	while (1) {
		int c;
		if ((c = BeagleRT_getopt_long(argc, argv, "hf:", customOptions, &settings)) < 0)
				break;
		switch (c) {
		case 'h':
				usage(basename(argv[0]));
				exit(0);
		case 'f':
				fileName = string((char *)optarg);
				break;
		case '?':
		default:
				usage(basename(argv[0]));
				exit(1);
		}
	}

	if(fileName.empty()){
		//fileName = "singing.wav";
		fileName = "singing44.wav"; //load anechoic wav file it has to be mono, 16 bit and 44.1 KHz
		//fileName = "drumtest.wav";
	}

	// Initialise GPIO pins
	if(gpio_export(gOutputPinLo)) {
		printf("Warning: unable to export GPIO output pin\n");
	}
	if(gpio_set_dir(gOutputPinLo, OUTPUT_PIN)) {
		printf("Warning: unable to set GPIO output direction\n");
	}
	if(gpio_export(gOutputPinHi)) {
		printf("Warning: unable to export GPIO output pin\n");
	}
	if(gpio_set_dir(gOutputPinHi, OUTPUT_PIN)) {
		printf("Warning: unable to set GPIO output direction\n");
	}

	// Initialise GPIO pins 1
	if(gpio_export(gInputPin1)) {
		printf("Unable to export GPIO input pin\n");
	}
	if(gpio_set_dir(gInputPin1, INPUT_PIN)) {
		printf("Unable to set GPIO input direction\n");
	}


	// Initialise GPIO pins 2
	if(gpio_export(gInputPin2)) {
		printf("Unable to export GPIO input pin\n");

	}
	if(gpio_set_dir(gInputPin2, INPUT_PIN)) {
		printf("Unable to set GPIO input direction\n");

	}

	// Load file
	if(initFile(fileName, &sampleData) != 0)
	{
		cout << "Error: unable to load samples " << endl;
		return -1;
	}

	if(settings.verbose)
		cout << "File contains " << sampleData.sampleLen << " samples" << endl;


	// Initialise the PRU audio device
	if(BeagleRT_initAudio(&settings, &sampleData) != 0) {
		cout << "Error: unable to initialise audio" << endl;
		return -1;
	}

	// Initialise time
	gettimeofday(&tv, NULL);
	gFirstSeconds = tv.tv_sec;
	gFirstMicroseconds = tv.tv_usec;

	// Start button thread
	pthread_create(&gpioThread, NULL, triggerFunction, NULL);

	// Start the audio device running
	if(BeagleRT_startAudio()) {
		cout << "Error: unable to start real-time audio" << endl;
		return -1;
	}

	// Set up interrupt handler to catch Control-C
	signal(SIGINT, interrupt_handler);
	signal(SIGTERM, interrupt_handler);

	// Run until told to stop
	while(!gShouldStop) {
		usleep(100000);
	}

	// Stop the audio device
	BeagleRT_stopAudio();

	// Clean up any resources allocated for audio
	BeagleRT_cleanupAudio();

	// Unexport GPIO input pin
	if(gpio_unexport(gOutputPinLo)) {
		printf("Warning: unable to unexport GPIO input pin\n");
	}
	if(gpio_unexport(gOutputPinHi)) {
		printf("Warning: unable to unexport GPIO input pin\n");
	}

	// Unexport GPIO input pin
	if(gpio_unexport(gInputPin1)) {
		printf("Warning: unable to unexport GPIO input pin\n");
	}
	// Unexport GPIO input pin
	if(gpio_unexport(gInputPin2)) {
		printf("Warning: unable to unexport GPIO input pin\n");
	}


	pthread_join(gpioThread, NULL);

	// All done!
	return 0;
}


