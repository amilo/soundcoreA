/*
 * RTAudioCommandLine.cpp
 *
 *  Created on: Nov 8, 2014
 *      Author: parallels
 */

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <getopt.h>

#include "../include/RTAudio.h"

// Default command-line options for RTAudio
struct option gDefaultLongOptions[] =
{
	{"period", 1, NULL, 'p'},
	{"verbose", 0, NULL, 'v'},
	{"use-matrix", 1, NULL, 'm'},
	{"matrix-channels", 1, NULL, 'C'},
	{"mute-speaker", 1, NULL, 'M'},
	{"dac-level", 1, NULL, 'D'},
	{"adc-level", 1, NULL, 'A'},
	{"hp-level", 1, NULL, 'H'},
	{NULL, 0, NULL, 0}
};

const char gDefaultShortOptions[] = "p:vm:M:C:D:A:H:";

// This function sets the default settings for the RTAudioSettings structure
void BeagleRT_defaultSettings(RTAudioSettings *settings)
{
	// Set default values for settings
	settings->periodSize = 8;
	settings->beginMuted = 0;
	settings->dacLevel = DEFAULT_DAC_LEVEL;
	settings->adcLevel = DEFAULT_ADC_LEVEL;
	settings->headphoneLevel = DEFAULT_HP_LEVEL;
	settings->useMatrix = 1;
	settings->numMatrixChannels = 8;
	settings->verbose = 0;
	settings->codecI2CAddress = CODEC_I2C_ADDRESS;
	settings->ampMutePin = kAmplifierMutePin;
}

// This function drops in place of getopt() in the main() function
// and handles the initialisation of the RTAudio settings using
// standard command-line arguments. System default arguments will
// be stored in settings, otherwise arguments will be returned
// as getopt() normally does.

int BeagleRT_getopt_long(int argc, char *argv[], const char *customShortOptions, const struct option *customLongOptions, RTAudioSettings *settings)
{
	static int firstRun = 1;
	static char totalShortOptions[256];
	static struct option totalLongOptions[256];

	int c;

	// Prep total option string the first time this is
	// run. As a getopt() substitute, it will be called repeatedly working its
	// way through argc and argv.
	if(firstRun) {
		firstRun = 0;

		// Copy short options into one string
		strcpy(totalShortOptions, gDefaultShortOptions);
		strncat(totalShortOptions, customShortOptions, 256 - strlen(gDefaultShortOptions) - 1);

		// Copy long options into one array
		int n = 0;
		while(1) {
			if(gDefaultLongOptions[n].name == NULL)
				break;
			totalLongOptions[n].name = gDefaultLongOptions[n].name;
			totalLongOptions[n].has_arg = gDefaultLongOptions[n].has_arg;
			totalLongOptions[n].flag = gDefaultLongOptions[n].flag;
			totalLongOptions[n].val = gDefaultLongOptions[n].val;
			n++;
		}

		// Copy custom options into the array, if present
		if(customLongOptions == 0) {
			// Terminate the array
			totalLongOptions[n].name = NULL;
			totalLongOptions[n].has_arg = 0;
			totalLongOptions[n].flag = NULL;
			totalLongOptions[n].val = 0;
		}
		else {
			int customIndex = 0;
			while(n < 256) {
				if(customLongOptions[customIndex].name == NULL)
					break;
				totalLongOptions[n].name = customLongOptions[customIndex].name;
				totalLongOptions[n].has_arg = customLongOptions[customIndex].has_arg;
				totalLongOptions[n].flag = customLongOptions[customIndex].flag;
				totalLongOptions[n].val = customLongOptions[customIndex].val;
				n++;
				customIndex++;
			}

			// Terminate the array
			totalLongOptions[n].name = NULL;
			totalLongOptions[n].has_arg = 0;
			totalLongOptions[n].flag = NULL;
			totalLongOptions[n].val = 0;
		}
	}

	while(1) {
		if ((c = getopt_long(argc, argv, totalShortOptions, totalLongOptions, NULL)) < 0)
			return c;

		switch (c) {
		case 'p':
			settings->periodSize = atoi(optarg);
			if(settings->periodSize < 1)
				settings->periodSize = 1;
			break;
		case 'v':
			settings->verbose = 1;
			break;
		case 'm':
			settings->useMatrix = atoi(optarg);
			break;
		case 'C':
			settings->numMatrixChannels = atoi(optarg);
			if(settings->numMatrixChannels >= 8)
				settings->numMatrixChannels = 8;
			else if(settings->numMatrixChannels >= 4)
				settings->numMatrixChannels = 4;
			else
				settings->numMatrixChannels = 2;
			break;
		case 'M':
			settings->beginMuted = atoi(optarg);
			break;
		case 'D':
			settings->dacLevel = atof(optarg);
			break;
		case 'A':
			settings->adcLevel = atof(optarg);
			break;
		case 'H':
			settings->headphoneLevel = atof(optarg);
			break;
		case '?':
		default:
			return c;
		}
	}
}

// This function prints standard usage information for default arguments
// Call from within your own usage function
void BeagleRT_usage()
{
	std::cerr << "   --period [-p] period:       Set the hardware period (buffer) size in matrix samples\n";
	std::cerr << "   --dac-level [-D] dBs:       Set the DAC output level (0dB max; -63.5dB min)\n";
	std::cerr << "   --adc-level [-A] dBs:       Set the ADC input level (0dB max; -12dB min)\n";
	std::cerr << "   --hp-level [-H] dBs:        Set the headphone output level (0dB max; -63.5dB min)\n";
	std::cerr << "   --mute-speaker [-M] val:    Set whether to mute the speaker initially (default: no)\n";
	std::cerr << "   --use-matrix [-m] val:      Set whether to use ADC/DAC matrix\n";
	std::cerr << "   --matrix-channels [-C] val: Set the number of ADC/DAC channels (default: 8)\n";
	std::cerr << "   --verbose [-v]:             Enable verbose logging information\n";
}
