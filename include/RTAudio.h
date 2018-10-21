/*
 *  RTAudio.h
 *
 *  Central control code for hard real-time audio on BeagleBone Black
 *  using PRU and Xenomai Linux extensions. This code began as part
 *  of the Hackable Instruments project (EPSRC) at Queen Mary University
 *  of London, 2013-14.
 *
 *  (c) 2014 Victor Zappi and Andrew McPherson
 *  Queen Mary University of London
 */


#ifndef RTAUDIO_H_
#define RTAUDIO_H_

#include "render.h"

// Useful constants
#define DBOX_CAPE					// New custom cape

#ifdef DBOX_CAPE
#define CODEC_I2C_ADDRESS  0x18		// Address of TLV320AIC3104 codec
#else
#define CODEC_I2C_ADDRESS  0x1B		// Address of TLV320AIC3106 codec
#endif

// Default volume levels
#define DEFAULT_DAC_LEVEL	0.0
#define DEFAULT_ADC_LEVEL	-6.0
#define DEFAULT_HP_LEVEL	-6.0

enum {
	kAmplifierMutePin = 61	// P8-26 controls amplifier mute
};

// Structure which contains initialisation parameters for the
// real-time audio system
typedef struct {
	// These items might be adjusted by the user:
	int periodSize;			// Number of (matrix) frames per period; audio is twice this
	int beginMuted;			// Whether to begin with the speakers muted
	float dacLevel;			// Level for the audio DAC output
	float adcLevel;			// Level for the audio ADC input
	float headphoneLevel;	// Level for the headphone output
	int useMatrix;			// Whether to use the matrix
	int numMatrixChannels;	// How many channels for the ADC and DAC
	int verbose;			// Whether to use verbose logging

	// These items are hardware-dependent and should only be changed
	// to run on different hardware
	int codecI2CAddress;	// Where the codec can be found on the I2C bus
	int ampMutePin;			// Pin where amplifier mute can be found
} RTAudioSettings;

typedef void* AuxiliaryTask;	// Opaque data type to keep track of aux tasks

// Flag that indicates when the audio will stop; can be read or
// set by other components which should end at the same time as the audio
extern bool gShouldStop;

// Command-line settings
void BeagleRT_defaultSettings(RTAudioSettings *settings);
int BeagleRT_getopt_long(int argc, char *argv[], const char *customShortOptions,
				   const struct option *customLongOptions, RTAudioSettings *settings);
void BeagleRT_usage();

// Basic audio control functions: init, start, stop and clean up
int BeagleRT_initAudio(RTAudioSettings *settings, void *userData);
int BeagleRT_startAudio();
void BeagleRT_stopAudio();
void BeagleRT_cleanupAudio();

// Volume/level controls
// These return 0 on success

// Set the level of the DAC; affects all outputs (headphone, line, speaker)
// 0dB is the maximum, -63.5dB is the minimum; 0.5dB steps
int BeagleRT_setDACLevel(float decibels);

// Set the level of the ADC
// 0dB is the maximum, -12dB is the minimum; 1.5dB steps
int BeagleRT_setADCLevel(float decibels);

// Set the level of the onboard headphone amplifier; affects headphone
// output only (not line out or speaker)
// 0dB is the maximum, -63.5dB is the minimum; 0.5dB steps
int BeagleRT_setHeadphoneLevel(float decibels);

// Mute or unmute the onboard speaker amplifiers
// mute == 0 means unmute; otherwise mute
// Returns 0 on success
int BeagleRT_muteSpeakers(int mute);

// Functions for creating auxiliary tasks
AuxiliaryTask createAuxiliaryTaskLoop(void (*functionToCall)(void), int priority, const char *name);
void scheduleAuxiliaryTask(AuxiliaryTask task);

void setVerboseLevel(int level);

#endif /* RTAUDIO_H_ */
