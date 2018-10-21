/*
 *  RTAudio.cpp
 *
 *  Central control code for hard real-time audio on BeagleBone Black
 *  using PRU and Xenomai Linux extensions. This code began as part
 *  of the Hackable Instruments project (EPSRC) at Queen Mary University
 *  of London, 2013-14.
 *
 *  (c) 2014 Victor Zappi and Andrew McPherson
 *  Queen Mary University of London
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <math.h>
#include <iostream>
#include <assert.h>
#include <vector>

// Xenomai-specific includes
#include <sys/mman.h>
#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>

#include "../include/RTAudio.h"
#include "../include/PRU.h"
#include "../include/I2c_Codec.h"
#include "../include/render.h"
#include "../include/GPIOcontrol.h"

using namespace std;

// Data structure to keep track of auxiliary tasks we
// can schedule
typedef struct {
	RT_TASK task;
	void (*function)(void);
	char *name;
	int priority;
} InternalAuxiliaryTask;

const char gRTAudioThreadName[] = "beaglert-audio";

// Real-time tasks and objects
RT_TASK gRTAudioThread;
PRU *gPRU = 0;
I2c_Codec *gAudioCodec = 0;

vector<InternalAuxiliaryTask*> gAuxTasks;

// Flag which tells the audio task to stop
bool gShouldStop = false;

// general settings
int gRTAudioVerbose = 0;   						// Verbosity level for debugging
int gAmplifierMutePin = -1;
int gAmplifierShouldBeginMuted = 0;

// Number of audio and matrix channels, globally accessible
// At least gNumMatrixChannels needs to be global to be used
// by the analogRead() and analogWrite() macros without creating
// extra confusion in their use cases by passing this argument
int gNumAudioChannels = 0;
int gNumMatrixChannels = 0;

// initAudio() prepares the infrastructure for running PRU-based real-time
// audio, but does not actually start the calculations.
// periodSize indicates the number of _sensor_ frames per period: the audio period size
// is twice this value. In total, the audio latency in frames will be 4*periodSize,
// plus any latency inherent in the ADCs and DACs themselves.
// useMatrix indicates whether to enable the ADC and DAC or just use the audio codec.
// numMatrixChannels indicates how many ADC and DAC channels to use.
// userData is an opaque pointer which will be passed through to the initialise_render()
// function for application-specific use
//
// Returns 0 on success.

int BeagleRT_initAudio(RTAudioSettings *settings, void *userData)
{
	rt_print_auto_init(1);
	setVerboseLevel(settings->verbose);

	if(gRTAudioVerbose == 1)
		rt_printf("Running with Xenomai\n");

	if(gRTAudioVerbose) {
		cout << "Starting with period size " << settings->periodSize << "; ";
		if(settings->useMatrix)
			cout << "matrix enabled\n";
		else
			cout << "matrix disabled\n";
		cout << "DAC level " << settings->dacLevel << "dB; ADC level " << settings->adcLevel;
		cout << "dB; headphone level " << settings->headphoneLevel << "dB\n";
		if(settings->beginMuted)
			cout << "Beginning with speaker muted\n";
	}

	// Prepare GPIO pins for amplifier mute and status LED
	if(settings->ampMutePin >= 0) {
		gAmplifierMutePin = settings->ampMutePin;
		gAmplifierShouldBeginMuted = settings->beginMuted;

		if(gpio_export(settings->ampMutePin)) {
			if(gRTAudioVerbose)
				cout << "Warning: couldn't export amplifier mute pin\n";
		}
		if(gpio_set_dir(settings->ampMutePin, OUTPUT_PIN)) {
			if(gRTAudioVerbose)
				cout << "Couldn't set direction on amplifier mute pin\n";
			return -1;
		}
		if(gpio_set_value(settings->ampMutePin, LOW)) {
			if(gRTAudioVerbose)
				cout << "Couldn't set value on amplifier mute pin\n";
			return -1;
		}
	}

	// Limit the matrix channels to sane values
	if(settings->numMatrixChannels >= 8)
		settings->numMatrixChannels = 8;
	else if(settings->numMatrixChannels >= 4)
		settings->numMatrixChannels = 4;
	else
		settings->numMatrixChannels = 2;

	// Sanity check the combination of channels and period size
	if(settings->numMatrixChannels <= 4 && settings->periodSize < 2) {
		cout << "Error: " << settings->numMatrixChannels << " channels and period size of " << settings->periodSize << " not supported.\n";
		return 1;
	}
	if(settings->numMatrixChannels <= 2 && settings->periodSize < 4) {
		cout << "Error: " << settings->numMatrixChannels << " channels and period size of " << settings->periodSize << " not supported.\n";
		return 1;
	}

	// Use PRU for audio
	gPRU = new PRU();
	gAudioCodec = new I2c_Codec();

	if(gPRU->prepareGPIO(settings->useMatrix, 1, 1)) {
		cout << "Error: unable to prepare GPIO for PRU audio\n";
		return 1;
	}
	if(gPRU->initialise(0, settings->periodSize, settings->numMatrixChannels, true)) {
		cout << "Error: unable to initialise PRU\n";
		return 1;
	}
	if(gAudioCodec->initI2C_RW(2, settings->codecI2CAddress, -1)) {
		cout << "Unable to open codec I2C\n";
		return 1;
	}
	if(gAudioCodec->initCodec()) {
		cout << "Error: unable to initialise audio codec\n";
		return 1;
	}

	// Set default volume levels
	BeagleRT_setDACLevel(settings->dacLevel);
	BeagleRT_setADCLevel(settings->adcLevel);
	BeagleRT_setHeadphoneLevel(settings->headphoneLevel);

	// Initialise the rendering environment: pass the number of audio and matrix
	// channels, the period size for matrix and audio, and the sample rates

	int audioPeriodSize = settings->periodSize * 2;
	float audioSampleRate = 44100.0;
	float matrixSampleRate = 22050.0;
	if(settings->useMatrix) {
		audioPeriodSize = settings->periodSize * settings->numMatrixChannels / 4;
		matrixSampleRate = audioSampleRate * 4.0 / (float)settings->numMatrixChannels;
	}

	gNumAudioChannels = 2;
	gNumMatrixChannels = settings->useMatrix ? settings->numMatrixChannels : 0;

	if(!initialise_render(gNumMatrixChannels, gNumAudioChannels,
				          settings->useMatrix ? settings->periodSize : 0, /* matrix period size */
				          audioPeriodSize,
				          matrixSampleRate, audioSampleRate,
				          userData)) {
		cout << "Couldn't initialise audio rendering\n";
		return 1;
	}

	return 0;
}

// audioLoop() is the main function which starts the PRU audio code
// and then transfers control to the PRU object. The PRU object in
// turn will call the audio render() callback function every time
// there is new data to process.

void audioLoop(void *)
{
	if(gRTAudioVerbose==1)
		rt_printf("_________________Audio Thread!\n");

	// PRU audio
	assert(gAudioCodec != 0 && gPRU != 0);

	if(gAudioCodec->startAudio(0)) {
		rt_printf("Error: unable to start I2C audio codec\n");
		gShouldStop = 1;
	}
	else {
		if(gPRU->start()) {
			rt_printf("Error: unable to start PRU\n");
			gShouldStop = 1;
		}
		else {
			// All systems go. Run the loop; it will end when gShouldStop is set to 1

			if(!gAmplifierShouldBeginMuted) {
				// First unmute the amplifier
				if(BeagleRT_muteSpeakers(0)) {
					if(gRTAudioVerbose)
						rt_printf("Warning: couldn't set value (high) on amplifier mute pin\n");
				}
			}

			gPRU->loop();

			// Now clean up
			// gPRU->waitForFinish();
			gPRU->disable();
			gAudioCodec->stopAudio();
			gPRU->cleanupGPIO();
		}
	}

	if(gRTAudioVerbose == 1)
		rt_printf("audio thread ended\n");
}

// Create a calculation loop which can run independently of the audio, at a different
// (equal or lower) priority. Audio priority is 99; priority should be generally be less than this.
// Returns an (opaque) pointer to the created task on success; 0 on failure
AuxiliaryTask createAuxiliaryTaskLoop(void (*functionToCall)(void), int priority, const char *name)
{
	InternalAuxiliaryTask *newTask = (InternalAuxiliaryTask*)malloc(sizeof(InternalAuxiliaryTask));

	// Attempt to create the task
	if(rt_task_create(&(newTask->task), name, 0, priority, T_JOINABLE | T_FPU)) {
		  cout << "Error: unable to create auxiliary task " << name << endl;
		  free(newTask);
		  return 0;
	}

	// Populate the rest of the data structure and store it in the vector
	newTask->function = functionToCall;
	newTask->name = strdup(name);
	newTask->priority = priority;

	gAuxTasks.push_back(newTask);

	return (AuxiliaryTask)newTask;
}

// Schedule a previously created auxiliary task. It will run when the priority rules next
// allow it to be scheduled.
void scheduleAuxiliaryTask(AuxiliaryTask task)
{
	InternalAuxiliaryTask *taskToSchedule = (InternalAuxiliaryTask *)task;

	rt_task_resume(&taskToSchedule->task);
}

// Calculation loop that can be used for other tasks running at a lower
// priority than the audio thread. Simple wrapper for Xenomai calls.
// Treat the argument as containing the task structure
void auxiliaryTaskLoop(void *taskStruct)
{
	// Get function to call from the argument
	void (*auxiliary_function)(void) = ((InternalAuxiliaryTask *)taskStruct)->function;
	const char *name = ((InternalAuxiliaryTask *)taskStruct)->name;

	// Wait for a notification
	rt_task_suspend(NULL);

	while(!gShouldStop) {
		// Then run the calculations
		auxiliary_function();

		// Wait for a notification
		rt_task_suspend(NULL);
	}

	if(gRTAudioVerbose == 1)
		rt_printf("auxiliary task %s ended\n", name);
}

// startAudio() should be called only after initAudio() successfully completes.
// It launches the real-time Xenomai task which runs the audio loop. Returns 0
// on success.

int BeagleRT_startAudio()
{
	// Create audio thread with the highest priority
	if(rt_task_create(&gRTAudioThread, gRTAudioThreadName, 0, 99, T_JOINABLE | T_FPU)) {
		  cout << "Error: unable to create Xenomai audio thread" << endl;
		  return -1;
	}

	// Start all RT threads
	if(rt_task_start(&gRTAudioThread, &audioLoop, 0)) {
		  cout << "Error: unable to start Xenomai audio thread" << endl;
		  return -1;
	}

	// The user may have created other tasks. Start those also.
	vector<InternalAuxiliaryTask*>::iterator it;
	for(it = gAuxTasks.begin(); it != gAuxTasks.end(); it++) {
		InternalAuxiliaryTask *taskStruct = *it;

		if(rt_task_start(&(taskStruct->task), &auxiliaryTaskLoop, taskStruct)) {
			cerr << "Error: unable to start Xenomai task " << taskStruct->name << endl;
			return -1;
		}
	}

	return 0;
}

// Stop the PRU-based audio from running and wait
// for the tasks to complete before returning.

void BeagleRT_stopAudio()
{
	// Tell audio thread to stop (if this hasn't been done already)
	gShouldStop = true;

	if(gRTAudioVerbose)
		cout << "Stopping audio...\n";

	// Now wait for threads to respond and actually stop...
	rt_task_join(&gRTAudioThread);

	// Stop all the auxiliary threads too
	vector<InternalAuxiliaryTask*>::iterator it;
	for(it = gAuxTasks.begin(); it != gAuxTasks.end(); it++) {
		InternalAuxiliaryTask *taskStruct = *it;

		// Wake up each thread and join it
		rt_task_resume(&(taskStruct->task));
		rt_task_join(&(taskStruct->task));
	}
}

// Free any resources associated with PRU real-time audio
void BeagleRT_cleanupAudio()
{
	cleanup_render();

	// Clean up the auxiliary tasks
	vector<InternalAuxiliaryTask*>::iterator it;
	for(it = gAuxTasks.begin(); it != gAuxTasks.end(); it++) {
		InternalAuxiliaryTask *taskStruct = *it;

		// Free the name string and the struct itself
		free(taskStruct->name);
		free(taskStruct);
	}
	gAuxTasks.clear();

	if(gPRU != 0)
		delete gPRU;
	if(gAudioCodec != 0)
		delete gAudioCodec;

	if(gAmplifierMutePin >= 0)
		gpio_unexport(gAmplifierMutePin);
	gAmplifierMutePin = -1;
}

// Set the level of the DAC; affects all outputs (headphone, line, speaker)
// 0dB is the maximum, -63.5dB is the minimum; 0.5dB steps
int BeagleRT_setDACLevel(float decibels)
{
	if(gAudioCodec == 0)
		return -1;
	return gAudioCodec->setDACVolume((int)floorf(decibels * 2.0 + 0.5));
}

// Set the level of the ADC
// 0dB is the maximum, -12dB is the minimum; 1.5dB steps
int BeagleRT_setADCLevel(float decibels)
{
	if(gAudioCodec == 0)
		return -1;
	return gAudioCodec->setADCVolume((int)floorf(decibels * 2.0 + 0.5));
}

// Set the level of the onboard headphone amplifier; affects headphone
// output only (not line out or speaker)
// 0dB is the maximum, -63.5dB is the minimum; 0.5dB steps
int BeagleRT_setHeadphoneLevel(float decibels)
{
	if(gAudioCodec == 0)
		return -1;
	return gAudioCodec->setHPVolume((int)floorf(decibels * 2.0 + 0.5));
}

// Mute or unmute the onboard speaker amplifiers
// mute == 0 means unmute; otherwise mute
// Returns 0 on success
int BeagleRT_muteSpeakers(int mute)
{
	int pinValue = mute ? LOW : HIGH;

	// Check that we have an enabled pin for controlling the mute
	if(gAmplifierMutePin < 0)
		return -1;

	return gpio_set_value(gAmplifierMutePin, pinValue);
}

// Set the verbosity level
void setVerboseLevel(int level)
{
	gRTAudioVerbose = level;
}
