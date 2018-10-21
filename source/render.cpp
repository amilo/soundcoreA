/*
 * render.cpp
 *
 *  Created on: Oct 24, 2014
 *      Author: parallels
 *
 *
 *
 *      Author: Alessia Milo MAT PhD Student
 *      Real-Time DSP - ECS732 - Andrew McPherson - TA - Giulio Moro
 *      April 2015
 *      Title: SoundCoreA
 *
 */
//Licence below for the sound library libsndfile 1.0.25 to write wav
/*
** Copyright (C) 2007-2011 Erik de Castro Lopo <erikd@mega-nerd.com>
**
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
*/




#include "../include/render.h"
#include "../include/RTAudio.h"
#include <rtdk.h>
#include <NE10.h>					// NEON FFT library
#include <cmath>
#include "SampleData.h"
#include	<cstring>
#include	<cstdio>
#include	<sndfile.hh>


//#define BUFFER_SIZE 32768
#define BUFFER_SIZE 65536
#define	BUFFER_SIZE_IN				1000000	//THE POINTER WRAPS ACCORDING TO POTENTIOMETER
#define	BUFFER_SIZE_WRITE			1000000 //TO WRITE THE WAV FILE


float gStoreBuffer[BUFFER_SIZE_IN];		//buffer to copy the audio track loaded
float gWriteBuffer[BUFFER_SIZE_IN];		//buffer to write

float gSweepBufferOut[BUFFER_SIZE_IN];	//buffer to copy the sweepOut for wav writing
float gSweepBufferIn[BUFFER_SIZE_IN];	//buffer to copy the sweepIn for wav writing and deconvolving the sweepOut later


int gPointer=0;	//pointer for reading gStoreBuffer
int gWritePointer=0;
int gReadPointer=0;
int gSweepPointer=0;


//Buffer and pointers for the FFT
float gOutputBuffer[BUFFER_SIZE];
int gOutputBufferWritePointer = 0;
int gOutputBufferReadPointer = 0;

//Input Buffer is shared and used mainly to store the Input from the Left and Right Binaural Microphones
float gInputBuffer[BUFFER_SIZE_IN];
int gInputBufferPointer = 0;

//Buffer to copy long buffer to short buffer for the FFT
float gInputBufferCopy[BUFFER_SIZE];
int gInputBufferPointerCopy=0;


float gPhase;
float gFFTPhase;
float gFrequency = 440;
float gInverseSampleRate;
float gEnvelopeAmplitude = 1.000;


int gSampleCount = 0;
//int gFrequencyIntervalMilliseconds = 600;
int gFrequencyIntervalMilliseconds = 200; //for the synth
int gSamplesIntervalLength = 0;
int gSamplesFrequencyCount = 0 ;

extern int gIsPlaying;
int gFirstTime;

float *gWindowBuffer;

float in;

int gFFTSize = 1024;
int gHopSize = gFFTSize / 2;
float gFFTScaleFactor = 0;
extern int gLEDLoOutput, gLEDHiOutput;

//thresholds
//float gThreshold = 5000.0;
float gThreshold = 50.0;
float gTotalThreshold = 4.0;
float gHiThreshold = 0.01;

//conditions for thresholds
int gThresholdLevelDanger = 0;
int gHiLevelDanger =0;

#define AVERAGING_SIZE 50
float gEnergyAveragingBins[AVERAGING_SIZE];

int gSamplesEnergyCount ;
int gSamplesAveragingInterval;
float gEnergyIntervalMilliseconds = 62.5;  //1/16 of second
//float gEnergyIntervalMilliseconds = 500;
int gEnergyAveragingPointer = 0;
int gEnergyCount=0;
float averagedEnergyInTime;

int gNumberIteration = 0;


float f1 = 22.5;
float f2 = 55.0;

// FFT vars
ne10_fft_cpx_float32_t* timeDomainIn;
ne10_fft_cpx_float32_t* timeDomainOut;
ne10_fft_cpx_float32_t* frequencyDomain;
ne10_fft_cfg_float32_t cfg;

// Sample info
SampleData gSampleData;	// User defined structure to get complex data from main
int gReadPtr = 0;		// Position of last read sample from file

int gOneSweep = 0;

float gMinInput = 0.000427;		// min value from Potentiometer;
float gMaxInput = 0.823;		// max value from Potentiometer;

int gMinSamples = 64;				//MinSample Interval
int gMaxSamples = 450000;			//MaxSample Interval

// Auxiliary task for calculating FFT
AuxiliaryTask gFFTTask;
int gFFTInputBufferPointer = 0;
int gFFTOutputBufferPointer = 0;

float gAudioSampleRate;

//states for the switch button
enum{
		kSwitchOff = 0,
		kSwitchSing,
		kSwitchScale,
		kSwitchSweep,
		kSwitchSine,

	};

extern int gTriggerButton1;
extern int gTriggerButton2;

//unsigned int gButtonStart= 1;
extern int gButtonSwitch;

//float increment = 1.029302; //for 50 cents scale
float increment = 1.059463; //for chromatic scale 100 cents


float envelopeIncrement = 0.0001;
float gScalef1 = 55;
float gScalef2 = 7040;
int gCount;
float gGain=-0.0f;
float gGainLevel=0.5f;

int gNumSamplesTaken = BUFFER_SIZE-1;
int gLastNumSamplesTaken;





void process_fft_background();

// initialise_render() is called once before the audio rendering starts.
// Use it to perform any initialisation and allocation which is dependent
// on the period size or sample rate.
//
// userData holds an opaque pointer to a data structure that was passed
// in from the call to initAudio().
//
// Return true on success; returning false halts the program.

bool initialise_render(int numMatrixChannels, int numAudioChannels,
					   int numMatrixFramesPerPeriod,
					   int numAudioFramesPerPeriod,
					   float matrixSampleRate, float audioSampleRate,
					   void *userData)
{

	// Retrieve a parameter passed in from the initAudio() call
	gSampleData = *(SampleData *)userData;

	gInverseSampleRate = 1.0 / audioSampleRate;
	gAudioSampleRate = audioSampleRate;
	gPhase = 0.0;		// gPhase for synth

	gSamplesIntervalLength = (int)(audioSampleRate*gFrequencyIntervalMilliseconds)/1000;
	gSamplesAveragingInterval = (int)(audioSampleRate*gEnergyIntervalMilliseconds)/1000;

	gFFTScaleFactor = 1.0f / (float)gFFTSize;
	gOutputBufferWritePointer += gHopSize;

	timeDomainIn = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * sizeof (ne10_fft_cpx_float32_t));
	timeDomainOut = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * sizeof (ne10_fft_cpx_float32_t));
	frequencyDomain = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * sizeof (ne10_fft_cpx_float32_t));
	cfg = ne10_fft_alloc_c2c_float32 (gFFTSize);

	memset(timeDomainOut, 0, gFFTSize * sizeof (ne10_fft_cpx_float32_t));
	memset(gOutputBuffer, 0, BUFFER_SIZE * sizeof(float));

	// Allocate the window buffer based on the FFT size
	gWindowBuffer = (float *)malloc(gFFTSize * sizeof(float));
	if(gWindowBuffer == 0)
		return false;

	// Calculate a Hann window
	for(int n = 0; n < gFFTSize; n++) {
		gWindowBuffer[n] = 0.5f * (1.0f - cosf(2.0 * M_PI * n / (float)(gFFTSize - 1)));
	}

	// Initialise auxiliary tasks
	if((gFFTTask = createAuxiliaryTaskLoop(&process_fft_background, 90, "fft-calculation")) == 0)
		return false;

	return true;
}
//////////////////

static void create_file (const char * fnameIn, const char * fnameOut,int format)
{	static float bufferIn [BUFFER_SIZE_WRITE] ;
	static float bufferOut [BUFFER_SIZE_WRITE] ;

	SndfileHandle fileIn ;
	SndfileHandle fileOut ;
	int channels = 2 ;
	int srate = 44100 ;

	printf ("Creating file named '%s'\n", fnameIn) ;
	printf ("Creating file named '%s'\n", fnameOut) ;
	fileIn = SndfileHandle (fnameIn, SFM_WRITE, format, channels, srate) ;
	fileOut = SndfileHandle (fnameOut, SFM_WRITE, format, channels, srate) ;
	memset (bufferIn, 0, sizeof (bufferIn)) ;
	memset (bufferOut, 0, sizeof (bufferOut)) ;

for (int n=0; n< BUFFER_SIZE_WRITE; n++){
	bufferIn[n] = gSweepBufferIn[n];
	bufferOut[n] = gSweepBufferOut[n];
}
	fileIn.write (bufferIn, BUFFER_SIZE_WRITE) ;
	fileOut.write (bufferOut, BUFFER_SIZE_WRITE) ;
	puts ("") ;
	/*
	**	The SndfileHandle object will automatically close the file and
	**	release all allocated memory when the object goes out of scope.
	**	This is the Resource Acquisition Is Initailization idom.
	**	See : http://en.wikipedia.org/wiki/Resource_Acquisition_Is_Initialization
	*/
} /* create_file */

void write_file()
{
	const char * fnameIn = "testIn.wav" ;
	const char * fnameOut = "testOut.wav" ;
		//puts ("\nSimple example showing usage of the C++ SndfileHandle object.\n") ;
		create_file (fnameIn, fnameOut, SF_FORMAT_WAV | SF_FORMAT_PCM_16) ;
		puts ("Done.\n") ;
}
//////////////////////////

// This function handles the FFT processing in this example once the buffer has
// been assembled.
void process_fft(float *inBuffer, int inWritePointer, float *outBuffer, int outWritePointer)
{
	// Copy buffer into FFT input
	int pointer = (inWritePointer - gFFTSize + BUFFER_SIZE) % BUFFER_SIZE;
	for(int n = 0; n < gFFTSize; n++) {
		timeDomainIn[n].r = (ne10_float32_t) inBuffer[pointer] * gWindowBuffer[n];
		timeDomainIn[n].i = 0;

		pointer++;
		if(pointer >= BUFFER_SIZE)
			pointer = 0;
	}

	// Run the FFT
	ne10_fft_c2c_1d_float32_neon (frequencyDomain, timeDomainIn, cfg->twiddles, cfg->factors, gFFTSize, 0);

	// Look for energy in lowest bins and set LED if above a threshold
	float totalEnergy = 0;
	float totalEnergyHi = 0;

	for(int n = 0; n < gFFTSize; n++) {
		totalEnergy += frequencyDomain[n].r * frequencyDomain[n].r + frequencyDomain[n].i * frequencyDomain[n].i;
	}
	/*
	for(int n = 0; n < gFFTSize/32 ; n++) {
			totalEnergy += frequencyDomain[n].r * frequencyDomain[n].r + frequencyDomain[n].i * frequencyDomain[n].i;
		}
	*/
	for(int n = gFFTSize/2 - gFFTSize/8; n < gFFTSize / 2; n++) {
		totalEnergyHi += frequencyDomain[n].r * frequencyDomain[n].r + frequencyDomain[n].i * frequencyDomain[n].i;
	}

	//totalEnergy /= (float)(gFFTSize / 32);
	totalEnergy /= (float)(gFFTSize );
	totalEnergyHi /= (float)(gFFTSize / 8);

	//rt_printf("%f %f\n", totalEnergy, totalEnergyHi);
	//if(totalEnergy > gThreshold){

	if(totalEnergy > gTotalThreshold){ //empirical values may be different with other systems
		gLEDLoOutput = 1; //blink
		gThresholdLevelDanger = 1; //set the condition to do something
		}
	else if(totalEnergy < 0.1) { //if too quiet
		gThresholdLevelDanger = -1; //different condition
		}
	else{ //otherwise all ok
		gLEDLoOutput = 0;
		gThresholdLevelDanger =0;
		}

	if(totalEnergyHi > gHiThreshold ){ //look at the high frequencies
		gLEDHiOutput = 1; 	//Hi Led ON
		gHiLevelDanger =1; // hi to high
		//gGainLevel = 0.001; //go quiet - not really helpful
	}
	else{ //otherwise all ok
		gLEDHiOutput = 0;
		gHiLevelDanger =0;
	}

	if (gSamplesEnergyCount >= gSamplesAveragingInterval){ // read values after an interval of 1/16 sec

		if (gThresholdLevelDanger == 1 && averagedEnergyInTime >= 10){ //if volume too high
		gGain= -0.005f;	//reduce gain
		}
		else if (gThresholdLevelDanger == -1  && averagedEnergyInTime <=0.5) { //if too quiet
		gGain = +0.0001f;  //amplify gain
		//gButtonSwitch = 3; //dynamic behaviour still unstable requires microphone and speaker measurement
		}
		else if (gHiLevelDanger == 1 && averagedEnergyInTime < 10) {  // there are high frequencies
		//gGain = -0.1;
		//gGain = +0.002f;
		//gButtonSwitch = 0; //dynamic behaviour : reset to no Audio - Listening if energy too high
		}

		else if (gGainLevel < 0){ //this shouldn't happen
			//gButtonSwitch = 0; //stop in case
			//gGainLevel = 0.5f; //reset to default
		}
		else if (gGainLevel >= 1){ //also this shouldn't happen
			//gGain = -gGain; //just for testing
		}
		else{
			gGain = 0; //do nothing to gain
		}

		gGainLevel = gGainLevel+(gGain*gSamplesEnergyCount/gSamplesAveragingInterval*0.5); //just one expression, not really stable

		if (gEnergyCount % 10 == 0){
		//rt_printf("gGain, gGLev, %f, %f\n", gGain, gGainLevel); //debug for gain control reading
		}

		//Energy Calculation

		gEnergyAveragingPointer++; // keep the energy pointer moving
		//rt_printf(" energyPointer %d\n", gEnergyAveragingPointer);

		gSamplesEnergyCount = 0;//reset the Interval (we are still inside)
		gEnergyCount++;

		//energyAveragingBins[n]=totalEnergy;
		gEnergyAveragingBins[gEnergyAveragingPointer] = (float)totalEnergy;
		//rt_printf("%f %f\n", totalEnergy, totalEnergyHi);

		for (int n = 0; n < AVERAGING_SIZE; n++){
		averagedEnergyInTime += gEnergyAveragingBins[n];
		}

		averagedEnergyInTime = averagedEnergyInTime/AVERAGING_SIZE;  //divide the energy for the number of bins
		//rt_printf(" averagedEnergyInTime %f\n", averagedEnergyInTime); //uncomment to print averaged values

		if (gEnergyAveragingPointer>= AVERAGING_SIZE) //wrap energy pointer time array
		gEnergyAveragingPointer=0;
		}

/*
	if (averagedEnergyInTime >= 5 && totalEnergy>=10){
	//gLEDHiOutput = 1;
	//rt_printf(" LED on %f\n", averagedEnergyInTime);
	for (int n = 0; n < AVERAGING_SIZE; n++){  //reset the bins if energy is getting too high
			gEnergyAveragingBins[n]=0;
			}
	}
*/
		//else
			//gLEDHiOutput = 0;

	//if(totalEnergyHi > gHiThreshold && totalEnergy < 10)
			//gLEDHiOutput = 1;
		//else
			//gLEDHiOutput = 0;


		if (gButtonSwitch == 0){ //calculate relative A-weighting coefficients dbA on bins for the FFT

			if (gEnergyCount % 10 == 0){ //not all the time for debug

			for(int n = 0; n < gFFTSize; n++) {

				float f = gAudioSampleRate/gFFTSize * n ; //get the frequency for the slice n

				 // get normalized bin magnitude
				 double normBinMag = sqrtf(frequencyDomain[n].r * frequencyDomain[n].r + frequencyDomain[n].i * frequencyDomain[n].i)/gFFTSize;

				 // convert to dB value
				 double amplitude = 20.0 * log10( normBinMag );
				 //rt_printf("dBamp = %f /// %f ;", f, amplitude);

				 float f2 = f*f;
				 float f4 = f*f*f*f;

				 // Return spectral A-weighting for this frequency.

			   double firstPart = (12200*12200 * f4)/(  (f2 + 20.6*20.6) * sqrt((f2+107.7*107.7)*(f2+(737.9*737.9))) *(f2+(12200*12200))  );
			   double Aweighting = 2.0 + 20 * log10(firstPart);

			   double AMag = amplitude + Aweighting;
			   // rt_printf(" F-Amp-dBA = %d-%f-%f-%f;", (int)f,  amplitude, Aweighting, AMag); //uncomment for debug
			}//close FFT
		}//close button 0 condition
	}//close Energy reading update
	//rt_printf("  \n"); //uncomment together with upper line for debug


	/*
	// Run the inverse FFT //nothing has been done to the signal so we don't need this for the moment

	ne10_fft_c2c_1d_float32_neon (timeDomainOut, frequencyDomain, cfg->twiddles, cfg->factors, gFFTSize, 1);

	// Add timeDomainOut into the output buffer
	pointer = outWritePointer;
	for(int n = 0; n < gFFTSize; n++) {
	outBuffer[pointer] += timeDomainOut[n].r * gFFTScaleFactor;
	pointer++;
	if(pointer >= BUFFER_SIZE)
		pointer = 0;
	}
	*/

}//close process FFT

// Function to process the FFT in a thread at lower priority
void process_fft_background() {
	process_fft(gInputBufferCopy, gFFTInputBufferPointer, gOutputBuffer, gFFTOutputBufferPointer);
}

// render() is called regularly at the highest priority by the audio engine.
// Input and output are given from the audio hardware and the other
// ADCs and DACs (if available). If only audio is available, numMatrixFrames
// will be 0.

void render(int numMatrixFrames, int numAudioFrames, float *audioIn, float *audioOut,
			float *matrixIn, float *matrixOut)
{



	if (gOneSweep==2 && gFirstTime==1){ //if we have done the sweep and it's the first time we switch to 3
			write_file();	//record input and output to extract impulse response in the future
			gOneSweep=3;	//go out of the case
		}

	if (gIsPlaying && gTriggerButton1 == 1){
		rt_printf(" ON  %d\n", gIsPlaying);
		gFirstTime=0;
		gTriggerButton1 = 0;
		gButtonSwitch = 0;}

	else if (!gIsPlaying && gTriggerButton1 == 1){
		rt_printf(" OFF  %d\n", gIsPlaying);
		gTriggerButton1 = 0;
		gButtonSwitch = 0;}

	if (gIsPlaying && gTriggerButton2 == 1){

	gNumberIteration=0;

	switch(gButtonSwitch)
	{
	case 0:  //

		rt_printf(" 0 is %d\n", gButtonSwitch);
		gTriggerButton2=0;
		break;

	case 1:  //
		rt_printf(" sitting in a room is %d\n", gButtonSwitch);
		gReadPointer=0;
		gReadPtr=0;
		gPointer=0;
		gWritePointer=0;
		gGainLevel=0.5f;
		gNumberIteration=0;
		gNumSamplesTaken=gSampleData.sampleLen;
		gTriggerButton2=0;
	    break;

	case 2:
		rt_printf(" scale is  %d\n", gButtonSwitch);
		gTriggerButton2=0;
		in=0;
		gNumberIteration=-1;
		gFrequency = 220;
	    break;

	case 3:
		rt_printf(" sweep is  %d\n", gButtonSwitch);
		gTriggerButton2=0;
		in=0;
		gSamplesFrequencyCount=0;
		gSweepPointer=0;
		gNumberIteration=-1;
		gOneSweep = 1;
		gFirstTime++;
		//rt_printf("(gOneSweep gFirstTime = %d, %d)\n", gOneSweep,gFirstTime );
		//f1=22.5;//set different values for different speeds
		//f2=110;
		break;

	case 4:
		rt_printf(" tone is  %d\n", gButtonSwitch);
		gTriggerButton2=0;
		break;


	default:
		rt_printf(" never here  %d\n", gButtonSwitch);
		gTriggerButton2=0;
	    //return 0;
	}
	}


	for(int n = 0; n < numAudioFrames; n++) {
				// read the potentiometer
				float analogReading = matrixIn[(n/2)*8];//half of the audioRate
				//rt_printf("raw: %f \n", analogReading);
				float normReading = (analogReading - gMinInput) / (gMaxInput - gMinInput);
					//use readings only on change;
				if (gNumberIteration != 0)
					gNumSamplesTaken = (int)(normReading*(gMaxSamples-gMinSamples)+gMinSamples);

					//execute code on call
				if (gButtonSwitch == 1 && gIsPlaying){
						//copy the track only the first time
					if(gNumberIteration==0){
						if(gReadPtr < gSampleData.sampleLen)
							gStoreBuffer[2*n]=gStoreBuffer[2*n+1]=gSampleData.samples[gReadPtr];

						else gStoreBuffer[2*n]=gStoreBuffer[2*n+1]=0;

						if(++gReadPtr >= gSampleData.sampleLen){
							gReadPtr = 0;
							gNumberIteration++;
						rt_printf("(Track played times = %d)\n", gNumberIteration);
						}

					}

				}

			}
	gPointer=0;

	for(int n = 0; n < numAudioFrames; n++) {

		gSamplesFrequencyCount ++; //update counting (depends on audioSampleRate)
		gSamplesEnergyCount ++;		//update counting (depends on audioSampleRate)

//////////////////////////////// play chromatic scale

		 if (gButtonSwitch == 2 && gIsPlaying){

			 //the following behaviour is meant to give the possibility to have sweeps controlled by the potentiometer
			 //making use of the chromatic scale. This could be also used for measurements.
			 //for this reason after some tests, it hase been chosen to give some creative control
			 //to the potentiometer, to allow both unpredictable results and a controlled scale varying in time.
			 //there is a particular threshold (around 1000 samples) when the sweep is too fast, where it can happen that the code
			 //misses to go back descending, in this case it starts emitting digital glitches Hi-Freq that may be
			 //slowed down and played in some cases - (frequencies may be too high and dangerous).
			 //Resetting the frequency is possible going down to 220 or up to 880,
			 //the normal behaviour can be restored in this way. In some other cases, it's not possible and it's
			 //necessary to reboot the code. This happens only if staying in that threshold looking for this thing to happen.


					if (gNumSamplesTaken<200){ //comment these two lines for measured behaviour on interval
						 gNumSamplesTaken = gSamplesIntervalLength; } //comment these two lines for measured behaviour on interval

					//if (gSamplesFrequencyCount > gSamplesIntervalLength  ){ //uncomment this for measured behaviour
					if (gSamplesFrequencyCount > gNumSamplesTaken  ){ //comment this for measured behaviour

					gFrequency = increment * gFrequency;
					gEnvelopeAmplitude = 0.01;
					gSamplesFrequencyCount = 0;


				}
					//rt_printf("(gNumSamplesTaken = freq %d, %f)\n", gNumSamplesTaken, gFrequency);
					/*
					if (gFrequency > 1000000)//dangerous version - when it get stuck it may cause damage
					gFrequency = 440;
					*/

					if (gFrequency > 10000) //safer version
					gFrequency = 440;

					if (gFrequency < 20) //we don't hear below this
					gFrequency = 220;

					if (gNumSamplesTaken > 200000) //too slow, send a signal to say it's better to go on the other way
					gFrequency = 880;

					if (gNumSamplesTaken < 400) //between 200 and 400 samples the sweep goes around low frequencies
					gFrequency = 220;

				gEnvelopeAmplitude -= envelopeIncrement;


				if(gFrequency >= gScalef2  )
					increment= 1/increment;

				if(gFrequency <= gScalef1  )
					increment= 1/increment;

				gPhase += 2.0 * M_PI * gFrequency * gInverseSampleRate;

				if(gPhase > 2.0 * M_PI)
				gPhase -= 2.0 * M_PI;

				//in = gGainLevel * sin(gPhase); //to be improved...
				in = 0.5f * sin(gPhase);

		}

//////////////////////////////// play Exponential Sine Sweep

		 if (gButtonSwitch == 3 && gIsPlaying){

			 f1=11.25;
			 f2=22.5; //change this for different speeds since time depends on gNumSamplesTaken from pot

			// rt_printf("(gNumSamplesTaken = freq %d, %d)\n", gNumSamplesTaken, (int)(gNumSamplesTaken/gAudioSampleRate));

			double argument;
			float omega1 = 2.0 * M_PI * f1 * gInverseSampleRate;

			//formula
			argument= ((omega1 * gAudioSampleRate *(int)(gNumSamplesTaken/gAudioSampleRate))/log(f2/f1))*(exp((gSamplesFrequencyCount/gAudioSampleRate)*log(f2/f1))-1.0);

			//in = gGainLevel * sin(argument); //in case of frequency dependent equalisation/correction for speaker - to implement
			in = 0.8f * sin(argument);

				if (gSamplesFrequencyCount > gAudioSampleRate *(int)(gNumSamplesTaken/gAudioSampleRate)){
			gSamplesFrequencyCount =0;
			}
		 }

//////////////////////////////// play Test Tone 1Khz

		 if (gButtonSwitch == 4){

			gPhase += 2.0 * M_PI * 1000 * gInverseSampleRate;
			//gPhase += 2.0 * M_PI * f1 * gInverseSampleRate;
			//gPhase += 2.0 * M_PI * gNumSamplesTaken * gInverseSampleRate;

			if(gPhase > 2.0 * M_PI)
			gPhase -= 2.0 * M_PI;

			//in = gGainLevel * sin(gPhase);
			in = 0.5f * sin(gPhase); //value of gain needs to be tested

		 }


		for(int channel = 0; channel < gNumAudioChannels; channel++){

			if (gIsPlaying==1){ // if g Is Playing (the main switch is on)

				float x = audioIn[n*gNumAudioChannels+channel]; //get the audio stereo from the microphones

				switch(gButtonSwitch) //do something according to cases
					{
					case 0:  //listen only

						in=0;
						audioOut[n * gNumAudioChannels + channel] = in;
						gInputBuffer[gInputBufferPointer] = ((audioIn[n*gNumAudioChannels] + audioIn[n*gNumAudioChannels+1]) * 0.5); //mix to mono
						//gInputBuffer[gInputBufferPointer] = x; //stereo
						break;

					case 1:  //iterative dynamic playback - inspired on Sitting in a Room by Alvin Lucier
						if (gNumberIteration==0){ //first playback - original audio out - input recording
							audioOut[n * gNumAudioChannels + channel] = gStoreBuffer[gPointer];
							gWriteBuffer[gWritePointer] = x; //interleaved (size of wrapping is gNumSamplesTaken)
						}

						if (gNumberIteration>0 && gNumberIteration%2 == 1 ){ //alternate buffers
							audioOut[n * gNumAudioChannels + channel] = gGainLevel * gInputBuffer[gReadPointer];
							gWriteBuffer[gWritePointer] = x; //interleaved (size of wrapping is gNumSamplesTaken)
						}

						if (gNumberIteration>0 && gNumberIteration%2 == 0 ){ //alternate buffers
							audioOut[n * gNumAudioChannels + channel] = gGainLevel * gWriteBuffer[gReadPointer];
							gInputBuffer[gWritePointer] = x;
						}
					    break;

					case 2:  //chromatic scale
						audioOut[n * gNumAudioChannels + channel] = in; //mono input to stereo
						gInputBuffer[gInputBufferPointer] = audioIn[n*gNumAudioChannels+channel];
					    break;

					case 3:	//ESS
						audioOut[n * gNumAudioChannels + channel] = in; //mono input to stereo
						gInputBuffer[gInputBufferPointer] = audioIn[n*gNumAudioChannels+channel];
						gSweepBufferOut[gSweepPointer]  = in;//copy the input to the sweep buffer - note: it should be mixed to mono when out
						gSweepBufferIn[gSweepPointer]  = audioIn[n*gNumAudioChannels+channel]; //record the audio in the SweepBuffer
						break;

					case 4: //test tone
						audioOut[n * gNumAudioChannels + channel] = in ; //mono input to stereo (double of gain...)
						gInputBuffer[gInputBufferPointer] = audioIn[n*gNumAudioChannels+channel];
						break;

					default:
						rt_printf(" never here  %d\n", gButtonSwitch);
						/* no break */
					    //return 0;
					}

		gSweepPointer++;
		if (gSweepPointer>=BUFFER_SIZE_WRITE ){
			gSweepPointer=0;
			if(gOneSweep ==1 && gButtonSwitch == 3) //write only in this case
				gOneSweep = 2;
		}

		// Clear the output sample in the buffer so it is ready for the next overlap-add
		gOutputBuffer[gOutputBufferReadPointer] = 0;
		gOutputBufferReadPointer++;

		if(gOutputBufferReadPointer >= BUFFER_SIZE) 	//wrap (this for FFT)
			gOutputBufferReadPointer = 0;
		gOutputBufferWritePointer++;
		if(gOutputBufferWritePointer >= BUFFER_SIZE)	//wrap (this for FFT)
			gOutputBufferWritePointer = 0;

		//system to copy the buffers for the FFT analysis

		if (gNumberIteration>0 && gNumberIteration%2 == 0 )
			gInputBufferCopy[gInputBufferPointerCopy]=gInputBuffer[gWritePointer];

		else if  (gNumberIteration>0 && gNumberIteration%2 == 1 )
			gInputBufferCopy[gInputBufferPointerCopy]=gInputBuffer[gWritePointer];

		else if  (gNumberIteration== 0 )
			gInputBufferCopy[gInputBufferPointerCopy]=gWriteBuffer[gReadPointer];

		gInputBufferPointer++;

		//from the potentiometer
		//wrap the input buffer according to the length of the block of samples
			if(gInputBufferPointer >= gNumSamplesTaken){
				gInputBufferPointer = 0;
			}

		gInputBufferPointerCopy++;

		if(gInputBufferPointerCopy >= BUFFER_SIZE) //wrap the pointer for the FFT
			gInputBufferPointerCopy = 0;

		gSampleCount++;

		if(gSampleCount >= gHopSize) { //if we go over the HopSize
			if (gButtonSwitch != 3 ){ //do not do FFT when doing EES (case 3)

			process_fft(gInputBufferCopy, gInputBufferPointerCopy, gOutputBuffer, gOutputBufferReadPointer);
			gFFTInputBufferPointer = gInputBufferPointerCopy;
			gFFTOutputBufferPointer = gOutputBufferWritePointer;
			scheduleAuxiliaryTask(gFFTTask);
			}
			else if (gButtonSwitch ==3 ){ //keep wrapping the Pointers
				gFFTInputBufferPointer = gInputBufferPointerCopy;
				gFFTOutputBufferPointer = gOutputBufferWritePointer;
			}
			gSampleCount = 0;
		}

		gPointer++; //this needed to copy properly the original audio track

		}
		else  //if g is not Playing stop the audio
		audioOut[n * gNumAudioChannels + channel] = 0;

	}//close channel

	if(gReadPointer >= gNumSamplesTaken && gNumberIteration > 0){
		gReadPointer = 0;
	gNumberIteration++;
		rt_printf("(IterationNumber is = %d)\n", gNumberIteration);}
	gReadPointer++;

	if(gWritePointer >= (gSampleData.sampleLen*2) && gNumberIteration == 0)
		gWritePointer = 0;
	if(gWritePointer >= gNumSamplesTaken && gNumberIteration > 0)
		gWritePointer = 0;
	gWritePointer++;


	}//close Audio frame

}//close render

// cleanup_render() is called once at the end, after the audio has stopped.
// Release any resources that were allocated in initialise_render().

void cleanup_render()
{
	NE10_FREE(timeDomainIn);
	NE10_FREE(timeDomainOut);
	NE10_FREE(frequencyDomain);
	NE10_FREE(cfg);
	free(gWindowBuffer);
}

