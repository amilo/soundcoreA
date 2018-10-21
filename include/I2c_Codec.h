/*
 * I2c_Codec.h
 *
 * Handle writing the registers to the TLV320AIC310x
 * series audio codecs, used on the BeagleBone Audio Cape.
 * This code is designed to bypass the ALSA driver and
 * configure the codec directly in a sensible way. It
 * is complemented by code running on the PRU which uses
 * the McASP serial port to transfer audio data.
 *
 *  Created on: May 25, 2014
 *      Author: Andrew McPherson
 */


#ifndef I2CCODEC_H_
#define I2CCODEC_H_

#include "I2c.h"


class I2c_Codec : public I2c
{
public:
	int writeRegister(unsigned int reg, unsigned int value);

	int initCodec();
	int startAudio(int dual_rate);
	int stopAudio();

	int setDACVolume(int halfDbSteps);
	int writeDACVolumeRegisters(bool mute);
	int setADCVolume(int halfDbSteps);
	int writeADCVolumeRegisters(bool mute);
	int setHPVolume(int halfDbSteps);
	int writeHPVolumeRegisters();

	int readI2C();

	I2c_Codec();
	~I2c_Codec();

private:
	bool running;
	int dacVolumeHalfDbs;
	int adcVolumeHalfDbs;
	int hpVolumeHalfDbs;
};


#endif /* I2CCODEC_H_ */
