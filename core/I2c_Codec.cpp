/*
 * I2c_Codec.cpp
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

#include "../include/I2c_Codec.h"

I2c_Codec::I2c_Codec()
: running(false), dacVolumeHalfDbs(0), adcVolumeHalfDbs(0), hpVolumeHalfDbs(0)
{}

// This method initialises the audio codec to its default state
int I2c_Codec::initCodec()
{
	// Write the reset register of the codec
	if(writeRegister(0x01, 0x80)) // Software reset register
	{
		cout << "Failed to reset codec\n";
		return 1;
	}

	// Wait for codec to process the reset (for safety)
	usleep(5000);

	return 0;
}

// Tell the codec to start generating audio
// See the TLV320AIC3106 datasheet for full details of the registers
// The dual_rate flag, when true, runs the codec at 88.2kHz; otherwise
// it runs at 44.1kHz
int I2c_Codec::startAudio(int dual_rate)
{
	if(writeRegister(0x02, 0x00))	// Codec sample rate register: fs_ref / 1
		return 1;
	if(writeRegister(0x03, 0x91))	// PLL register A: enable
		return 1;
	if(writeRegister(0x04, 0x1C))	// PLL register B
		return 1;
	if(writeRegister(0x05, 0x52))	// PLL register C
		return 1;
	if(writeRegister(0x06, 0x40))	// PLL register D
		return 1;
	if(dual_rate) {
		if(writeRegister(0x07, 0xEA))	// Codec datapath register: 44.1kHz; dual rate; standard datapath
			return 1;
	}
	else {
		if(writeRegister(0x07, 0x8A))	// Codec datapath register: 44.1kHz; std rate; standard datapath
			return 1;
	}
	if(writeRegister(0x08, 0xC0))	// Audio serial control register A: BLCK, WCLK outputs
		return 1;
	if(writeRegister(0x09, 0x40))	// Audio serial control register B: DSP mode, word len 16 bits
		return 1;
	if(writeRegister(0x0A, 0x00))	// Audio serial control register C: 0 bit offset
		return 1;
	if(writeRegister(0x0B, 0x01))	// Audio codec overflow flag register: PLL R = 1
		return 1;
	if(writeRegister(0x0C, 0x00))	// Digital filter register: disabled
		return 1;
	if(writeRegister(0x0D, 0x00))	// Headset / button press register A: disabled
		return 1;
	if(writeRegister(0x0E, 0x00))	// Headset / button press register B: disabled
		return 1;
	if(writeRegister(0x0F, 0x20))	// Left ADC PGA gain control: not muted; 0x20 = 16dB
		return 1;
	if(writeRegister(0x10, 0x20))	// Right ADC PGA gain control: not muted; 0x20 = 16dB
		return 1;

	if(writeRegister(0x25, 0xC0))	// DAC power/driver register: DAC power on (left and right)
		return 1;
	if(writeRegister(0x26, 0x04))	// High power output driver register: Enable short circuit protection
		return 1;
	if(writeRegister(0x28, 0x02))	// High power output stage register: disable soft stepping
		return 1;

	if(writeRegister(0x52, 0x80))	// DAC_L1 to LEFT_LOP volume control: routed, volume 0dB
		return 1;
	if(writeRegister(0x5C, 0x80))	// DAC_R1 to RIGHT_LOP volume control: routed, volume 0dB
		return 1;

	if(writeHPVolumeRegisters())	// Send DAC to high-power outputs
		return 1;

	if(writeRegister(0x66, 0x02))			// Clock generation control register: use MCLK, PLL N = 2
		return 1;

	if(writeRegister(0x33, 0x0D))	// HPLOUT output level control: output level = 0dB, not muted, powered up
		return 1;
	if(writeRegister(0x41, 0x0D))	// HPROUT output level control: output level = 0dB, not muted, powered up
		return 1;
	if(writeRegister(0x56, 0x09))	// LEFT_LOP output level control: 0dB, not muted, powered up
		return 1;
	if(writeRegister(0x5D, 0x09))	// RIGHT_LOP output level control: 0dB, not muted, powered up
		return 1;

	if(writeDACVolumeRegisters(false))	// Unmute and set volume
		return 1;

	if(writeRegister(0x65, 0x00))	// GPIO control register B: disabled; codec uses PLLDIV_OUT
		return 1;

	if(writeADCVolumeRegisters(false))	// Unmute and set ADC volume
		return 1;

	running = true;
	return 0;
}

// Set the volume of the DAC output
int I2c_Codec::setDACVolume(int halfDbSteps)
{
	dacVolumeHalfDbs = halfDbSteps;
	if(running)
		return writeDACVolumeRegisters(false);

	return 0;
}

// Set the volume of the DAC output
int I2c_Codec::setADCVolume(int halfDbSteps)
{
	adcVolumeHalfDbs = halfDbSteps;
	if(running)
		return writeADCVolumeRegisters(false);

	return 0;
}

// Update the DAC volume control registers
int I2c_Codec::writeDACVolumeRegisters(bool mute)
{
	int volumeBits = 0;

	if(dacVolumeHalfDbs < 0) { // Volume is specified in half-dBs with 0 as full scale
		volumeBits = -dacVolumeHalfDbs;
		if(volumeBits > 127)
			volumeBits = 127;
	}

	if(mute) {
		if(writeRegister(0x2B, volumeBits | 0x80))	// Left DAC volume control: muted
			return 1;
		if(writeRegister(0x2C, volumeBits | 0x80))	// Right DAC volume control: muted
			return 1;
	}
	else {
		if(writeRegister(0x2B, volumeBits))	// Left DAC volume control: not muted
			return 1;
		if(writeRegister(0x2C, volumeBits))	// Right DAC volume control: not muted
			return 1;
	}

	return 0;
}

// Update the ADC volume control registers
int I2c_Codec::writeADCVolumeRegisters(bool mute)
{
	int volumeBits = 0;

	// Volume is specified in half-dBs with 0 as full scale
	// The codec uses 1.5dB steps so we divide this number by 3
	if(adcVolumeHalfDbs < 0) {
		volumeBits = -adcVolumeHalfDbs / 3;
		if(volumeBits > 8)
			volumeBits = 8;
	}

	if(mute) {
		if(writeRegister(0x13, 0x00))		// Line1L to Left ADC control register: power down
			return 1;
		if(writeRegister(0x16, 0x00))		// Line1R to Right ADC control register: power down
			return 1;
	}
	else {
		if(writeRegister(0x13, 0x7C))	// Line1L disabled; left ADC powered up with soft step
			return 1;
		if(writeRegister(0x16, 0x7C))	// Line1R disabled; right ADC powered up with soft step
			return 1;
		if(writeRegister(0x11, (volumeBits << 4) | 0x0F))	// Line2L connected to left ADC
			return 1;
		if(writeRegister(0x12, volumeBits | 0xF0))		    // Line2R connected to right ADC
			return 1;
	}

	return 0;
}

// Set the volume of the headphone output
int I2c_Codec::setHPVolume(int halfDbSteps)
{
	hpVolumeHalfDbs = halfDbSteps;
	if(running)
		return writeHPVolumeRegisters();

	return 0;
}


// Update the headphone volume control registers
int I2c_Codec::writeHPVolumeRegisters()
{
	int volumeBits = 0;

	if(hpVolumeHalfDbs < 0) { // Volume is specified in half-dBs with 0 as full scale
		volumeBits = -hpVolumeHalfDbs;
		if(volumeBits > 127)
			volumeBits = 127;
	}

	if(writeRegister(0x2F, volumeBits | 0x80)) // DAC_L1 to HPLOUT register: route to HPLOUT, volume 0dB
		return 1;
	if(writeRegister(0x40, volumeBits | 0x80)) // DAC_R1 to HPROUT register: route to HPROUT, volume 0dB
		return 1;

	return 0;
}

// This tells the codec to stop generating audio and mute the outputs
int I2c_Codec::stopAudio()
{
	if(writeDACVolumeRegisters(true))	// Mute the DACs
		return 1;
	if(writeADCVolumeRegisters(true))	// Mute the ADCs
		return 1;

	usleep(10000);

	if(writeRegister(0x33, 0x0C))		// HPLOUT output level register: muted
		return 1;
	if(writeRegister(0x41, 0x0C))		// HPROUT output level register: muted
		return 1;
	if(writeRegister(0x56, 0x08))		// LEFT_LOP output level control: muted
		return 1;
	if(writeRegister(0x5D, 0x08))		// RIGHT_LOP output level control: muted
		return 1;
	if(writeRegister(0x25, 0x00))		// DAC power/driver register: power off
		return 1;
	if(writeRegister(0x03, 0x11))		// PLL register A: disable
		return 1;
	if(writeRegister(0x01, 0x80))		// Reset codec to defaults
		return 1;

	running = false;
	return 0;
}

// Write a specific register on the codec
int I2c_Codec::writeRegister(unsigned int reg, unsigned int value)
{
	char buf[2] = { reg & 0xFF, value & 0xFF };

	if(write(i2C_file, buf, 2) != 2)
	{
		cout << "Failed to write register " << reg << " on codec\n";
		return 1;
	}

	return 0;
}


int I2c_Codec::readI2C()
{
	// Nothing to do here, we only write the registers
	return 0;
}


I2c_Codec::~I2c_Codec()
{
	if(running)
		stopAudio();
}

