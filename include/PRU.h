/*
 * PRU.h
 *
 *  Created on: May 27, 2014
 *      Author: andrewm
 */

#ifndef PRU_H_
#define PRU_H_

#include <stdint.h>

class PRU
{
private:
	static const unsigned int kPruGPIODACSyncPin;
	static const unsigned int kPruGPIOADCSyncPin;
	static const unsigned int kPruGPIOTestPin;
	static const unsigned int kPruGPIOTestPin2;
	static const unsigned int kPruGPIOTestPin3;

public:
	// Constructor
	PRU();

	// Destructor
	~PRU();

	// Prepare the GPIO pins needed for the PRU
	int prepareGPIO(int use_spi, int include_test_pin, int include_led);

	// Clean up the GPIO at the end
	void cleanupGPIO();

	// Initialise and open the PRU
	int initialise(int pru_num, int frames_per_buffer, int spi_channels,
				   bool xenomai_test_pin = false);

	// Run the code image in pru_rtaudio_bin.h
	int start();

	// Loop: read and write data from the PRU
	void loop();

	// Wait for an interrupt from the PRU indicate it is finished
	void waitForFinish();

	// Turn off the PRU when done
	void disable();

	// For debugging:
	void setGPIOTestPin();
	void clearGPIOTestPin();

private:
	int pru_number;		// Which PRU we use
	bool running;		// Whether the PRU is running
	bool spi_enabled;	// Whether SPI ADC and DAC are used
	bool gpio_enabled;	// Whether GPIO has been prepared
	bool led_enabled;	// Whether a user LED is enabled
	bool gpio_test_pin_enabled; // Whether the test pin was also enabled
	int spi_num_channels; // How many channels to use for SPI ADC/DAC

	volatile uint32_t *pru_buffer_comm;
	uint16_t *pru_buffer_spi_dac;
	uint16_t *pru_buffer_spi_adc;
	int16_t *pru_buffer_audio_dac;
	int16_t *pru_buffer_audio_adc;
	unsigned int spi_buffer_frames;
	unsigned int audio_buffer_frames;

	int xenomai_gpio_fd;	// File descriptor for /dev/mem for fast GPIO
	uint32_t *xenomai_gpio;	// Pointer to GPIO registers
};


#endif /* PRU_H_ */
