/*
 * PRU.cpp
 *
 * Code for communicating with the Programmable Realtime Unit (PRU)
 * on the BeagleBone AM335x series processors. The PRU loads and runs
 * a separate code image compiled from an assembly file. Here it is
 * used to handle audio and SPI ADC/DAC data.
 *
 * This code is specific to the PRU code in the assembly file; for example,
 * it uses certain GPIO resources that correspond to that image.
 *
 *  Created on: May 27, 2014
 *      Author: andrewm
 */

#include "../include/PRU.h"
#include "../include/prussdrv.h"
#include "../include/pruss_intc_mapping.h"
#include "../include/GPIOcontrol.h"
#include "../include/render.h"
#include "../include/pru_rtaudio_bin.h"

#include <iostream>
#include <stdlib.h>
#include <cstdio>
#include <cerrno>
#include <fcntl.h>
#include <sys/mman.h>

// Xenomai-specific includes
#include <sys/mman.h>
#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>

using namespace std;

#define PRU_MEM_MCASP_OFFSET 0x2000  // Offset within PRU-SHARED RAM
#define PRU_MEM_MCASP_LENGTH 0x2000  // Length of McASP memory, in bytes
#define PRU_MEM_DAC_OFFSET 0x0     // Offset within PRU0 RAM
#define PRU_MEM_DAC_LENGTH 0x2000  // Length of ADC+DAC memory, in bytes
#define PRU_MEM_COMM_OFFSET 0x0    // Offset within PRU-SHARED RAM

#define PRU_SHOULD_STOP 	0
#define PRU_CURRENT_BUFFER  1
#define PRU_BUFFER_FRAMES   2
#define PRU_SHOULD_SYNC     3
#define PRU_SYNC_ADDRESS    4
#define PRU_SYNC_PIN_MASK   5
#define PRU_LED_ADDRESS		6
#define PRU_LED_PIN_MASK	7
#define PRU_FRAME_COUNT		8
#define PRU_USE_SPI			9
#define PRU_SPI_NUM_CHANNELS 10

#define PRU_SAMPLE_INTERVAL_NS 11338	// 88200Hz per SPI sample = 11.338us

#define GPIO0_ADDRESS 		0x44E07000
#define GPIO1_ADDRESS 		0x4804C000
#define GPIO_SIZE			0x198
#define GPIO_CLEARDATAOUT 	(0x190 / 4)
#define GPIO_SETDATAOUT 	(0x194 / 4)

#define TEST_PIN_GPIO_BASE	GPIO0_ADDRESS	// Use GPIO0(31) for debugging
#define TEST_PIN_MASK		(1 << 31)
#define TEST_PIN2_MASK		(1 << 26)

#define USERLED3_GPIO_BASE  GPIO1_ADDRESS // GPIO1(24) is user LED 3
#define USERLED3_PIN_MASK   (1 << 24)

const unsigned int PRU::kPruGPIODACSyncPin = 5;	// GPIO0(5); P9-17
const unsigned int PRU::kPruGPIOADCSyncPin = 48; // GPIO1(16); P9-15

const unsigned int PRU::kPruGPIOTestPin = 60;	// GPIO1(28); P9-12
const unsigned int PRU::kPruGPIOTestPin2 = 31;	// GPIO0(31); P9-13
const unsigned int PRU::kPruGPIOTestPin3 = 26;	// GPIO0(26); P8-14

extern int gShouldStop;
extern int gRTAudioVerbose;

// Constructor: specify a PRU number (0 or 1)
PRU::PRU()
: pru_number(0), running(false), spi_enabled(false), gpio_enabled(false), led_enabled(false),
  gpio_test_pin_enabled(false), spi_num_channels(0), xenomai_gpio_fd(-1), xenomai_gpio(0)
{

}

// Destructor
PRU::~PRU()
{
	if(running)
		disable();
	if(gpio_enabled)
		cleanupGPIO();
	if(xenomai_gpio_fd >= 0)
		close(xenomai_gpio_fd);
}

// Prepare the GPIO pins needed for the PRU
// If include_test_pin is set, the GPIO output
// is also prepared for an output which can be
// viewed on a scope. If include_led is set,
// user LED 3 on the BBB is taken over by the PRU
// to indicate activity
int PRU::prepareGPIO(int use_spi, int include_test_pin, int include_led)
{
	if(use_spi) {
		// Prepare DAC CS/ pin: output, high to begin
		if(gpio_export(kPruGPIODACSyncPin)) {
			if(gRTAudioVerbose)
				cout << "Warning: couldn't export DAC sync pin\n";
		}
		if(gpio_set_dir(kPruGPIODACSyncPin, OUTPUT_PIN)) {
			if(gRTAudioVerbose)
				cout << "Couldn't set direction on DAC sync pin\n";
			return -1;
		}
		if(gpio_set_value(kPruGPIODACSyncPin, HIGH)) {
			if(gRTAudioVerbose)
				cout << "Couldn't set value on DAC sync pin\n";
			return -1;
		}

		// Prepare ADC CS/ pin: output, high to begin
		if(gpio_export(kPruGPIOADCSyncPin)) {
			if(gRTAudioVerbose)
				cout << "Warning: couldn't export ADC sync pin\n";
		}
		if(gpio_set_dir(kPruGPIOADCSyncPin, OUTPUT_PIN)) {
			if(gRTAudioVerbose)
				cout << "Couldn't set direction on ADC sync pin\n";
			return -1;
		}
		if(gpio_set_value(kPruGPIOADCSyncPin, HIGH)) {
			if(gRTAudioVerbose)
				cout << "Couldn't set value on ADC sync pin\n";
			return -1;
		}

		spi_enabled = true;
	}

	if(include_test_pin) {
		// Prepare GPIO test output (for debugging), low to begin
		if(gpio_export(kPruGPIOTestPin)) {
			if(gRTAudioVerbose)
				cout << "Warning: couldn't export GPIO test pin\n";
		}
		if(gpio_set_dir(kPruGPIOTestPin, OUTPUT_PIN)) {
			if(gRTAudioVerbose)
				cout << "Couldn't set direction on GPIO test pin\n";
			return -1;
		}
		if(gpio_set_value(kPruGPIOTestPin, LOW)) {
			if(gRTAudioVerbose)
				cout << "Couldn't set value on GPIO test pin\n";
			return -1;
		}

		if(gpio_export(kPruGPIOTestPin2)) {
			if(gRTAudioVerbose)
				cout << "Warning: couldn't export GPIO test pin 2\n";
		}
		if(gpio_set_dir(kPruGPIOTestPin2, OUTPUT_PIN)) {
			if(gRTAudioVerbose)
				cout << "Couldn't set direction on GPIO test pin 2\n";
			return -1;
		}
		if(gpio_set_value(kPruGPIOTestPin2, LOW)) {
			if(gRTAudioVerbose)
				cout << "Couldn't set value on GPIO test pin 2\n";
			return -1;
		}

		if(gpio_export(kPruGPIOTestPin3)) {
			if(gRTAudioVerbose)
				cout << "Warning: couldn't export GPIO test pin 3\n";
		}
		if(gpio_set_dir(kPruGPIOTestPin3, OUTPUT_PIN)) {
			if(gRTAudioVerbose)
				cout << "Couldn't set direction on GPIO test pin 3\n";
			return -1;
		}
		if(gpio_set_value(kPruGPIOTestPin3, LOW)) {
			if(gRTAudioVerbose)
				cout << "Couldn't set value on GPIO test pin 3\n";
			return -1;
		}
		gpio_test_pin_enabled = true;
	}

	if(include_led) {
		// Turn off system function for LED3 so it can be reused by PRU
		led_set_trigger(3, "none");
		led_enabled = true;
	}

	gpio_enabled = true;

	return 0;
}

// Clean up the GPIO at the end
void PRU::cleanupGPIO()
{
	if(!gpio_enabled)
		return;
	if(spi_enabled) {
		gpio_unexport(kPruGPIODACSyncPin);
		gpio_unexport(kPruGPIOADCSyncPin);
	}
	if(gpio_test_pin_enabled) {
		gpio_unexport(kPruGPIOTestPin);
		gpio_unexport(kPruGPIOTestPin2);
		gpio_unexport(kPruGPIOTestPin3);
	}
	if(led_enabled) {
		// Set LED back to default eMMC status
		// TODO: make it go back to its actual value before this program,
		// rather than the system default
		led_set_trigger(3, "mmc1");
	}

	gpio_enabled = gpio_test_pin_enabled = false;
}

// Initialise and open the PRU
int PRU::initialise(int pru_num, int frames_per_buffer, int spi_channels, bool xenomai_test_pin)
{
	uint32_t *pruMem = 0;

	if(!gpio_enabled) {
		rt_printf("initialise() called before GPIO enabled\n");
		return 1;
	}

	pru_number = pru_num;

	/* Set number of SPI ADC / DAC channels to use. This implicitly
	 * also determines the sample rate relative to the audio clock
	 * (half audio clock for 8 channels, full audio clock for 4,
	 * double audio clock for 2)
	 */
	spi_num_channels = spi_channels;

    /* Initialize structure used by prussdrv_pruintc_intc   */
    /* PRUSS_INTC_INITDATA is found in pruss_intc_mapping.h */
    tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

    /* Allocate and initialize memory */
    prussdrv_init();
    if(prussdrv_open(pru_number == 0 ? PRU_EVTOUT_0 : PRU_EVTOUT_1)) {
    	rt_printf("Failed to open PRU driver\n");
    	return 1;
    }

    /* Map PRU's INTC */
    prussdrv_pruintc_init(&pruss_intc_initdata);

    spi_buffer_frames = frames_per_buffer;
    audio_buffer_frames = spi_buffer_frames * spi_num_channels / 4;

    /* Map PRU memory to pointers */
	prussdrv_map_prumem (PRUSS0_SHARED_DATARAM, (void **)&pruMem);
    pru_buffer_comm = (uint32_t *)&pruMem[PRU_MEM_COMM_OFFSET/sizeof(uint32_t)];
	pru_buffer_audio_dac = (int16_t *)&pruMem[PRU_MEM_MCASP_OFFSET/sizeof(uint32_t)];

	/* ADC memory starts 2(ch)*2(buffers)*bufsize samples later */
	pru_buffer_audio_adc = &pru_buffer_audio_dac[4 * audio_buffer_frames];

	if(spi_enabled) {
		prussdrv_map_prumem (pru_number == 0 ? PRUSS0_PRU0_DATARAM : PRUSS0_PRU1_DATARAM, (void **)&pruMem);
		pru_buffer_spi_dac = (uint16_t *)&pruMem[PRU_MEM_DAC_OFFSET/sizeof(uint32_t)];

		/* ADC memory starts after N(ch)*2(buffers)*bufsize samples */
		pru_buffer_spi_adc = &pru_buffer_spi_dac[2 * spi_num_channels * spi_buffer_frames];
	}
	else {
		pru_buffer_spi_dac = pru_buffer_spi_adc = 0;
	}

    /* Set up flags */
    pru_buffer_comm[PRU_SHOULD_STOP] = 0;
    pru_buffer_comm[PRU_CURRENT_BUFFER] = 0;
    pru_buffer_comm[PRU_BUFFER_FRAMES] = spi_buffer_frames;
    pru_buffer_comm[PRU_SHOULD_SYNC] = 0;
    pru_buffer_comm[PRU_SYNC_ADDRESS] = 0;
    pru_buffer_comm[PRU_SYNC_PIN_MASK] = 0;
    if(led_enabled) {
    	pru_buffer_comm[PRU_LED_ADDRESS] = USERLED3_GPIO_BASE;
    	pru_buffer_comm[PRU_LED_PIN_MASK] = USERLED3_PIN_MASK;
    }
    else {
    	pru_buffer_comm[PRU_LED_ADDRESS] = 0;
    	pru_buffer_comm[PRU_LED_PIN_MASK] = 0;
    }
    if(spi_enabled) {
    	pru_buffer_comm[PRU_USE_SPI] = 1;
    	pru_buffer_comm[PRU_SPI_NUM_CHANNELS] = spi_num_channels;
    }
    else {
    	pru_buffer_comm[PRU_USE_SPI] = 0;
    	pru_buffer_comm[PRU_SPI_NUM_CHANNELS] = 0;
    }

    /* Clear ADC and DAC memory */
    if(spi_enabled) {
		for(int i = 0; i < PRU_MEM_DAC_LENGTH / 2; i++)
			pru_buffer_spi_dac[i] = 0;
    }
	for(int i = 0; i < PRU_MEM_MCASP_LENGTH / 2; i++)
		pru_buffer_audio_dac[i] = 0;

	/* If using GPIO test pin for Xenomai (for debugging), initialise the pointer now */
	if(xenomai_test_pin && xenomai_gpio_fd < 0) {
		xenomai_gpio_fd = open("/dev/mem", O_RDWR);
		if(xenomai_gpio_fd < 0)
			rt_printf("Unable to open /dev/mem for GPIO test pin\n");
		else {
			xenomai_gpio = (uint32_t *)mmap(0, GPIO_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, xenomai_gpio_fd, TEST_PIN_GPIO_BASE);
			if(xenomai_gpio == MAP_FAILED) {
				rt_printf("Unable to map GPIO address for test pin\n");
				xenomai_gpio = 0;
				close(xenomai_gpio_fd);
				xenomai_gpio_fd = -1;
			}
		}
	}

	return 0;
}

// Run the code image in the specified file
int PRU::start()
{
	/* Clear any old interrupt */
	if(pru_number == 0)
		prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);
	else
		prussdrv_pru_clear_event(PRU_EVTOUT_1, PRU1_ARM_INTERRUPT);

    /* Load and execute binary on PRU */
	if(prussdrv_exec_code(pru_number, PRUcode, sizeof(PRUcode))) {
    	rt_printf("Failed to execute PRU code\n");
    	return 1;
    }

    running = true;
    return 0;
}

// Main loop to read and write data from/to PRU
void PRU::loop()
{
	// Polling interval is 1/4 of the period
	RTIME sleepTime = PRU_SAMPLE_INTERVAL_NS * (spi_num_channels / 2) * spi_buffer_frames / 4;
	float *audioInBuffer, *audioOutBuffer;
	float *matrixInBuffer, *matrixOutBuffer;

	audioInBuffer = (float *)malloc(2 * audio_buffer_frames * sizeof(float));
	audioOutBuffer = (float *)malloc(2 * audio_buffer_frames * sizeof(float));
	matrixInBuffer = (float *)malloc(spi_num_channels * spi_buffer_frames * sizeof(float));
	matrixOutBuffer = (float *)malloc(spi_num_channels * spi_buffer_frames * sizeof(float));

	if(audioInBuffer == 0 || audioOutBuffer == 0) {
		rt_printf("Error: couldn't allocate audio buffers\n");
		return;
	}
	if(matrixInBuffer == 0 || matrixOutBuffer == 0) {
		rt_printf("Error: couldn't allocate matrix buffers\n");
		return;
	}

	while(!gShouldStop) {
		// Wait for PRU to move to buffer 1
		while(pru_buffer_comm[PRU_CURRENT_BUFFER] == 0 && !gShouldStop) {
			rt_task_sleep(sleepTime);
		}
		if(gShouldStop)
			break;

		if(xenomai_gpio != 0) {
			// Set the test pin high
			xenomai_gpio[GPIO_SETDATAOUT] = TEST_PIN_MASK;
		}

		// Render from/to buffer 0

		// Convert short (16-bit) samples to float
		for(unsigned int n = 0; n < 2 * audio_buffer_frames; n++)
			audioInBuffer[n] = (float)pru_buffer_audio_adc[n] / 32768.0;

		if(spi_enabled) {
			for(unsigned int n = 0; n < spi_num_channels * spi_buffer_frames; n++)
				matrixInBuffer[n] = (float)pru_buffer_spi_adc[n] / 65536.0;

			render(spi_buffer_frames, audio_buffer_frames, audioInBuffer, audioOutBuffer,
					matrixInBuffer, matrixOutBuffer);

			for(unsigned int n = 0; n < spi_num_channels * spi_buffer_frames; n++) {
				int out = matrixOutBuffer[n] * 65536.0;
				if(out < 0) out = 0;
				else if(out > 65535) out = 65535;
				pru_buffer_spi_dac[n] = (uint16_t)out;
			}
		}
		else
			render(0, audio_buffer_frames, audioInBuffer, audioOutBuffer, 0, 0);

		// Convert float back to short
		for(unsigned int n = 0; n < 2 * audio_buffer_frames; n++) {
			int out = audioOutBuffer[n] * 32768.0;
			if(out < -32768) out = -32768;
			else if(out > 32767) out = 32767;
			pru_buffer_audio_dac[n] = (int16_t)out;
		}

		if(xenomai_gpio != 0) {
			// Set the test pin high
			xenomai_gpio[GPIO_CLEARDATAOUT] = TEST_PIN_MASK;
		}

		// Wait for PRU to move to buffer 0
		while(pru_buffer_comm[PRU_CURRENT_BUFFER] != 0 && !gShouldStop) {
			rt_task_sleep(sleepTime);
		}

		if(gShouldStop)
			break;

		if(xenomai_gpio != 0) {
			// Set the test pin high
			xenomai_gpio[GPIO_SETDATAOUT] = TEST_PIN_MASK;
		}

		// Render from/to buffer 1

		// Convert short (16-bit) samples to float
		for(unsigned int n = 0; n < 2 * audio_buffer_frames; n++)
			audioInBuffer[n] = (float)pru_buffer_audio_adc[n + audio_buffer_frames * 2] / 32768.0;

		if(spi_enabled) {
			for(unsigned int n = 0; n < spi_num_channels * spi_buffer_frames; n++)
				matrixInBuffer[n] = (float)pru_buffer_spi_adc[n + spi_buffer_frames * spi_num_channels] / 65536.0;

			render(spi_buffer_frames, audio_buffer_frames, audioInBuffer, audioOutBuffer,
					matrixInBuffer, matrixOutBuffer);

			for(unsigned int n = 0; n < spi_num_channels * spi_buffer_frames; n++) {
				int out = matrixOutBuffer[n] * 65536.0;
				if(out < 0) out = 0;
				else if(out > 65535) out = 65535;
				pru_buffer_spi_dac[n + spi_buffer_frames * spi_num_channels] = (uint16_t)out;
			}
		}
		else
			render(0, audio_buffer_frames, audioInBuffer, audioOutBuffer, 0, 0);

		// Convert float back to short
		for(unsigned int n = 0; n < 2 * audio_buffer_frames; n++) {
			int out = audioOutBuffer[n] * 32768.0;
			if(out < -32768) out = -32768;
			else if(out > 32767) out = 32767;
			pru_buffer_audio_dac[n + audio_buffer_frames * 2] = (int16_t)out;
		}

		if(xenomai_gpio != 0) {
			// Set the test pin high
			xenomai_gpio[GPIO_CLEARDATAOUT] = TEST_PIN_MASK;
		}
	}

	// Tell PRU to stop
	pru_buffer_comm[PRU_SHOULD_STOP] = 1;

	free(audioInBuffer);
	free(audioOutBuffer);
	free(matrixInBuffer);
	free(matrixOutBuffer);
}

// Wait for an interrupt from the PRU indicate it is finished
void PRU::waitForFinish()
{
	if(!running)
		return;
    prussdrv_pru_wait_event (pru_number == 0 ? PRU_EVTOUT_0 : PRU_EVTOUT_1);
	if(pru_number == 0)
		prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);
	else
		prussdrv_pru_clear_event(PRU_EVTOUT_1, PRU1_ARM_INTERRUPT);
}

// Turn off the PRU when done
void PRU::disable()
{
    /* Disable PRU and close memory mapping*/
    prussdrv_pru_disable(pru_number);
    prussdrv_exit();
	running = false;
}

// Debugging
void PRU::setGPIOTestPin()
{
	if(!xenomai_gpio)
		return;
	xenomai_gpio[GPIO_SETDATAOUT] = TEST_PIN2_MASK;
}

void PRU::clearGPIOTestPin()
{
	if(!xenomai_gpio)
		return;
	xenomai_gpio[GPIO_CLEARDATAOUT] = TEST_PIN2_MASK;
}
