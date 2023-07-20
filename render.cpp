// mixing the getData and setSensor examples. The encoder's value sets the
// frequency of a sinewave and can be reset by pressing the button

#include <Bela.h>
#include <cmath>
#include "DFRobot_VisualRotaryEncoder.h"
DFRobot_VisualRotaryEncoder_I2C sensor(/*i2cAddr = */0x54, /*i2cBus = */1);

float gFrequency = 440.0;
float gPhase;
float gInverseSampleRate;

// I2C I/O cannot be performed from the audio thread, so we
// do it in this other thread, polling the encoder every 10 ms
void readEncoder(void*) {
	uint16_t pastValue = -1;
	while(!Bela_stopRequested())
	{
		/**
		 * get the encoder current count
		 * return value range： 0-1023
		 */
		uint16_t encoderValue = sensor.getEncoderValue();
		if(encoderValue != pastValue)
			printf("The encoder current counts: %d\n", encoderValue);
		pastValue = encoderValue;
		// set the frequency of the oscillator accordingly
		gFrequency = encoderValue + 100;
		/**
		 * Detect if the button is pressed
		 * return true when the button pressed，otherwise, return false
		 */
		if(sensor.detectButtonDown()){
			/**
			 * set the encoder count value
			 * value range[0, 1023], the setting is invalid when out of range
			 * In this example, set the encoder value to zero when detecting the button pressed, and you can see all the LEDs that light up before turning off
			 */
			sensor.setEncoderValue(0);
		}
		usleep(10000);
	}
}

bool setup(BelaContext *context, void *userData)
{
	if(NO_ERR != sensor.begin()) {
		fprintf(stderr, "Error while initialising sensor. Are the address and bus correct?\n");
		return false;
	}
	/**
	* Retrieve basic information from the sensor and buffer it into basicInfo,
	* the structure that stores information
	* Members of basicInfo structure: PID, VID, version, i2cAddr
	*/
	sensor.refreshBasicInfo();
	/* Module PID, default value 0x01F6 (The highest two of 16-bit data are used to determine SKU type: 00: SEN, 01: DFR, 10: TEL, the next 14 are numbers.)(SEN0502) */
	printf("PID: %#x\n", sensor.basicInfo.PID);

	/* Module VID, default value 0x3343(for manufacturer DFRobot) */
	printf("VID: %#x\n", sensor.basicInfo.VID);

	/* Firmware version number: 0x0100 represents V0.1.0.0 */
	printf("versions: %#x\n", sensor.basicInfo.version);

	/* Module communication address, default value 0x54, module device address (0x54~0x57) */
	printf("communication address: %#x\n", sensor.basicInfo.i2cAddr);

	usleep(1000);
	/**
	 * Get the encoder current gain factor, and the numerical accuracy for turning one step
	 * Accuracy range：1~51，the minimum is 1 (light up one LED about every 2.5 turns), the maximum is 51 (light up one LED every one step rotation)
	 * Return value range： 1-51
	 */
	uint8_t gainCoefficient = sensor.getGainCoefficient();
	printf("Encoder current gain coefficient: %d\n", gainCoefficient);

	/**
	 * set the current gain factor of the encoder, and the numerical accuracy of turning one step
	 * accuracy range：1~51，the minimum is 1 (light up one LED about every 2.5 turns), the maximum is 51 (light up one LED every one step rotation)
	 * gainValue range[1, 51], the setting is invalid when out of range, tip: small amplitude adjustment of gain factor has little effect on LED change
	 */
	gainCoefficient = 25;
	printf("Setting encoder gain coefficient to: %d\n", gainCoefficient);
	sensor.setGainCoefficient(gainCoefficient);
	usleep(10000);

	// read it back to verify it was set correctly
	gainCoefficient = sensor.getGainCoefficient();
	printf("Encoder current gain coefficient: %d\n", gainCoefficient);

	// start a separate thread to read the encoder value.
	Bela_runAuxiliaryTask(readEncoder);

	gInverseSampleRate = 1.0 / context->audioSampleRate;
	gPhase = 0.0;

	return true;
}

void render(BelaContext *context, void *userData)
{
	for(unsigned int n = 0; n < context->audioFrames; n++) {
		float out = 0.8 * sinf(gPhase);
		gPhase += 2.0 * M_PI * gFrequency * gInverseSampleRate;
		if(gPhase > 2.0 * M_PI)
			gPhase -= 2.0 * M_PI;

		for(unsigned int channel = 0; channel < context->audioOutChannels; channel++) {
			// Two equivalent ways to write this code

			// The long way, using the buffers directly:
			// context->audioOut[n * context->audioOutChannels + channel] = out;

			// Or using the macros:
			audioWrite(context, n, channel, out);
		}
	}
}

void cleanup(BelaContext *context, void *userData)
{

}
