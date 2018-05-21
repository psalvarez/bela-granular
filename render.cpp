/*Code created by Pedro Sánchez Álvarez
*
*Code from bbb_ola_complete project's render.cpp was used as a template
*/

#include <Bela.h>
#include <ne10/NE10.h>					// neon library
#include <cmath>


#define NUM_CHANNELS 2
#define BUFFER_SIZE 32768
#define N_MODES 6



// Input and output buffers
float gInputBuffer[BUFFER_SIZE];
int gInputBufferPointer = 0;
float gOutputBuffer[BUFFER_SIZE];
int gOutputBufferReadPointer;
int gOutputBufferWritePointer;
int gFFTInputBufferPointer;
int gFFTOutputBufferPointer;

int gSampleCount = 0; //Counts samples on each block

// -----------------------------------------------
// These variables used internally in the example:
int gGrainSize;
int gHopSize;
float gFFTScaleFactor = 0;
int gFFTSize = 0;

// GPIO
int gInputButtonPin = P8_09;
int gInputPotPin = 0;
int gButton = 0;
int gPrevButton = 0;
int gBounceTimer = 0;
bool gChangeDone = false; //Flag to check whether a nmode change has been made
int gAudioFramesPerAnalogFrame;

// FFT vars
ne10_fft_cpx_float32_t* timeDomainIn;
ne10_fft_cpx_float32_t* timeDomainOut;
ne10_fft_cpx_float32_t* frequencyDomain;
ne10_fft_cfg_float32_t cfg;

//Granularisation variables
int gMode = 0; //Granulation mode
bool gMute = false; //Flag to mute every other sample

// Auxiliary task for calculating FFT
AuxiliaryTask gFFTTask;

void process_grain_background(void *);

// setup() is called once before the audio rendering starts.
// Use it to perform any initialisation and allocation which is dependent
// on the period size or sample rate.
//
// Return true on success; returning false halts the program.

bool setup(BelaContext *context, void *userData)
{
	// Set up FFT
	gFFTSize = (int) *(float *)userData;
	gGrainSize = gFFTSize;
	gHopSize = gGrainSize/2;
	gFFTScaleFactor = 1.0f / (float)gFFTSize * 1000;
	rt_printf("scale factor: %f\n", gFFTScaleFactor);

	timeDomainIn = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * sizeof (ne10_fft_cpx_float32_t));
	timeDomainOut = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * sizeof (ne10_fft_cpx_float32_t));
	frequencyDomain = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * sizeof (ne10_fft_cpx_float32_t));
	cfg = ne10_fft_alloc_c2c_float32_neon (gFFTSize);


	// Initialise auxiliary tasks
	if((gFFTTask = Bela_createAuxiliaryTask(&process_grain_background, 90, "grain-calculation")) == 0)
		return false;

	memset(gInputBuffer, 0, BUFFER_SIZE * sizeof(float));
	memset(timeDomainOut, 0, gFFTSize * sizeof (ne10_fft_cpx_float32_t));
	memset(gOutputBuffer, 0, BUFFER_SIZE * sizeof(float));

	// Initialise output pointers
	gOutputBufferReadPointer = 0;
	gOutputBufferWritePointer = gHopSize;

	//Button Pin
	pinMode(context, 0, gInputButtonPin, INPUT);

	//Potentiometer
	gAudioFramesPerAnalogFrame = context->audioFrames / context->analogFrames;

	return true;
}

// This function handles the granular processing once the buffer has
// been assembled.
void process_grain(float *inBuffer, int inWritePointer, float *outBuffer, int outWritePointer, int mode)
{
	// Copy buffer into FFT input

	int pointer = (inWritePointer - gGrainSize + BUFFER_SIZE) % BUFFER_SIZE; //Pointer to the earliest sample needed
	float windowVal = 0;

	for(int n = 0; n < gFFTSize; n++) {
		if(n < gGrainSize) {
			windowVal = 0.5f * (1.0f - cosf(2.0 * M_PI * n / (float)(gGrainSize - 1))); //Value of the window in this sample (Hann window). Calculated online due to changing window sizes

			timeDomainIn[n].r = (ne10_float32_t) inBuffer[pointer] * windowVal;
			timeDomainIn[n].i = 0;

			// Increase circular buffer pointer (wrap around when necessary)
			pointer++;
			if (pointer >= BUFFER_SIZE) {
				pointer = 0;
			}
		} else { //Zero Pad
			timeDomainIn[n].r = 0;
			timeDomainIn[n].i = 0;
		}
	}

	if (mode == N_MODES) {
		mode = rand() % N_MODES - 1;
	}

	if (mode == 0) { //FFT Bypass. Only used to test if the FFT is working
		// Run the FFT
		ne10_fft_c2c_1d_float32_neon (frequencyDomain, timeDomainIn, cfg, 0);

		// Run the inverse FFT
		ne10_fft_c2c_1d_float32_neon (timeDomainOut, frequencyDomain, cfg, 1);

	}	else if (mode == 1) { //Reverse the grain
		for(int n = 0; n < gGrainSize; n++) {
			timeDomainOut[n].r = timeDomainIn[gGrainSize - n].r;
		}

	} else if (mode == 2) { //Mute every other grain
		if(gMute) { //If mute is on, output is 0
			for(int n = 0; n < gGrainSize; n++) {
				timeDomainOut[n].r = 0;
			}
		} else {
			for(int n = 0; n < gGrainSize; n++) {
				timeDomainOut[n].r = timeDomainIn[n].r;
			}
		}

		gMute = !gMute; //Change the mute toggle

	} else if (mode == 3) { //Robotisation
		ne10_fft_c2c_1d_float32_neon (frequencyDomain, timeDomainIn, cfg, 0);
		for(int n = 0; n < gFFTSize; n++) {
			float amplitude = sqrtf(frequencyDomain[n].r * frequencyDomain[n].r + frequencyDomain[n].i * frequencyDomain[n].i);
			frequencyDomain[n].r = amplitude; //Save amplitude
			frequencyDomain[n].i = 0; //Set Phase to 0
		}
		ne10_fft_c2c_1d_float32_neon (timeDomainOut, frequencyDomain, cfg, 1);

	} else if (mode == 4) { //Twist
		for(int n = 0; n < floor(gGrainSize)/2; ++n) {
			timeDomainOut[n].r = timeDomainIn[(int) floor(gGrainSize)/2 + n].r;
			timeDomainOut[(int) floor(gGrainSize)/2 + n].r = timeDomainIn[n].r;
		}

	} else if (mode == 5){ //True Bypass
		for(int n = 0; n < gGrainSize; n++) {
			timeDomainOut[n].r = timeDomainIn[n].r;
		}
	}

	// Copy into Output Buffer
	pointer = outWritePointer;
	for(int n = 0; n < gGrainSize; n++) {
		if ((mode == 3) || (mode == 0)) {
			outBuffer[pointer] += timeDomainOut[n].r * gFFTScaleFactor;
			//Only apply scale factor when FFT is done
		} else {
			outBuffer[pointer] += timeDomainOut[n].r;
		}
		pointer++;
		if(pointer >= BUFFER_SIZE)
			pointer = 0;
	}
}

// Function to process the FFT in a thread at lower priority
void process_grain_background(void *) {
	process_grain(gInputBuffer, gFFTInputBufferPointer, gOutputBuffer, gFFTOutputBufferPointer, gMode);
}


void render(BelaContext *context, void *userData)
{
	for(unsigned int n = 0; n < context->audioFrames; n++) {

		/*GPIO*/

		//Button
		gButton = digitalRead(context, n, gInputButtonPin); //Pin is low when pressed
		int bounceThres = 1000; //Bouncing time in samples

		if ((gButton == 0) && (gPrevButton != 0)) { //Action happens when pressing
			//If the button has just been pressed, start timer
			gBounceTimer = 0;
			gChangeDone = false;
		} else if ((gButton == 0) && (gPrevButton == 0)) {
			//If button was pressed and stays pressed, increment count
			++gBounceTimer;
			//rt_printf("Hey Ho\n");
			if((gBounceTimer >= bounceThres) && !gChangeDone) { //If the timer has run long enough, and the mode hasn't been switched before, switch mode
				gMode++; //Switching between modes when pressing button
				if(gMode > N_MODES) {
					gMode = 0;
				}
				rt_printf("Mode: %d\n", gMode);
				gBounceTimer = 0;
				gChangeDone = true;
			}
		}

		gPrevButton = gButton; //Save button state

		//Potentiometer
		if ((gMode == 0) || (gMode == 3)) {
			gGrainSize = gFFTSize; //Fix grain size if doing the FFT
		} else {
			if(!(n % gAudioFramesPerAnalogFrame)) {
				// On even audio sample read analog inputs
				gGrainSize = map(analogRead(context, n/gAudioFramesPerAnalogFrame, gInputPotPin), 0, 1, gFFTSize/4, gFFTSize);

				//rt_printf("Window Size: %d\n", gGrainSize);
			}
		}
		gHopSize = gGrainSize /2; //Update Hop size



		/*Audio*/

        float inSample = 0;
        for(int channel = 0; channel < context->audioOutChannels; channel++) {

    		// Read data from Audio Input into In Buffer
            inSample += audioRead(context, n, channel);

    		// Copy Out Buffer to Audio Output
            audioWrite(context, n, channel, gOutputBuffer[gOutputBufferReadPointer]);
		}
        gInputBuffer[gInputBufferPointer] = inSample/context->audioOutChannels; //Turn into Mono
        gOutputBuffer[gOutputBufferReadPointer] = 0; //Clear sample in Out Buffer


        //Increment pointers
		gOutputBufferReadPointer++;
		if(gOutputBufferReadPointer >= BUFFER_SIZE)
			gOutputBufferReadPointer = 0;

		gOutputBufferWritePointer++;
		if(gOutputBufferWritePointer >= BUFFER_SIZE)
			gOutputBufferWritePointer = 0;

		gInputBufferPointer++;
		if(gInputBufferPointer >= BUFFER_SIZE)
			gInputBufferPointer = 0;


		//Increment sample count and check if the block has been filled
		gSampleCount++;
		if(gSampleCount >= gHopSize) {

			gFFTInputBufferPointer = gInputBufferPointer;
			gFFTOutputBufferPointer = gOutputBufferWritePointer;
			Bela_scheduleAuxiliaryTask(gFFTTask);

			//Reset SampleCount once the block has been filled
			gSampleCount = 0;
		}
	}
}

// cleanup() is called once at the end, after the audio has stopped.
// Release any resources that were allocated in setup().

void cleanup(BelaContext *context, void *userData)
{
	NE10_FREE(timeDomainIn);
	NE10_FREE(timeDomainOut);
	NE10_FREE(frequencyDomain);
	NE10_FREE(cfg);
}
