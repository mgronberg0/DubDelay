#include "daisy_field.h"
#include "daisysp.h"

using namespace daisy;
using namespace daisysp;

DaisyField hw;
constexpr int N = 96000;
constexpr int Fs = 48000;
constexpr int D = static_cast<int> (Fs * 0.5f);
DSY_SDRAM_BSS float Buffer[N];
int writeIndex = 0;
// read index will eventually be a float
int readIndex = 0;
float delayed = 0.0f; 
float feedbackGain = 0.8f;
float lp_state = 0.0f;
float lp_coeff = 0.7f;
float G = 4.0f; // saturation scaling


void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	hw.ProcessAllControls();
	float y = 0.0f; //output of saturator
	for (size_t i = 0; i < size; i++)
	{
		readIndex = writeIndex - D;
		if(readIndex<0){
			readIndex = readIndex+N;
		}
		// read position will have interpolation eventualy
		delayed = Buffer[readIndex];
		lp_state = (1-lp_coeff) * delayed + (lp_coeff*lp_state);

		// Saturation
		// Needs to be in control loop or parameter sanity area:
		if(G < 0.1){
			G = 0.1;
		}
		y = tanhf(lp_state * G) / G;
		Buffer[writeIndex] = in[0][i] + feedbackGain*y;

		writeIndex++;
		if (writeIndex>=N){
			writeIndex = 0;
		}

		out[0][i] = in[0][i] + y;
		out[1][i] = in[1][i] + y;
	}
}

void init(void)
{
	memset(Buffer, 0, sizeof(Buffer));
}

int main(void)
{
	
	hw.Init();
	init();
	hw.SetAudioBlockSize(4); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	hw.StartAdc();
	hw.StartAudio(AudioCallback);
	while(1) {}
}
