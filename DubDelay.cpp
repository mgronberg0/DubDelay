#include "daisy_field.h"
#include "daisysp.h"

using namespace daisy;
using namespace daisysp;

DaisyField hw;
constexpr int N = 96000;
constexpr int Fs = 48000;

DSY_SDRAM_BSS float Buffer[N];
int writeIndex = 0;
// read index is fractional
float readIndexF = 0;

float delayed = 0.0f; 
float feedbackGain = 0.8f;
float lp_state = 0.0f;
float lp_coeff = 0.7f;
float G = 4.0f; // saturation scaling

// Control smooting state vars
float motor_coeff = 0.00007f;
float smooth_lpFc = 0.5f;
float smooth_delayT = Fs * 0.5f;
float smooth_feedback = 0.3f;
float smooth_saturationG = 1.0f;
float ctrl_coeff = 0.001f;

float interpolateSample(float *buffer, float fractional_idx, int buf_length);


void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	hw.ProcessAllControls();
	// Controls update
	float raw = hw.GetKnobValue(0);
	// Delay smoothing is handled per sample
	float dKnob = (480.0f + raw*raw * (N - 480));
	// 1.1 coeff allows for explosive feedback
	smooth_feedback += (hw.GetKnobValue(1)*(1.1) - smooth_feedback) * ctrl_coeff;
	smooth_lpFc += (hw.GetKnobValue(2) - smooth_lpFc) * ctrl_coeff;
	// squre the raw 0 - 1 value for G: low G values are more drastic
	raw = hw.GetKnobValue(3);
	smooth_saturationG += ((1.0f + (raw*raw * 9.0f)) - smooth_saturationG) * ctrl_coeff;

	// Delay Math
	float y = 0.0f; //output of saturator
	for (size_t i = 0; i < size; i++)
	{
		// Delay smoothing is handled per sample
		smooth_delayT += (dKnob - smooth_delayT) * motor_coeff;
		readIndexF = writeIndex - smooth_delayT;
		if(readIndexF<0){
			readIndexF = readIndexF+N;
		}
		// read position will have interpolation eventualy
		delayed = interpolateSample(Buffer, readIndexF, N);
		lp_state = (1-smooth_lpFc) * delayed + (smooth_lpFc*lp_state);

		// Saturation
		y = tanhf(lp_state * smooth_saturationG) / smooth_saturationG;
		Buffer[writeIndex] = in[0][i] + smooth_feedback*y;

		writeIndex++;
		if (writeIndex>=N){
			writeIndex = 0;
		}

		out[0][i] = in[0][i] + y;
		out[1][i] = in[1][i] + y;
	}
}

float interpolateSample(float *buffer, float fractional_idx, int buf_length)
{
	// remove fraction, aka round down
	int lowIdx = (int)fractional_idx;
	int highIdx = (lowIdx+1) % buf_length;
	float fraction = fractional_idx - lowIdx;
	return ((buffer[highIdx] - buffer[lowIdx])*fraction) + buffer[lowIdx];
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
