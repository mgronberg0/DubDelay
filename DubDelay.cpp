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

float delayed0 = 0.0f;
float delayed1 = 0.0f;
float feedbackGain = 0.8f;
float lp_state = 0.0f;
float hp_state = 0.0f;
float hp_coeff = expf(-2.0f * PI_F * 120 / Fs);

float G = 4.0f; // saturation scaling

// Control smooting state vars
float motor_coeff = 0.0005f;
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
	// 1.1 coeff allows for deeper feedback
	smooth_feedback += (hw.GetKnobValue(1)*(1.1f) - smooth_feedback) * ctrl_coeff;
	// low pass filter fc
	raw = 100.0f * powf(120.0f, hw.GetKnobValue(2)); // 100 hz to 12khz logscale
	raw = expf(-2.0f * PI_F * raw / Fs); // convert hz to 1 pole lowpass coeff
	smooth_lpFc = fmaxf( 0.001f, fminf(0.995f, smooth_lpFc + ((raw - smooth_lpFc) * ctrl_coeff)));
	// squre the raw 0 - 1 value for G: low G values are more drastic
	raw = hw.GetKnobValue(3);
	smooth_saturationG += ((1.0f + (raw*raw * 9.0f)) - smooth_saturationG) * ctrl_coeff;

	// Delay Math
	float y = 0.0f; //output of saturator
	for (size_t i = 0; i < size; i++)
	{
		// Delay smoothing is handled per sample
		smooth_delayT += (dKnob - smooth_delayT) * motor_coeff;
		smooth_delayT = fmaxf(1.0f, fminf(static_cast<float>(N - 1),smooth_delayT));
		readIndexF = writeIndex - smooth_delayT;
		if(readIndexF<0){
			readIndexF = readIndexF+N;
		}
		delayed1 = delayed0;
		delayed0 = interpolateSample(Buffer, readIndexF, N);
		// apply high pass
		hp_state = delayed0 - delayed1 + hp_coeff * hp_state;

		// apply low pass
		lp_state = (1-smooth_lpFc) * hp_state + (smooth_lpFc*lp_state);

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
	hw.SetAudioBlockSize(64); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	hw.StartAdc();
	hw.StartAudio(AudioCallback);
	while(1) {}
}
