// TODO: Improve parameter ranges for broader sweet spots
// Create better saturation algo
// fix dub macros, currently not engaging
// add second delay line
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
float readIndexF = 0.0f;

float delayed0 = 0.0f;
float delayed1 = 0.0f;
float feedbackGain = 0.8f;
float lp_state = 0.0f;
float hp_state = 0.0f;
float hp_coeff = expf(-2.0f * PI_F * 120 / Fs);

float G = 4.0f; // saturation scaling

// Control smooting state vars
float motor_coeff = 0.00005f;
float mix_coeff   = 0.01f;
float ctrl_coeff = 0.01f;
float smooth_lpFc = 0.5f;
float smooth_delayT = Fs * 0.5f;
float smooth_feedback = 0.3f;
float smooth_saturationG = 1.0f;
float smooth_gain = 0.5f;
float smooth_dry = 1.0f;
float smooth_wet = 0.5f;

// input parameters
float input_gain = 0.5f;
float input_wet_mix = 0.5f;
float input_dry_mix = 1.0f;
float input_feedback = 0.3f;
float input_lpFc = 12000.0f;
float input_saturationG = 1.0f;
// wow and flutter
float wow_freqHz = 0.4f;
float wow_phase = 0.0f;
float wow_depth = 200.0f;
float wow_freq_motor_delta = 2.0f;
// Dub Macro Coeffs
bool dub_held = false;
float fast_coeff = 0.0002f;
float slow_coeff = 0.00005f;

float interpolateSample(float *buffer, float fractional_idx, int buf_length);


void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	hw.ProcessAllControls();
	// Controls update
	float raw = hw.GetKnobValue(0);
	// Delay smoothing is handled per sample
	float input_delay = (4800.0f + raw*raw * (N - 4800));
	// input gain 
	input_gain = 1.5f * hw.GetKnobValue(4);
	//input_gain += (1.5f * raw - input_gain) * ctrl_coeff;
	// Volume Params
	raw = hw.GetKnobValue(7);
	if(raw<0.75f){
		input_dry_mix = 1.0f;
		input_wet_mix = (raw / 0.75f);
	}else if(raw>=0.75f){
		// Roll off dry at high mix values
		input_dry_mix = 1.0f - 4*(raw-0.75f);
		input_wet_mix = 1.0f;
	}
	// Feedback
	raw = hw.GetKnobValue(1);
	input_feedback = raw*(1.1f);
	// smooth_feedback += (hw.GetKnobValue(1)*(1.4f) - smooth_feedback) * ctrl_coeff;
	// low pass filter fc
	raw = 100.0f * powf(120.0f, hw.GetKnobValue(2)); // 100 hz to 12khz logscale
	input_lpFc = expf(-2.0f * PI_F * raw / Fs); // convert hz to 1 pole lowpass coeff
	// squre the raw 0 - 1 value for G: low G values are more drastic
	raw = hw.GetKnobValue(3);
	input_saturationG = (1.0f + (raw * 2.0f));
	

	// Dub Macros:
	if (hw.sw[DaisyField::LED_KEY_A1].RisingEdge()){
		dub_held = true;
	}
	if (hw.sw[DaisyField::LED_KEY_A1].FallingEdge()){
		dub_held = false;
	}
	if(dub_held){
		// augment input gain target
		input_gain = 1.71;
		input_dry_mix = 0.01;
		input_wet_mix = 1.0;
	}
	// Input smoothing
	smooth_gain += (input_gain - smooth_gain) * mix_coeff;
	smooth_dry += (input_dry_mix - smooth_dry) * mix_coeff;
	smooth_wet += (input_wet_mix - smooth_wet) * mix_coeff;
	smooth_feedback += (input_feedback - smooth_feedback) * ctrl_coeff;
	smooth_saturationG += (input_saturationG - smooth_saturationG) * ctrl_coeff;
	smooth_lpFc = fmaxf(0.001f, fminf(0.995f,
                  smooth_lpFc + (input_lpFc - smooth_lpFc) * ctrl_coeff));
	// Delay smoothing is handled per sample
	// Delay Math
	float y = 0.0f; //output of saturator
	for (size_t i = 0; i < size; i++)
	{
		
		// Delay smoothing is handled per sample
		smooth_delayT += (input_delay - smooth_delayT) * motor_coeff;
		// wow and flutter calc
		// we want wow_freqHz to go up when the smooth delay is smaller
		// if the wow is at it's lowest speed when delay is at it's maximum of 3 seconds,
		// at 1.5 seconds it's twice as fast, at 0.75 it's 4 times as fast, etc
		wow_phase += (static_cast<float>(N)/smooth_delayT)* wow_freqHz * TWOPI_F / Fs;
		if (wow_phase>=TWOPI_F){
			wow_phase -= TWOPI_F;
		}
		float mod = (wow_depth*smooth_delayT/(static_cast<float>(N))) * sinf(wow_phase);
		float read_delay = smooth_delayT + mod;
		read_delay = fmaxf(1.0f, fminf(static_cast<float>(N - 1),read_delay));
		
		readIndexF = writeIndex - read_delay;
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
		Buffer[writeIndex] = smooth_gain*in[0][i] + smooth_feedback*y;

		writeIndex++;
		if (writeIndex>=N){
			writeIndex = 0;
		}

		out[0][i] = smooth_dry*in[0][i] + smooth_wet*y;
		out[1][i] = smooth_dry*in[1][i] + smooth_wet*y;
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
	hw.led_driver.SetAllTo(0.0f);
	hw.led_driver.SwapBuffersAndTransmit();
	while(1) {}
}
