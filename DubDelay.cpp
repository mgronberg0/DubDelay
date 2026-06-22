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

// Named constants for all hardcoded values
constexpr float kHpCutoffHz         = 120.0f;
constexpr float kHissHpCutoffHz     = 800.0f;
constexpr float kMinDelaySamples    = 4800.0f;   // 0.1 s * Fs
constexpr float kInputGainScale     = 1.5f;
constexpr float kMixCrossover       = 0.75f;
constexpr float kMixDryRolloffScale = 4.0f;
constexpr float kFeedbackScale      = 1.1f;
constexpr float kLpfBaseHz          = 100.0f;
constexpr float kLpfOctaveRange     = 120.0f;
constexpr float kSaturationGRange   = 2.0f;
constexpr float kWowDepthMin        = 100.0f;
constexpr float kWowDepthRange      = 2000.0f;
constexpr float kDropoutRange       = 0.9f;
constexpr float kHissMinDb          = -100.0f;
constexpr float kHissDbRange        = 60.0f;
constexpr float kWowAugmentScale    = 0.2f;
constexpr float kDubInputGain       = 1.71f;
constexpr float kDubDryMix          = 0.01f;
constexpr float kDubWetMix          = 1.0f;
constexpr uint32_t kDisplayRefreshMs = 40;
// Display Variables
float knobPrev[DaisyField::KNOB_LAST];
int activeKnob = 0;
uint32_t displayTimer = 0;
const float knobThreshold = 0.007f;
enum knobID { 
	KNOB_DELAY, 		//1
	KNOB_FEEDBACK, 		//2
	KNOB_LPF,  			//3
	KNOB_SATURATION, 	//4
	KNOB_INPUT_GAIN, 	//5
	KNOB_WOW, 			//6
	KNOB_AGE, 			//7
	KNOB_MIX 			//8
	};
static const char* const knobNames[DaisyField::KNOB_LAST] = {
	"DelayT", "Fdbk", "LPF", "Satur", "InGain", "Wow", "Age", "Mix"
};
// Buffer Management
DSY_SDRAM_BSS float Buffer[N];
int writeIndex = 0;
// read index is fractional
float readIndexF = 0.0f;

float delayed0 = 0.0f;
float delayed1 = 0.0f;
float feedbackGain = 0.8f;
float lp_state = 0.0f;
float hp_state = 0.0f;
float hp_coeff = expf(-2.0f * PI_F * kHpCutoffHz / Fs);

float G = 4.0f; // saturation scaling

// Control smooting state vars
float motor_coeff = 0.00005f;
float mix_coeff   = 0.99f;
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
float wow_freqHz = 0.80f;
float wow_phase = 0.0f;
float wow_depth = 100.0f;
float wow_freq_motor_delta = 2.0f;
// Hiss level (between -120dB and -40dB)
float hiss_level_scalar = 0.0f;
float hiss_hp_coeff = expf(-2.0f * PI_F * kHissHpCutoffHz / Fs);
float hiss_n1 = 0.0f;
float hiss_hp_state = 0.0f;
float dropout_threshold = 1.01f;
// Dub Macro Coeffs
bool dub_held = false;
float fast_coeff = 0.0002f;
float slow_coeff = 0.00005f;

float interpolateSample(float *buffer, float fractional_idx, int buf_length);
float MagToDB(float scalar);
float DBToMag(float dB);


void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	hw.ProcessAllControls();
	// Controls update
	float raw = hw.GetKnobValue(KNOB_DELAY);
	// Delay smoothing is handled per sample
	float input_delay = (kMinDelaySamples + raw*raw * (N - kMinDelaySamples));
	// input gain 
	input_gain = kInputGainScale * hw.GetKnobValue(KNOB_INPUT_GAIN);
	//input_gain += (1.5f * raw - input_gain) * ctrl_coeff;
	// Volume Params
	raw = hw.GetKnobValue(KNOB_MIX);
	if(raw < kMixCrossover){
		input_dry_mix = 1.0f;
		input_wet_mix = (raw / kMixCrossover);
	} else if(raw >= kMixCrossover){
		// Roll off dry at high mix values
		input_dry_mix = 1.0f - kMixDryRolloffScale*(raw - kMixCrossover);
		input_wet_mix = 1.0f;
	}
	// Feedback
	raw = hw.GetKnobValue(KNOB_FEEDBACK);
	input_feedback = raw * kFeedbackScale;
	// smooth_feedback += (hw.GetKnobValue(1)*(1.4f) - smooth_feedback) * ctrl_coeff;
	// low pass filter fc
	raw = kLpfBaseHz * powf(kLpfOctaveRange, hw.GetKnobValue(KNOB_LPF)); // 100 hz to 12khz logscale
	input_lpFc = expf(-2.0f * PI_F * raw / Fs); // convert hz to 1 pole lowpass coeff
	// squre the raw 0 - 1 value for G: low G values are more drastic
	raw = hw.GetKnobValue(KNOB_SATURATION);
	input_saturationG = (1.0f + (raw * kSaturationGRange));

	raw = hw.GetKnobValue(KNOB_WOW);
	wow_depth = kWowDepthMin + kWowDepthRange * raw;

	// Age
	raw = hw.GetKnobValue(KNOB_AGE);
	// Hiss level (between -100dB and -40dB)
	dropout_threshold = 1.01f - kDropoutRange * raw;
	hiss_level_scalar = DBToMag(kHissMinDb + kHissDbRange * raw);
	// get random value [0 1], bias around 0.0, scale by age, decimate. this value is applied to
	// scale the wow lfo 
	float wow_freq_augment = 1.0f - (((daisy::Random::GetFloat() - 0.5f) * raw) * kWowAugmentScale);
	

	// Dub Macros:
	if (hw.sw[DaisyField::LED_KEY_A1].RisingEdge()){
		dub_held = true;
	}
	if (hw.sw[DaisyField::LED_KEY_A1].FallingEdge()){
		dub_held = false;
	}
	if(dub_held){
		// augment input gain target
		input_gain = kDubInputGain;
		input_dry_mix = kDubDryMix;
		input_wet_mix = kDubWetMix;
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
		wow_phase += (static_cast<float>(N)/smooth_delayT)* wow_freqHz*wow_freq_augment* TWOPI_F / Fs;
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
		// Feedback filtering
		delayed1 = delayed0;
		delayed0 = interpolateSample(Buffer, readIndexF, N);
		// apply high pass
		hp_state = delayed0 - delayed1 + hp_coeff * hp_state;

		// apply low pass
		lp_state = (1-smooth_lpFc) * hp_state + (smooth_lpFc*lp_state);

		// Saturation
		y = tanhf(lp_state * smooth_saturationG) / smooth_saturationG;
		// hiss filtering
		float hiss = (2.0f*daisy::Random::GetFloat()-1.0f)*hiss_level_scalar; 
		hiss_hp_state = hiss - hiss_n1 + hiss_hp_coeff * hiss_hp_state;
		hiss_n1 = hiss;

		Buffer[writeIndex] = smooth_gain*in[0][i] + smooth_feedback*y + hiss_hp_state;

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
float MagToDB(float scalar)
{
	return 8.6858896f * logf(fmaxf(scalar, 1e-7f)); // 20/log(10)
}
float DBToMag(float dB)
{
	return pow10f(dB * 0.05f);
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
	// initilize knob probing
	for(size_t i = 0; i < DaisyField::KNOB_LAST; i++){
		knobPrev[i] = hw.GetKnobValue(i);
	}
	int num_lines = 4;
	int cur_line = 0;
	int mru[DaisyField::KNOB_LAST];
	int mru_count = 0;
	while(1) {
		// Display manager
		uint32_t now = System::GetNow();
		if(now - displayTimer >= kDisplayRefreshMs)
		{
			displayTimer = now;

			for(size_t i = 0; i < DaisyField::KNOB_LAST; i++){
				float v = hw.GetKnobValue(i);
				if(fabsf(v - knobPrev[i]) > knobThreshold){
					int from = mru_count;
					for(int k = 0; k < mru_count; k++){
						if(mru[k]==i){
							from = k; break; 
						}
					}
					if(from==mru_count && mru_count < DaisyField::KNOB_LAST){
						mru_count++;
					}
					for(int k = (from < mru_count ? from : mru_count - 1); k > 0; k--){
						mru[k] = mru[k - 1];
					}
					mru[0]=i;
				}
				knobPrev[i] = v;
			}

			hw.display.Fill(false);
			int rows = (mru_count < 4) ? mru_count : 4;
			for(int r = 0; r < rows; r++){
				int id = mru[r];
				FixedCapStr<32> str(knobNames[id]);
				str.Append(": ");
				str.AppendFloat(hw.GetKnobValue(id), 3);
				hw.display.SetCursor(0, r*12);
				hw.display.WriteString(str.Cstr(), Font_7x10, true);
			}
			hw.display.Update();
		}
	}
}
