
#ifndef _AudioSettings_F32_
#define _AudioSettings_F32_

class AudioSettings_F32 {
	public:
		AudioSettings_F32(float fs_Hz, int block_size) :
			sample_rate_Hz(fs_Hz),  bit_depth(16), audio_block_samples(block_size) {}
		AudioSettings_F32(float fs_Hz, int block_size, int _bit_depth) :
			sample_rate_Hz(fs_Hz), bit_depth(_bit_depth), audio_block_samples(block_size) {}
		
		const float sample_rate_Hz;   	//sample rate from Audio Codec
		const int bit_depth;		//bit depth for audio codec
		const int audio_block_samples;	//default size of audio block
		
		float cpu_load_percent(const int n);
		float processorUsage(void);
		float processorUsageMax(void);
		void processorUsageMaxReset(void);
};

#endif