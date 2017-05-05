/* Audio Library for Teensy 3.X
 * Copyright (c) 2014, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "input_i2s_32bit_f32.h"
#include "output_i2s_32bit_f32.h"
#include <arm_math.h>

float AudioInputI2S_32bit_F32::sample_rate_Hz = AUDIO_SAMPLE_RATE;
int AudioInputI2S_32bit_F32::audio_block_samples = AUDIO_BLOCK_SAMPLES;
uint16_t AudioInputI2S_32bit_F32::block_offset = 0;
bool AudioInputI2S_32bit_F32::update_responsibility = false;
DMAChannel AudioInputI2S_32bit_F32::dma(false);

//16-bit audio data
//typedef int16_t audio_data_type_t;
//DMAMEM static uint32_t i2s_rx_buffer[AUDIO_BLOCK_SAMPLES]; //for int16, this is good for a stereo set that is AUDIO_BLOCK_SAMPLES long
//audio_block_t * AudioInputI2S_32bit_F32::block_left = NULL;
//audio_block_t * AudioInputI2S_32bit_F32::block_right = NULL;
//int AudioInputI2S_32bit_F32::bit_depth = 16;   //not used?
//#define I2S_BUFFER_TO_USE_BYTES (AudioOutputI2S_32bit_F32::audio_block_samples*sizeof(i2s_rx_buffer[0]))


//32-bit audio data
typedef float32_t audio_data_type_t;
DMAMEM static int32_t i2s_rx_buffer[2*AUDIO_BLOCK_SAMPLES]; //was uint16_t with size = AUDIO_BLOCK_SAMPLES when using int16 data
audio_block_f32_t * AudioInputI2S_32bit_F32::block_left = NULL;
audio_block_f32_t * AudioInputI2S_32bit_F32::block_right = NULL;
int AudioInputI2S_32bit_F32::bit_depth = 32;  //not used?
#define I2S_BUFFER_TO_USE_BYTES (AudioOutputI2S_32bit_F32::audio_block_samples*2*sizeof(i2s_rx_buffer[0]))


void AudioInputI2S_32bit_F32::begin(void)
{
	dma.begin(true); // Allocate the DMA channel first

	//block_left_1st = NULL;
	//block_right_1st = NULL;

	// TODO: should we set & clear the I2S_RCSR_SR bit here?
	AudioOutputI2S_32bit_F32::sample_rate_Hz = sample_rate_Hz;
	AudioOutputI2S_32bit_F32::audio_block_samples = audio_block_samples;
	AudioOutputI2S_32bit_F32::bit_depth = bit_depth;
	AudioOutputI2S_32bit_F32::config_i2s();

	CORE_PIN13_CONFIG = PORT_PCR_MUX(4); // pin 13, PTC5, I2S0_RXD0
#if defined(KINETISK)
	dma.TCD->SADDR = &I2S0_RDR0;  //Source Address for data.  I2SO_RDR0 is Synchronous Audio Interface receive data register (kinetis.h)
	dma.TCD->SOFF = 0;  		  //Source Address offset. zero means that it doesn't move.
	//dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);  //Transfer Attributes for 16-bit.  Source and Desitnation transfer sizes.  The DMA codes are from kinetis.h
	dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(3) | DMA_TCD_ATTR_DSIZE(3); //32-bit is 0b10.  Source and Desitnation transfer sizes.  The DMA codes are from kinetis.h
	//dma.TCD->NBYTES_MLNO = 2;   //number of bytes to transfer.  16-bit data
	dma.TCD->NBYTES_MLNO = 2*2;   //number of bytes to transfer.  32-bit data?
	dma.TCD->SLAST = 0;   //last source address adjustment at end of major iteration
	dma.TCD->DADDR = i2s_rx_buffer;  //destination address
	//dma.TCD->DOFF = 2;   //destination address offset after each destination write.  16-bit data.
	dma.TCD->DOFF = 2*2;   //destination address offset after each destination write.  32-bit data?
	dma.TCD->CITER_ELINKNO = I2S_BUFFER_TO_USE_BYTES / 2; //Major loop count...half of the buffer
	dma.TCD->DLASTSGA = -I2S_BUFFER_TO_USE_BYTES;  //last destination address adjustment when major iteration is completed
	dma.TCD->BITER_ELINKNO = I2S_BUFFER_TO_USE_BYTES / 2;  //beginning major loop count...half of the buffer
	dma.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;  //channel service request? (DMA fields in kinetis.h...0x0004 and 0x0002, respectively)
#endif
	dma.triggerAtHardwareEvent(DMAMUX_SOURCE_I2S0_RX);
	update_responsibility = update_setup();
	dma.enable();

	I2S0_RCSR |= I2S_RCSR_RE | I2S_RCSR_BCE | I2S_RCSR_FRDE | I2S_RCSR_FR; //Synchronous Audio Interface receive control register
	I2S0_TCSR |= I2S_TCSR_TE | I2S_TCSR_BCE; // TX clock enable, because sync'd to TX
	dma.attachInterrupt(isr);
	
	
};

void AudioInputI2S_32bit_F32::isr(void)
{
	uint32_t daddr, offset;
	const audio_data_type_t *src, *end;
	audio_data_type_t *dest_left, *dest_right;
	audio_block_f32_t *left, *right;

#if defined(KINETISK)
	daddr = (uint32_t)(dma.TCD->DADDR);
#endif
	dma.clearInterrupt();

	if (daddr < (uint32_t)i2s_rx_buffer + I2S_BUFFER_TO_USE_BYTES / 2) {		
		// DMA is receiving to the first half of the buffer
		// need to remove data from the second half
		src = (audio_data_type_t *)&i2s_rx_buffer[2*audio_block_samples/2]; //start at midpoint
		end = (audio_data_type_t *)&i2s_rx_buffer[2*audio_block_samples];	//go until the end
		if (AudioInputI2S_32bit_F32::update_responsibility) AudioStream_F32::update_all(); //why is this called now instead of at the end of this function?
	} else {
		// DMA is receiving to the second half of the buffer
		// need to remove data from the first half
		src = (audio_data_type_t *)&i2s_rx_buffer[0];	//start at beginning
		end = (audio_data_type_t *)&i2s_rx_buffer[2*audio_block_samples/2]; //got to midpoint
	}
	left = AudioInputI2S_32bit_F32::block_left;
	right = AudioInputI2S_32bit_F32::block_right;
	if (left != NULL && right != NULL) {
		offset = AudioInputI2S_32bit_F32::block_offset;
		if (offset <= ((uint32_t) audio_block_samples)) { //make sure it's a legal value
			dest_left = &(left->data[offset]);
			dest_right = &(right->data[offset]);
			AudioInputI2S_32bit_F32::block_offset = offset + audio_block_samples/2;
			do {
				//copy left/right interleaved data from i2s_rx_buffer into destination buffers
				*dest_left++ = (float32_t) *src++;  //copy int32 data out of i2s_rx_buffer into dest_left float32 buffer
				*dest_right++ = (float32_t) *src++; //copy int32 data out of i2s_rx_buffer into dest_right float32 buffer
			} while (src < end);
		}
	}
}

//#define I16_TO_F32_NORM_FACTOR (3.051850947599719E-05)  //which is 1/32767, which is 1/(2^(16-1)-1)
//void AudioInputI2S_32bit_F32::convert_i16_to_f32( int16_t *p_i16, float32_t *p_f32, int len) {
//	for (int i=0; i<len; i++) { *p_f32++ = ((float32_t)(*p_i16++)) * I16_TO_F32_NORM_FACTOR; }
//}
#define I32_TO_F32_NORM_FACTOR (4.656612875245797E-10)  //which is , which is 1/(2^(32-1)-1)
void AudioInputI2S_32bit_F32::scale_i32_to_f32( float32_t *p_in_f32, float32_t *p_out_f32, int len) {
	for (int i=0; i<len; i++) { *p_out_f32++ = (*p_in_f32++) * I32_TO_F32_NORM_FACTOR; }
}

void AudioInputI2S_32bit_F32::update(void)
{
	audio_block_f32_t *new_left=NULL, *new_right=NULL, *out_left=NULL, *out_right=NULL;

	// allocate 2 new blocks, but if one fails, allocate neither
	new_left = AudioStream_F32::allocate_f32();
	if (new_left != NULL) {
		new_right = AudioStream_F32::allocate_f32();
		if (new_right == NULL) {
			AudioStream_F32::release(new_left);
			new_left = NULL;
		}
	}
	__disable_irq();

	if (block_offset >= audio_block_samples) {	
		// the DMA filled 2 blocks, so grab them and get the
		// 2 new blocks to the DMA, as quickly as possible
		out_left = block_left;
		block_left = new_left;
		out_right = block_right;
		block_right = new_right;
		block_offset = 0;
		__enable_irq();
		
		//scale int32 to float 32
		scale_i32_to_f32(out_left->data, out_left->data, audio_block_samples);
		scale_i32_to_f32(out_right->data, out_right->data, audio_block_samples);
		
		// then transmit the DMA's former blocks		
		AudioStream_F32::transmit(out_left,0);
		AudioStream_F32::transmit(out_right,1);		
		AudioStream_F32::release(out_left);
		AudioStream_F32::release(out_right);

	} else if (new_left != NULL) {
		// the DMA didn't fill blocks, but we allocated blocks
		if (block_left == NULL) {
			// the DMA doesn't have any blocks to fill, so
			// give it the ones we just allocated
			block_left = new_left;
			block_right = new_right;
			block_offset = 0;
			__enable_irq();
		} else {
			// the DMA already has blocks, doesn't need these
			__enable_irq();
			AudioStream_F32::release(new_left);
			AudioStream_F32::release(new_right);
		}
	} else {
		// The DMA didn't fill blocks, and we could not allocate
		// memory... the system is likely starving for memory!
		// Sadly, there's nothing we can do.
		__enable_irq();
	}
}


/******************************************************************/

/*
void AudioInputI2Sslave::begin(void)
{
	dma.begin(true); // Allocate the DMA channel first

	//block_left_1st = NULL;
	//block_right_1st = NULL;

	AudioOutputI2Sslave::config_i2s();

	CORE_PIN13_CONFIG = PORT_PCR_MUX(4); // pin 13, PTC5, I2S0_RXD0
#if defined(KINETISK)
	dma.TCD->SADDR = &I2S0_RDR0;
	dma.TCD->SOFF = 0;
	dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
	dma.TCD->NBYTES_MLNO = 2;
	dma.TCD->SLAST = 0;
	dma.TCD->DADDR = i2s_rx_buffer;
	dma.TCD->DOFF = 2;
	dma.TCD->CITER_ELINKNO = sizeof(i2s_rx_buffer) / 2;
	dma.TCD->DLASTSGA = -sizeof(i2s_rx_buffer);
	dma.TCD->BITER_ELINKNO = sizeof(i2s_rx_buffer) / 2;
	dma.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;
#endif
	dma.triggerAtHardwareEvent(DMAMUX_SOURCE_I2S0_RX);
	update_responsibility = update_setup();
	dma.enable();

	I2S0_RCSR |= I2S_RCSR_RE | I2S_RCSR_BCE | I2S_RCSR_FRDE | I2S_RCSR_FR;
	I2S0_TCSR |= I2S_TCSR_TE | I2S_TCSR_BCE; // TX clock enable, because sync'd to TX
	dma.attachInterrupt(isr);
}
*/



