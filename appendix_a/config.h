/*
 * config.h
 *
 *  Created on: Sep 24, 2017
 *      Author: edgco
 */

#ifndef APPENDIX_A_CONFIG_H_
#define APPENDIX_A_CONFIG_H_

#define SAMPLING_FREQUENCY 48000
#define SAMPLED_LUT_FREQUENCY 10
#define NUM_SAMPLES ((SAMPLING_FREQUENCY / SAMPLED_LUT_FREQUENCY) / 2)
#define MAX_WAVEFORM_INDEX ((NUM_SAMPLES) * 2)
#define NUM_OUTPUT_FREQS 2

typedef enum wave_type
{
	SINE_WAVE,
	COS_WAVE
} wavetype_t;



#endif /* APPENDIX_A_CONFIG_H_ */
