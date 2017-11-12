/*
 * waveforms.c
 *
 *  Created on: Sep 24, 2017
 *      Author: edgco
 */

#include <stdbool.h>
#include <math.h>

#include "config.h"
#include "waveforms.h"
#include "sine_wave.h"

float sine_wave( float total_index )
{
	float lut_weighting_low = 0.0;
	float lut_weighting_high = 0.0;

	bool reversed = true;
	int multiplier = 1;
	int lut_index[2] = { -1, -1 };

	// Determine which sample index we're looking at
	int lut_sample_index = (int)floor(total_index) % NUM_SAMPLES;

	// Determine which quadrant we're in
	switch( (int)floor(total_index/ NUM_SAMPLES) % 2)
	{
	case 0:
		// No changes to index
		//reversed = false;
		multiplier = 1;
		break;

	case 1:
		// Reverse index, negative value
		//reversed = false;
		multiplier = -1;
		break;
//
//	case 2:
//		// Negative value
//		reversed = false;
//		multiplier = -1;
//
//		break;
//
//	case 3:
//		// Reverse index
//		reversed = true;
//		multiplier = -1;
//		break;
	}

	// If index is an integer value, return index
	if(total_index == floor(total_index)) {
		if(reversed) {
			return multiplier * SINE_WAVE_LUT[NUM_SAMPLES - lut_sample_index - 1];
		}
		else {
			return multiplier * SINE_WAVE_LUT[lut_sample_index];
		}
	}
	else {
		// Calculate element weighting
		lut_weighting_high = total_index - floor(total_index);
		lut_weighting_low = 1 - lut_weighting_high;

		// Determine LUT position indexes
		if(reversed) {
			lut_index[0] = NUM_SAMPLES - 1 - lut_sample_index; // Low LUT index

			if(lut_sample_index == NUM_SAMPLES - 1) {
				lut_index[1] = NUM_SAMPLES - 1; // Wrap around
			}
			else {
				lut_index[1] = lut_index[0] - 1;
			}
		}
		else {
			lut_index[0] = lut_sample_index; // Low LUT index

			if(lut_sample_index == NUM_SAMPLES - 1) {
				lut_index[1] = 0; // Wrap around
			}
			else {
				lut_index[1] = lut_index[0] + 1; //
			}
		}
	}

	return multiplier * (lut_weighting_low * SINE_WAVE_LUT[lut_index[0]] +
			lut_weighting_high * SINE_WAVE_LUT[lut_index[1]]);
}

float cosine_wave( float total_index ) {
	float lut_weighting_low = 0.0;
	float lut_weighting_high = 0.0;

	bool reversed = true;
	int multiplier = 1;
	int lut_index[2] = { -1, -1 };

	// Determine which sample index we're looking at
	int lut_sample_index = (int)floor(total_index) % NUM_SAMPLES;

	// Determine which quadrant we're in
	switch( (int)floor(total_index/ NUM_SAMPLES) % 2)
	{
	case 0:
		// No changes to index
		//reversed = false;
		multiplier = -1;
		break;

	case 1:
		// Reverse index, negative value
		//reversed = false;
		multiplier = 1;
		break;
		//
		//	case 2:
		//		// Negative value
		//		reversed = false;
		//		multiplier = -1;
		//
		//		break;
		//
		//	case 3:
		//		// Reverse index
		//		reversed = true;
		//		multiplier = -1;
		//		break;
	}

	// If index is an integer value, return index
	if(total_index == floor(total_index)) {
		if(reversed) {
			return multiplier * COS_WAVE_LUT[NUM_SAMPLES - lut_sample_index - 1];
		}
		else {
			return multiplier * COS_WAVE_LUT[lut_sample_index];
		}
	}
	else {
		// Calculate element weighting
		lut_weighting_high = total_index - floor(total_index);
		lut_weighting_low = 1 - lut_weighting_high;

		// Determine LUT position indexes
		if(reversed) {
			lut_index[0] = NUM_SAMPLES - 1 - lut_sample_index; // Low LUT index

			if(lut_sample_index == NUM_SAMPLES - 1) {
				lut_index[1] = NUM_SAMPLES - 1; // Wrap around
			}
			else {
				lut_index[1] = lut_index[0] - 1;
			}
		}
		else {
			lut_index[0] = lut_sample_index; // Low LUT index

			if(lut_sample_index == NUM_SAMPLES - 1) {
				lut_index[1] = 0; // Wrap around
			}
			else {
				lut_index[1] = lut_index[0] + 1;
			}
		}
	}

	return multiplier * (lut_weighting_low * COS_WAVE_LUT[lut_index[0]] +
			lut_weighting_high * COS_WAVE_LUT[lut_index[1]]);
}

float square_wave( float total_index ) {
	float lut_weighting_low = 0.0;
	float lut_weighting_high = 0.0;

	bool reversed = true;
	int multiplier = 1;
	int lut_index[2] = { -1, -1 };

	// Determine which sample index we're looking at
	int lut_sample_index = (int)floor(total_index) % NUM_SAMPLES;

	// Determine which quadrant we're in
	switch( (int)floor(total_index/ NUM_SAMPLES) % 2)
	{
	case 0:
		// No changes to index
		//reversed = false;
		multiplier = 1;
		break;

	case 1:
		// Reverse index, negative value
		//reversed = false;
		multiplier = -1;
		break;
		//
		//	case 2:
		//		// Negative value
		//		reversed = false;
		//		multiplier = -1;
		//
		//		break;
		//
		//	case 3:
		//		// Reverse index
		//		reversed = true;
		//		multiplier = -1;
		//		break;
	}

	// If index is an integer value, return index
	if(total_index == floor(total_index)) {
		if(reversed) {
			return multiplier * SQUARE_WAVE_LUT[NUM_SAMPLES - lut_sample_index - 1];
		}
		else {
			return multiplier * SQUARE_WAVE_LUT[lut_sample_index];
		}
	}
	else {
		// Calculate element weighting
		lut_weighting_high = total_index - floor(total_index);
		lut_weighting_low = 1 - lut_weighting_high;

		// Determine LUT position indexes
		if(reversed) {
			lut_index[0] = NUM_SAMPLES - 1 - lut_sample_index; // Low LUT index

			if(lut_sample_index == NUM_SAMPLES - 1) {
				lut_index[1] = NUM_SAMPLES - 1; // Wrap around
			}
			else {
				lut_index[1] = lut_index[0] - 1;
			}
		}
		else {
			lut_index[0] = lut_sample_index; // Low LUT index

			if(lut_sample_index == NUM_SAMPLES - 1) {
				lut_index[1] = 0; // Wrap around
			}
			else {
				lut_index[1] = lut_index[0] + 1;
			}
		}
	}

	return multiplier * (lut_weighting_low * SQUARE_WAVE_LUT[lut_index[0]] +
			lut_weighting_high * SQUARE_WAVE_LUT[lut_index[1]]);
}


float sawtooth_wave( float total_index ) {
	float lut_weighting_low = 0.0;
	float lut_weighting_high = 0.0;

	bool reversed = true;
	int multiplier = 1;
	int lut_index[2] = { -1, -1 };

	// Determine which sample index we're looking at
	int lut_sample_index = (int)floor(total_index) % NUM_SAMPLES;

	// Determine which quadrant we're in
//	switch( (int)floor(total_index/ NUM_SAMPLES) % 2)
//	{
//	case 0:
//		// No changes to index
//		//reversed = false;
//		multiplier = 1;
//		break;
//
//	case 1:
//		// Reverse index, negative value
//		//reversed = false;
//		multiplier = -1;
//		break;
//		//
//		//	case 2:
//		//		// Negative value
//		//		reversed = false;
//		//		multiplier = -1;
//		//
//		//		break;
//		//
//		//	case 3:
//		//		// Reverse index
//		//		reversed = true;
//		//		multiplier = -1;
//		//		break;
//	}

	// If index is an integer value, return index
	if(total_index == floor(total_index)) {
		if(reversed) {
			return multiplier * SAWTOOTH_WAVE_LUT[NUM_SAMPLES - lut_sample_index - 1];
		}
		else {
			return multiplier * SAWTOOTH_WAVE_LUT[lut_sample_index];
		}
	}
	else {
		// Calculate element weighting
		lut_weighting_high = total_index - floor(total_index);
		lut_weighting_low = 1 - lut_weighting_high;

		// Determine LUT position indexes
		if(reversed) {
			lut_index[0] = NUM_SAMPLES - 1 - lut_sample_index; // Low LUT index

			if(lut_sample_index == NUM_SAMPLES - 1) {
				lut_index[1] = NUM_SAMPLES - 1; // Wrap around
			}
			else {
				lut_index[1] = lut_index[0] - 1;
			}
		}
		else {
			lut_index[0] = lut_sample_index; // Low LUT index

			if(lut_sample_index == NUM_SAMPLES - 1) {
				lut_index[1] = 0; // Wrap around
			}
			else {
				lut_index[1] = lut_index[0] + 1;
			}
		}
	}

	return multiplier * (lut_weighting_low * SAWTOOTH_WAVE_LUT[lut_index[0]] +
			lut_weighting_high * SAWTOOTH_WAVE_LUT[lut_index[1]]);
}
