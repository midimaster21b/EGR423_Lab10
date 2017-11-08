// Welch, Wright, & Morrow, 
// Real-time Digital Signal Processing, 2011

///////////////////////////////////////////////////////////////////////
// Filename: frames.h
//
// Synopsis: Frame buffering/processing function declarations
//
///////////////////////////////////////////////////////////////////////

#include "fft.h"

// Necessary definitions
// frame buffer declarations
#define BUFFER_COUNT		1024   // buffer length in McASP samples (L+R)
#define BUFFER_LENGTH		BUFFER_COUNT*2 // two Int16 read from McASP each time
#define NUM_BUFFERS		3     // don't change this!
#define SAMPLING_FREQ           48000.0
#define SAMPLE_WINDOW           20
#define NUM_PEAKS               2

// defined in ISRs.c
void ZeroBuffers();
void ProcessBuffer(COMPLEX *twiddle_factors);
int IsBufferReady();
int IsOverRun();
void EDMA_Init();

