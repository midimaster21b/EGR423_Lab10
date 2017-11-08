// Welch, Wright, & Morrow,
// Real-time Digital Signal Processing, 2011

///////////////////////////////////////////////////////////////////////
// Filename: main.c
//
// Synopsis: Main program file for frame-based processing using EDMA
//
///////////////////////////////////////////////////////////////////////

#include "DSP_Config.h"
#include "frames.h"
#include "fft.h"

#define NUM_TWIDDLE_FACTORS BUFFER_COUNT

COMPLEX Twiddle_Factors[NUM_TWIDDLE_FACTORS] = { 0 };

int main()
{
  // initialize all buffers to 0
  ZeroBuffers();

  // Compute twiddle factors
  init_W(NUM_TWIDDLE_FACTORS, Twiddle_Factors);

  // initialize EDMA controller
  EDMA_Init();

  // initialize DSP for EDMA operation
  DSP_Init_EDMA();

  // call to StartUp not needed here

  // main loop here, process buffer when ready
  while(1) {
    if(IsBufferReady()) // process buffers in background
      ProcessBuffer(Twiddle_Factors);
  }
}
