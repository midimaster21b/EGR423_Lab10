///////////////////////////////////////////////////////////////////////
// Filename: ISRs.c
//
// Synopsis: Interrupt service routines for OMAP-L138 EDMA
//
///////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include "math.h"
#include "DSP_Config.h"
#include "config.h"
#include "frames.h"
#include "fft.h"
#include "waveforms.h"
#include "dtfm.h"

#pragma DATA_SECTION (buffer, "CE0"); // allocate buffers in SDRAM
Int16 buffer[NUM_BUFFERS][BUFFER_LENGTH];

// there are 3 buffers in use at all times, one being filled from the McBSP,
// one being operated on, and one being emptied to the McBSP
// ready_index --> buffer ready for processing
volatile Int16 buffer_ready = 0, over_run = 0, ready_index = 0;

// values used for EDMA channel initialization
#define EDMA_CONFIG_RX_OPTION				0x00100000	// TCINTEN, event 0
#define EDMA_CONFIG_RX_SRC_ADDR				((Uint32)(&(McASP0_Base->rbuf[12])))
#define EDMA_CONFIG_RX_SRC_DEST_B_INDEX		((4 << 16) + 0)	// src_b_index = 0, dest_b_index = 4
#define EDMA_CONFIG_RX_A_B_COUNT			((BUFFER_COUNT << 16) + 4)	// 4-byte transfers
#define EDMA_CONFIG_TX_OPTION				0x00101000	// TCINTEN, event 1
#define EDMA_CONFIG_TX_DEST_ADDR			((Uint32)(&(McASP0_Base->xbuf[11])))
#define EDMA_CONFIG_TX_SRC_DEST_B_INDEX		((0 << 16) + 4)	// src_b_index = 4, dest_b_index = 0
#define EDMA_CONFIG_TX_A_B_COUNT			EDMA_CONFIG_RX_A_B_COUNT
#define EDMA_CONFIG_EVENT_MASK				3	// using events 0 (rx) and 1 (tx)
#define EDMA_CONFIG_INTERRUPT_MASK			1	// interrupt on rx reload only

extern COMPLEX Twiddle_Factors[];

#pragma DATA_SECTION (Input_Total, "CE0"); // allocate buffers in SDRAM
static COMPLEX Input_Total[BUFFER_COUNT] = { 0 }; // Summed left & right inputs

#pragma DATA_SECTION (Output_Magnitude_Total, "CE0"); // allocate buffers in SDRAM
static float Output_Magnitude_Total[BUFFER_COUNT] = { 0 };

uint16_t max_peak = 0;

char detected_char = '0';

/* ENCODER GLOBALS */
#define LEFT  0
#define RIGHT 1

volatile union {
  Uint32 UINT;
  Int16 Channel[2];
} CodecDataIn, CodecDataOut;

/* add any global variables here */
static float output_frequencies[NUM_OUTPUT_FREQS] = {1000.0, 1300.0};
static float running_waveform_indices[NUM_OUTPUT_FREQS] = { 0.0 };
static float output_gain = 15000;

void EDMA_Init()
////////////////////////////////////////////////////////////////////////
// Purpose:   Configure EDMA controller to perform all McASP servicing.
//            EDMA is setup so buffer[2] is outbound to McASP, buffer[0] is
//            available for processing, and buffer[1] is being loaded.
//            Both the EDMA transmit and receive events are set to automatically
//            reload upon completion, cycling through the 3 buffers.
//            The EDMA completion interrupt occurs when a buffer has been filled
//            by the EDMA from the McASP.
//            The EDMA interrupt service routine updates the ready buffer index,
//            and sets the buffer ready flag which is being polled by the main
//            program loop
//
// Input:     None
//
// Returns:   Nothing
//
// Calls:     Nothing
//
// Notes:     None
///////////////////////////////////////////////////////////////////////
{
  EDMA_params* param;

  // McASP tx event params
  param = (EDMA_params*)EDMA3_0_PARAM(EDMA3_EVENT_MCASP0_TX);
  param->option = EDMA_CONFIG_TX_OPTION;
  param->source = (Uint32)(&buffer[2][0]);
  param->a_b_count = EDMA_CONFIG_TX_A_B_COUNT;
  param->dest = EDMA_CONFIG_TX_DEST_ADDR;
  param->src_dest_b_index = EDMA_CONFIG_TX_SRC_DEST_B_INDEX;
  param->link_reload = (BUFFER_COUNT << 16) + (EDMA3_0_PARAM(64) & 0xFFFF);
  param->c_count = 1;

  // set up first tx link param
  param = (EDMA_params*)EDMA3_0_PARAM(64);
  param->option = EDMA_CONFIG_TX_OPTION;
  param->source = (Uint32)(&buffer[0][0]);
  param->a_b_count = EDMA_CONFIG_TX_A_B_COUNT;
  param->dest = EDMA_CONFIG_TX_DEST_ADDR;
  param->src_dest_b_index = EDMA_CONFIG_TX_SRC_DEST_B_INDEX;
  param->link_reload = (BUFFER_COUNT << 16) + (EDMA3_0_PARAM(65) & 0xFFFF);
  param->c_count = 1;

  // set up second tx link param
  param = (EDMA_params*)EDMA3_0_PARAM(65);
  param->option = EDMA_CONFIG_TX_OPTION;
  param->source = (Uint32)(&buffer[1][0]);
  param->a_b_count = EDMA_CONFIG_TX_A_B_COUNT;
  param->dest = EDMA_CONFIG_TX_DEST_ADDR;
  param->src_dest_b_index = EDMA_CONFIG_TX_SRC_DEST_B_INDEX;
  param->link_reload = (BUFFER_COUNT << 16) + (EDMA3_0_PARAM(66) & 0xFFFF);
  param->c_count = 1;

  // set up third tx link param
  param = (EDMA_params*)EDMA3_0_PARAM(66);
  param->option = EDMA_CONFIG_TX_OPTION;
  param->source = (Uint32)(&buffer[2][0]);
  param->a_b_count = EDMA_CONFIG_TX_A_B_COUNT;
  param->dest = EDMA_CONFIG_TX_DEST_ADDR;
  param->src_dest_b_index = EDMA_CONFIG_TX_SRC_DEST_B_INDEX;
  param->link_reload = (BUFFER_COUNT << 16) + (EDMA3_0_PARAM(64) & 0xFFFF);
  param->c_count = 1;


  // McASP rx event params
  param = (EDMA_params*)(EDMA3_0_PARAM(EDMA3_EVENT_MCASP0_RX));
  param->option = EDMA_CONFIG_RX_OPTION;
  param->source = EDMA_CONFIG_RX_SRC_ADDR;
  param->a_b_count = EDMA_CONFIG_RX_A_B_COUNT;
  param->dest = (Uint32)(&buffer[1][0]);
  param->src_dest_b_index = EDMA_CONFIG_RX_SRC_DEST_B_INDEX;
  param->link_reload = (BUFFER_COUNT << 16) + (EDMA3_0_PARAM(67) & 0xFFFF);
  param->c_count = 1;

  // set up first rx link param
  param = (EDMA_params*)EDMA3_0_PARAM(67);
  param->option = EDMA_CONFIG_RX_OPTION;
  param->source = EDMA_CONFIG_RX_SRC_ADDR;
  param->a_b_count = EDMA_CONFIG_RX_A_B_COUNT;
  param->dest = (Uint32)(&buffer[2][0]);
  param->src_dest_b_index = EDMA_CONFIG_RX_SRC_DEST_B_INDEX;
  param->link_reload = (BUFFER_COUNT << 16) + (EDMA3_0_PARAM(68) & 0xFFFF);
  param->c_count = 1;

  // set up second rx link param
  param = (EDMA_params*)EDMA3_0_PARAM(68);
  param->option = EDMA_CONFIG_RX_OPTION;
  param->source = EDMA_CONFIG_RX_SRC_ADDR;
  param->a_b_count = EDMA_CONFIG_RX_A_B_COUNT;
  param->dest = (Uint32)(&buffer[0][0]);
  param->src_dest_b_index = EDMA_CONFIG_RX_SRC_DEST_B_INDEX;
  param->link_reload = (BUFFER_COUNT << 16) + (EDMA3_0_PARAM(69) & 0xFFFF);
  param->c_count = 1;

  // set up third rx link param
  param = (EDMA_params*)EDMA3_0_PARAM(69);
  param->option = EDMA_CONFIG_RX_OPTION;
  param->source = EDMA_CONFIG_RX_SRC_ADDR;
  param->a_b_count = EDMA_CONFIG_RX_A_B_COUNT;
  param->dest = (Uint32)(&buffer[1][0]);
  param->src_dest_b_index = EDMA_CONFIG_RX_SRC_DEST_B_INDEX;
  param->link_reload = (BUFFER_COUNT << 16) + (EDMA3_0_PARAM(67) & 0xFFFF);
  param->c_count = 1;

  // configure EDMA to start servicing events
  *(volatile Uint32 *)EDMA3_0_CC_ECR  = EDMA_CONFIG_EVENT_MASK;	// clear pending events
  *(volatile Uint32 *)EDMA3_0_CC_EESR = EDMA_CONFIG_EVENT_MASK;	// enable events
  *(volatile Uint32 *)EDMA3_0_CC_DRAE1 = EDMA_CONFIG_EVENT_MASK;	// enable events for region 1
  *(volatile Uint32 *)EDMA3_0_CC_IESR = EDMA_CONFIG_INTERRUPT_MASK;	// enable CPU interrupt
}

void ZeroBuffers()
////////////////////////////////////////////////////////////////////////
// Purpose:   Sets all buffer locations to 0
//
// Input:     None
//
// Returns:   Nothing
//
// Calls:     Nothing
//
// Notes:     None
///////////////////////////////////////////////////////////////////////
{
  Int32 i = BUFFER_COUNT * NUM_BUFFERS;
  Int32 *p = (Int32 *)buffer;

  while(i--)
    *p++ = 0;
}

void ProcessBuffer(COMPLEX *twiddle_factors)
///////////////////////////////////////////////////////////////////////
// Purpose:   Processes the data in buffer[ready_index] and stores
//		  the results back into the buffer
//            Data is packed into the buffer, alternating right/left
//
// Input:     None
//
// Returns:   Nothing
//
// Calls:     Nothing
//
// Notes:     None
///////////////////////////////////////////////////////////////////////
{
  Int16 *pBuf = buffer[ready_index];
  Int32 i;

  // Used for peak finding
  uint16_t num_peaks = 0;
  uint16_t peakIndices[BUFFER_COUNT/2] = { 0 };

  float real_component, imag_component;

  WriteDigitalOutputs(0); // set digital outputs low - for time measurement

  // Extract data from signal
  for(i = 0;i < BUFFER_COUNT;i++) { // extract data to float buffers

    // Sum left and right signal
    /* Input_Total[i].re = *pBuf + *(pBuf + 1); */

    Input_Total[i].re = *pBuf;
    Input_Total[i].im = 0.0;

    pBuf++;
    pBuf++;
  }


  /********* END PRE FFT *********/

  // Compute FFT's
  fft_c(BUFFER_COUNT, Input_Total, twiddle_factors);  // Input Total

  /********* BEGIN POST FFT *********/

  // Calculate magnitudes of FFT
  for(i = 0;i < BUFFER_COUNT;i++) {
    real_component = Input_Total[i].re * Input_Total[i].re;
    imag_component = Input_Total[i].im * Input_Total[i].im;
    Output_Magnitude_Total[i] = pow( real_component + imag_component, 0.5);
  }

  // Find all peaks (Identified by being greater than both neighboring magnitudes
  for(i=1; i < BUFFER_COUNT/2; i++) {
    if(Output_Magnitude_Total[i] > Output_Magnitude_Total[i-1] && Output_Magnitude_Total[i] > Output_Magnitude_Total[i+1]) {
      peakIndices[num_peaks] = i;
      num_peaks += 1;
    }
  }

  uint16_t j = 0;
  float localMax = 0;
  uint16_t localMaxIndex = 0;
  uint16_t localMaxTemp = 0;

  // Bubble sort... cause it's quick and simple... to write...
  for(j=0; j < num_peaks; j++) {
    localMax = 0;
    localMaxIndex = 0;

    // Find max
    for(i=j; i < num_peaks; i++) {
      if(Output_Magnitude_Total[peakIndices[i]] > localMax) {
	localMax = Output_Magnitude_Total[peakIndices[i]];
	localMaxIndex = i;
      }
    }

    // Swap with jth item
    localMaxTemp = peakIndices[j];
    peakIndices[j] = peakIndices[localMaxIndex];
    peakIndices[localMaxIndex] = localMaxTemp;
  }

  if(peakIndices[0] != max_peak) {
    max_peak = peakIndices[0];
    /* printf("New peak index detected: %d\n", max_peak); */
  }

  float dtfm_freq_one = peakIndices[0] * (SAMPLING_FREQUENCY / NUM_FFT_SAMPLES);
  float dtfm_freq_two = peakIndices[1] * (SAMPLING_FREQUENCY / NUM_FFT_SAMPLES);

  detected_char = determine_character(dtfm_freq_one, dtfm_freq_two);

  /* Your code should be done by here */
  WriteDigitalOutputs(1); // set digital output bit 0 high - for time measurement
  buffer_ready = 0; // signal we are done
}

///////////////////////////////////////////////////////////////////////
// Purpose:   Access function for buffer ready flag
//
// Input:     None
//
// Returns:   Non-zero when a buffer is ready for processing
//
// Calls:     Nothing
//
// Notes:     None
///////////////////////////////////////////////////////////////////////
int IsBufferReady()
{
  return buffer_ready;
}

///////////////////////////////////////////////////////////////////////
// Purpose:   Access function for buffer overrun flag
//
// Input:     None
//
// Returns:   Non-zero if a buffer overrun has occurred
//
// Calls:     Nothing
//
// Notes:     None
///////////////////////////////////////////////////////////////////////
int IsOverRun()
{
  return over_run;
}

interrupt void EDMA_ISR()
///////////////////////////////////////////////////////////////////////
// Purpose:   EDMA interrupt service routine.  Invoked on every buffer
//            completion
//
// Input:     None
//
// Returns:   Nothing
//
// Calls:     Nothing
//
// Notes:     None
///////////////////////////////////////////////////////////////////////
{
  *(volatile Uint32 *)EDMA3_0_CC_ICR = EDMA_CONFIG_INTERRUPT_MASK; // clear interrupt
  if(++ready_index >= NUM_BUFFERS) // update buffer index
    ready_index = 0;
  if(buffer_ready == 1) // set a flag if buffer isn't processed in time
    over_run = 1;
  buffer_ready = 1; // mark buffer as ready for processing
}

interrupt void Codec_ISR()
///////////////////////////////////////////////////////////////////////
// Purpose:   Codec interface interrupt service routine
//
// Input:     None
//
// Returns:   Nothing
//
// Calls:     CheckForOverrun, ReadCodecData, WriteCodecData
//
// Notes:     None/
//////////////////////////////////////////////////////////////////////
{
  /* add any local variables here */
  float output_signal = 0.0;
  uint8_t i=0;
  float waveform_step_size;

  if(CheckForOverrun())	// overrun error occurred (i.e. halted DSP)
    return;             // so serial port is reset to recover

  CodecDataIn.UINT = ReadCodecData(); // THIS LINE IS CRUCIAL. WILL NOT RUN WITHOUT.

  for(i=0; i<NUM_OUTPUT_FREQS; i++) {
    // Calculate the appropriate step size
    waveform_step_size = output_frequencies[i] / SAMPLED_LUT_FREQUENCY;

    // Add the current freq signal to the total output signal
    output_signal += sine_wave(running_waveform_indices[i]);

    // Increment by step size
    running_waveform_indices[i] += waveform_step_size;

    // Wrap around if index goes too high
    if(running_waveform_indices[i] >= MAX_WAVEFORM_INDEX) {
      running_waveform_indices[i] -= MAX_WAVEFORM_INDEX;
    }
  }

  output_signal *= output_gain;

  CodecDataOut.Channel[ LEFT] = output_signal;
  CodecDataOut.Channel[RIGHT] = output_signal;

  /* end your code here */

  WriteCodecData(CodecDataOut.UINT);		// send output data to port
}
