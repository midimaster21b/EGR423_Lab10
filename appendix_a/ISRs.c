///////////////////////////////////////////////////////////////////////
// Filename: ISRs.c
//
// Synopsis: Interrupt service routines for OMAP-L138 EDMA
//
///////////////////////////////////////////////////////////////////////

#include "math.h"
#include "DSP_Config.h"
#include "frames.h"
#include "fft.h"

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
static COMPLEX Input_Left[BUFFER_COUNT] = { 0 };
static float Output_Magnitude_Left[BUFFER_COUNT] = { 0 };

#pragma DATA_SECTION (Input_Right, "CE0"); // allocate buffers in SDRAM
static COMPLEX Input_Right[BUFFER_COUNT] = { 0 };

#pragma DATA_SECTION (Output_Magnitude_Right, "CE0"); // allocate buffers in SDRAM
static float Output_Magnitude_Right[BUFFER_COUNT] = { 0 };

#pragma DATA_SECTION (Input_Total, "CE0"); // allocate buffers in SDRAM
static COMPLEX Input_Total[BUFFER_COUNT] = { 0 }; // Summed left & right inputs

#pragma DATA_SECTION (Output_Magnitude_Total, "CE0"); // allocate buffers in SDRAM
static float Output_Magnitude_Total[BUFFER_COUNT] = { 0 };

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
	static float Left[BUFFER_COUNT], Right[BUFFER_COUNT];
	float *pL = Left, *pR = Right;
	Int32 i;

	// Used for peak finding
	float maxL = *pL;
	float maxR = *pR; // Used for p

	// Right values
	float meanR=0;
	float minR=*pR;
	float varianceR=0;
	float averageMagR=0;

	// Left values
	float sumL=0;
	float meanL=0;
	float minL=*pL;
	float varianceL=0;
	float averageMagL=0;

	uint16_t indexR = 0;
	uint16_t indexL = 0;

	float peakR = 0;
	float peakL = 0;

	uint16_t num_peaks = 0;
	uint16_t maxIndex[BUFFER_COUNT/2] = { 0 };

	for(i = 0;i < BUFFER_COUNT;i++) { // extract data to float buffers

	  // Sum left and right signal
	  Input_Total[i].re = *pBuf + *(pBuf + 1);
	  Input_Total[i].im = 0.0;

	  // Store input in real part of COMPLEX typedef before increment
	  Input_Right[i].re = *pBuf;
	  Input_Right[i].im = 0.0;

	  // Increment buffer pointer and right pointer (left and right interwoven)
	  *pR++ = *pBuf++;

	  // Store input in real part of COMPLEX typedef before increment
	  Input_Left[i].re = *pBuf;
	  Input_Left[i].im = 0.0;

	  // Increment buffer pointer and left pointer (left and right interwoven)
	  *pL++ = *pBuf++;

	}

	WriteDigitalOutputs(0); // set digital outputs low - for time measurement

	// reinitialize pointers
	pL = Left;
	pR = Right;

	/********* END PRE FFT *********/

	// Compute FFT's
	fft_c(BUFFER_COUNT, Input_Left, twiddle_factors);   // Input Left
	fft_c(BUFFER_COUNT, Input_Right, twiddle_factors);  // Input Right
	fft_c(BUFFER_COUNT, Input_Total, twiddle_factors);  // Input Total

	/********* BEGIN POST FFT *********/

	// Calculate magnitudes of outputs
	for(i = 0;i < BUFFER_COUNT;i++) {

	  // Calculate signal magnitudes (
	  Output_Magnitude_Total[i] = pow(pow(Input_Total[i].re, 2.0) + pow(Input_Total[i].im, 2.0), 0.5);
	  Output_Magnitude_Left[i] = pow(pow(Input_Left[i].re, 2.0) + pow(Input_Left[i].im, 2.0), 0.5);
	  Output_Magnitude_Right[i] = pow(pow(Input_Right[i].re, 2.0) + pow(Input_Right[i].im, 2.0), 0.5);

	  // Store max values and indices for left input signal
	  maxL = maxL < Output_Magnitude_Left[i] ? Output_Magnitude_Left[i] : maxL;
	  indexL = maxL == Output_Magnitude_Left[i] ? i : indexL;

	  // Store max values and indices for right input signal
	  maxR = maxR < Output_Magnitude_Right[i] ? Output_Magnitude_Right[i] : maxR;
	  indexR = maxR == Output_Magnitude_Right[i] ? i : indexR;
	}

	// If max peak is greater than half way, flip to first half equivalent freq.
	peakR = (indexR > 512 ? 1024 - indexR : indexR) * (SAMPLING_FREQ / BUFFER_COUNT);
	peakL = (indexL > 512 ? 1024 - indexL : indexL) * (SAMPLING_FREQ / BUFFER_COUNT);

	for(i=1; i < BUFFER_COUNT/2; i++) {
	  if(Output_Magnitude_Total[i] > Output_Magnitude_Total[i-1] && Output_Magnitude_Total[i] > Output_Magnitude_Total[i+1]) {
	    maxIndex[num_peaks] = i;
	    num_peaks += 1;
	  }
	}

	uint16_t j = 0;
	uint16_t localMax = 0;
	uint16_t localMaxIndex = 0;
	uint16_t localMaxTemp = 0;

	// Bubble sort... cause it's quick and simple...
	for(j=0; j<2; j++) {
	  localMax = 0;
	  localMaxIndex = 0;

	  // Find max
	  for(i=j; i<num_peaks; i++) {
	    if(Output_Magnitude_Total[maxIndex[i]] > localMax) {
	      localMax = Output_Magnitude_Total[maxIndex[i]];
	      localMaxIndex = i;
	    }
	  }

	  // Swap with jth item
	  localMaxTemp = maxIndex[j];
	  maxIndex[j] = maxIndex[localMaxIndex];
	  maxIndex[localMaxIndex] = localMaxTemp;
	}

	float peakOne = Output_Magnitude_Total[maxIndex[0]];
	float peakTwo = Output_Magnitude_Total[maxIndex[1]];


	/* Your code should be done by here */
	pBuf = buffer[ready_index];
	pL = Left;
	pR = Right;


	for(i = 0;i < BUFFER_COUNT;i++) { // pack into buffer after bounding
		*pBuf++ = _spint(*pR++ * 65536) >> 16;
		*pBuf++ = _spint(*pL++ * 65536) >> 16;
	}

	pBuf = buffer[ready_index];

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
