////////////////////////////////////////////////////////////////
// Filename: fft.c
//
// Synopsis: Example FFT routine. Can run on nearly any CPU that
//   supports a C compiler. This code calculates the FFT 
//   (decimation-in-time) of an N point complex data sequence.
//
//   This algorithm is based on the discussion in
//   "C Algorithms for Real-Time DSP", by Paul M. Embree
//   Prentice-Hall PTR, copyright 1995.
//
////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "fft.h"
/* int main(argc,argv) */
/* int argc; */
/* char *argv[]; */
/* { */
/*     int M, N, i; */
/*     COMPLEX *W;  // for twiddle factors */
/*     COMPLEX *x;  // for input data */

/*     if(argc != 2) */
/*     { */
/* 	fprintf(stderr,USE,argv[0]); */
/* 	exit(-1); */
/*     } */
/*     M = atoi(argv[1]); */
/*     N = pow(2.0, M); */

/*     W = malloc(N*sizeof(COMPLEX)); */
/*     x = malloc(N*sizeof(COMPLEX)); */

/*     // read in data */
/*     for(i = 0; i < N; i++) */
/*     { */
/* 	    scanf("%f", &x[i].re); */
/* 	    x[i].im = 0.0; */
/*     } */

/*     // initialize real and imaginary parts of the twiddle factors  */
/*     init_W(N, W); */
    
/*     // calculate the FFT of x[N]  */
/*     fft_c(N, x, W); */

/*     // display the results   */
/*     printf("\n  i    real part    imag part \n\n"); */
/*     for(i = 0 ; i < N ; i++)  */
/*         printf("%3d %12.5f %12.5f \n", i, x[i].re, x[i].im); */
/*     printf("\n"); */

/*     return(0); */
/* } */

void fft_c(int n, COMPLEX *x, COMPLEX *W)
///////////////////////////////////////////////////////////////////////
// Purpose:   Calculate the radix-2 decimation-in-time FFT.
//
// Input:     n: length of FFT, x: input array of complex numbers,
//            W: array of precomputed twiddle factors
//
// Returns:   values in array x are replaced with result
//
// Calls:     Nothing
//
// Notes:     Bit-reversed address reordering of the sequence 
//            is performed in this function.
///////////////////////////////////////////////////////////////////////
{
    COMPLEX u, temp, tm;
    COMPLEX *Wptr;

    int i, j, k, len, Windex; 
    
    // perform fft butterfly
    Windex = 1;
    for(len = n/2 ; len > 0 ; len /= 2) {
        Wptr = W;
        for (j = 0 ; j < len ; j++) {
            u = *Wptr;
            for (i = j ; i < n ; i = i + 2*len) {
                temp.re = x[i].re + x[i+len].re;
                temp.im = x[i].im + x[i+len].im;
                tm.re = x[i].re - x[i+len].re;
                tm.im = x[i].im - x[i+len].im;             
                x[i+len].re = tm.re*u.re - tm.im*u.im;
                x[i+len].im = tm.re*u.im + tm.im*u.re;
                x[i] = temp;
            }
            Wptr = Wptr + Windex;
        }
        Windex = 2*Windex;
    }
    
    // rearrange data by bit reversed addressing 
    // this step must occur after the fft butterfly
    j = 0;
    for (i = 1; i < (n-1); i++) {
        k = n/2;
        while(k <= j) {
            j -= k;
            k /= 2;
        }
        j += k;
        if (i < j) {
            temp = x[j];
            x[j] = x[i];
            x[i] = temp;
        }
    }
    
}  // end of fft_c function

void init_W(int n, COMPLEX *W)
///////////////////////////////////////////////////////////////////////
// Purpose:   Calculate the twiddle factors needed by the FFT.
//
// Input:     n: length of FFT, W: array to store twiddle factors
//
// Returns:   values are stored in array W
//
// Calls:     Nothing
//
// Notes:     Floats used rather than double as this is intended
//            for a DSP CPU target.  Could change to double.
///////////////////////////////////////////////////////////////////////
{
    int i;

    float a = 2.0*MYPI/n;

    for(i = 0 ; i < n ; i++) {
        W[i].re = (float) cos(-i*a);
        W[i].im = (float) sin(-i*a);
    }
}
