#ifndef FFT_H_INCLUDED
#define FFT_H_INCLUDED

// define the COMPLEX structure
typedef struct {
    float re;
    float im;
} COMPLEX;

// function prototypes
void fft_c(int n, COMPLEX *x, COMPLEX *W);
void init_W(int n, COMPLEX *W);

#define MYPI 3.1415926535897932
#define USE "Usage:%s M(N=2^M) < in_real > out_fft_cmplx\n"
#endif
