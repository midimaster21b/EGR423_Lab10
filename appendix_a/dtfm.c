#include <stdint.h>
#include "dtfm.h"

char determine_character(float dtfm_freq_one, float dtfm_freq_two) {

  char match_values[DTFM_NUM_ROWS][DTFM_NUM_COLS];

  match_values[0][0] = '1';
  match_values[0][1] = '2';
  match_values[0][2] = '3';
  match_values[0][3] = 'A';
  match_values[1][0] = '4';
  match_values[1][1] = '5';
  match_values[1][2] = '6';
  match_values[1][3] = 'B';
  match_values[2][0] = '7';
  match_values[2][1] = '8';
  match_values[2][2] = '9';
  match_values[2][3] = 'C';
  match_values[3][0] = '*';
  match_values[3][1] = '0';
  match_values[3][2] = '#';
  match_values[3][3] = 'D';

  uint16_t boundries[DTFM_NUM_TONES]  = { 730, 810, 900, 1050, 1270, 1400, 1550, 1800 };
  /* float dtfm_freqs[DTFM_NUM_TONES] = { 697, 770, 852, 941,  1209, 1336, 1477, 1633 }; */
  float dtfm_freqs[DTFM_NUM_TONES];
  uint16_t dtfm_freq_one_num, dtfm_freq_two_num, dtfm_low_freq, dtfm_high_freq;
  float dtfm_margin;
  uint8_t i;

  dtfm_freqs[0] = 697.0;
  dtfm_freqs[1] = 770.0;
  dtfm_freqs[2] = 852.0;
  dtfm_freqs[3] = 941.0;
  dtfm_freqs[4] = 1209.0;
  dtfm_freqs[5] = 1336.0;
  dtfm_freqs[6] = 1477.0;
  dtfm_freqs[7] = 1633.0;

  // Determine what expected frequencies were
  for(i=0; i<DTFM_NUM_TONES; i++) {
    dtfm_margin =  0.035 * dtfm_freqs[i];

    if(dtfm_freq_one < dtfm_freqs[i] + dtfm_margin && dtfm_freq_one > dtfm_freqs[i] - dtfm_margin) {
      dtfm_freq_one_num = i;
      break;
    }

    if(i == DTFM_NUM_TONES - 1) {
      return '\0';
    }
  }

  for(i=0; i<DTFM_NUM_TONES; i++) {
    dtfm_margin =  0.035 * dtfm_freqs[i];

    if(dtfm_freq_two < dtfm_freqs[i] + dtfm_margin && dtfm_freq_two > dtfm_freqs[i] - dtfm_margin) {
      dtfm_freq_two_num = i;
      break;
    }

    if(i == DTFM_NUM_TONES - 1) {
      return '\0';
    }
  }

  // Determine low and high freqs
  if(dtfm_freq_one_num < dtfm_freq_two_num) {
    dtfm_low_freq = dtfm_freq_one_num;
    dtfm_high_freq = dtfm_freq_two_num;
  }
  else {
    dtfm_low_freq = dtfm_freq_two_num;
    dtfm_high_freq = dtfm_freq_one_num;
  }

  // No approriate value to return
  if(dtfm_low_freq >= 4 || dtfm_high_freq < 4) {
    return '\0';
  }

  return match_values[dtfm_low_freq][dtfm_high_freq - 4];
}
