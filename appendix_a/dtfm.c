#include <stdint.h>
#include "dtfm.h"

const char match_values[DTFM_NUM_ROWS][DTFM_NUM_COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

char determine_character(float dtfm_freq_one, float dtfm_freq_two) {

  uint16_t boundries[DTFM_NUM_TONES]  = { 730, 810, 900, 1050, 1270, 1400, 1550, 1800 };
  uint16_t dtfm_freqs[DTFM_NUM_TONES] = { 697, 770, 852, 941,  1209, 1336, 1477, 1633 };
  uint16_t dtfm_freq_one_num, dtfm_freq_two_num, dtfm_low_freq, dtfm_high_freq;
  uint8_t i;

  // Determine what expected frequencies were
  for(i=0; i<DTFM_NUM_TONES; i++) {
    if(dtfm_freq_one < boundries[i]) {
      dtfm_freq_one_num = i;
      break;
    }
  }

  for(i=0; i<DTFM_NUM_TONES; i++) {
    if(dtfm_freq_two < boundries[i]) {
      dtfm_freq_two_num = i;
      break;
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

  return match_values[dtfm_low_freq][dtfm_high_freq];
}
