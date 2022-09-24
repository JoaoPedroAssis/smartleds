#include "SoundAnalyzer.h"


SoundAnalyzer::SoundAnalyzer() {
    real_fft = fft_init(SAMPLES, FFT_REAL, FFT_FORWARD, NULL, NULL);
}

SoundAnalyzer::~SoundAnalyzer() {
    fft_destroy(real_fft);
}

void SoundAnalyzer::set_data(const float* sample_data, size_t byte_size) {
    memcpy(real_fft->input, sample_data, SAMPLES * sizeof(float));
}

void SoundAnalyzer::clear_data() {
    max_mag = 0;
    fundamental_freq = 0;
    peak_idx = 0;
    memset(bands, 0, sizeof_bands);
}

void SoundAnalyzer::hamming_window() {
    float samplesMinusOne = (float(real_fft->size) - 1.0);
    for (uint16_t i = 0; i < (real_fft->size >> 1); i++) {
        float indexMinusOne = float(i);
        float ratio = (indexMinusOne / samplesMinusOne);
        float weighingFactor = 0.54 - (0.46 * cos(TWO_PI * ratio));
        real_fft->input[i] *= weighingFactor;
        real_fft->input[real_fft->size - (i + 1)] *= weighingFactor;
    }
}

void SoundAnalyzer::complex_to_magnitude() {

    for (int k = 1 ; k < real_fft->size/ 2 ; k++) {
        float mag = sqrt(pow(real_fft->output[2*k],2) + pow(real_fft->output[2*k+1],2))/1;
        float freq = frequency(k);
        if(mag > max_mag) {
            max_mag = mag;
            fundamental_freq = freq;
            peak_idx = k;
        }
        real_fft->output[k] = mag;
    }
}

double * SoundAnalyzer::get_bands() {
    
    clear_data();
    FFT();

    for (int i = 0; i < frequency_cutoff_idx; i++) {
      bands[idx_to_band(i)] += scale_magnitude * real_fft->output[i];
    }

    double min, max;
    min_max(bands, 7, &min, &max);
    normalize(bands, 7, min, max);

    return bands;
}

void SoundAnalyzer::FFT() {
    hamming_window();
    fft_execute(real_fft);
    complex_to_magnitude();    
}

int SoundAnalyzer::idx_to_band(int idx) {

  if (idx <= 2)   return 0; // 60    Hz
  if (idx <= 6)   return 1; // 250   Hz
  if (idx <= 12)  return 2; // 500   Hz
  if (idx <= 47)  return 3; // 2000  Hz
  if (idx <= 93)  return 4; // 4000  Hz
  if (idx <= 140) return 5; // 6000  Hz
  if (idx <= 232) return 6; // 10000 Hz

  return 6;
}

void SoundAnalyzer::min_max(double * array, size_t size, double * min, double * max) {

  double ma = array[0];
  double mi = array[0];

  for (int i = 1; i < size; i++) {
    if (array[i] > ma) {
      ma = array[i];
    } else if (array[i] < mi) {
      mi = array[i];
    }
  }

  *min = mi;
  *max = ma;
}

double SoundAnalyzer::mapd(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void SoundAnalyzer::normalize(double * array, size_t size, double min, double max) {
  for (int i = 0; i < size; i++) {
    array[i] = mapd(array[i], min, max, 0, 1);
  }
}