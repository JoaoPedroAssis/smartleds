#pragma once

#include <stdint.h>
#include <Arduino.h>
#include <string.h>
#include <ESP32_fft.h>

#define SAMPLES 1024
#define BUFFER_SIZE 4
#define SAMPLING_FREQUENCY 44100
#define NUM_BANDS 7


class SoundAnalyzer {
    private:

        double bands[NUM_BANDS];
        size_t sizeof_bands = 7 * sizeof(double);
        fft_config_t * real_fft;
        float max_mag = 0;
        float fundamental_freq = 0;
        float _bin_width = (double) SAMPLING_FREQUENCY / (double) SAMPLES;
        size_t frequency_cutoff_idx = 232; // idx * bin_width = 10kHz
        float scale_magnitude = 0.3f;
        int peak_idx = 0;

        // FFT related stuff
        void clear_data();
        void FFT();
        void hamming_window();
        void complex_to_magnitude();
        int idx_to_band(int);
     

    public:

        SoundAnalyzer();
        ~SoundAnalyzer();

        float bin_width()        { return _bin_width; }
        float frequency(int i)   { return i * _bin_width; }
        float major_peak()       { return max_mag; }
        float major_peak_req()   { return fundamental_freq; }
        int major_peak_idx()   { return peak_idx; }

        // Copies the sample_data into input_buffer array
        void set_data(const float*, size_t);
        
        double * get_bands();

        // Utility functions
        void min_max(double *, size_t, double *, double *);
        double mapd(double, double, double, double, double);
        void normalize(double*, size_t, double, double);
};