#pragma once

#include <stdint.h>
#include <arduinoFFT.h>

#define NUM_BANDS 16
#define SAMPLES 1024
#define BUFFER_SIZE 4
#define SAMPLING_FREQUENCY 44100
#define NOISE 500

static int cutOffs16Band[16] =
{
	100, 250, 450, 565, 715, 900, 1125, 1400, 1750, 2250, 2800, 3150, 4000, 5000, 6400, 12500
};

class SoundAnalyzer {
    private:
    
        double * vReal;
        double * vImag;
        float * peaks;
        arduinoFFT _FFT;
        float amplitude = 200.0;

        void FFT() {
            _FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
            _FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
            _FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
        }

        int bucketFrequency(int i) const {
            if (i <= 1)
                return 0;

            int iOffset = i - 2;
            return iOffset * (SAMPLING_FREQUENCY / 2) / (SAMPLES / 2);
	    }

        int getBand(int i) {

            // if (i <= 4)     return 0; // 20Hz
            // if (i <= 5)     return 1; // 25Hz
            // if (i <= 7)     return 2; // 500Hz
            // if (i <= 8)     return 3; // 1000Hz
            // if (i <= 10)    return 4; // 2000Hz
            // if (i <= 13)    return 5; // 4000Hz
            // if (i <= 16)    return 6; // 8000Hz
            // if (i <= 20)    return 7; // 20Hz
            // if (i <= 25)    return 8; // 25Hz
            // if (i <= 31)    return 9; // 20Hz
            // if (i <= 39)    return 10; // 25Hz
            // if (i <= 48)    return 11; // 500Hz
            // if (i <= 61)    return 12; // 1000Hz
            // if (i <= 76)    return 13; // 2000Hz
            // if (i <= 95)    return 14; // 4000Hz
            // if (i <= 118)   return 15; // 8000Hz
            // if (i <= 148)   return 16; // 20Hz
            // if (i <= 185)   return 17; // 25Hz
            // if (i <= 231)   return 18; // 20Hz
            // if (i <= 288)   return 19; // 25Hz
            // if (i <= 360)   return 20; // 500Hz
            // if (i <= 450)   return 21; // 1000Hz
            // if (i <= 562)   return 22; // 2000Hz
            // if (i <= 703)   return 23; // 4000Hz
            // if (i <= 878)   return 24; // 8000Hz
            // if (i <= 1098)  return 25; // 20Hz
            // if (i <= 1371)  return 26; // 25Hz
            // if (i <= 1714)  return 27; // 20Hz
            // if (i <= 2142)  return 28; // 25Hz
            // if (i <= 2676)  return 29; // 500Hz
            // if (i <= 3344)  return 30; // 1000Hz
            // if (i <= 4179)  return 31; // 2000Hz


            if (i <= 1)     return 0; // 20Hz
            if (i <= 1)     return 1; // 25Hz
            if (i <= 2)     return 2; // 500Hz
            if (i <= 2)     return 3; // 1000Hz
            if (i <= 4)     return 4; // 2000Hz
            if (i <= 6)     return 5; // 4000Hz
            if (i <= 10)    return 6; // 8000Hz
            if (i <= 15)    return 7; // 20Hz
            if (i <= 24)    return 8; // 25Hz
            if (i <= 38)    return 9; // 20Hz
            if (i <= 60)    return 10; // 25Hz
            if (i <= 95)    return 11; // 500Hz
            if (i <= 151)   return 12; // 1000Hz
            if (i <= 239)   return 13; // 2000Hz
            if (i <= 379)   return 14; // 4000Hz
            if (i <= 600)   return 15; // 8000Hz
            else return 0;
        }

        void createBands() {
            for (uint8_t band = 0; band < NUM_BANDS; band++) {
                peaks[band] = 0;
            }

            for (int i = 2; i < (SAMPLES / 2); i++) {
                if (vReal[i] > NOISE) { // Add a crude noise filter, 10 x amplitude or more
                    peaks[getBand(i)] = (int) vReal[i];
                }
            }
        }

    public:

        SoundAnalyzer() {
            vReal = (double *) malloc(SAMPLES * sizeof(vReal[0]));
            vImag = (double *) malloc(SAMPLES * sizeof(vImag[0]));
            peaks = (float *)  malloc(NUM_BANDS * sizeof(peaks[0]));
        }

        ~SoundAnalyzer() {
            free(vReal);
            free(vImag);
            free(peaks);
        }

        // Copies the sample_data into vReal array and clears vImag
        void setData(const double* sample_data) {
            memcpy(vReal, sample_data, SAMPLES * sizeof(double));
            for (int i = 0; i < SAMPLES; i++) {
                vImag[i] = 0.0;
            }
        }
        
        int getPeak() {

            FFT();
            createBands();

            float max_peak = peaks[0];
            int max_peak_idx = 0;
            for (int i = 0; i < NUM_BANDS; i++) {
                if (peaks[i] > max_peak) {
                    max_peak = peaks[i];
                    max_peak_idx = i;
                }
            }

            return max_peak_idx;
        }
};