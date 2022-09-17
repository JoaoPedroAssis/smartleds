#pragma once

#include <stdint.h>
#include "ESP32_fft.h"

#define NUM_BANDS 16
#define SAMPLES 256
#define BUFFER_SIZE 4
#define SAMPLING_FREQUENCY 11025
#define NOISE 500

static int cutOffs16Band[16] =
{
	100, 250, 450, 565, 715, 900, 1125, 1400, 1750, 2250, 2800, 3150, 4000, 5000, 6400, 12500
};



class SoundAnalyzer {
    private:
    
        // float * input_buffer;
        // float * output_buffer;
        float * peaks;
        fft_config_t * _FFT;
        float amplitude = 200.0;
        float max_mag = 0;
        float fundamental_freq = 0;
        float bin_width;


        void FFT() {
            hammingWindow();
            fft_execute(_FFT);
            complexToMagnitude();
        }

        void hammingWindow() {
            float samplesMinusOne = (float(_FFT->size) - 1.0);
            for (uint16_t i = 0; i < (_FFT->size >> 1); i++) {
                float indexMinusOne = float(i);
                float ratio = (indexMinusOne / samplesMinusOne);
                float weighingFactor = 0.54 - (0.46 * cos(TWO_PI * ratio));
                _FFT->input[i] *= weighingFactor;
                _FFT->input[_FFT->size - (i + 1)] *= weighingFactor;
            }
        }

        void complexToMagnitude() {

            for (int k = 1 ; k < _FFT->size/ 2 ; k++) {
                float mag = sqrt(pow(_FFT->output[2*k],2) + pow(_FFT->output[2*k+1],2))/1;
                float freq = frequency(k);
                if(mag > max_mag) {
                    max_mag = mag;
                    fundamental_freq = freq;
                }
                _FFT->output[k] = mag;
            }
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


            if (i <= 2)     return 0; // 20Hz
            if (i <= 3)     return 1; // 25Hz
            if (i <= 5)     return 2; // 500Hz
            if (i <= 6)     return 3; // 1000Hz
            if (i <= 9)     return 4; // 2000Hz
            if (i <= 13)     return 5; // 4000Hz
            if (i <= 19)    return 6; // 8000Hz
            if (i <= 27)    return 7; // 20Hz
            if (i <= 38)    return 8; // 25Hz
            if (i <= 54)    return 9; // 20Hz
            if (i <= 77)    return 10; // 25Hz
            if (i <= 110)    return 11; // 500Hz
            if (i <= 156)   return 12; // 1000Hz
            if (i <= 222)   return 13; // 2000Hz
            if (i <= 316)   return 14; // 4000Hz
            if (i <= 450)   return 15; // 8000Hz
            else return 0;
        }

        // void createBands() {
        //     for (uint8_t band = 0; band < NUM_BANDS; band++) {
        //         peaks[band] = 0;
        //     }

        //     for (int i = 2; i < (SAMPLES / 2); i++) {
        //         if (input_buffer[i] > NOISE) { // Add a crude noise filter, 10 x amplitude or more
        //             peaks[getBand(i)] = (int) input_buffer[i];
        //         }
        //     }
        // }

    public:

        SoundAnalyzer() {
            // input_buffer = (float *) malloc(SAMPLES * sizeof(input_buffer[0]));
            // output_buffer = (float *) malloc(SAMPLES * sizeof(output_buffer[0]));
            peaks = (float *)  malloc(NUM_BANDS * sizeof(peaks[0]));

            bin_width = (float) (SAMPLING_FREQUENCY/ (SAMPLES/2.0));
            // Serial.printf("BINWIDTH: %.3f\n", bin_width);

            _FFT = fft_init(SAMPLES, FFT_REAL, FFT_FORWARD, NULL, NULL);
        }

        ~SoundAnalyzer() {
            // free(input_buffer);
            // free(output_buffer);
            free(peaks);
            fft_destroy(_FFT);
        }

        float binWidth()        {return bin_width; }
        float frequency(int i)  {return i * bin_width; }
        float majorPeak()       {return max_mag ;}
        float majorPeakFreq()   {return fundamental_freq ;}

        // Copies the sample_data into input_buffer array
        void setData(const float* sample_data) {
            // memcpy(_FFT->input, sample_data, SAMPLES * sizeof(float));
            for (int i = 0; i < SAMPLES; i++) {
                _FFT->input[i] = sample_data[i];
            }
        }
        
        float getPeak() {

            
            FFT();


            return majorPeakFreq();
        }
};