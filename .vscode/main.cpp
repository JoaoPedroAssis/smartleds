#include <Arduino.h>
#include "BluetoothA2DPSink.h"
#include <FastLED.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>

#include "../include/ESP32_fft.h"

// LED settings
#define NUM_LEDS 60
#define LED_PIN 32
#define INIT_BRIGHTNESS 100

// FFT
#define NUM_BANDS 16
#define SAMPLES 1024
#define TRIMMED_SAMPLES 128
// #define SAMPLING_FREQUENCY 44100
#define NOISE 500

#define BUFFER_SIZE 4
#define BUFFER_TRIGGER 2

CRGB leds[NUM_LEDS];

DEFINE_GRADIENT_PALETTE( greenblue_gp ) { 
  0,   0,  255, 245,
  46,  0,  21,  255,
  179, 12, 250, 0,
  255, 0,  255, 245
};
CRGBPalette16 greenblue = greenblue_gp;
BluetoothA2DPSink a2dp_sink;
StreamBufferHandle_t sample_buffer;
// SoundAnalyzer *sound_analyzer;

fft_config_t *real_fft_plan;

float *tmp_buffer;
int cont = 0;

void read_data_stream(const uint8_t *data, uint32_t length) { 
  int16_t l_sample, r_sample;
  int buff_idx = 0;
  size_t buffer_byte_size = TRIMMED_SAMPLES * sizeof(float);
  size_t bytes_sent;

  for (int i = 0; i < SAMPLES; i += 8) {
    l_sample = (int16_t) data[i];
    r_sample = (int16_t) data[i+1];

    tmp_buffer[buff_idx] = (l_sample + r_sample) / 2.0f;
    buff_idx++;
  }

  // Serial.printf("                                                     IDX FINAL: %d\n", buff_idx);

  bytes_sent = xStreamBufferSend(sample_buffer, tmp_buffer, buffer_byte_size, 0);
  if (bytes_sent != buffer_byte_size) {
    Serial.printf("Nem todos os bytes foram enviados!: %d\n", xPortGetCoreID());
  } else {
    Serial.printf("Todos os bytes foram enviados!: %d\n", xPortGetCoreID());
  }
  cont++;
}

void calculateFFT(void *parameters) {
  size_t bytes_received;
  size_t buffer_byte_size = TRIMMED_SAMPLES * BUFFER_TRIGGER * sizeof(float); //256
  float max_magnitude = 0;  
  float *fft_buffer = (float*) malloc(buffer_byte_size);
  while (1) {
    bytes_received = xStreamBufferReceive(sample_buffer, fft_buffer, buffer_byte_size, portMAX_DELAY);

    if (bytes_received != buffer_byte_size) {
      Serial.printf("Nem todos os bytes foram recebidos!: %d\n", xPortGetCoreID());
    } else {
      Serial.printf("Todos os bytes foram recebidos!: %d\n", xPortGetCoreID());
    }

    for (int i = 0; i < real_fft_plan->size; i++) {
      real_fft_plan->input[i] = fft_buffer[i];
    }
    
    fft_execute(real_fft_plan);

    for (int i = 1; real_fft_plan->size / 2; i++) {
      float mag = sqrt(pow(real_fft_plan->output[2*i],2) + pow(real_fft_plan->output[2*i+1],2));

      if(mag > max_magnitude) {
        max_magnitude = mag;
      }
    }

    // sound_analyzer->setData(fft_buffer);
    // float max_peak = sound_analyzer->getPeak(); //FFT
    // float freq = sound_analyzer->majorPeakFreq();

    Serial.printf("                                           MAG: %.3f\n", max_magnitude);
  }
}


void setup() {

  Serial.begin(115200);

  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(INIT_BRIGHTNESS);
  FastLED.clear(true);
  for (int i = 0; i < NUM_LEDS; i++) {
    int rand = random8();
    leds[i] = ColorFromPalette(greenblue, rand);
  }
  FastLED.show();

  // static const i2s_config_t i2s_config {
  //   .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
  //   .sample_rate = 44100, // corrected by info from bluetooth
  //   .bits_per_sample = (i2s_bits_per_sample_t) 16, /* the DAC module will only take the 8bits from MSB */
  //   .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
  //   .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_MSB,
  //   .intr_alloc_flags = 0, // default interrupt priority
  //   .dma_buf_count = 8,
  //   .dma_buf_len = 64,
  //   .use_apll = false
  // };

  i2s_pin_config_t my_pin_config = {
        .bck_io_num = 18,
        .ws_io_num = 19,
        .data_out_num = 21,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

  xTaskCreatePinnedToCore(
    calculateFFT,
    "FFT",
    25000,
    NULL,
    1,
    NULL,
    1
  );

  // sound_analyzer = new SoundAnalyzer();

  real_fft_plan = fft_init(SAMPLES / 4, FFT_REAL, FFT_FORWARD, NULL, NULL);

  tmp_buffer = (float*) malloc(TRIMMED_SAMPLES * sizeof(float));

  int stream_size = TRIMMED_SAMPLES * BUFFER_SIZE * sizeof(float);
  int buffer_trigger = BUFFER_TRIGGER * TRIMMED_SAMPLES * sizeof(float);
  sample_buffer = xStreamBufferCreate(stream_size, buffer_trigger);

  if( sample_buffer == NULL ) {
    Serial.println("SAMPLE BUFFER NOT INITIALIZED");
  } else {
    Serial.println("SAMPLE BUFFER INITIALIZED");
  }

  a2dp_sink.set_pin_config(my_pin_config);
  // a2dp_sink.set_i2s_config(i2s_config);
  a2dp_sink.start("JP-Bluetooth");
  a2dp_sink.set_stream_reader(read_data_stream);

  
}

void loop() {
  EVERY_N_SECONDS(1) {
    Serial.printf("                                                                                                 CONT: %d\n", cont);
    cont = 0;
  }
}