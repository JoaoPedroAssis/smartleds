#include <Arduino.h>
#include "BluetoothA2DPSink.h"
#include <FastLED.h>
#include <arduinoFFT.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>

#include "../include/SoundAnalyzer.h"

// LED settings
#define NUM_LEDS 60
#define LED_PIN 32
#define INIT_BRIGHTNESS 100

// FFT
#define NUM_BANDS 16
#define SAMPLES 1024
#define TRIMMED_SAMPLES 256
#define SAMPLING_FREQUENCY 44100
#define NOISE 500

#define BUFFER_SIZE 4

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
SoundAnalyzer *sound_analyzer;

double *tmp_buffer;

void read_data_stream(const uint8_t *data, uint32_t length) { 
  int16_t l_sample, r_sample;
  int byte_offset, idx = 0;
  size_t buffer_byte_size = TRIMMED_SAMPLES * sizeof(double);
  size_t bytes_sent;

  for (int i = 0; i < SAMPLES; i += 4) {
    l_sample = (int16_t)(((*(data + byte_offset + 1) << 8) | *(data + byte_offset)));
    r_sample = (int16_t)(((*(data + byte_offset + 3) << 8) | *(data + byte_offset + 2)));

    tmp_buffer[idx] = (l_sample + r_sample) / 2.0f;
    byte_offset = byte_offset + 16;
    idx++;
  }

  // Serial.printf("    IDX FINAL: %d\n", idx);

  bytes_sent = xStreamBufferSend(sample_buffer, tmp_buffer, buffer_byte_size, 0);
  if (bytes_sent != buffer_byte_size) {
    Serial.printf("Nem todos os bytes foram enviados!: %d\n", xPortGetCoreID());
  } else {
    Serial.printf("Todos os bytes foram enviados!: %d\n", xPortGetCoreID());
  }
}

void calculateFFT(void *parameters) {
  size_t bytes_received;
  size_t buffer_byte_size = TRIMMED_SAMPLES * BUFFER_SIZE * sizeof(double);
  double *fft_buffer = (double*) malloc(buffer_byte_size);
  while (1) {
    bytes_received = xStreamBufferReceive(sample_buffer, fft_buffer, buffer_byte_size, portMAX_DELAY);

    if (bytes_received != buffer_byte_size) {
      Serial.printf("Nem todos os bytes foram recebidos!: %d\n", xPortGetCoreID());
    } else {
      Serial.printf("Todos os bytes foram recebidos!: %d\n", xPortGetCoreID());
    }

    sound_analyzer->setData(fft_buffer);
    // int max_peak = sound_analyzer->getPeak();

    // Serial.printf("                                   PEAK: %d\n", max_peak);
    // int led_brightness = map(max_peak, 0, 15, 0, 255);
    // FastLED.setBrightness(led_brightness);
    // FastLED.show();
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

  static const i2s_config_t i2s_config {
    .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = 44100, // corrected by info from bluetooth
    .bits_per_sample = (i2s_bits_per_sample_t) 16, /* the DAC module will only take the 8bits from MSB */
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_MSB,
    .intr_alloc_flags = 0, // default interrupt priority
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false
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

  sound_analyzer = new SoundAnalyzer();

  tmp_buffer = (double*) malloc(TRIMMED_SAMPLES * sizeof(double));

  int stream_size = TRIMMED_SAMPLES * BUFFER_SIZE * sizeof(double);
  sample_buffer = xStreamBufferCreate(stream_size, stream_size);

  if( sample_buffer == NULL ) {
    Serial.println("SAMPLE BUFFER NOT INITIALIZED");
  } else {
    Serial.println("SAMPLE BUFFER INITIALIZED");
  }

  a2dp_sink.set_i2s_config(i2s_config);
  a2dp_sink.start("JP-Bluetooth");
  a2dp_sink.set_stream_reader(read_data_stream);

  
}

void loop() {
  
}