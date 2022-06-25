#include <Arduino.h>
#include "BluetoothA2DPSink.h"
#include <FastLED.h>
#include <arduinoFFT.h>

// LED settings
#define NUM_LEDS 60
#define LED_PIN 4
#define INIT_BRIGHTNESS 100

CRGB leds[NUM_LEDS];

DEFINE_GRADIENT_PALETTE( greenblue_gp ) { 
  0,   0,  255, 245,
  46,  0,  21,  255,
  179, 12, 250, 0,
  255, 0,  255, 245
};
CRGBPalette16 greenblue = greenblue_gp;

// FFT
#define NUM_BANDS 16
#define SAMPLES 1024
#define SAMPLING_FREQUENCY 44100
#define NOISE 500

arduinoFFT FFT = arduinoFFT();
BluetoothA2DPSink a2dp_sink;

float amplitude = 200.0;

int32_t peak[NUM_BANDS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
double vReal[SAMPLES];
double vImag[SAMPLES];

// Multitask stuff
QueueHandle_t bl_queue;

void createBands(int i, int dsize) {
  uint8_t band = 0;

  if (i<=2 )           band = 1;
  if (i>2   && i<=3  ) band = 2;
  if (i>3   && i<=5  ) band = 3;
  if (i>5   && i<=7  ) band = 4;
  if (i>7   && i<=9  ) band = 5;
  if (i>9   && i<=13 ) band = 6;
  if (i>13  && i<=18 ) band = 7;
  if (i>18  && i<=25 ) band = 8;
  if (i>25  && i<=36 ) band = 9;
  if (i>36  && i<=50 ) band = 10;
  if (i>50  && i<=69 ) band = 11;
  if (i>69  && i<=97 ) band = 12;
  if (i>97  && i<=135) band = 13;
  if (i>135 && i<=189) band = 14;
  if (i>189 && i<=264) band = 15;

  int dmax = amplitude;
  if (dsize > dmax)
    dsize = dmax;
  if (dsize > peak[band]) {
    peak[band] = dsize;
  }
}

void read_data_stream(const uint8_t *data, uint32_t length) {
  int item = 0;
  int16_t l_sample, r_sample;

  if (uxQueueMessagesWaiting(bl_queue) == 0) {
    int byte_offset = 0;

    for (int i = 0; i < SAMPLES; i++) {
      l_sample = (int16_t)(((*(data + byte_offset + 1) << 8) | *(data + byte_offset)));
      r_sample = (int16_t)(((*(data + byte_offset + 3) << 8) | *(data + byte_offset + 2)));

      vReal[i] = (l_sample + r_sample) / 2.0f;
      vImag[i] = 0;
      byte_offset = byte_offset + 4;
    }

    xQueueSend(bl_queue, &item, portMAX_DELAY);
  }
}

void calculateFFT(void *parameters) {
  int item = 0;
  while (1) {
    if (uxQueueMessagesWaiting(bl_queue) > 0) {

      FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
      FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

      for (uint8_t band = 0; band < NUM_BANDS; band++) {
        peak[band] = 0;
      }

      for (int i = 2; i < (SAMPLES / 2); i++) {
        if (vReal[i] > NOISE) { // Add a crude noise filter, 10 x amplitude or more
          createBands(i, (int)vReal[i] / amplitude);
        }
      }

      xQueueReceive(bl_queue, &item, 0);

      int32_t max_peak = peak[0];
      int max_peak_idx = 0;
      for (int i = 0; i < NUM_BANDS; i++) {
        if (peak[i] > max_peak) {
          max_peak = peak[i];
          max_peak_idx = i;
        }
      }

      Serial.println(max_peak_idx);

      int led_brightness = map(max_peak_idx, 0, 15, 0, 255);
      // fill_solid(leds, NUM_LEDS, CHSV(led_brightness, 255, 255));
      FastLED.setBrightness(led_brightness);
      FastLED.show();
    }
  }
}


void setup() {

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

  bl_queue = xQueueCreate(1, sizeof(int));

  xTaskCreatePinnedToCore(
    calculateFFT,
    "FFT",
    10000,
    NULL,
    1,
    NULL,
    1
  );

  a2dp_sink.set_i2s_config(i2s_config);
  a2dp_sink.start("JP-Bluetooth");
  a2dp_sink.set_stream_reader(read_data_stream);

  Serial.begin(115200);
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  
}