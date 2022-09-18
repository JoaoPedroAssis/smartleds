#include <Arduino.h>
#include "BluetoothA2DPSink.h"
#include <FastLED.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>
#include <freertos/ringbuf.h>
#include "../include/ESP32_fft.h"

// LED settings
#define NUM_LEDS 60
#define LED_PIN 32
#define INIT_BRIGHTNESS 100

// FFT
#define NUM_BANDS 16
#define SAMPLES 1024
#define TRIMMED_SAMPLES 128
#define SAMPLING_FREQUENCY 44100
#define NOISE 500

// StreamBuffer
#define BUFFER_SIZE 4
#define BUFFER_SLOT 128
#define BUFFER_TRIGGER 1


/* --------- VARIABLES ----------- */

// LED STUFF
CRGB leds[NUM_LEDS];
// uint8_t idx;
CRGBPalette16 purplePalette = CRGBPalette16 (
    CRGB::DarkViolet,
    CRGB::DarkViolet,
    CRGB::DarkViolet,
    CRGB::DarkViolet,
    
    CRGB::Magenta,
    CRGB::Magenta,
    CRGB::Linen,
    CRGB::Linen,
    
    CRGB::Magenta,
    CRGB::Magenta,
    CRGB::DarkViolet,
    CRGB::DarkViolet,

    CRGB::DarkViolet,
    CRGB::DarkViolet,
    CRGB::Linen,
    CRGB::Linen
);

DEFINE_GRADIENT_PALETTE ( sunset ) {
  0  , 120 , 0  , 0,
  22 , 179 , 22 , 0,
  51 , 255 , 104, 0,
  85 , 167 , 22 , 18,
  135, 100 , 0  , 103,
  198 , 16  , 0  , 130,
  255 , 0 , 0, 160};

CRGBPalette16 SunsetPallete = sunset;


// BLUETOOTH STUFF
BluetoothA2DPSink a2dp_sink;
int cont = 0;

// BUFFERs
RingbufHandle_t sample_buffer;
float *blt_output_buffer, *fft_input_buffer;

// FFT
fft_config_t *real_fft_plan = fft_init(SAMPLES, FFT_REAL, FFT_FORWARD, NULL, NULL);
double bands[7] = {0,0,0,0,0,0,0};
double fundamental_freq = (double) SAMPLING_FREQUENCY / (double) SAMPLES;


int idx_to_band(int idx) {

  if (idx <= 2)   return 0; // 60    Hz
  if (idx <= 6)   return 1; // 250   Hz
  if (idx <= 12)  return 2; // 500   Hz
  if (idx <= 47)  return 3; // 2000  Hz
  if (idx <= 93)  return 4; // 4000  Hz
  if (idx <= 140) return 5; // 6000  Hz
  if (idx <= 232) return 6; // 10000 Hz

  return 6;
}

void min_max(double * array, size_t size, double * min, double * max) {

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

double mapd(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void normalize(double * array, size_t size, double min, double max) {
  for (int i = 0; i < size; i++) {
    array[i] = mapd(array[i], min, max, 0, 1);
  }
}

void read_data_stream(const uint8_t *data, uint32_t length) {
  cont++;

  int16_t *values = (int16_t*) data;

  int buff_idx = 0;
  size_t buffer_bytes_to_send = SAMPLES * sizeof(float);
  size_t bytes_sent;
  for (int i = 0; i < SAMPLES * 2; i+=2) {
    blt_output_buffer[buff_idx] = (values[i] + values[i+1]) /2.0f;
    buff_idx++;
  }

  UBaseType_t res =  xRingbufferSend(sample_buffer, blt_output_buffer, buffer_bytes_to_send, pdMS_TO_TICKS(1000));
  if (res != pdTRUE) {
    // Serial.printf("                                       NAO ENVIOU!\n");
  } else {
    // Serial.printf("ENVIOU!\n");
  }
}

void calculateFFT(void *parameters) {
  size_t bytes_received;
  size_t buffer_bytes_to_receive = SAMPLES * BUFFER_TRIGGER * sizeof(float);

  while (1) {
    size_t item_byte_size;   
    fft_input_buffer = (float *) xRingbufferReceive(sample_buffer, &item_byte_size, portMAX_DELAY);

    size_t item_size = item_byte_size / sizeof(float);

    //Check received item
    if (fft_input_buffer != NULL) {
      vRingbufferReturnItem(sample_buffer, (void *) fft_input_buffer);
      // Serial.printf("RECEBEU! | SIZE: %d\n", item_size);
    } else {
        // //Failed to receive item
        // Serial.printf("NAO RECEBEU!\n");
        continue;
    }

    for (int i = 0; i < item_size; i++) {
      real_fft_plan->input[i] = fft_input_buffer[i];
    }

    fft_execute(real_fft_plan);

    float max_magnitude = 0;
    int idx = 0;
    for (int k = 1 ; k < real_fft_plan->size / 2 ; k++) {
      float mag = sqrt(pow(real_fft_plan->output[2*k],2) + pow(real_fft_plan->output[2*k+1],2));
      // Serial.printf("                                                       MAG: %.2f\n", mag);
      
      real_fft_plan->output[k] = mag;

      if(mag > max_magnitude) {
        max_magnitude = mag;
        idx = k;
      }
    }

    memset(bands, 0, 7 * sizeof(double));
    for (int i = 0; i < 232; i++) {
      bands[idx_to_band(i)] += 0.3 * real_fft_plan->output[i];
    }

    double min, max;

    min_max(bands, 7, &min, &max);

    normalize(bands, 7, min, max);

    Serial.printf("ENERGY: ");
    for (int i = 0; i < 7; i++) {
      Serial.printf("%.3f ", bands[i]);
    }
    Serial.printf("\n");

    for (int i = 0; i < NUM_LEDS; i++) {
      // uint8_t noise = inoise8(i * idx + 15);
      // uint8_t hue = map(noise, 50, 190, 0, 255);
      idx = map(idx, 1, 511, 0, 5000);
      uint8_t brightness = inoise8(i * 150, idx);
      uint8_t index = inoise8(i * 20, idx);
      leds[i] = ColorFromPalette(SunsetPallete, index, brightness);
    }
  }  
}

void setup() {
  Serial.begin(115200);

  // LEDS
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(INIT_BRIGHTNESS);
  FastLED.clear(true);
  for (int i = 0; i < NUM_LEDS; i++) {
      int rand = random8();
      leds[i] = ColorFromPalette(SunsetPallete, rand);
  }
  FastLED.show();

  // BLUETOOTH
  i2s_pin_config_t my_pin_config = {
    .bck_io_num = 18,
    .ws_io_num = 19,
    .data_out_num = 21,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  a2dp_sink.set_pin_config(my_pin_config);
  a2dp_sink.start("SMARTLEDS");
  a2dp_sink.set_stream_reader(read_data_stream);

  // BUFFERS
  int ring_buffer_size    = BUFFER_SIZE    * SAMPLES * sizeof(float);
  int stream_buffer_trigger = BUFFER_TRIGGER * SAMPLES * sizeof(float);

  // sample_buffer = xRingbufferCreate(ring_buffer_size, RINGBUF_TYPE_NOSPLIT);
  sample_buffer = xRingbufferCreateNoSplit(SAMPLES * sizeof(float), BUFFER_SIZE);

  if( sample_buffer == NULL ) {
    Serial.println("SAMPLE BUFFER NOT INITIALIZED");
  } else {
    Serial.println("SAMPLE BUFFER INITIALIZED");
  }

  blt_output_buffer = (float*) malloc(SAMPLES * sizeof(float));
  fft_input_buffer  = (float*) malloc(SAMPLES * BUFFER_TRIGGER * sizeof(float));

  xTaskCreatePinnedToCore(
    calculateFFT,
    "FFT",
    25000,
    NULL,
    1,
    NULL,
    1
  );
}

void loop() {
  // fill_palette(leds, NUM_LEDS, idx, 255 / NUM_LEDS, purplePalette, 100, LINEARBLEND);
  
  // EVERY_N_MILLISECONDS(10){
  //   idx++;
  // }
  
  EVERY_N_SECONDS(1) {
    Serial.printf("                                                                                                      CONT: %d\n", cont);
    cont = 0;
  }

  FastLED.show();
}
