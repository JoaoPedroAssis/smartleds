#include <Arduino.h>
#include "BluetoothA2DPSink.h"
#include <FastLED.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>
#include <freertos/ringbuf.h>
#include "../include/FFT_Class.h"

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
uint8_t idx;
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

// BLUETOOTH STUFF
BluetoothA2DPSink a2dp_sink;
int cont = 0;

// BUFFERs
RingbufHandle_t sample_buffer;
float *blt_output_buffer, *fft_input_buffer;


void read_data_stream(const uint8_t *data, uint32_t length) {
  // Serial.printf("UM PACOTE CHEGOU! SIZE: %d\n", length);
  cont++;

  int16_t *values = (int16_t*) data;

  int buff_idx = 0;
  float mean = 0;
  size_t buffer_bytes_to_send = SAMPLES * sizeof(float);
  size_t bytes_sent;
  for (int i = 0; i < SAMPLES * 2; i+=2) {
    blt_output_buffer[buff_idx] = (values[i] + values[i+1]) /2.0f;
    mean += blt_output_buffer[buff_idx];
    buff_idx++;
  }

  mean /= (float) SAMPLES; 

  Serial.printf("NUMERO: %.2f %.2f %.2f | MEAN: %.2f\n", blt_output_buffer[0], blt_output_buffer[1], blt_output_buffer[2], mean);

  // Serial.printf("NUMERO: %.3f\n", max);
  UBaseType_t res =  xRingbufferSend(sample_buffer, blt_output_buffer, buffer_bytes_to_send, pdMS_TO_TICKS(1000));
  if (res != pdTRUE) {
    Serial.printf("                                       NAO ENVIOU!\n");
  } else {
    Serial.printf("ENVIOU!\n");
  }
}

void calculateFFT(void *parameters) {
  size_t bytes_received;
  size_t buffer_bytes_to_receive = SAMPLES * BUFFER_TRIGGER * sizeof(float);

  while (1) {
    // bytes_received = xStreamBufferReceive(sample_buffer, fft_input_buffer, buffer_bytes_to_receive, portMAX_DELAY);
    size_t tam;   
    fft_input_buffer = (float *) xRingbufferReceive(sample_buffer, &tam, portMAX_DELAY);

    //Check received item
    if (fft_input_buffer != NULL) {
      vRingbufferReturnItem(sample_buffer, (void *) fft_input_buffer);
      Serial.printf("RECEBEU! | SIZE: %d\n", tam);
    } else {
        //Failed to receive item
        Serial.printf("NAO RECEBEU!\n");
        continue;
    }

    float mean = 0;
    for (int i = 0; i < SAMPLES; i++) {
      mean += fft_input_buffer[i];
    }

    mean /= (float) SAMPLES;

    Serial.printf("HOLY: %.2f %.2f %.2f | MEAN: %.2f\n", fft_input_buffer[0], fft_input_buffer[1], fft_input_buffer[2], mean);


    // Serial.printf("HOLY: %ld\n", max);
    
    // mean = mean / 2048.0;
    // Serial.printf("MEAN: %.3f\n", mean);
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
      leds[i] = ColorFromPalette(purplePalette, rand);
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

  sample_buffer = xRingbufferCreate(ring_buffer_size, RINGBUF_TYPE_NOSPLIT);
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
  fill_palette(leds, NUM_LEDS, idx, 255 / NUM_LEDS, purplePalette, 100, LINEARBLEND);
  
  EVERY_N_MILLISECONDS(10){
    idx++;
  }
  
  EVERY_N_SECONDS(1) {
    Serial.printf("                                                                                                      CONT: %d\n", cont);
    cont = 0;
  }

  FastLED.show();
}
