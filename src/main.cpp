#include <Arduino.h>
#include "BluetoothA2DPSink.h"
#include <FastLED.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>
#include <freertos/ringbuf.h>

#include <SoundAnalyzer.h>


// LED settings
#define NUM_LEDS 60
#define LED_PIN 32
#define INIT_BRIGHTNESS 100
#define BASS_LEDS 15

// StreamBuffer
#define BUFFER_SIZE 4
#define BUFFER_SLOT 128
#define BUFFER_TRIGGER 1

// Bands
enum Bands {
  SUB_BASS,
  BASS,
  LOWER_MID,
  MID,
  HIGH_MID,
  PRESENCE,
  BRILLIANCE,
};


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
SoundAnalyzer *sound_analyzer = new SoundAnalyzer();

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

    sound_analyzer->set_data(fft_input_buffer, item_byte_size);
    double * bands = sound_analyzer->get_bands();
    int peak_idx = sound_analyzer->major_peak_idx();

    double bass = bands[BASS];
    
    int draw_bass = (int) sound_analyzer->mapd(bass, 0.0, 1.0, 0.0, 14.0);

    if (draw_bass > 14) {
      continue;
    }

    uint16_t t = millis() / 5;
    int scale = beatsin8(10, 10, 30);

    for (int i = 0; i < NUM_LEDS; i++) {
      uint8_t noise = inoise8(i * scale + peak_idx, t);
      uint8_t hue = map(noise, 50, 190, 0, 255);
      leds[i] = CHSV(hue, 255, 255);
    }

    fill_noise16(leds, NUM_LEDS, 1, 0, peak_idx, 1, 0, peak_idx, t, 5 );

    for (int i = 0; i < draw_bass; i++) {
      leds[i] = CRGB::Red;
    }
    FastLED.show();
  }  
}

void setup() {
  Serial.begin(115200);

  // LEDS
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(INIT_BRIGHTNESS);
  FastLED.clear(true);
  for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = ColorFromPalette(SunsetPallete, i);
  }
  FastLED.show();

  static const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,
    .bits_per_sample = (i2s_bits_per_sample_t) 16,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0, // default interrupt priority
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false
  };

  a2dp_sink.set_i2s_config(i2s_config);
  a2dp_sink.set_stream_reader(read_data_stream, true);
  a2dp_sink.start("SMARTLEDS");

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
  
  // EVERY_N_SECONDS(1) {
  //   Serial.printf("                                                                                                      CONT: %d\n", cont);
  //   cont = 0;
  // }
  vTaskDelete(NULL);
  
}
