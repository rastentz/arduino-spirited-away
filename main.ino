#include <FastLED.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>
#include <time.h>

// If using the breakout with SPI, define the pins for SPI communication.
#define PN532_SCK  (2)
#define PN532_MOSI (3)
#define PN532_SS   (4)
#define PN532_MISO (5)

// If using the breakout or shield with I2C, define just the pins connected
// to the IRQ and reset lines.  Use the values below (2, 3) for the shield!
#define PN532_IRQ   (2)
#define PN532_RESET (3)  // Not connected by default on the NFC Shield

// Uncomment just _one_ line below depending on how your breakout or shield
// is connected to the Arduino:

// Use this line for a breakout with a software SPI connection (recommended):
//Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

// Use this line for a breakout with a hardware SPI connection.  Note that
// the PN532 SCK, MOSI, and MISO pins need to be connected to the Arduino's
// hardware SPI SCK, MOSI, and MISO pins.  On an Arduino Uno these are
// SCK = 13, MOSI = 11, MISO = 12.  The SS line can be any digital IO pin.
//Adafruit_PN532 nfc(PN532_SS);

// Or use this line for a breakout or shield with an I2C connection:
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
// also change #define in Adafruit_PN532.cpp library file
   #define Serial SerialUSB
#endif

// SETUP
#define LED_PIN     5
#define NUM_LEDS    138
#define BRIGHTNESS  64
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define TOTAL_TAGS  10
#define TAG_SIZE     7
#define TAGS_PER_ROUND 3
#define PUMP_PIN 10
CRGB leds[NUM_LEDS];
int SEGMENTS[3][2] = {{0, 46}, {46, 92}, {92, 138}}; // [start, end)

enum SHAPE {
  SQUARE   = 0,
  CIRCLE   = 1,
  TRIANGLE = 2,
};

typedef struct Tag {
  SHAPE shape;
  CRGB color;
  uint8_t key[TAG_SIZE];
  uint8_t key_size;
} tag_t;

tag_t TAGS[TOTAL_TAGS] = {{.shape = SHAPE::SQUARE,   .color = CRGB::Blue,   .key = {0x04, 0xA9, 0xC3, 0x12, 0xC3, 0x58, 0x80}, .key_size = 7},
                          {.shape = SHAPE::SQUARE,   .color = CRGB::Red,    .key = {0x04, 0x2C, 0xC3, 0x12, 0xC3, 0x58, 0x80}, .key_size = 7},
                          {.shape = SHAPE::SQUARE,   .color = CRGB::White,  .key = {0x04, 0xE5, 0xC3, 0x12, 0xC3, 0x58, 0x80}, .key_size = 7},
                          {.shape = SHAPE::CIRCLE,   .color = CRGB::Blue,   .key = {0x04, 0xD1, 0xC3, 0x12, 0xC3, 0x58, 0x80}, .key_size = 7},
                          {.shape = SHAPE::CIRCLE,   .color = CRGB::Red,    .key = {0x04, 0xD0, 0xC3, 0x12, 0xC3, 0x58, 0x80}, .key_size = 7},
                          {.shape = SHAPE::CIRCLE,   .color = CRGB::White,  .key = {0x04, 0x0E, 0xC3, 0x12, 0xC3, 0x58, 0x81}, .key_size = 7},
                          {.shape = SHAPE::TRIANGLE, .color = CRGB::Red,    .key = {0x04, 0x1A, 0xC3, 0x12, 0xC3, 0x58, 0x80}, .key_size = 7},
                          {.shape = SHAPE::TRIANGLE, .color = CRGB::Green,  .key = {0x04, 0x42, 0xC3, 0x12, 0xC3, 0x58, 0x80}, .key_size = 7},
                          {.shape = SHAPE::TRIANGLE, .color = CRGB::White,  .key = {0x04, 0x0F, 0xC3, 0x12, 0xC3, 0x58, 0x81}, .key_size = 7},
                          {.shape = SHAPE::TRIANGLE, .color = CRGB::Blue,   .key = {0x04, 0xA8, 0xC3, 0x12, 0xC3, 0x58, 0x80}, .key_size = 7}};

tag_t START_TAG = {.shape = SHAPE::TRIANGLE, .color = CRGB::Blue,   .key = {0x62, 0x43, 0xE5, 0x1C, 0x00, 0x00, 0x00}, .key_size = 4};

void fill_segment(int segment, CRGB color) {
  int start = SEGMENTS[segment][0];
  int finish = SEGMENTS[segment][1];
  for(int i = start; i < finish; i++) {
    leds[i] = color;
  }
  FastLED.show();
}

void fill_all(CRGB color) {
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = color;
  }
  FastLED.show();
}

void clear_all() {
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
}

void select_tags(tag_t **ret) {
  int i = -1, j = -1, k = -1;
  i = rand() % TOTAL_TAGS;
  do {
    j = rand() % TOTAL_TAGS;
  } while(j == i || j == k);
  do {
    k = rand() % TOTAL_TAGS;
  } while(k == j || k == i);
  // assign 
  ret[0] = &TAGS[i];
  ret[1] = &TAGS[j];
  ret[2] = &TAGS[k];
}

void flash(int iter, int period, CRGB color) {
  for(int i = 0; i < iter; i++) {
    fill_all(color);
    delay(period);
    clear_all();
    delay(period);
  }
}

void setup() {
  delay( 3000 ); // power-up safety delay
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  BRIGHTNESS );
  Serial.begin(115200);

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }
  
  // configure board to read RFID tags
  nfc.SAMConfig();

    // some setup for output pins
  pinMode(PUMP_PIN, OUTPUT);
}

void waitForTag(tag_t *tag) {
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  while(true) {
    success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
    Serial.println(uidLength);
    if (success) {
      if(arrMatch(tag->key, uid, tag->key_size, uidLength)){
        return;
      }
    }
  }
}

void loop()
{
  srand(time(0));
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  tag_t* selected_tags[TAGS_PER_ROUND];
  while(true) {
    select_tags(selected_tags);
    for(int i = 0; i < TAGS_PER_ROUND; i++) {
      bool match = false;
      fill_segment((int) (selected_tags[i]->shape), selected_tags[i]->color);
      while(!match) {
        success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
        if (success) {
          if(arrMatch(selected_tags[i]->key, uid, selected_tags[i]->key_size, uidLength)){
            Serial.println("It's a match");
            match = true;
            fill_all(CRGB::Green);
            delay(500);
            clear_all();
          }
          else{
            fill_all(CRGB::Red);
            delay(500);
            clear_all();
            fill_segment((int) (selected_tags[i]->shape), selected_tags[i]->color);
          }
        }
      }
    }
    digitalWrite(PUMP_PIN, HIGH);
    Serial.println("PUMP ON");
    flash(3, 250, CRGB::Green);
    waitForTag(&START_TAG);
    digitalWrite(PUMP_PIN, LOW);
    Serial.println("PUMP OFF");
  }
}
   
int arrMatch(uint8_t arr1[], uint8_t arr2[], uint8_t arr1_len, uint8_t arr2_len){
  if(arr1_len != arr2_len) return 0;
  for (int i = 0; i < arr1_len; i = i+1){
    if(arr1[i] != arr2[i]){
      return 0;
    }
  }
  return 1;
}
