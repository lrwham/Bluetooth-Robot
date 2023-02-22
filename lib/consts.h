#ifndef CONSTS_H
#define CONSTS_H

#include <Arduino.h>

// using Adafruit Bluefruit Connect app on Apple iOS
// key    pressed released
// up     !B516   !B507
// right  !B813   !B804
// down   !B615   !B606
// left   !B714   !B705
// 1 key  !B11:   !B10;
// 2 key  !B219   !B20:
// 3 key  !B318   !B309
// 4 key  !B417   !B408
const std::string UP_PRESSED = "!B516";
const std::string UP_RELEASED = "!B507";
const std::string RIGHT_PRESSED = "!B813";
const std::string RIGHT_RELEASED = "!B804";
const std::string DOWN_PRESSED = "!B615";
const std::string DOWN_RELEASED = "!B606";
const std::string LEFT_PRESSED = "!B714";
const std::string LEFT_RELEASED = "!B705";
const std::string ONE_KEY_PRESSED = "!B11:";
const std::string ONE_KEY_RELEASED = "!B10;";
const std::string TWO_KEY_PRESSED = "!B219";
const std::string TWO_KEY_RELEASED = "!B20:";
const std::string THREE_KEY_PRESSED = "!B318";
const std::string THREE_KEY_RELEASED = "!B309";
const std::string FOUR_KEY_PRESSED = "!B417";
const std::string FOUR_KEY_RELEASED = "!B408";


// Arbitrary values for motor speed
// between 0-255 according to the Adafruit Motor Shield library
const uint8_t MAX_SPEED = 255;
const uint8_t FAST_SPEED = 190;
const uint8_t SLOW_SPEED = 125;
const uint8_t MIN_SPEED = 80;
const uint8_t STOPPED = 0;


// enum for keypad codes from the Adafruit Bluefruit Connect app
enum keypad_string_code
{
  up_pressed,
  up_released,
  right_pressed,
  right_released,
  down_pressed,
  down_released,
  left_pressed,
  left_released,
  one_pressed,
  one_released,
  two_pressed,
  two_released,
  three_pressed,
  three_released,
  four_pressed,
  four_released,
  error
};


// enum for motor speed levels
enum speed{
  max_speed,
  fast_speed,
  slow_speed,
  min_speed,
  stopped
};

#endif