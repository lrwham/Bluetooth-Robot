/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE"
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second.
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HardwareSerial.h>
#include <Arduino.h>
#include <Adafruit_MotorShield.h>
// #include <Adafruit_I2CDevice.h>
#include <SPI.h>
#include "consts.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// And connect a DC motor to port M1
Adafruit_DCMotor *leftWheel = AFMS.getMotor(1);
Adafruit_DCMotor *rightWheel = AFMS.getMotor(2);

speed speedSetting = stopped;
uint8_t rightWheelSpeed = 0;
uint8_t leftWheelSpeed = 0;
uint8_t rightWheelForward = FORWARD;
uint8_t leftWheelForward = FORWARD;

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;
uint16_t loopCounter = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};

keypad_string_code hashit(std::string const &inString)
{
  if (inString == UP_PRESSED)
    return up_pressed;
  if (inString == UP_RELEASED)
    return up_released;

  if (inString == RIGHT_PRESSED)
    return right_pressed;
  if (inString == RIGHT_RELEASED)
    return right_released;

  if (inString == DOWN_PRESSED)
    return down_pressed;
  if (inString == DOWN_RELEASED)
    return down_released;

  if (inString == LEFT_PRESSED)
    return left_pressed;
  if (inString == LEFT_RELEASED)
    return left_released;

  if (inString == ONE_KEY_PRESSED)
    return one_pressed;
  if (inString == ONE_KEY_RELEASED)
    return one_released;

  if (inString == TWO_KEY_PRESSED)
    return two_pressed;
  if (inString == TWO_KEY_RELEASED)
    return two_released;

  if (inString == THREE_KEY_PRESSED)
    return three_pressed;
  if (inString == THREE_KEY_RELEASED)
    return three_released;

  if (inString == FOUR_KEY_PRESSED)
    return four_pressed;
  if (inString == FOUR_KEY_RELEASED)
    return four_released;

  return error;
}

void stopAllMotors()
{
  rightWheelSpeed = STOPPED;
  leftWheelSpeed = STOPPED;
  rightWheelForward = RELEASE;
  leftWheelForward = RELEASE;
}

void serialPrintRxValue(std::string const &rxValue)
{
  if (rxValue.length() > 0)
  {
    Serial.println("*********");
    Serial.print("Received Value Length: ");
    Serial.println(rxValue.length());
    Serial.print("Received Value: ");
    for (int i = 0; i < rxValue.length(); i++)
    {
      Serial.print(rxValue[i]);
    }
    Serial.println();
    Serial.println("*********");
  }
}
uint8_t speedValue(speed speedEnum)
{
  switch (speedEnum)
  {
  case max_speed:
    return MAX_SPEED;
  case fast_speed:
    return FAST_SPEED;
  case slow_speed:
    return SLOW_SPEED;
  case min_speed:
    return MIN_SPEED;
  default:
    return STOPPED;
  }
}
class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string rxValue = pCharacteristic->getValue();

    switch (hashit(rxValue))
    {
    case up_pressed:
      Serial.println("Up Pressed");
      rightWheelSpeed = speedValue(speedSetting);
      leftWheelSpeed = speedValue(speedSetting);
      rightWheelForward = FORWARD;
      leftWheelForward = FORWARD;
      break;
    case up_released:
      Serial.println("Up Released");
      stopAllMotors();
      break;
    case right_pressed:
      Serial.println("Right Pressed");
      rightWheelSpeed = speedValue(speedSetting)/2;
      leftWheelSpeed = speedValue(speedSetting)/2;
      rightWheelForward = BACKWARD;
      leftWheelForward = FORWARD;
      break;
    case right_released:
      Serial.println("Right Released");
      stopAllMotors();
      break;
    case down_pressed:
      Serial.println("Down Pressed");
      rightWheelSpeed = speedValue(speedSetting);
      leftWheelSpeed = speedValue(speedSetting);
      rightWheelForward = BACKWARD;
      leftWheelForward = BACKWARD;
      break;
    case down_released:
      Serial.println("Down Released");
      stopAllMotors();
      break;
    case left_pressed:
      Serial.println("Left Pressed");
      rightWheelSpeed = speedValue(speedSetting)/2;
      leftWheelSpeed = speedValue(speedSetting)/2;
      rightWheelForward = FORWARD;
      leftWheelForward = BACKWARD;
      break;
    case left_released:
      Serial.println("Left Released");
      stopAllMotors();
      break;
    case one_pressed:
      speedSetting = min_speed;
      break;
    case two_pressed:
      speedSetting = slow_speed;
      break;
    case three_pressed:
      speedSetting = fast_speed;
      break;
    case four_pressed:
      speedSetting = max_speed;
      break;
    default:
      serialPrintRxValue(rxValue);
    }
  }
};

void setup()
{
  Serial.begin(115200);

  if (!AFMS.begin())
  { // create with the default frequency 1.6KHz
    // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1)
      ;
  }
  Serial.println("Motor Shield found.");

  // Create the BLE Device
  BLEDevice::init("UART Service");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      BLECharacteristic::PROPERTY_NOTIFY);

  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a client connection to notify...");
}



void loop()
{
  

  rightWheel->setSpeed(rightWheelSpeed);
  leftWheel->setSpeed(leftWheelSpeed);
  rightWheel->run(rightWheelForward);
  leftWheel->run(leftWheelForward);
  /*
    if (deviceConnected)
    {
      pTxCharacteristic->setValue(&txValue, 1);
      pTxCharacteristic->notify();
      txValue++;
      delay(20); // bluetooth stack will go into congestion, if too many packets are sent
    }

    // disconnecting
    if (!deviceConnected && oldDeviceConnected)
    {
      delay(500);                  // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected)
    {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
    }
    */
}
