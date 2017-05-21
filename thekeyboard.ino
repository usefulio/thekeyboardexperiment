/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/*
  This example shows how to send HID (keyboard/mouse/etc) data via BLE
  Note that not all devices support BLE keyboard! BLE Keyboard != Bluetooth Keyboard
*/

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined(ARDUINO_ARCH_SAMD)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

// SX1509

#include <Wire.h> // Include the I2C library (required)
#include <SparkFunSX1509.h> // Include SX1509 library

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define KEY_ROWS                    4
    #define KEY_COLS                    3
    #define INTERRUPT_PIN               18 // ARDUINO pin 18 connected to SX1509 interrupt
/*=========================================================================*/

// Handy array we'll use to map row/column pairs to 
// character values:
char keyMap[KEY_ROWS][KEY_COLS] = {
  {'I', 'A', 'N'},
  {'L', 'E', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}};

const byte SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
SX1509 io; // Create an SX1509 object

// Create the bluefruit object, hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit HID Keyboard Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  if (!io.begin(SX1509_ADDRESS))
  {
    error(F("Failed to communicate to SX1509."));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'Bluefruit Keyboard': "));
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=Bluefruit Keyboard" )) ) {
    error(F("Could not set device name?"));
  }

  /* Enable HID Service */
  Serial.println(F("Enable HID Service (including Keyboard): "));
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    if ( !ble.sendCommandCheckOK(F( "AT+BleHIDEn=On" ))) {
      error(F("Could not enable Keyboard"));
    }
  }else
  {
    if (! ble.sendCommandCheckOK(F( "AT+BleKeyboardEn=On"  ))) {
      error(F("Could not enable Keyboard"));
    }
  }

  /* Add or remove service requires a reset */
  Serial.println(F("Performing a SW reset (service changes require a reset): "));
  if (! ble.reset() ) {
    error(F("Couldn't reset??"));
  }

  Serial.println();
  Serial.println(F("Go to your phone's Bluetooth settings to pair your device"));
  Serial.println(F("then open an application that accepts keyboard input"));

  Serial.println();
  Serial.println(F("Enter the character(s) to send:"));
  Serial.println(F("- \\r for Enter"));
  Serial.println(F("- \\n for newline"));
  Serial.println(F("- \\t for tab"));
  Serial.println(F("- \\b for backspace"));

  Serial.println();

  // To initialize the keypad engine, you at least need
  // to tell it how many rows and columns are in the matrix.
  // io.keypad(KEY_ROWS, KEY_COLS);
  // You can customize the keypad behavior further, by
  // defining scan time, debounce time, and sleep time:
  // Sleep time range: 128 ms - 8192 ms (powers of 2) 0=OFF
  unsigned int sleepTime = 0;
  // Scan time range: 1-128 ms, powers of 2
  byte scanTime = 16; // Scan time per row, in ms
  // Debounce time range: 0.5 - 64 ms (powers of 2)
  byte debounceTime = 8; // Debounce time
  io.keypad(KEY_ROWS, KEY_COLS, sleepTime, scanTime, debounceTime);

  // Set the ARDUINO pin as an input, to monitor the interrupt
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  Serial.println("Row | Col | Key");
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  checkForKeyPresses();
  
//  // Display prompt
//  Serial.print(F("keyboard > "));
//
//  // Check for user input and echo it back if anything was found
//  char keys[BUFSIZE+1];
//  getUserInput(keys, BUFSIZE);
//
//  Serial.print("\nSending ");
//  Serial.println(keys);
//
//  ble.print("AT+BleKeyboard=");
//  ble.println(keys);
//
  if( ble.waitForOK() )
  {
    Serial.println( F("OK!") );
  }else
  {
    Serial.println( F("FAILED!") );
  }
}

/**************************************************************************/
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/
void getUserInput(char buffer[], uint8_t maxSize)
{
  memset(buffer, 0, maxSize);
  while( Serial.available() == 0 ) {
    delay(1);
  }

  uint8_t count=0;

  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && !(Serial.available() == 0) );
}

void checkForKeyPresses(void)
{
  // If the interrupt pin goes active-low, a keypad button
  // is begin pressed:
  if (!digitalRead(INTERRUPT_PIN))
  {
    // Use readKeypad() to get a binary representation for
    // which row and column are pressed
    unsigned int keyData = io.readKeypad();

    // Use the getRow, and getCol helper functions to find
    // which row and column keyData says are active.
    byte row = io.getRow(keyData);
    byte col = io.getCol(keyData);
    char key = keyMap[row][col];
    Serial.print(String(row) + " | " + String(col) + " | ");
    Serial.println(key);
    ble.print("AT+BleKeyboard=");
    ble.println(key);
    if( ble.waitForOK() )
    {
      Serial.println( F("OK!") );
//    }else
//    {
//      Serial.println( F("FAILED!") );
    }
  }
}

