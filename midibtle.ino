#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEMIDI.h"
#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#include "BluefruitConfig.h"

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.7.0"

// This app was tested on iOS with the following apps:
//
// https://itunes.apple.com/us/app/midimittr/id925495245?mt=8
// https://itunes.apple.com/us/app/igrand-piano-free-for-ipad/id562914032?mt=8
//
// To test:
// - Run this sketch and open the Serial Monitor
// - Open the iGrand Piano Free app
// - Open the midimittr app on your phone and under Clients select "Adafruit Bluefruit LE"
// - When you see the 'Connected' label switch to the Routing panel
// - Set the Destination to 'iGrand Piano'
// - Switch to the iGrand Piano Free app and you should see notes playing one by one

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                              BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

Adafruit_BLEMIDI midi(ble);

bool isConnected = false;
int current_note = 60;

int s0 = 9;
int s1 = 10;
int s2 = 11;
int s3 = 12;
int SIG_pin = 0;

byte ccChannels[16] = {
  0xB0,
  0xB1,
  0xB2,
  0xB3,
  0xB4,
  0xB5,
  0xB6,
  0xB7,
  0xB8,
  0xB9,
  0xBA,
  0xBB,
  0xBC,
  0xBD,
  0xBE,
  0xBF,
};

int currentValues[16];

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// callback
void connected(void)
{
  isConnected = true;

  Serial.println(F(" CONNECTED!"));
  delay(1000);

}

void disconnected(void)
{
  Serial.println("disconnected");
  isConnected = false;
}

void BleMidiRX(uint16_t timestamp, uint8_t status, uint8_t byte1, uint8_t byte2)
{
  Serial.print("[MIDI ");
  Serial.print(timestamp);
  Serial.print(" ] ");

  Serial.print(status, HEX); Serial.print(" ");
  Serial.print(byte1 , HEX); Serial.print(" ");
  Serial.print(byte2 , HEX); Serial.print(" ");

  Serial.println();
}

void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  Serial.begin(115200);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  
  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);

  Serial.println(F("Adafruit Bluefruit MIDI Example"));
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
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  //ble.sendCommandCheckOK(F("AT+uartflow=off"));
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Set BLE callbacks */
  ble.setConnectCallback(connected);
  ble.setDisconnectCallback(disconnected);

  // Set MIDI RX callback
  midi.setRxCallback(BleMidiRX);

  Serial.println(F("Enable MIDI: "));
  if ( ! midi.begin(true) )
  {
    error(F("Could not enable MIDI"));
  }

  ble.verbose(false);
  Serial.print(F("Waiting for a connection..."));
}

void loop(void)
{
  // interval for each scanning ~ 500ms (non blocking)
  ble.update(500);

  // bail if not connected
  if (! isConnected)
    return;

  for(int i = 0; i <= 15; i++){
    int val = map(readMux(i), 0, 1023, 0, 127);
    int currentVal = currentValues[i];
    // Serial.println(currentVal);
    byte chan = ccChannels[i];
    // Serial.print("Value at channel ");
    // Serial.print(i);
    // Serial.print(" is : ");
    if (currentValues[i] != val) {
      Serial.print(i);
      Serial.print(" currentVal: ");
      Serial.print(currentVal);
      Serial.print(" val: ");
      Serial.print(val);
      Serial.print(" chan: ");
      Serial.print(chan);
      // Serial.print(ccChannels[0]);
      // Serial.print(ccChannels[1]);
      Serial.print(": ");
      Serial.println(val);
      midi.send(chan, i, val);
      // currentValues[i] = val;
      // delay(20);
    }
    currentValues[i] = val;
    
    // // Serial.println(readMux(i));
    // Serial.println(i);
    // delay(50);
  }

  // Serial.print("Sending pitch ");
  // Serial.println(current_note, HEX);

  // // send note on
  // midi.send(0x90, current_note, 0x64);
  // delay(500);

  // // send note off
  // midi.send(0x80, current_note, 0x64);
  // delay(500);

}

int readMux(int channel){
  int controlPin[] = {s0, s1, s2, s3};
  int muxChannel[16][4]={
    {0,0,0,0}, //channel 0
    {1,0,0,0}, //channel 1 
    {0,1,0,0}, //channel 2
    {1,1,0,0}, //channel 3
    {0,0,1,0}, //channel 4
    {1,0,1,0}, //channel 5
    {0,1,1,0}, //channel 6
    {1,1,1,0}, //channel 7
    {0,0,0,1}, //channel 8
    {1,0,0,1}, //channel 9
    {0,1,0,1}, //channel 10
    {1,1,0,1}, //channel 11
    {0,0,1,1}, //channel 12
    {1,0,1,1}, //channel 13
    {0,1,1,1}, //channel 14
    {1,1,1,1} //channel 15
  }; 
  //loop through the 4 sig
  for(int s = 0; s < 4; s ++){
    digitalWrite(controlPin[s], muxChannel[channel][s]);
  } //read the value at the SIG pin
  int val = analogRead(SIG_pin); //return the value
  return val;
}
