// Wire1 connnected to Wire. (scl <-> scl1, sda <-> sda1)

// Demonstrates use of the Wire library on a single Arduino board
// with 2 Wire interfaces (like Arduino Due).
// Uses the option of customizing the buffers.
//
// Wire data to an I2C/TWI slave device
// Wire1 receives data as an I2C/TWI slave device

// Created 02 Jan 2025

// This example code is in the public domain.


#include <Wire.h>
#include <WireBuffer.h>
#include "Arduino.h"

static_assert(WIRE_INTERFACES_COUNT > 1, "You need two I2C interfaces on the Arduino board to run this sketch");

static const char text[] = "You really won't believe it, but x is ";

// Wire is the master writer
constexpr size_t M_RECEIVE_BUFFER_SIZE  = 0;  // There is no receive in this sketch.
constexpr size_t M_TRANSMIT_BUFFER_SIZE = 42; // Enhance the buffer to 42 characters.
SET_Wire_BUFFERS(M_RECEIVE_BUFFER_SIZE, M_TRANSMIT_BUFFER_SIZE,
    true /* master buffers needed */, false /* no slave buffers needed */ );

// Wire1 is the slave receiver
constexpr size_t S_RECEIVE_BUFFER_SIZE  = 42; // Be able receive up to 42 characters in one message.
constexpr size_t S_TRANSMIT_BUFFER_SIZE = 0;  // There is no transmit in this sketch.
SET_Wire1_BUFFERS(S_RECEIVE_BUFFER_SIZE, S_TRANSMIT_BUFFER_SIZE,
    false /* no master buffers needed */, true /* slave buffers needed */ );

void setup() {
  Serial.begin(9600);            // start serial for output
  Wire.begin();                  // master joins I2C bus (address optional for master)
  Wire1.begin(8);                // slave joins I2C bus with address #8
  Wire1.onReceive(receiveEvent); // register event

  // This is just for curiosity and could be removed
  printWireBufferSize(Serial);
  printWire1BufferSize(Serial);
}

static byte x = 0;

void loop() {
  Wire.beginTransmission(8); // transmit to device #8
  Wire.write(text);          // sends multiple bytes
  Wire.write(x);             // sends one byte
  Wire.endTransmission();    // stop transmitting

  x++;
  delay(500);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
//
// Hint: This function is called within an interrupt context.
// That means, that there must be enough space in the Serial output
// buffer for the characters to be printed. Otherwise the
// Serial.print() call will lock up.
void receiveEvent(int howMany) {
  while (1 < Wire1.available()) { // loop through all but the last
    const char c = Wire1.read();  // receive byte as a character
    Serial.print(c);              // print the character
  }
  const int x = Wire1.read();     // receive byte as an integer
  Serial.println(x);              // print the integer
}

void printWireBufferSize(Stream& stream) {
  using namespace WireBuffer;
  stream.print("Wire receive buffer size is ");
  stream.println(buffers.RX_BUFFER_SIZE);
  stream.print("Wire transmit buffer size is ");
  stream.println(buffers.TX_BUFFER_SIZE);
  stream.print("Wire service buffer size is ");
  stream.println(buffers.SRV_BUFFER_SIZE);
  delay(250); // Give time to free up Serial output buffer.
}

void printWire1BufferSize(Stream& stream) {
  using namespace Wire1Buffer;
  stream.print("Wire1 receive buffer size is ");
  stream.println(buffers.RX_BUFFER_SIZE);
  stream.print("Wire1 transmit buffer size is ");
  stream.println(buffers.TX_BUFFER_SIZE);
  stream.print("Wire1 service buffer size is ");
  stream.println(buffers.SRV_BUFFER_SIZE);
  delay(250); // Give time to free up Serial output buffer.
}
