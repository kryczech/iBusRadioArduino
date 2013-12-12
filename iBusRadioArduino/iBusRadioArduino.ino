/* This file is part of iBusRadioArduino.

    iBusRadioArduino is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    iBusRadioArduino is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with iBusRadioArduino.  If not, see <http://www.gnu.org/licenses/>.
    
*/

#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX

byte readbuffer[64];
int i;
int cspin = 7;
boolean read_byte = false;
int buffer_index = 0;
int buffer_max = 64;
int cksum;
long lastrcv;
const int faultPin = 6;

// Initialize array of all output byte arrays
// Make sure to set second bracket value to max array length. Max message plus one.
// First value in each array is the length of the message.
const byte outputMessages[][7] =
{
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x11 }, // radio 1 pressed
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x51 }, // radio 1 pressed more than 1 second
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x91 }, // radio 1 released
  
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x01 }, // radio 2 pressed
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x41 }, // radio 2 pressed more than 1 second
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x81 }, // radio 2 released
  
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x12 }, // radio 3 pressed
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x52 }, // radio 3 pressed more than 1 second
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x93 }, // radio 3 released
  
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x02 }, // radio 4 pressed
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x42 }, // radio 4 pressed more than 1 second
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x82 }, // radio 4 released
  
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x13 }, // radio 5 pressed
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x53 }, // radio 5 pressed more than 1 second
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x93 }, // radio 5 released
  
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x03 }, // radio 6 pressed
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x43 }, // radio 6 pressed more than 1 second
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x83 }, // radio 6 released
  
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x10 }, // tune left pressed
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x50 }, // tune left pressed more than 1 second
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x90 }, // tune left released
  
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x00 }, // tune right pressed
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x40 }, // tune right pressed more than 1 second
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x80 }, // tune right released
  
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x23 }, // mode pressed
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x63 }, // mode pressed more than 1 second
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0xA3 }, // mode released
  
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x04 }, // tone pressed
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x44 }, // tone pressed more than 1 second
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x84 }, // tone released
  
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x20 }, // select pressed
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x60 }, // select pressed more than 1 second
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0xA0 }, // select released
  
  { 0x05, 0xF0, 0x04, 0x68, 0x32, 0x11 }, // turn volume knob right (vol up)
  { 0x05, 0xF0, 0x04, 0x68, 0x32, 0x10 }, // turn volume knob left (vol down)
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x06 }, // volume knob pressed
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x46 }, // volume knob pressed more than 1 second
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x86 }, // volume knob released
  // { 0x05, 0xF0, 0x04, 0x68, 0x4B, 0x05 }, // radio off????
  
  { 0x05, 0xF0, 0x04, 0x68, 0x49, 0x81 }, // turn nav knob right
  { 0x05, 0xF0, 0x04, 0x68, 0x49, 0x01 }, // turn nav knob left
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x05 }, // nav knob pressed
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x45 }, // nav knob pressed more than 1 second
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x85 }, // nav knob released
  
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x31 }, // FM pressed
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x71 }, // FM pressed more than 1 second
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0xB1 }, // FM released
  
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x21 }, // AM pressed
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x61 }, // AM pressed more than 1 second
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0xA1 }, // AM released
  
  { 0x05, 0xF0, 0x04, 0xFF, 0x48, 0x08 }, // Phone pressed
  { 0x05, 0xF0, 0x04, 0xFF, 0x48, 0x48 }, // Phone pressed more than 1 second
  { 0x05, 0xF0, 0x04, 0xFF, 0x48, 0x88 }, // Phone released
  
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x24 }, // eject pressed
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x64 }, // eject pressed more than 1 second
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0xA4 }, // eject released
  
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x14 }, // tape reverse pressed
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x54 }, // tape reverse pressed more than 1 second
  { 0x05, 0xF0, 0x04, 0x68, 0x48, 0x94 }, // tape reverse released
  
};

  

void setup()  
{
  // initialize buffer array to 0's
  memset(readbuffer, 0, sizeof(readbuffer));
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600, SERIAL_8E1);
  Serial.println("Hello Brad.");
  // Serial.begin(9600);
  
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  mySerial.println("Hello, world?");
  mySerial.flush();
  
  pinMode(cspin, OUTPUT);
  digitalWrite(cspin, HIGH);
  pinMode(faultPin, OUTPUT);
  digitalWrite(faultPin, HIGH);
  
  lastrcv = millis();
}

void loop() // run over and over
{
// Somehow I think I'm having to slow this down to accurately read messages. 
// Trying to just add a delay x number of microseconds to see how it does.
  delayMicroseconds(1100);

// Do timeout of buffer after not receiving anything for 10ms
  if ((millis() - lastrcv) > 15) {
    memset(readbuffer, 0, sizeof(readbuffer));
    buffer_index = 0;
    read_byte = false;
    buffer_max = 64;
    lastrcv = millis();
    // mySerial.println("Timeout.");
    return;
  } 
  
  // Check mySerial to see if we need to send something. 
  // Do I really need to time constrain it here?
  // if ((millis() - lastsend) > 500) {
    if (mySerial.available()) {
      sendMessage(mySerial.read());
    }
  // }
  
  // If there's something to be read, read it.  
  if (Serial.available()) {
    readbuffer[buffer_index] = Serial.read();
    read_byte = true;
  }
  
  // If this is byte 2, then set buffer_max to it's value.
  // Also set cksum to xor of byte 1 and byte 2.
  if (read_byte) {
    if (buffer_index == 1) {
      buffer_max = readbuffer[buffer_index] + 2;
      cksum = readbuffer[0] ^ readbuffer[1];
    } else if ((buffer_index > 1 ) && (buffer_index < buffer_max)) {
      cksum = cksum ^ readbuffer[buffer_index];
    }
  }
  
  // Reset buffer_index when it is buffer_max - 1.
  if (buffer_index == (buffer_max - 1)) {
    if ((readbuffer[0] == 0x68) || (readbuffer[2] == 0x68)) {
      if (cksum == 0) {
        mySerial.print("Good message: ");
        mySerial.print(millis());
        mySerial.print(": ");
        for (i = 0; i < buffer_max; i++) {
          mySerial.print(readbuffer[i], HEX);
          mySerial.print(" ");
        }
        mySerial.println();
      } else {
        mySerial.print("Invalid message. cksum: ");
        mySerial.println(cksum, HEX);
        for (i = 0; i < buffer_max; i++) {
          mySerial.print(readbuffer[i], HEX);
          mySerial.print(" ");
        }
        mySerial.println();
      }
    }
    memset(readbuffer, 0, sizeof(readbuffer));
    buffer_index = 0;
    read_byte = false;
    lastrcv = millis();
  }
    
  // Increment index if we put something into the buffer
  if (read_byte == true) {
    read_byte = false;
    buffer_index++;
    lastrcv = millis();
  }
}

void sendMessage(byte messageType)
{ 
  // mySerial.print("Sending ...  ");
  
  // digitalWrite(faultPin, HIGH);
  
  // This is the test case for the new method.
  // mySerial.println("Sending radio 2.");
  /*
  if (messageType == 0) {
    for (int k = 1; k < outputMessages[0][0] + 1; k++) {
      Serial.write(outputMessages[messageType][k]);
    }
    delay(150);
    for (int k = 1; k < outputMessages[2][0] + 1; k++) {
      Serial.write(outputMessages[11][k]);
    }
  } else if (messageType == 3) {
    for (int k = 1; k < outputMessages[3][0] + 1; k++) {
      Serial.write(outputMessages[messageType][k]);
    }
  }
  */
  
  byte message_cksum = gen_cksum(outputMessages[messageType]);
  
  for (int k = 1; k < outputMessages[messageType][0] + 1; k++) {
    Serial.write(outputMessages[messageType][k]);
  }
  Serial.write(message_cksum);
  delay(150);
    
  
  // digitalWrite(faultPin, LOW);  
}

byte gen_cksum(const byte message[]) {
  byte cksum = 0x00;
  for (int i = 1; i <= message[0]; i++) {
    cksum = cksum ^ message[i];
  }
  return cksum;
}

