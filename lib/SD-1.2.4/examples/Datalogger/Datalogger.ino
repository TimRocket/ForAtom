/*
  SD card datalogger

  This example shows how to log data from three analog sensors
  to an SD card using the SD library.

  The circuit:
   analog sensors on analog ins 0, 1, and 2
   SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

  created  24 Nov 2010
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

*/

#include <SPI.h>
#include <SD.h>

const int chipSelect = 10;
// the logging file
File logfile;

void setup() {

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    tone(4, 400, 100);
    // don't do anything more:
    while (1);
  }
    // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE);
      break;  // leave the loop!
    }
  }
}

void loop() {
  
//  // make a string for assembling the data to log:
//  String dataString = "";
//
//  // read three sensors and append to the string:
//  for (int analogPin = 0; analogPin < 3; analogPin++) {
//    int sensor = analogRead(analogPin);
//    dataString += String(sensor);
//    if (analogPin < 2) {
//      dataString += ",";
//    }
//  }


  // if the file is available, write to it:
  if (logfile) {
    logfile.println(millis());
    logfile.close();

  }
  // if the file isn't open, pop up an error:
  else {
    tone(4, 800, 100);
  }
}
