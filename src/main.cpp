#define debugger true

#include <Arduino.h>

#include <Wire.h> //I2C
#include <SD.h>//SD
#include <SPI.h>//SPI
#include <Adafruit_BMP280.h>//Pression
#include "rgb_lcd.h"//LCD

#define buttonGo 2
#define ledReady 6
#define ledFlight 7

Adafruit_BMP280 bmp; // I2C

float newZero = 0;
float pres = 0;
float mini = 9999999999;
float maxi = -999999999;
float sum = 0;
float altMax = 0;
float alt = 0;
float firstAlt = 0;
float prevAlt = 0;
float currentPres = 0;

//bool debugger= false;
bool desc = false;
bool error = false;
bool notLanded = true;

byte initi = 0; //à quoi il sert ?

unsigned long timeStart;
unsigned long timeApog;

float average_pres(byte n);

File dataLogger;
rgb_lcd lcd;

//status of the state machine
typedef enum  {
  READY,
  FLIGHT,
  LANDED,
  PROBLEM
} rocket_state;


// interrupt on START button
// status used inside the interrupt function (volatile mandatory)
// https://arduino.stackexchange.com/questions/30968/how-do-interrupts-work-on-the-arduino-uno-and-similar-boards
volatile rocket_state current_state;

//void start_button_pressed() { // need hardware debouncing?
//  // check on the current state, start button is only usable during attract mode, not during a game
//  if ((current_state == ATTRACT_MODE));
//  {
//    current_state = START;
//  }
//}
//


float average_pres(byte n)
{
  for (int i = 0; i <= n; i++) {
    pres = bmp.readPressure();
    sum = pres + sum;
    if (pres < mini) {
      mini = pres;
    }
    if (pres > maxi) {
      maxi = pres;
    }
  }
  pres = (sum - (mini + maxi)) / ((n - 3) * 100);
  return pres;
}


void setup()
{
  pinMode(buttonGo, INPUT_PULLUP);
  pinMode(ledReady, OUTPUT);
    pinMode(ledFlight, OUTPUT);
    digitalWrite(ledReady, LOW);
    digitalWrite(ledFlight, LOW);
#if (debugger== true)
  {
    Serial.begin(9600);
    Serial.println(F("setup"));  // ecrire les etats des composants qui marchent ou inverse?
  }
  #endif

  tone(5, 1000, 100);
  if (!bmp.begin()) {
    error = true;
  }
  bmp.setSampling(Adafruit_BMP280::MODE_SLEEP,      /* Operating Mode.       */
                  Adafruit_BMP280::SAMPLING_NONE,   /* Temp. oversampling    */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering.            */
                  Adafruit_BMP280::STANDBY_MS_1);   /* Standby time.         */

  if (!SD.begin(10)) {
    error = true;
  }
 dataLogger = SD.open("EXAMPLE.txt", FILE_WRITE);
     dataLogger.println("Time;Alt;AltMax");
dataLogger.close();
  if (error == false) {
    current_state = READY;
  }
  else {
    current_state = PROBLEM;
  }
}


void loop()
{
  //  READY,
  //  FLIGHT,
  //  LANDED,
  //  PROBLEM
  switch (current_state) {
    case READY :
      while (digitalRead(buttonGo) == 1)  {
        digitalWrite(ledReady, HIGH);

        Serial.println("ready");
      }
      timeStart = millis();
      digitalWrite(ledFlight, HIGH);

      current_state = FLIGHT;
      break;

    case FLIGHT :
      bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                      Adafruit_BMP280::SAMPLING_NONE,
                      Adafruit_BMP280::SAMPLING_X16,
                      Adafruit_BMP280::FILTER_X16,
                      Adafruit_BMP280::STANDBY_MS_1);
      dataLogger = SD.open("EXAMPLE.txt", FILE_WRITE);
      Serial.println("flight");
      while (notLanded == 1) {
        while (millis() - timeStart < 5000) {
          currentPres = average_pres(10);
          newZero = currentPres;
          pres = 0;   //reset les parametre de la fonction average_pres
          mini = 9999999999;
          maxi = -999999999;
          sum = 0;
        } //set the newZero
        digitalWrite(ledReady, desc); //

        currentPres = average_pres(10);
        pres = 0;   //reset les parametre de la fonction average_pres
        mini = 9999999999;
        maxi = -999999999;
        sum = 0;


        alt = 44330 * (1.0 - pow(currentPres / newZero, 0.1903));

        #if (debugger == true)
        {
          Serial.print(alt * 100); //serial plotter/monitor
          Serial.print(" ");
          Serial.println(altMax * 100);
          Serial.print(" ");
        }
        #endif

        dataLogger.print(millis());
        dataLogger.print(";");
        dataLogger.print(alt);
        dataLogger.print(";");
        dataLogger.print(altMax);
        dataLogger.println(";");

        if (alt > altMax) {
          altMax = alt;
          timeApog = millis();
        }

        if ((alt - prevAlt < 0) && (alt - firstAlt < 0) && (alt > 1))  // Falling detection (+beep)
        {
          desc = true;
          digitalWrite(ledFlight, LOW);
          tone(5, 1000, 100);

        }

        firstAlt = prevAlt;
        prevAlt = alt;

        if ((desc == 1) && (alt < 1)){
          notLanded=0;
        }
      }
      #if (debugger== true)
      {
        Serial.print (F("It has "));
      }
      #endif
      current_state = LANDED;

      break;
    case LANDED :
    #if (debugger== true)
    {
      Serial.print (F("LANDED"));
    }
    #endif
        bmp.setSampling(Adafruit_BMP280::MODE_SLEEP,      /* Operating Mode.       */
                  Adafruit_BMP280::SAMPLING_NONE,   /* Temp. oversampling    */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering.            */
                  Adafruit_BMP280::STANDBY_MS_1);   /* Standby time.         */
                  digitalWrite(ledReady, HIGH);
                  digitalWrite(ledFlight, HIGH);
      dataLogger.close();
      break;
    case PROBLEM :
    #if (debugger== 1)
    {
      Serial.print (F("PROBLEM"));
    }
    #endif


      break;
    default : break;
  }
}
