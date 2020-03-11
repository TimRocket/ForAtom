#define debugger false  // change to TRUE to enter debugger mode

#include <Arduino.h> // needed to convert C++

#include <Wire.h> //I2C
#include <SD.h>//SD
#include <SPI.h>//SPI
#include <Adafruit_BMP280.h>//Pression
#include "rgb_lcd.h"//LCD
#include <Adafruit_MMA8451.h>//Acce
#include <Adafruit_Sensor.h>//Acce
#include "DS1307.h"//RTC
#include <PWMServo.h>//servo

#define buttonGo 2
#define ledRed 6
#define ledGreen 5
#define ledBlue 7

#define TCAADDR 0x70

int pres = 0; //variable for the pressure
int newZero1 = 0; //Pressure at ground level for each barometer
int newZero2 = 0;
int newZero3 = 0;
int alt = 0; //variable for the altitude
int altZero = 0;
int prevAlt = 0;
int firstAlt = 0;
int altMax = 0; //maximum altitude
int offsetAcce = 0; //recalibration of the accelerometer

bool takeOff;
bool descBaro = false;
bool descAcce = false;
bool notLanded = true;
bool acce = false;
bool baro = false;
bool apog = false;
bool ascent = 0;

char filename[] = "00-00_00.CSV";

byte error = 0;
byte lastError = 0;

unsigned long timeApog;
unsigned long timeCligno;
unsigned long timeStart;
unsigned long timeTakeoff;
unsigned long timeBip;

File dataLogger;
rgb_lcd lcd;
Adafruit_BMP280 bmp1;//I2C
Adafruit_BMP280 bmp2;
Adafruit_BMP280 bmp3;
Adafruit_MMA8451 mma = Adafruit_MMA8451();
DS1307 clock;
PWMServo Servomoteur;

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


void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}



void setup() //Led Vert
{
  clock.begin();
  lcd.begin(16, 2);
  Servomoteur.attach(SERVO_PIN_A);
  pinMode(buttonGo, INPUT_PULLUP);
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledBlue, OUTPUT);
  digitalWrite(ledRed, LOW);
  digitalWrite(ledGreen, HIGH);
  digitalWrite(ledBlue, LOW);
  Servomoteur.write(0);
#if (debugger == true)
  {
    Serial.begin(9600);
    Serial.println(F("setup"));  // ecrire les etats des composants qui marchent ou inverse?
  }
#endif

  Wire.begin();
  tcaselect(2);
  if (!bmp3.begin())
  {
    error = 1;
  }
  else {
    bmp3.setSampling(Adafruit_BMP280::MODE_SLEEP,      /* Operating Mode.       */
                     Adafruit_BMP280::SAMPLING_NONE,   /* Temp. oversampling    */
                     Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                     Adafruit_BMP280::FILTER_X2,      /* Filtering.            */
                     Adafruit_BMP280::STANDBY_MS_1);
  }

  tcaselect(1);
  if (!bmp2.begin())
  {
    error = error + 2;
  }
  else {
    bmp2.setSampling(Adafruit_BMP280::MODE_SLEEP,     /* Operating Mode.       */
                     Adafruit_BMP280::SAMPLING_NONE,   /* Temp. oversampling   */
                     Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                     Adafruit_BMP280::FILTER_X2,       /* Filtering.            */
                     Adafruit_BMP280::STANDBY_MS_1);   /* Standby time.         */
  }

  tcaselect(0);
  if (!bmp1.begin())
  {
    error = error + 4;
  }
  else {
    bmp1.setSampling(Adafruit_BMP280::MODE_SLEEP,     /* Operating Mode.       */
                     Adafruit_BMP280::SAMPLING_NONE,   /* Temp. oversampling    */
                     Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                     Adafruit_BMP280::FILTER_X2,       /* Filtering.            */
                     Adafruit_BMP280::STANDBY_MS_1);   /* Standby time.*/
  }

  if (!mma.begin())
  {
    error = error + 8;
  }
  else
  {
    mma.setRange(MMA8451_RANGE_8_G);
  }

  if (!SD.begin(10)) {// pin CS chipselect
    error = error + 16;
  }
  else {
    clock.getTime();


    filename[0] = clock.dayOfMonth / 10 + '0';

    filename[1] = clock.dayOfMonth % 10 + '0';

    filename[3] = clock.month / 10 + '0';

    filename[4] = clock.month % 10 + '0';

    for (uint8_t i = 0; i < 100; i++) {
      filename[6] = i / 10 + '0';
      filename[7] = i % 10 + '0';
      if (! SD.exists(filename)) {
        // only open a new file if it doesn't exist
        dataLogger = SD.open(filename, FILE_WRITE);
        break;  // leave the loop!
      }
    }
  }

  dataLogger.print(clock.hour, DEC);
  dataLogger.print(":");
  dataLogger.print(clock.minute, DEC);
  dataLogger.print(":");
  dataLogger.print(clock.second, DEC);
  dataLogger.print("  ");
  dataLogger.print(clock.dayOfWeek);
  dataLogger.print(" ");
  dataLogger.print(clock.dayOfMonth, DEC);
  dataLogger.print("/");
  dataLogger.print(clock.month, DEC);
  dataLogger.print("/");
  dataLogger.println(clock.year+2000, DEC);
  dataLogger.println("Time;Alt;Acce;Baro;Acce;descBaro;descAcce;Apog");

  if (error == false) {
    current_state = READY;
  }
  else {
    current_state = PROBLEM;
  }
  tone(4, 1000, 100);
}


void loop()
{
  //  READY,
  //  FLIGHT,
  //  LANDED,
  //  PROBLEM
  switch (current_state) {
    case READY : //clignotement led verte
      while (digitalRead(buttonGo) == 1)  {

        digitalWrite(ledGreen, !digitalRead(ledGreen) );
        delay(200);

#if (debugger == true)
        {
          Serial.println("ready");
        }
#endif
      }

      timeStart = millis();

      //Led orange, nouveau zéro
      digitalWrite(ledGreen, HIGH);
      digitalWrite(ledRed, HIGH);

      current_state = FLIGHT;
      break;

    case FLIGHT :
      tcaselect(2);
      bmp3.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode.       */
                       Adafruit_BMP280::SAMPLING_NONE,   /* Temp. oversampling   */
                       Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                       Adafruit_BMP280::FILTER_X2,       /* Filtering.            */
                       Adafruit_BMP280::STANDBY_MS_1);   /* Standby time.         */


      tcaselect(1);
      bmp2.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode.       */
                       Adafruit_BMP280::SAMPLING_NONE,   /* Temp. oversampling   */
                       Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                       Adafruit_BMP280::FILTER_X2,       /* Filtering.            */
                       Adafruit_BMP280::STANDBY_MS_1);   /* Standby time.         */

      tcaselect(0);
      bmp1.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode.       */
                       Adafruit_BMP280::SAMPLING_NONE,   /* Temp. oversampling    */
                       Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                       Adafruit_BMP280::FILTER_X2,       /* Filtering.            */
                       Adafruit_BMP280::STANDBY_MS_1);   /* Standby time.*/
#if (debugger == true)
      {
        Serial.println("flight");
      }
#endif
      while (notLanded == 1) {

        //Recalibration
        while (millis() - timeStart < 5000) {

          newZero1 = 0;
          newZero2 = 0;
          newZero3 = 0;

          tcaselect(2);
          for (int i = 0; i < 3; i++)
          {
            pres = (int) bmp3.readPressure();
            newZero3 = pres + newZero3;
          }
          newZero3 = newZero3 / 3;

          tcaselect(1);
          for (int i = 0; i < 3; i++)
          {
            pres = (int) bmp2.readPressure();
            newZero2 = pres + newZero2;
          }
          newZero2 = newZero2 / 3;

          tcaselect(0);
          for (int i = 0; i < 3; i++)
          {
            pres = (int) bmp1.readPressure();
            newZero1 = pres + newZero1;
          }
          newZero1 = newZero1 / 3;


          mma.read();
          sensors_event_t event;
          mma.getEvent(&event);
          offsetAcce = -9.81 - event.acceleration.y;


        }



        if (apog==1)
        {
        //Led jaune clignotant, descente
        if (millis()-timeCligno > 200)
        {
        digitalWrite(ledBlue, LOW);
        digitalWrite(ledGreen, !digitalRead(ledGreen) );
        digitalWrite(ledRed, !digitalRead(ledRed) );
        timeCligno = millis();
        }
        }
        else
        {
        //Led bleue, vol
        digitalWrite(ledRed, LOW);
        digitalWrite(ledGreen, LOW);
        digitalWrite(ledBlue, HIGH);
        }

        // Accelerometer
        mma.read();
        sensors_event_t event;
        mma.getEvent(&event);

        if (((event.acceleration.y + offsetAcce ) / 9.81) < -2 && takeOff == 0) { //needs some real tests
          takeOff = 1;
          timeTakeoff = millis();
        }

        if ((event.acceleration.y + offsetAcce / 9.81) > 0)
        {
          descAcce = 1;
        }

        if ((takeOff == 1) && (descAcce == 1) && ((event.acceleration.y + offsetAcce / 9.81) < 0.00))
        {
          acce = 1;
        }


        //Barometer
          //Read the pressure
        tcaselect(0);
        alt = 100 * 44330 * (1.0 - pow(bmp1.readPressure() / newZero1, 0.1903));

        tcaselect(1);
        alt = alt + 100 * 44330 * (1.0 - pow(bmp2.readPressure() / newZero2, 0.1903));

        tcaselect(2);
        alt = alt + 100 * 44330 * (1.0 - pow(bmp3.readPressure() / newZero3, 0.1903));

          //Average
        alt = alt / 3;

          //Test of the maximum altitude
        if (alt > altMax) {
          altMax = alt;
        }

        if ((alt - prevAlt > 0) && (prevAlt - firstAlt > 0) && (alt > 300) && (ascent==0))
        {
          ascent = 1;
        }

          //descent and apogee
        if ((alt - prevAlt < 0) && (prevAlt - firstAlt < 0) && (alt > 500))
        {
          if (descBaro == false) {
            timeApog = millis();
            timeCligno = millis();
            baro = 1;
          }

          descBaro = true;
          tone(4, 1000, 100);
        }

        //voting
        if (((acce == 1) || (baro == 1)) && (apog == 0)) {
          apog = 1;
          Servomoteur.write(180);
          dataLogger.close();
          dataLogger = SD.open(filename, FILE_WRITE);
        }

        //Data Logging
        dataLogger.print(millis());
        dataLogger.print(";");
        dataLogger.print(alt);
        dataLogger.print(";");
        dataLogger.print((int)(100*(event.acceleration.y + offsetAcce) / 9.81));
        dataLogger.print(";");
        dataLogger.print(baro);
        dataLogger.print(";");
        dataLogger.print(acce);
        dataLogger.print(";");
        dataLogger.print(descBaro);
        dataLogger.print(";");
        dataLogger.print(descAcce);
        dataLogger.print(";");
        dataLogger.print(apog);
        dataLogger.println(";");

        //Debugger Serial
        #if (debugger == true)
                {
                  Serial.print(alt * 100);
                  Serial.print(" ");
                  Serial.println(altMax * 100);
                  Serial.print(" ");
                }
        #endif

        //Stores the last values
        firstAlt = prevAlt;
        prevAlt = alt;


        //Condition for the simple state machine
        if ((ascent == 1) && (apog == 1) && ((alt < 70) || ((event.acceleration.y + offsetAcce) / 9.81) < -20 )) {
          notLanded = 0;
        }
      }

      current_state = LANDED;
      break;


    case LANDED :
    //Debugger Serial
#if (debugger== true)
      {
        Serial.print (F("LANDED"));
      }
#endif


      tcaselect(2);
      bmp3.setSampling(Adafruit_BMP280::MODE_SLEEP,      /* Operating Mode.       */
                       Adafruit_BMP280::SAMPLING_NONE,   /* Temp. oversampling    */
                       Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                       Adafruit_BMP280::FILTER_X2,      /* Filtering.            */
                       Adafruit_BMP280::STANDBY_MS_1);

      tcaselect(1);
      bmp2.setSampling(Adafruit_BMP280::MODE_SLEEP,     /* Operating Mode.       */
                       Adafruit_BMP280::SAMPLING_NONE,   /* Temp. oversampling   */
                       Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                       Adafruit_BMP280::FILTER_X2,       /* Filtering.            */
                       Adafruit_BMP280::STANDBY_MS_1);   /* Standby time.         */
      tcaselect(0);
      bmp1.setSampling(Adafruit_BMP280::MODE_SLEEP,     /* Operating Mode.       */
                       Adafruit_BMP280::SAMPLING_NONE,   /* Temp. oversampling    */
                       Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                       Adafruit_BMP280::FILTER_X2,       /* Filtering.            */
                       Adafruit_BMP280::STANDBY_MS_1);   /* Standby time.*/


      dataLogger.print(timeApog);
      dataLogger.print(timeTakeoff);
      dataLogger.close();

      if (millis()-timeBip > 4000)
      {
        tone(4, 1000, 2000);
      timeBip = millis();
      }

      if (millis()-timeCligno > 200)
      {
        digitalWrite(ledBlue, LOW);
        digitalWrite(ledGreen, LOW);
      digitalWrite(ledRed, !digitalRead(ledRed) );
      timeCligno = millis();
      }

      break;

    case PROBLEM :

    //Debugger
#if (debugger== 1)
      {
        Serial.print (F("PROBLEM"));
      }
#endif
      lastError = error;

      //Led rouge, problème
      digitalWrite(ledRed, HIGH);
      digitalWrite(ledGreen, LOW);
      digitalWrite(ledBlue, LOW);

      if ((error & B00010000) != 0) {
        lcd.setCursor (0, 0);
        lcd.print ("Error SD");
      }
      else {
        lcd.setCursor (0, 0);
        lcd.print ("SD OK");
      }
      delay (2000);
      lcd.clear();

      if ((error & B00001000) != 0) {
        lcd.setCursor (0, 0);
        lcd.print ("Error MMA");
      } else {
        lcd.setCursor (0, 0);
        lcd.print ("MMA OK");
      }

      delay (2000);
      lcd.clear();

      if ((error & B00000100) != 0) {
        lcd.setCursor (0, 0);
        lcd.print ("Error BMP1");
      }
      else {
        lcd.setCursor (0, 0);
        lcd.print ("BMP1 OK");
      }

      delay (2000);
      lcd.clear();

      if ((error & B00000010) != 0) {
        lcd.setCursor (0,0);
        lcd.print ("Error BMP2");
      }
      else {
        lcd.setCursor (0, 0);
        lcd.print ("BMP2 OK");
      }

      delay (2000);
      lcd.clear();

      if ((error & B00000001) != 0) {
        lcd.setCursor (0, 0);
        lcd.print ("Error BMP3");
      }
      else {
        lcd.setCursor (0, 0);
        lcd.print ("BMP3 OK");
      }

      delay (2000);
      lcd.clear();

      error = lastError;
      break;
    default : break;
  }
}
