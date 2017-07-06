/* Garden Party Main code

Initializing code and main loop for Garden Party.

*/

#include "Adafruit_Sensor.h"
#include "DHT.h"

#include <Arduino.h>
#include <SPI.h>


#include <Adafruit_MAX31865.h>
#include <Adafruit_MAX31856.h>


const int LEDPin = 13; //LED_BUILTIN; // the number of a LED pin
int ledState = LOW; // LED state used to set the LED

// Generally, we want to use an unsigned long for variables that
// hold time, because the value will quickly become too large for an int
unsigned long previousMillis = 0; // stores last time LED updated

const long interval = 2000; // interval at which to blink (msec)
//also the DHT sensor requires 2 seconds between reads.

// Use software SPI: CS, DI, DO, CLK
//                            originally (10, 11, 12, 13)
Adafruit_MAX31865 myRTD = Adafruit_MAX31865(9, 10, 11, 12); // chip select is pin 9
// keeping pin 13 clear for use as indicator LED

Adafruit_MAX31856 myThermocouple = Adafruit_MAX31856(8, 10, 11, 12); // chip select is pin 8


// The value of the Rref resistor. Use 430.0!
#define RREF 430.0

float tempRTD = 0;
float tempThermo = 0;

//configure DHT
#define DHTPIN 2
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);


void setup()
{
  Serial.begin(115200);
  Serial.println("Sensors Testing Commence!\n");

  myRTD.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary

  myThermocouple.begin(); // type K Thermocouple
  myThermocouple.setThermocoupleType(MAX31856_TCTYPE_K);

  dht.begin();

  pinMode(LEDPin, OUTPUT);
}


void loop()
{
  unsigned long currentMillis = millis();

  //uint16_t rtd = myRTD.readRTD();
  tempRTD = myRTD.temperature(100, RREF);
  tempThermo = myThermocouple.readThermocoupleTemperature();



  if (currentMillis - previousMillis >= interval)
  {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    if ( (tempRTD >= 28.0) || (tempThermo >= 28.0) ) ledState = HIGH;
    else ledState = LOW;

    digitalWrite(LEDPin, ledState);

    //Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
    Serial.print("RTD Temp = "); Serial.println(tempRTD);
    Serial.print("\n");

    Serial.print("Thermocouple Temp: "); Serial.println(tempThermo);
    Serial.print("\n");

    //DHT
    Serial.print("DHT humidity: "); Serial.println(dht.readHumidity());
  }
}













// void loop()
// {
//
//   uint16_t rtd = myRTD.readRTD();
//   delay(500);
//
//
//   // Check and print any faults
//   uint8_t fault = myRTD.readFault();
//   // if (fault) {
//   //   Serial.print("Fault 0x"); Serial.println(fault, HEX);
//   //   if (fault & MAX31865_FAULT_HIGHTHRESH) {
//   //     Serial.println("RTD High Threshold");
//   //   }
//   //   if (fault & MAX31865_FAULT_LOWTHRESH) {
//   //     Serial.println("RTD Low Threshold");
//   //   }
//   //   if (fault & MAX31865_FAULT_REFINLOW) {
//   //     Serial.println("REFIN- > 0.85 x Bias");
//   //   }
//   //   if (fault & MAX31865_FAULT_REFINHIGH) {
//   //     Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
//   //   }
//   //   if (fault & MAX31865_FAULT_RTDINLOW) {
//   //     Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
//   //   }
//   //   if (fault & MAX31865_FAULT_OVUV) {
//   //     Serial.println("Under/Over voltage");
//   //   }
//     myRTD.clearFault();
//
//   Serial.println();
//   delay(250);
//
//   if (temperature >= 32.0)
//   {digitalWrite(7, HIGH);
//   Serial.println("here");
// delay(500);}
//   else digitalWrite(7, LOW);
//
// }
