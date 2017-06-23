/* Blink without Delay

Turns on and off a light emitting diode (LED) connected to a digital
pin, without using the delay() function.  This means that other code
can run at the same time without being interrupted by the LED code.

The circuit:
 * Use the onboard LED.
 * Note: Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino model, check
  the Technical Specs of your board  at https://www.arduino.cc/en/Main/Products

created 2005
by David A. Mellis
modified 8 Feb 2010
by Paul Stoffregen
modified 11 Nov 2013
by Scott Fitzgerald
modified 9 Jan 2017
by Arturo Guadalupi

This example code is in the public domain.

http://www.arduino.cc/en/Tutorial/BlinkWithoutDelay
*/

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_MAX31856.h>

const int LEDPin = 13; //LED_BUILTIN; // the number of a LED pin

int ledState = LOW; // LED state used to set the LED

// Generally, we want to use an unsigned long for variables that
// hold time, because the value will quickly become too large for an int
unsigned long previousMillis = 0; // stores last time LED updated

const long interval = 1000; // interval at which to blink (msec)

// Use software SPI: CS, DI, DO, CLK

//                            originally (10, 11, 12, 13)
Adafruit_MAX31865 myRTD = Adafruit_MAX31865(9, 10, 11, 12); // chip select is pin 9
// keeping pin 13 clear for use as indicator LED

Adafruit_MAX31856 myThermocouple = Adafruit_MAX31856(8, 10, 11, 12); // chip select is pin 8


// The value of the Rref resistor. Use 430.0!
#define RREF 430.0

float tempRTD = 0;
float tempThermo = 0;


void setup()
{
  Serial.begin(115200);
  Serial.println("Temperature Sensors Tests.\n");

  myRTD.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary

  myThermocouple.begin(); // type K Thermocouple
  myThermocouple.setThermocoupleType(MAX31856_TCTYPE_K);


  pinMode(LEDPin, OUTPUT);
}


void loop()
{
  unsigned long currentMillis = millis();
  //Serial.println(currentMillis);

  //uint16_t rtd = myRTD.readRTD();
  tempRTD = myRTD.temperature(100, RREF);
  tempThermo = myThermocouple.readThermocoupleTemperature();

  //Serial.print("RTD value: "); Serial.println(rtd);
  //float ratio = rtd;
  //ratio /= 32768;


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
