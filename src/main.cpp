/* Garden Party Main code

Initializing code and main loop for Garden Party.

*/

#include <DHT.h>
#include <SHT1x.h>
#include <Adafruit_Sensor.h>

#include <Arduino.h>
#include <SPI.h>

#include <Adafruit_MAX31865.h>
#include <Adafruit_MAX31856.h>

#include <Temperature.h>


const int LEDPin = 13; //LED_BUILTIN; // the number of a LED pin
int ledState = LOW; // LED state used to set the LED

// Generally, we want to use an unsigned long for variables that
// hold time, because the value will quickly become too large for an int
unsigned long previousMillis = 0; // stores last time LED updated

const long interval = 2500; // interval at which to blink (msec)
//also the DHT sensor requires 2 seconds between reads.

// Use software SPI: CS, DI, DO, CLK
//                            originally (10, 11, 12, 13)
const int SPI_CLOCK = 12;
const int SPI_DO = 11;
const int SPI_DI = 10;
const int SPI_CS_THERM = 9;
const int SPI_CS_RTD = 8;



// keeping pin 13 clear for use as indicator LED
Adafruit_MAX31856 myThermocouple = Adafruit_MAX31856(SPI_CS_THERM,
                                                     SPI_DI,
                                                     SPI_DO,
                                                     SPI_CLOCK); // chip select is pin 8

Adafruit_MAX31865 myRTD = Adafruit_MAX31865(SPI_CS_RTD,
                                            SPI_DI,
                                            SPI_DO,
                                            SPI_CLOCK); // chip select is pin 9


// The value of the Rref resistor. Use 430.0!
#define RREF 430.0

float tempRTD = 0;
float tempThermo = 0;

//configure Humidity Detectors
const uint8_t DHTPIN = 7;
const uint8_t DHTTYPE= 22; //DHT22 style sensor
DHT dht = DHT(DHTPIN, DHTTYPE);


//configure moisture sensor
//#define dataPin 5
//#define clockPin 4
//SHT1x sht1x(dataPin, clockPin);


#define dataPin 5
#define clockPin 4
#define clockPulse 1
#define voltage sht1xalt::VOLTAGE_5V
#define units sht1xalt::UNITS_CELCIUS
sht1xalt::Sensor sensor(dataPin, clockPin, clockPulse, voltage, units);



void setup()
{
  Serial.begin(115200);
  Serial.println("Sensors Testing Commence!\n");

  myRTD.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary

  myThermocouple.begin(); // type K Thermocouple
  myThermocouple.setThermocoupleType(MAX31856_TCTYPE_K);

  dht.begin();

  pinMode(LEDPin, OUTPUT);

  sensor.configureConnection();
  sensor.softReset();



}

float tempC = 0;
float relHum = 0;

void loop()
{
  unsigned long currentMillis = millis();

  //uint16_t rtd = myRTD.readRTD();
  tempRTD = myRTD.temperature(100, RREF);
  tempThermo = myThermocouple.readThermocoupleTemperature();



  if (currentMillis - previousMillis > interval)
  {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    if ( (tempRTD >= 28.0) || (tempThermo >= 28.0) ) ledState = HIGH;
    else ledState = LOW;

    digitalWrite(LEDPin, ledState);

    //Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
//    Serial.print("RTD Temp = "); Serial.println(tempRTD);
    //Serial.print("\n");

//    Serial.print("Thermocouple Temp: "); Serial.println(tempThermo);
    //Serial.print("\n");

    //DHT
//    Serial.print("DHT temperature: "); Serial.println(dht.readTemperature());
//    Serial.print("DHT humidity: "); Serial.println(dht.readHumidity());
//    Serial.print("\n");

    //tempF = sht1x.readTemperatureF();
    sensor.measure(tempC,relHum);
    Serial.print("SHT1x temperature: "); Serial.println(tempC);
    Serial.print("SHT1x humidity: "); Serial.println(relHum);
    Serial.print("\n");

    delay(100);
  }
}
