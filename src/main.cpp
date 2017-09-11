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

const long UPDATE_INTERVAL = 2000; // interval at which to blink (msec)
//also the DHT sensor requires 2 seconds between reads.

// Use software SPI: CS, DI, DO, CLK
//                            originally (10, 11, 12, 13)
// keeping pin 13 clear for use as indicator LED
const int SPI_CLOCK = 12;
const int SPI_DO = 11; //WRT to Arduino
const int SPI_DI = 10;
const int SPI_CS_THERM = 9;
const int SPI_CS_RTD = 8;


const float MAX31865_REFERENCE_RESISTOR = 430.0;
const float RTD_NOMINAL_RESISTANCE = 100.0;

float alarm_low = 10.0; // in degrees celsius
float alarm_high = 50.0;
float alarm_deadband = 2.0; // in degree celsius


// Initialize the temperature objects.
Temperature_Sensor myRTD(sensor_type::RTD_3WIRE,
                        SPI_CS_RTD,
                        SPI_DI,
                        SPI_DO,
                        SPI_CLOCK,
                        UPDATE_INTERVAL,
                        alarm_low,
                        alarm_high,
                        alarm_deadband,
                        RTD_NOMINAL_RESISTANCE,
                        MAX31865_REFERENCE_RESISTOR);


Temperature_Sensor myTC(sensor_type::TCTYPE_K,
                        SPI_CS_THERM,
                        SPI_DI,
                        SPI_DO,
                        SPI_CLOCK,
                        UPDATE_INTERVAL,
                        alarm_low,
                        alarm_high,
                        alarm_deadband);


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

  dht.begin();

  pinMode(LEDPin, OUTPUT);

  sensor.configureConnection();
  sensor.softReset();
}

float tempC = 0;
float relHum = 0;
temperature_channel_status RTD_data;

void loop()
{
  unsigned long currentMillis = millis();

  //uint16_t rtd = myRTD.readRTD();
  //tempRTD = myRTD.temperature(100, RREF);

  tempRTD = myRTD.getTemperature();
  RTD_data = myRTD.update();

  tempThermo = myTC.getTemperature();



  if (currentMillis - previousMillis > UPDATE_INTERVAL)
  {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    if ( (tempRTD >= 28.0) || (tempThermo >= 28.0) ) ledState = HIGH;
    else ledState = LOW;

    digitalWrite(LEDPin, ledState);

    //Serial.print("Resistance = "); Serial.println(RREF*ratio,8);

    // Serial.print("RTD Temp = "); Serial.println(tempRTD);
    // Serial.print("\n");
    Serial.print("RTD Temp = "); Serial.println(RTD_data.temperature);
    Serial.print("\n");



    Serial.print("Thermocouple Temp: "); Serial.println(tempThermo);
    Serial.print("\n");

    //DHT
//    Serial.print("DHT temperature: "); Serial.println(dht.readTemperature());
//    Serial.print("DHT humidity: "); Serial.println(dht.readHumidity());
//    Serial.print("\n");

    //tempF = sht1x.readTemperatureF();
    // sensor.measure(tempC,relHum);
    // Serial.print("SHT1x temperature: "); Serial.println(tempC);
    // Serial.print("SHT1x humidity: "); Serial.println(relHum);
    // Serial.print("\n");



    delay(100);
  }
}
