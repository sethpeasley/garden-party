/* Garden Party Main code

Initializing code and main loop for Garden Party.

*/

#include <Arduino.h>

#include <Temperature.h>
#include <Humidity.h>


const int LEDPin = 13; //LED_BUILTIN; // the number of a LED pin
int ledState = LOW; // LED state used to set the LED

// Generally, we want to use an unsigned long for variables that
// hold time, because the value will quickly become too large for an int
unsigned long previousMillis = 0; // stores last time LED updated

const unsigned long TEMPERATURE_UPDATE_INTERVAL = 500; // interval at which to blink (msec)
const unsigned long HUMIDITY_UPDATE_INTERVAL = 2000;
const unsigned long UI_UPDATE_INTERVAL = 1000;
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
Temperature_Sensor myRTD(sensor_type_temperature::RTD_3WIRE,
                        SPI_CS_RTD,
                        SPI_DI,
                        SPI_DO,
                        SPI_CLOCK,
                        TEMPERATURE_UPDATE_INTERVAL,
                        alarm_low,
                        alarm_high,
                        alarm_deadband,
                        RTD_NOMINAL_RESISTANCE,
                        MAX31865_REFERENCE_RESISTOR);


Temperature_Sensor myTC(sensor_type_temperature::TCTYPE_K,
                        SPI_CS_THERM,
                        SPI_DI,
                        SPI_DO,
                        SPI_CLOCK,
                        TEMPERATURE_UPDATE_INTERVAL,
                        alarm_low,
                        alarm_high,
                        alarm_deadband);




//configure Humidity Detectors


//DHT Sensor0
uint8_t HUMIDITY_SENSOR_0_DATA_PIN = 7;
const unsigned int HUMIDITY_SENSOR_0_TYPE = 22;// HUM_DHT22; //DHT22 style sensor
float humidity_setpoint_hi = 95.0;
float humidity_setpoint_lo = 5.0;// in Relative humidity
float humidity_setpoint_deadband = 2.5;
float humidity_temperature_setpoint_hi = 50.0; // in degrees celsius
float humidity_temperature_setpoint_lo = 10.0;
float humidity_temperature_setpoint_deadband = 2.0;
//DHT dht = DHT(DHTPIN, DHTTYPE);

Humidity_Sensor dht0(HUMIDITY_SENSOR_0_TYPE,
                    HUMIDITY_SENSOR_0_DATA_PIN,
                    0,
                    0,
                    HUMIDITY_UPDATE_INTERVAL,
                    humidity_setpoint_hi, humidity_setpoint_lo,
                    humidity_setpoint_deadband, humidity_temperature_setpoint_hi,
                    humidity_temperature_setpoint_lo, humidity_temperature_setpoint_deadband,
                    sht1xalt::VOLTAGE_5V, sht1xalt::UNITS_CELCIUS);

//configure moisture sensor
// #define dataPin 5
// #define clockPin 4
// #define clockPulse 1
// #define voltage sht1xalt::VOLTAGE_5V
// #define units sht1xalt::UNITS_CELCIUS
//sht1xalt::Sensor sensor(dataPin, clockPin, clockPulse, voltage, units);

uint8_t HUMIDITY_SENSOR_1_DATA_PIN = 5;
const int HUMIDITY_SENSOR_1_CLOCK_PIN = 4;
const unsigned int HUMIDITY_SENSOR_1_TYPE = 2;// SHT1x
const int HUMIDITY_SENSOR_1_CLOCK_PULSE_WIDTH = 1;
const sht1xalt::voltage_t HUMIDITY_SENSOR_1_VOLTAGE = sht1xalt::VOLTAGE_5V;
const sht1xalt::temp_units_t HUMIDITY_SENSOR_1_UNITS = sht1xalt::UNITS_CELCIUS;
Humidity_Sensor blurn(HUMIDITY_SENSOR_1_TYPE,
                      HUMIDITY_SENSOR_1_DATA_PIN,
                      HUMIDITY_SENSOR_1_CLOCK_PIN,
                      HUMIDITY_SENSOR_1_CLOCK_PULSE_WIDTH,
                      HUMIDITY_UPDATE_INTERVAL,
                      humidity_setpoint_hi, humidity_setpoint_lo,
                      humidity_setpoint_deadband, humidity_temperature_setpoint_hi,
                      humidity_temperature_setpoint_lo, humidity_temperature_setpoint_deadband,
                      HUMIDITY_SENSOR_1_VOLTAGE, HUMIDITY_SENSOR_1_UNITS);






void setup()
{
  Serial.begin(115200);
  Serial.println("Sensors Testing Commence!\n");

  // dht.begin();

  //set_alarm_setpoints(alarm_low, alarm_high, alarm_deadband)
  myRTD.set_alarm_setpoints(23.0, 26.0, 0.5);
  myTC.set_alarm_setpoints(21.2, 27.8, 0.7);


  pinMode(LEDPin, OUTPUT);

  // sensor.configureConnection();
  // sensor.softReset();
}

float tempC = 0;
float relHum = 0;
temperature_channel_status RTD_data;
temperature_channel_status TC_data;
humidity_channel_status humid0_data;
humidity_channel_status humid1_data;

void loop()
{
  unsigned long currentMillis = millis();

  //update the internal states of the objects
  RTD_data = myRTD.update();
  TC_data = myTC.update();
  humid0_data = dht0.update();
  //humid1_data = blurn.update();

  // process any alarms
  if ((RTD_data.channel_status == HIGH_OUT_OF_RANGE) ||
      (RTD_data.channel_status == LOW_OUT_OF_RANGE) ||
      (TC_data.channel_status == HIGH_OUT_OF_RANGE) ||
      (TC_data.channel_status == LOW_OUT_OF_RANGE) )
       ledState = HIGH;
  else ledState = LOW;

  digitalWrite(LEDPin, ledState);

  if (currentMillis - previousMillis > UI_UPDATE_INTERVAL)
  {
    // last time the screen was updated.
    previousMillis = currentMillis;

    Serial.print("RTD Temp = "); Serial.println(RTD_data.temperature);
    Serial.print("Thermocouple Temp: "); Serial.println(TC_data.temperature);
  //  Serial.print("\n");

    //DHT
    Serial.print("DHT temperature: "); Serial.println(humid0_data.temperature);
    Serial.print("DHT humidity: "); Serial.println(humid0_data.humidity);
    //Serial.print("\n");

    //tempF = sht1x.readTemperatureF();
    // sensor.measure(tempC,relHum);


    // Serial.print("SHT1x temperature: "); Serial.println(humid1_data.temperature);
    // Serial.print("SHT1x humidity: "); Serial.println(humid1_data.humidity);
    // Serial.print("\n");


    // sensor.measure(tempC, relHum);
    // Serial.print("SHT1x temperature: "); Serial.println(tempC);
    // Serial.print("SHT1x humidity: "); Serial.println(relHum);
    // Serial.print("\n");


    //delay(100);
  }
}
