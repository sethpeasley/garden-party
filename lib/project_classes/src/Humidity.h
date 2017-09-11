/* Humidity class header for Garden Party project.
 This class will be used for the humidity / moisture sensors.

 Creates the humidity sensor object, initializes it, reads it,
 processes updates, provides a data structure with humidity, and channel
 status values in it.

 Written by Seth Peasley, September 2017.
 */

 #include <Adafruit_Sensor.h>
 #include <DHT.h>
 #include <SHT1x.h>



 typedef enum sensor_type_humidity
 {
   HUM_SHT10 =   1, // SHT10
   HUM_DHT11 =  11, // DHT11
   HUM_DHT22 =  22, // DHT22
   HUM_DHT21 =  21, // DHT21
   HUM_AM2301 = 21 // AM2301
 } sensor_type_humidity_t;



class Humidity_Sensor
{
public:
  Humidity_Sensor(sensor_type_humidity humid_sensor, uint8_t data_pin,
                  int clock_pin, unsigned int clock_pulse_width,
                  unsigned long update_interval,
                  float alarm_low_humidity = 0.0, float alarm_high_humidity = 100.0,
                  float alarm_deadband_humidity = 2.0, float alarm_low_temperature = 0.0,
                  float alarm_high_temperature = 100.0, float alarm_deadband_temperature = 2.0,
                  sht1xalt::voltage_t sensor_voltage = sht1xalt::VOLTAGE_5V,
                  sht1xalt::temp_units_t expressed_units = sht1xalt::UNITS_CELCIUS);



private:
  //private member data -------------------------------------
  unsigned long _previousUpdate = 0;
  unsigned long _currentMillis;

  sensor_type_humidity _humid_sensor;
  uint8_t _data_pin;
  int _clock_pin;
  unsigned int _clock_pulse_width;
  sht1xalt::voltage_t sensor_voltage;
  sht1xalt::temp_units_t expressed_units;






  DHT* _myDHT;
  sht1xalt::Sensor* _mySHT1x;



};
