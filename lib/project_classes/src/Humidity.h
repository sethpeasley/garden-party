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
 #include <sht1xalt.h>

 typedef enum sensor_type_humidity
 {
    // =  0, // SHT10
   HUM_DHT11, // =  1, // DHT11
   HUM_DHT22, // =  2, // DHT22
   HUM_DHT21, // =  3, // DHT21
   HUM_AM2301, // = 4  // AM2301
   HUM_SHT10
 } sensor_type_humidity_t;


 // Is the parameter being measured in the normal range, high, or low out of range?
 typedef enum channel_conditions_humidity
 {
   HUMIDITY_NORMAL,
   HUMIDITY_HIGH_OUT_OF_RANGE,
   HUMIDITY_LOW_OUT_OF_RANGE
 }channel_conditions_humidity_t;

typedef enum channel_conditions_humidity_temperature
{
   HUMIDITY_TEMPERATURE_NORMAL,
   HUMIDITY_TEMPERATURE_HIGH_OUT_OF_RANGE,
   HUMIDITY_TEMPERATURE_LOW_OUT_OF_RANGE
 } channel_conditions_humidity_temperature_t;


 typedef enum fault_values_humidity // for future expansion
 {
   NO_FAULTS,
   placeholder_fault_0,
   placeholder_fault_1,
   placeholder_fault_2
 } fault_values_humidity_t;

 // Structure provided to the outside world. Contains the last measured
 // temperature, if the channel is high/normal/low in band, if the signal
 // processing device is faulted and it's fault value, is the data in this
 // channel good (device has no faults), and the current alarm setpoints as an
 // array, where alarm_settings[0] is HIGH_OUT_OF_RANGE setpoint,
 // alarm_settings[1] is LOW_OUT_OF_RANGE, and alarm_settings[2] is the
 // deadband (hysteresis value)
 struct humidity_channel_status
 {
   float humidity;
   float temperature;
   channel_conditions_humidity             humidity_channel_setpoint_status;
   channel_conditions_humidity_temperature humidity_temperature_channel_setpoint_status;
   fault_values_humidity channel_fault; //The DHT11 doesn't provide errors but the SHT does
   bool is_data_good;
   float alarm_settings[6]; // humidity lo, humidity hi, humidity deadband,
                            // temp lo, temp hi, temp deadband
 };



class Humidity_Sensor
{
public:         // sensor_type_humidity humidity_sensor,
  Humidity_Sensor(unsigned int  sensor_kind, unsigned int data_pin,
                  int clock_pin, word clock_pulse_width,
                  unsigned long update_interval,
                  float sensor_high_humidity_setpoint = 100.0, float sensor_low_humidity_setpoint = 0.0,
                  float sensor_deadband_humidity_setpoint = 2.0, float sensor_high_temperature_setpoint = 100.0,
                  float sensor_low_temperature_setpoint = 0.0, float sensor_deadband_temperature_setpoint = 2.0,
                  sht1xalt::voltage_t sensor_voltage = sht1xalt::VOLTAGE_5V,
                  sht1xalt::temp_units_t expressed_units = sht1xalt::UNITS_CELCIUS);
  // Provides the most recent data to a caller. Returns the STRUCT defined
  // above.
  humidity_channel_status update();

  // Provides a means to update the alarm setpoints after the object is
  // constructed.
  void set_alarm_setpoints_humidity(float sensor_high_humidity_setpoint, float sensor_low_humidity_setpoint, float sensor_deadband_humidity_setpoint);
  void set_alarm_setpoints_temperature(float sensor_high_temperature_setpoint, float sensor_low_temperature_setpoint, float sensor_deadband_temperature_setpoint);



private:
  //private member data -------------------------------------
  unsigned long _previousUpdate;
  unsigned long _currentMillis;
  unsigned long _update_interval;

  //sensor_type_humidity _sensor_kind;
  unsigned int _sensor_kind;
  uint8_t _data_pin;
  int _clock_pin;
  word _clock_pulse_width;
  sht1xalt::voltage_t _sensor_voltage;
  sht1xalt::temp_units_t _expressed_units;

  float _sensor_low_humidity_setpoint, _sensor_high_humidity_setpoint, _sensor_deadband_humidity_setpoint;
  float _sensor_low_temperature_setpoint, _sensor_high_temperature_setpoint, _sensor_deadband_temperature_setpoint;

  humidity_channel_status _current_channel_status;

  DHT* _myDHT;
  sht1xalt::Sensor* _mySHT1x;
  //------------------------------------- private member data

  // private member methods -------------------------------------
  float getHumidity();
  float getTemperature();

  channel_conditions_humidity status_setpoints_humidity();
  channel_conditions_humidity_temperature status_setpoints_humidity_temperature();
  fault_values_humidity status_faults();
  //------------------------------------- private member methods


};
