/* Humidity class implementation for Garden Party project.
 This class will be used for the humidity / moisture sensors.

 Creates the humidity sensor object, initializes it, reads it,
 processes updates, provides a data structure with humidity, and channel
 status values in it.

 Written by Seth Peasley, September 2017.
 */

 #include <Humidity.h>



Humidity_Sensor::Humidity_Sensor(sensor_type_humidity humid_sensor,
                                uint8_t data_pin, int clock_pin,
                                unsigned int clock_pulse_width,
                                unsigned long update_interval,
                                float sensor_high_humidity_setpoint, float sensor_low_humidity_setpoint,
                                float sensor_deadband_humidity_setpoint, float sensor_high_temperature_setpoint,
                                float sensor_low_temperature_setpoint, float sensor_deadband_temperature_setpoint,
                                sht1xalt::voltage_t sensor_voltage, sht1xalt::temp_units_t expressed_units)
{
  _sensor_kind = humid_sensor;
  _data_pin = data_pin;
  _clock_pin = clock_pin;
  _clock_pulse_width = clock_pulse_width;
  _sensor_voltage = sensor_voltage;
  _expressed_units = expressed_units;

  _previousUpdate = 0;
  _update_interval = update_interval;

  _sensor_low_humidity_setpoint = sensor_low_humidity_setpoint;
  _sensor_high_humidity_setpoint = sensor_high_humidity_setpoint;
  _sensor_deadband_humidity_setpoint = sensor_deadband_humidity_setpoint;
  _sensor_low_temperature_setpoint = sensor_low_temperature_setpoint;
  _sensor_high_temperature_setpoint = sensor_high_temperature_setpoint;
  _sensor_deadband_temperature_setpoint = sensor_deadband_temperature_setpoint;

  // Based on the input of the sensor to initialize, sensor_kind,
  // initializes the channel
  switch (_sensor_kind)
  {
    case HUM_SHT10:
    {
      _mySHT1x = new sht1xalt::Sensor(_data_pin, _clock_pin, _clock_pulse_width, _sensor_voltage, _expressed_units);
      this -> _mySHT1x -> configureConnection();
      this -> _mySHT1x -> softReset();
      break;
    }

    case HUM_DHT11:
    {
      _myDHT = new DHT(_data_pin, _sensor_kind);
      this -> _myDHT -> begin();
      break;
    }

    case HUM_DHT21: //Also the AM2301
    {
      _myDHT = new DHT(_data_pin, _sensor_kind);
      this -> _myDHT -> begin();
      break;
    }

    case HUM_DHT22:
    {
      _myDHT = new DHT(_data_pin, _sensor_kind);
      this -> _myDHT -> begin();
      break;
    }
  }



}


// Update the channel values for humidyt and temperature, alarming conditions,
// faulting conditions, good/bad channel status, and the current setpoint values.
// It only updates at the rate set by the update interval. This allows the
// devices to provide the most accuracy, or the most speed, as directed by
// the developer.
// Returns a STRUCT with current statuses.
humidity_channel_status Humidity_Sensor::update()
{
  _currentMillis = millis();
  if (_currentMillis - _previousUpdate > _update_interval)
  {
    _current_channel_status.humidity = getHumidity();
    _current_channel_status.temperature = getTemperature();
//    _current_channel_status.channel_conditions = status_setpoints();
    _current_channel_status.humidity_channel_setpoint_status = status_setpoints_humidity();
    _current_channel_status.humidity_temperature_channel_setpoint_status = status_setpoints_humidity_temperature();
    _current_channel_status.channel_fault = status_faults();
    // humidity lo, humidity hi, humidity deadband,
                             // temp lo, temp hi, temp deadband

    _current_channel_status.alarm_settings[0] = _sensor_high_humidity_setpoint;
    _current_channel_status.alarm_settings[1] = _sensor_low_humidity_setpoint;
    _current_channel_status.alarm_settings[2] = _sensor_deadband_humidity_setpoint;
    _current_channel_status.alarm_settings[3] = _sensor_high_temperature_setpoint;
    _current_channel_status.alarm_settings[4] = _sensor_low_temperature_setpoint;
    _current_channel_status.alarm_settings[5] = _sensor_deadband_temperature_setpoint;
  }
  return _current_channel_status;
}



float Humidity_Sensor::getHumidity()
{
  if (_sensor_kind == HUM_SHT10)
  {
    float temperature, humidity;
    this -> _mySHT1x -> measure(temperature, humidity); // temperature compensates, should be more accurate
    return humidity;
  }
  else
  {
    return this -> _myDHT -> readHumidity();
  }
}

float Humidity_Sensor::getTemperature()
{
  if (_sensor_kind == HUM_SHT10)
  {
    float temperature, humidity;
    this -> _mySHT1x -> measure(temperature, humidity); // temperature compensates, should be more accurate
    return temperature;
  }
  else
  {
    return this -> _myDHT -> readTemperature();
  }
}

// Provides the status of the sensor, including fault conditions, alarming conditions,
// and the current temperature
channel_conditions_humidity Humidity_Sensor::status_setpoints_humidity()
{
  float humidity = getHumidity();

  if (humidity > (_sensor_high_humidity_setpoint + _sensor_deadband_humidity_setpoint) ) return HUMIDITY_HIGH_OUT_OF_RANGE;
  else if (humidity < (_sensor_low_humidity_setpoint + _sensor_deadband_humidity_setpoint) ) return HUMIDITY_LOW_OUT_OF_RANGE;
  else return HUMIDITY_NORMAL;
}

channel_conditions_humidity_temperature Humidity_Sensor::status_setpoints_humidity_temperature()
{
  float temperature = getTemperature();

  if (temperature > (_sensor_high_temperature_setpoint + _sensor_deadband_temperature_setpoint) ) return HUMIDITY_TEMPERATURE_HIGH_OUT_OF_RANGE;
  else if (temperature < (_sensor_low_temperature_setpoint + _sensor_deadband_temperature_setpoint) ) return HUMIDITY_TEMPERATURE_LOW_OUT_OF_RANGE;
  else return HUMIDITY_TEMPERATURE_NORMAL;
}


fault_values_humidity Humidity_Sensor::status_faults()
{
  return NO_FAULTS; // placeholder, come bck to this later
}



// Provides a means to update the alarm setpoints after the object is
// constructed.
void Humidity_Sensor::set_alarm_setpoints_humidity(float sensor_high_humidity_setpoint, float sensor_low_humidity_setpoint, float sensor_deadband_humidity_setpoint)
{
  _sensor_high_humidity_setpoint = sensor_high_humidity_setpoint;
  _sensor_low_humidity_setpoint = sensor_low_humidity_setpoint;
  _sensor_deadband_humidity_setpoint = sensor_deadband_humidity_setpoint;
}


void Humidity_Sensor::set_alarm_setpoints_temperature(float sensor_high_temperature_setpoint, float sensor_low_temperature_setpoint, float sensor_deadband_temperature_setpoint)
{
  _sensor_high_temperature_setpoint = sensor_high_temperature_setpoint;
  _sensor_low_temperature_setpoint = sensor_low_temperature_setpoint;
  _sensor_deadband_temperature_setpoint = sensor_deadband_temperature_setpoint;
}
