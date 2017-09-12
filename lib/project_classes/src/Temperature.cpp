
/* Temperature class implementation for Garden Party project.
 This class will be used for both RTD and thermocouple inputs.

 Creates the thermocouple or RTD device, initializes it, reads it,
 processes updates, provides a data structure with temperature and channel
 status values in it.

 Written by Seth Peasley, September 2017.
 */
 #include <Temperature.h>


// I used one large constructor to handle all both RTD and TC sensors.
Temperature_Sensor::Temperature_Sensor(sensor_type_temperature sensor_kind, int8_t spi_chip_sel,
                                      int8_t spi_arduino_data_out, int8_t spi_arduino_data_in,
                                      int8_t spi_clock, unsigned long update_interval,
                                      float alarm_high_setpoint, float alarm_low_setpoint, float alarm_deadband_setpoint,
                                      float rtd_nominal, float max31865_ref_resistor)
{
  _sensor_kind = sensor_kind;
  _spi_clock = spi_clock;
  _spi_chip_select = spi_chip_sel;
  _spi_arduino_data_out = spi_arduino_data_out;
  _spi_arduino_data_in = spi_arduino_data_in;

  _update_interval = update_interval;
  _previousUpdate = 0;

  _alarm_low_setpoint = alarm_low_setpoint;
  _alarm_high_setpoint = alarm_high_setpoint;
  _alarm_deadband_setpoint = alarm_deadband_setpoint;

  _rtd_nominal =  rtd_nominal;
  _reference_resistor = max31865_ref_resistor;

  // Based on the input of the sensor to initialize, sensor_kind,
  // initializes the channel
  switch (_sensor_kind)
  {
    case RTD_2WIRE:
    {
     _myRTD = new Adafruit_MAX31865(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
      this ->_myRTD -> begin(MAX31865_2WIRE);
      break;
    }
    case RTD_3WIRE:
    {
     _myRTD = new Adafruit_MAX31865(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
      this ->_myRTD -> begin(MAX31865_3WIRE);
      break;
    }
    case RTD_4WIRE:
    {
     _myRTD = new Adafruit_MAX31865(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
      this ->_myRTD -> begin(MAX31865_4WIRE);
      break;
    }

    case TCTYPE_B:
    {
      _myTC = new Adafruit_MAX31856(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
      this -> _myTC -> begin();
      this -> _myTC -> setThermocoupleType(MAX31856_TCTYPE_B);
      break;
    }
    case TCTYPE_E:
    {
      _myTC = new Adafruit_MAX31856(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
      this -> _myTC -> begin();
      this -> _myTC -> setThermocoupleType(MAX31856_TCTYPE_E);
      break;
    }
    case TCTYPE_J:
    {
      _myTC = new Adafruit_MAX31856(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
      this -> _myTC -> begin();
      this -> _myTC -> setThermocoupleType(MAX31856_TCTYPE_J);
      break;
    }
    case TCTYPE_K:
    {
      _myTC = new Adafruit_MAX31856(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
      this -> _myTC -> begin();
      this -> _myTC -> setThermocoupleType(MAX31856_TCTYPE_K);
      break;
    }
    case TCTYPE_N:
    {
      _myTC = new Adafruit_MAX31856(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
      this -> _myTC -> begin();
      this -> _myTC -> setThermocoupleType(MAX31856_TCTYPE_N);
      break;
    }
    case TCTYPE_R:
    {
      _myTC = new Adafruit_MAX31856(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
      this -> _myTC -> begin();
      this -> _myTC -> setThermocoupleType(MAX31856_TCTYPE_R);
      break;
    }
    case TCTYPE_S:
    {
      _myTC = new Adafruit_MAX31856(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
      this -> _myTC -> begin();
      this -> _myTC -> setThermocoupleType(MAX31856_TCTYPE_S);
      break;
    }
    case TCTYPE_T:
    {
      _myTC = new Adafruit_MAX31856(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
      this -> _myTC -> begin();
      this -> _myTC -> setThermocoupleType(MAX31856_TCTYPE_T);
      break;
    }
    default:
      break;
  }
}

// Update the channel values for temperature, alarming conditions,
// faulting conditions, good/bad channel status, and the current setpoint values.
// It only updates at the rate set by the update interval. This allows the
// devices to provide the most accuracy, or the most speed, as directed by
// the developer.
// Returns a STRUCT with current statuses.
temperature_channel_status Temperature_Sensor::update()
{
  _currentMillis = millis();
  if (_currentMillis - _previousUpdate > _update_interval)
  {
    _current_channel_status.temperature = getTemperature();
    _current_channel_status.channel_status = status_setpoints();
    _current_channel_status.channel_fault = status_faults();
    _current_channel_status.alarm_settings[0] = _alarm_low_setpoint;
    _current_channel_status.alarm_settings[1] = _alarm_high_setpoint;
    _current_channel_status.alarm_settings[2] = _alarm_deadband_setpoint;
  }
  return _current_channel_status;
}

// Changes the object's alarm setpoints after the object has been created.
// Allows for more flexibility with alarming channels.
void Temperature_Sensor::set_alarm_setpoints(float alarm_low_setpoint, float alarm_high_setpoint, float alarm_deadband_setpoint)
{
  _alarm_low_setpoint = alarm_low_setpoint;
  _alarm_high_setpoint = alarm_high_setpoint;
  _alarm_deadband_setpoint = alarm_deadband_setpoint;
}


// Provides the status of the sensor, including fault conditions, alarming conditions,
// and the current temperature
channel_conditions_temperature Temperature_Sensor::status_setpoints()
{
  float temperature = getTemperature();

  if (temperature > (_alarm_high_setpoint + _alarm_deadband_setpoint) ) return HIGH_OUT_OF_RANGE;
  else if (temperature < (_alarm_low_setpoint + _alarm_deadband_setpoint) ) return LOW_OUT_OF_RANGE;
  else return NORMAL;
}

// Provides any fault conditions that may exist in the channel.
fault_values_temperatures Temperature_Sensor::status_faults()
{
  if ( (_sensor_kind == RTD_2WIRE) || (_sensor_kind == RTD_3WIRE) || (_sensor_kind == RTD_4WIRE) )
  {
    uint8_t fault = this -> _myRTD -> readFault();
    switch (fault)
    {
      case 0:
        return NO_FAULT;
        break;
      case MAX31865_FAULT_HIGHTHRESH:
        return RTD_FAULT_HIGHTHRESH;
        break;
      case MAX31865_FAULT_LOWTHRESH:
        return RTD_FAULT_LOWTHRESH;
        break;
      case MAX31865_FAULT_REFINLOW:
        return RTD_FAULT_REFINLOW;
        break;
      case MAX31865_FAULT_REFINHIGH:
        return RTD_FAULT_REFINHIGH;
        break;
      case MAX31865_FAULT_RTDINLOW:
        return RTD_FAULT_RTDINLOW;
        break;
      case MAX31865_FAULT_OVUV:
        return RTD_FAULT_OVUV;
        break;
      default:
        return NO_FAULT;
        break;
    }
    this -> _myRTD -> clearFault();
  }
  else
  {
    uint8_t fault = this -> _myTC -> readFault();
    switch (fault)
    {
      case 0:
        return NO_FAULT;
        break;
      case MAX31856_FAULT_CJRANGE:
        return TC_FAULT_CJRANGE;
        break;
      case MAX31856_FAULT_TCRANGE:
        return TC_FAULT_TCRANGE;
        break;
      case MAX31856_FAULT_CJHIGH:
        return TC_FAULT_CJHIGH;
        break;
      case MAX31856_FAULT_CJLOW:
        return TC_FAULT_CJLOW;
        break;
      case MAX31856_FAULT_TCHIGH:
        return TC_FAULT_TCHIGH;
        break;
      case MAX31856_FAULT_TCLOW:
        return TC_FAULT_TCLOW;
        break;
      case MAX31856_FAULT_OVUV:
        return TC_FAULT_OVUV;
        break;
      case MAX31856_FAULT_OPEN:
        return TC_FAULT_OPEN;
        break;
      default:
        return NO_FAULT;
        break;
    }
    //The thermocouple library does not have a clearFault() method.
  }
}

// Calls the appropriate function to return the TC or RTD temperature value
float Temperature_Sensor::getTemperature(void)
{
  if ( (_sensor_kind == RTD_2WIRE) || (_sensor_kind == RTD_3WIRE) || (_sensor_kind == RTD_4WIRE) )
  {
    return getRTD_temperature();
  }
  else return getTC_temperature();
}

float Temperature_Sensor::getRTD_temperature()
{
  return this ->_myRTD -> temperature(_rtd_nominal, _reference_resistor);
}

float Temperature_Sensor::getTC_temperature()
{
  return this -> _myTC -> readThermocoupleTemperature();
}
