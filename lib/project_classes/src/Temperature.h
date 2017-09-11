/* Temperature class header for Garden Party project.
 This class will be used for both RTD and thermocouple inputs.

 Creates the thermocouple or RTD device, initializes it, reads it,
 processes updates, provides a data structure with temperature and channel
 status values in it.

 Written by Seth Peasley, September 2017.
 */


#include <Adafruit_MAX31865.h>
#include <Adafruit_MAX31856.h>

#include <SPI.h>

// This provides a more generic way to address the various devices we might use.
typedef enum sensor_type_temperature
{
  RTD_2WIRE = 0, //MAX31865_2WIRE,
  RTD_3WIRE = 1, // MAX31865_3WIRE,
  RTD_4WIRE = 2, // MAX31865_4WIRE,
  TCTYPE_B =  3, // MAX31856_TCTYPE_B,
  TCTYPE_E =  4, // MAX31856_TCTYPE_E,
  TCTYPE_J =  5, // MAX31856_TCTYPE_J,
  TCTYPE_K =  6, // MAX31856_TCTYPE_K,
  TCTYPE_N =  7, // MAX31856_TCTYPE_N,
  TCTYPE_R =  8, // MAX31856_TCTYPE_R,
  TCTYPE_S =  9, // MAX31856_TCTYPE_S,
  TCTYPE_T = 10  // MAX31856_TCTYPE_T
  // Note: not currently supporting the "voltage mode" settings which can be used
  // in the MASC31856. See data sheet for more information.
} sensor_type_temperature_t;

// Is the parameter being measured in the normal range, high, or low out of range?
typedef enum channel_conditions
{
  NORMAL,
  HIGH_OUT_OF_RANGE,
  LOW_OUT_OF_RANGE
} channel_conditions_t;

// The library from Adafruit provides fault information. This information can be
// used to assess if the channel has good data.
typedef enum fault_values
{
  NO_FAULT,             // Normal No Fault Condition

  RTD_FAULT_HIGHTHRESH, // MAX31865_FAULT_HIGHTHRESH
  RTD_FAULT_LOWTHRESH,  // MAX31865_FAULT_REFINLOW
  RTD_FAULT_REFINLOW,   // MAX31865_FAULT_REFINHIGH
  RTD_FAULT_REFINHIGH,  // MAX31865_FAULT_REFINHIGH
  RTD_FAULT_RTDINLOW,   // MAX31865_FAULT_RTDINLOW
  RTD_FAULT_OVUV,       // MAX31865_FAULT_OVUV

  TC_FAULT_CJRANGE,     // MAX31856_FAULT_CJRANGE
  TC_FAULT_TCRANGE,     // MAX31856_FAULT_TCRANGE
  TC_FAULT_CJHIGH,      // MAX31856_FAULT_CJHIGH
  TC_FAULT_CJLOW,       // MAX31856_FAULT_CJLOW
  TC_FAULT_TCHIGH,      // MAX31856_FAULT_TCHIGH
  TC_FAULT_TCLOW,       // MAX31856_FAULT_TCLOW
  TC_FAULT_OVUV,         // MAX31856_FAULT_OVUV
  TC_FAULT_OPEN         // MAX31856_FAULT_OPEN
} fault_values_t;

// Structure provided to the outside world. Contains the last measured
// temperature, if the channel is high/normal/low in band, if the signal
// processing device is faulted and it's fault value, is the data in this
// channel good (device has no faults), and the current alarm setpoints as an
// array, where alarm_settings[0] is HIGH_OUT_OF_RANGE setpoint,
// alarm_settings[1] is LOW_OUT_OF_RANGE, and alarm_settings[2] is the
// deadband (hysteresis value)
struct temperature_channel_status
{
  float temperature;
  channel_conditions channel_status;
  fault_values channel_fault;
  bool is_data_good;
  float alarm_settings[3];
};

//This class has one constructor for use in both RTD and Thermocouple applications.
class Temperature_Sensor
{
  public:
    Temperature_Sensor(sensor_type_temperature temp_sensor, int8_t spi_chip_sel, int8_t spi_arduino_data_out,
                    int8_t spi_arduino_data_in, int8_t spi_clock, unsigned long update_interval,
                    float alarm_low = 0.0, float alarm_high = 100.0, float alarm_deadband = 2.0,
                    float rtd_nominal = 100.0, float max31865_ref_resistor = 430.0);


    // Provides the most recent data to a caller. Returns the STRUCT defined
    // above.
    temperature_channel_status update();

    // Provides a means to update the alarm setpoints after the object is
    // constructed.
    void set_alarm_setpoints(float alarm_low, float alarm_high, float alarm_deadband);


  private:
    //private member data -------------------------------------
    unsigned long _previousUpdate = 0;
    unsigned long _currentMillis;

    sensor_type_temperature _sensor_kind;
    int8_t _spi_clock, _spi_chip_select, _spi_arduino_data_out, _spi_arduino_data_in;
    unsigned long _update_interval;
    float _rtd_nominal, _reference_resistor;
    float _alarm_low, _alarm_high, _alarm_deadband;
    bool _temp_hi, _temp_lo;

    // Only one will be used per object, but to make this as generic as possible,
    // I include a pointer to each kind here. This simplifies the construction
    // of the object.
    Adafruit_MAX31856* _myTC;
    Adafruit_MAX31865* _myRTD;

    temperature_channel_status _current_channel_status;
    //------------------------------------- private member data

    // private member methods -------------------------------------

    float getTemperature();
    float getRTD_temperature();
    float getTC_temperature();
    channel_conditions status_setpoints();
    fault_values status_faults();
    //------------------------------------- private member methods
};
