/* Temperature class for Garden Party project.
 This class will be used for both RTD and thermocouple inputs.

 Creates the thermocouple or RTD device, initializes it, reads it,


 Written by Seth Peasley, September 2017.
 */


#include <Adafruit_MAX31865.h>
#include <Adafruit_MAX31856.h>

typedef enum sensor_type
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
} sensor_type_t;

typedef enum channel_conditions
{
  NORMAL,
  HIGH_OUT_OF_RANGE,
  LOW_OUT_OF_RANGE
} channel_conditions_t;

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

struct temperature_channel_status
{
  float temperature;
  channel_conditions channel_status;
  fault_values channel_fault;
  bool is_data_good;
  float alarm_settings[3];
};

class Temperature_Sensor {
 public:
  Temperature_Sensor(sensor_type temp_sensor, int8_t spi_chip_sel, int8_t spi_arduino_data_out,
                    int8_t spi_arduino_data_in, int8_t spi_clock, int update_rate,
                    float alarm_low = 0.0, float alarm_high = 100.0, float alarm_deadband = 2.0,
                    float rtd_nominal = 100.0, float max31865_ref_resistor = 430.0);


  temperature_channel_status update();
  float getTemperature();
  void set_alarm_setpoints(float alarm_low = 0.0, float alarm_high = 100.0, float alarm_deadband = 2.0);

 private:
  sensor_type _sensor_kind;
  int8_t _spi_clock, _spi_chip_select, _spi_arduino_data_out, _spi_arduino_data_in;
  int _update_rate;
  float _rtd_nominal, _reference_resistor;
  float _alarm_low, _alarm_high, _alarm_deadband;
  bool _temp_hi, _temp_lo;

  Adafruit_MAX31856* _myTC;
  Adafruit_MAX31865* _myRTD;

  float getRTD_temperature();
  float getTC_temperature();
  channel_conditions status_setpoints();
  fault_values status_faults();



};
