/* Temperature class for Garden Party project.
 This class will be used for both RTD and thermocouple inputs.

 Creates the thermocouple or RTD object, initializes it,


 Written by Seth Peasley, September 2017.
 */

//#define RREF 430.0

#include <Adafruit_MAX31865.h>
#include <Adafruit_MAX31856.h>

const float RREF = 430.0;

namespace sensor_types
{
   enum sensor_type {
    RTD_2WIRE = MAX31865_2WIRE,
    RTD_3WIRE = MAX31865_3WIRE,
    RTD_4WIRE = MAX31865_4WIRE,
    TCTYPE_B = MAX31856_TCTYPE_B,
    TCTYPE_E = MAX31856_TCTYPE_E,
    TCTYPE_J = MAX31856_TCTYPE_J,
    TCTYPE_K = MAX31856_TCTYPE_K,
    TCTYPE_N = MAX31856_TCTYPE_N,
    TCTYPE_R = MAX31856_TCTYPE_R,
    TCTYPE_S = MAX31856_TCTYPE_S,
    TCTYPE_T = MAX31856_TCTYPE_T
  } sensor_type_t;
}


class Temperature_Sensor {
 public:
  Temperature_Sensor(sensor_types::sensor_type temp_Sensor, int8_t spi_cs, int8_t spi_mosi, int8_t spi_miso, int8_t spi_clk, int update_rate);
  //Adafruit_MAX31865(int8_t spi_cs);

  // boolean begin(max31865_numwires_t x = MAX31865_2WIRE);
  //
  // uint8_t readFault(void);
  // void clearFault(void);
  // uint16_t readRTD();
  //
  //
  // void setWires(max31865_numwires_t wires);
  // void autoConvert(boolean b);
  // void enableBias(boolean b);
  //
  // float temperature(float RTDnominal, float refResistor);
  void update();

 private:
  int8_t _sclk, _miso, _mosi, _cs;
  int update_rate;
  //
  // void initDevice();
  // void readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n);
  //
  // uint8_t  readRegister8(uint8_t addr);
  // uint16_t readRegister16(uint8_t addr);
  //
  // void     writeRegister8(uint8_t addr, uint8_t reg);
  // uint8_t spixfer(uint8_t addr);

  Adafruit_MAX31856* myThermocouple;
  Adafruit_MAX31865* myRTD;

};
