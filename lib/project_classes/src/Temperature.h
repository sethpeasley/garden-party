/* Temperature class for Garden Party project.
 This class will be used for both RTD and thermocouple inputs.
 Written by Seth Peasley, September 2017.
 */

//#define RREF 430.0

const float RREF = 430.0;


typedef enum sensor_type {
  MAX31865_2WIRE = 0,
  MAX31865_3WIRE = 1,
  MAX31865_4WIRE = 0
} sensor_type_t;
typedef enum
{
  MAX31856_TCTYPE_B  = 0b0000,
  MAX31856_TCTYPE_E  = 0b0001,
  MAX31856_TCTYPE_J  = 0b0010,
  MAX31856_TCTYPE_K  = 0b0011,
  MAX31856_TCTYPE_N  = 0b0100,
  MAX31856_TCTYPE_R  = 0b0101,
  MAX31856_TCTYPE_S  = 0b0110,
  MAX31856_TCTYPE_T  = 0b0111,
  MAX31856_VMODE_G8  = 0b1000,
  MAX31856_VMODE_G32 = 0b1100,
} max31856_thermocoupletype_t;



// class Temperature_Sensor {
//  public:
//   Adafruit_MAX31865(int8_t spi_cs, int8_t spi_mosi, int8_t spi_miso, int8_t spi_clk);
//   Adafruit_MAX31865(int8_t spi_cs);
//
//   boolean begin(max31865_numwires_t x = MAX31865_2WIRE);
//
//   uint8_t readFault(void);
//   void clearFault(void);
//   uint16_t readRTD();
//
//
//   void setWires(max31865_numwires_t wires);
//   void autoConvert(boolean b);
//   void enableBias(boolean b);
//
//   float temperature(float RTDnominal, float refResistor);
//
//  private:
//   int8_t _sclk, _miso, _mosi, _cs;
//
//   void readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n);
//
//   uint8_t  readRegister8(uint8_t addr);
//   uint16_t readRegister16(uint8_t addr);
//
//   void     writeRegister8(uint8_t addr, uint8_t reg);
//   uint8_t spixfer(uint8_t addr);
// };
