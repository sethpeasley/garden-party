
#include <Temperature.h>


  // Software (bitbang) SPI
  Temperature_Sensor::Temperature_Sensor(sensor_type sensor_kind, int8_t spi_chip_sel, int8_t spi_arduino_data_out, int8_t spi_arduino_data_in, int8_t spi_clock, int update_rate)
  {
    _spi_clock = spi_clock;
    _spi_chip_select = spi_chip_sel;
    _spi_arduino_data_out = spi_arduino_data_out;
    _spi_arduino_data_in = spi_arduino_data_in;

    switch (sensor_kind)
    {
      case RTD_2WIRE:
      {
        myRTD = new Adafruit_MAX31865(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
        this -> myRTD -> begin(MAX31865_2WIRE);
        break;
      }
      case RTD_3WIRE:
      {
        myRTD = new Adafruit_MAX31865(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
        this -> myRTD -> begin(MAX31865_3WIRE);
        break;
      }
      case RTD_4WIRE:
      {
        myRTD = new Adafruit_MAX31865(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
        this -> myRTD -> begin(MAX31865_4WIRE);
        break;
      }

      case TCTYPE_B:
      {
        myThermocouple = new Adafruit_MAX31856(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
        this -> myThermocouple -> begin();
        this -> myThermocouple -> setThermocoupleType(MAX31856_TCTYPE_B);
        break;
      }
      case TCTYPE_E:
      {
        myThermocouple = new Adafruit_MAX31856(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
        this -> myThermocouple -> begin();
        this -> myThermocouple -> setThermocoupleType(MAX31856_TCTYPE_E);
        break;
      }
      case TCTYPE_J:
      {
        myThermocouple = new Adafruit_MAX31856(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
        this -> myThermocouple -> begin();
        this -> myThermocouple -> setThermocoupleType(MAX31856_TCTYPE_J);
        break;
      }
      case TCTYPE_K:
      {
        myThermocouple = new Adafruit_MAX31856(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
        this -> myThermocouple -> begin();
        this -> myThermocouple -> setThermocoupleType(MAX31856_TCTYPE_K);
        break;
      }
      case TCTYPE_N:
      {
        myThermocouple = new Adafruit_MAX31856(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
        this -> myThermocouple -> begin();
        this -> myThermocouple -> setThermocoupleType(MAX31856_TCTYPE_N);
        break;
      }
      case TCTYPE_R:
      {
        myThermocouple = new Adafruit_MAX31856(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
        this -> myThermocouple -> begin();
        this -> myThermocouple -> setThermocoupleType(MAX31856_TCTYPE_R);
        break;
      }
      case TCTYPE_S:
      {
        myThermocouple = new Adafruit_MAX31856(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
        this -> myThermocouple -> begin();
        this -> myThermocouple -> setThermocoupleType(MAX31856_TCTYPE_S);
        break;
      }
      case TCTYPE_T:
      {
        myThermocouple = new Adafruit_MAX31856(spi_chip_sel, spi_arduino_data_out, spi_arduino_data_in, spi_clock);
        this -> myThermocouple -> begin();
        this -> myThermocouple -> setThermocoupleType(MAX31856_TCTYPE_T);
        break;
      }
      default:
        break;
    }



  }

  // // Hardware SPI init
  // Adafruit_MAX31856::Adafruit_MAX31856(int8_t spi_cs) {
  //   _cs = spi_cs;
  //   _sclk = _miso = _mosi = -1;
  // }
  //
  // boolean Adafruit_MAX31856::begin(void) {
  //   pinMode(_cs, OUTPUT);
  //   digitalWrite(_cs, HIGH);
  //
  //   if (_sclk != -1) {
  //     //define pin modes
  //     pinMode(_sclk, OUTPUT);
  //     pinMode(_mosi, OUTPUT);
  //     pinMode(_miso, INPUT);
  //   } else {
  //     //start and configure hardware SPI
  //     SPI.begin();
  //   }
  // }
