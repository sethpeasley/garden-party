#include <Temperature.h>





  // Software (bitbang) SPI
  Temperature_Sensor::Temperature_Sensor(sensor_type sensor_kind, int8_t spi_cs, int8_t spi_mosi, int8_t spi_miso, int8_t spi_clk, int update_rate)
  {
    _sclk = spi_clk;
    _cs = spi_cs;
    _miso = spi_miso;
    _mosi = spi_mosi;

    switch (sensor_kind)
    {
      case RTD_2WIRE:
      {
        myRTD = new Adafruit_MAX31865(spi_cs, spi_mosi, spi_miso, spi_clk);
        this -> myRTD -> begin(MAX31865_2WIRE);
        break;
      }
      case RTD_3WIRE:
      {
        myRTD = new Adafruit_MAX31865(spi_cs, spi_mosi, spi_miso, spi_clk);
        this -> myRTD -> begin(MAX31865_3WIRE);
        break;
      }
      case RTD_4WIRE:
      {
        myRTD = new Adafruit_MAX31865(spi_cs, spi_mosi, spi_miso, spi_clk);
        this -> myRTD -> begin(MAX31865_4WIRE);
        break;
      }

      case TCTYPE_B:
      {
        myThermocouple = new Adafruit_MAX31856(spi_cs, spi_mosi, spi_miso, spi_clk);
        this -> myThermocouple -> begin();
        this -> myThermocouple -> setThermocoupleType(MAX31856_TCTYPE_B);
        break;
      }

      case TCTYPE_E:
      {
        myThermocouple = new Adafruit_MAX31856(spi_cs, spi_mosi, spi_miso, spi_clk);
        this -> myThermocouple -> begin();
        this -> myThermocouple -> setThermocoupleType(MAX31856_TCTYPE_E);
        break;
      }

      case TCTYPE_J:
      {
        myThermocouple = new Adafruit_MAX31856(spi_cs, spi_mosi, spi_miso, spi_clk);
        this -> myThermocouple -> begin();
        this -> myThermocouple -> setThermocoupleType(MAX31856_TCTYPE_J);
        break;
      }

      case TCTYPE_K:
      {
        myThermocouple = new Adafruit_MAX31856(spi_cs, spi_mosi, spi_miso, spi_clk);
        this -> myThermocouple -> begin();
        this -> myThermocouple -> setThermocoupleType(MAX31856_TCTYPE_K);
        break;
      }

      // TCTYPE_N = MAX31856_TCTYPE_N,
      // TCTYPE_R = MAX31856_TCTYPE_R,
      // TCTYPE_S = MAX31856_TCTYPE_S,
      // TCTYPE_T = MAX31856_TCTYPE_T
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
