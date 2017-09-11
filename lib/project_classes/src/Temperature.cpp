#include <Temperature.h>





  // Software (bitbang) SPI
  Temperature_Sensor::Temperature_Sensor(sensor_types::sensor_type sensor_kind, int8_t spi_cs, int8_t spi_mosi, int8_t spi_miso, int8_t spi_clk, int update_rate)
  {
    _sclk = spi_clk;
    _cs = spi_cs;
    _miso = spi_miso;
    _mosi = spi_mosi;

    switch (sensor_kind)
    {
      case sensor_types::RTD_2WIRE:
        myRTD = new Adafruit_MAX31865(spi_cs, spi_mosi, spi_miso, spi_clk);
        //this -> myRTD -> begin(MAX31865_2WIRE);
        break;

      case sensor_types::RTD_3WIRE:
        myRTD = new Adafruit_MAX31865(spi_cs, spi_mosi, spi_miso, spi_clk);
        //this -> myRTD -> begin(MAX31865_3WIRE);
        break;

      case sensor_types::RTD_4WIRE:
        myRTD = new Adafruit_MAX31865(spi_cs, spi_mosi, spi_miso, spi_clk);
        //this -> myRTD -> begin(MAX31865_4WIRE);
        break;


      // RTD_4WIRE = MAX31865_4WIRE,
      // TCTYPE_B = MAX31856_TCTYPE_B,
      // TCTYPE_E = MAX31856_TCTYPE_E,
      // TCTYPE_J = MAX31856_TCTYPE_J,
      // TCTYPE_K = MAX31856_TCTYPE_K,
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
