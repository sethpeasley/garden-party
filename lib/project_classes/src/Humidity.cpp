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
                                float alarm_low_humidity, float alarm_high_humidity,
                                float alarm_deadband_humidity, float alarm_low_temperature,
                                float alarm_high_temperature, float alarm_deadband_temperature,
                                sht1xalt::voltage_t sensor_volage, sht1xalt::temp_units_t expressed_units)
{

}
