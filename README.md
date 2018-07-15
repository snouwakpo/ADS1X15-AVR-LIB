# ADS1X15-AVR-LIB
Driver for ADS1X15 Analog to Digital Converters of Texas Instrument.

This library is heavily inspired from Adafruit's Adafruit_ADS1X15 library. In fact, the main difference between the Adafruit library (written for Arduino) and the current one relates to the i2c communication protocole which had to be written for a C implementation in addition to a new sensor structure that contains the ADS1X15 configurations.
The Peter Fleury i2c library is used to communicate with the sensor.
This library has been successfully tested with ATMEGA328 microcontrollers and will most likely work with other microcontrollers compatible with the Peter Fleury i2c library.

Usage:
//Create a new sensor instance that stores the ADS1X15 configurations
sensor ads = { .m_gain = GAIN_ONE, .m_conversionDelay = ADS1015_CONVERSIONDELAY, .m_bitShift = 4, .m_i2cAddress = ADS1015_ADDRESS };

sensor ads1 = { .m_gain = GAIN_SIXTEEN, .m_conversionDelay = ADS1015_CONVERSIONDELAY, .m_bitShift = 4, .m_i2cAddress = ADS1015_ADDRESS };

//Initiate i2c communication

i2c_init();

//Read ADS1X15 channels

int16_t curadc = readADC_SingleEnded(ads, 0); //Where 0 is the channel number

int16_t curadc1 = readADC_SingleEnded(ads1, 0); 

int16_t adc_diff = readADC_Differential_0_1(ads);


Voila!!

