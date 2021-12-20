#include <vector>
#include <stdint.h> 
#include <stdio.h>
#include "AMG8833IR.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#ifndef I2C_M_RD
#include <linux/i2c.h>
#endif
#include <unistd.h>
typedef unsigned char   u8;
// Global file descriptor used to talk to the I2C bus:
int i2c_fd = -1;
// Default RPi B device name for the I2C bus exposed on GPIO2,3 pins (GPIO2=SDA, GPIO3=SCL): 
const char *i2c_fname = "/dev/i2c-1";
using namespace std; 	
uint8_t _deviceAddress;
int i2c_init(void) {
    if ((i2c_fd = open(i2c_fname, O_RDWR)) < 0) {
        char err[200];
        sprintf(err, "open('%s') in i2c_init", i2c_fname);
        perror(err);
        return -1;
    }
    // NOTE we do not call ioctl with I2C_SLAVE here because we always use the I2C_RDWR ioctl operation to do
    // writes, reads, and combined write-reads. I2C_SLAVE would be used to set the I2C slave address to communicate
    // with. With I2C_RDWR operation, you specify the slave address every time. There is no need to use normal write()
    // or read() syscalls with an I2C device which does not support SMBUS protocol. I2C_RDWR is much better especially
    // for reading device registers which requires a write first before reading the response.
    return i2c_fd;
}
// deb777 new I2c stuff here.
void i2c_close(void) {
    close(i2c_fd);
}
// Write to an I2C slave device's register:
int i2c_write(u8 slave_addr, u8 reg, u8 data) {
    //int retval;
    u8 outbuf[2];
    struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset;//[1];
    msgs[0].addr = slave_addr;
    msgs[0].flags = 0;
    msgs[0].len = sizeof(outbuf);
    msgs[0].buf = (char*) outbuf;
    outbuf[0] = reg;
    outbuf[1] = data;
    msgset.msgs = msgs;
    msgset.nmsgs = 1;
    if (ioctl(i2c_fd, I2C_RDWR, &msgset) < 0) {
        perror("ioctl(I2C_RDWR) in i2c_write");
        return 0;
    }
    return 1;
}
// Read the given I2C slave device's register and return the read value in `*result`:
int i2c_read(u8 slave_addr, u8 reg, u8 *result) {
    //int retval;
    //unsigned char outbuf[1], inbuf[1];
    unsigned char outbuf, inbuf;
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset;
    outbuf = reg;
    msgs[0].addr = slave_addr;
    msgs[0].flags = 0;
    msgs[0].len = sizeof(outbuf);
    //msgs[0].buf = &outbuf;    //deb777
    msgs[0].buf = (char*)&outbuf;
    msgs[1].addr = slave_addr;
    msgs[1].flags = I2C_M_RD;// | I2C_M_NOSTART | I2C_M_STOP;
    msgs[1].len = sizeof(inbuf);
    msgs[1].buf = (char*)&inbuf;
    msgset.msgs = msgs;
    msgset.nmsgs = 2;
    if (ioctl(i2c_fd, I2C_RDWR, &msgset) < 0) {
        perror("ioctl(I2C_RDWR) in i2c_read");
        return -1;
    }
    *result = inbuf;
    return 1;
}
//The default I2C address for the THING on the SparkX breakout is 0x69. 0x68 is also possible.
void AMG8833IR::begin(uint8_t deviceAddress /*, TwoWire &wirePort*/)  //deb100
{
	_deviceAddress = deviceAddress;
	i2c_init();
  	//_i2cPort = &wirePort;
}
float AMG8833IR::getPixelTemperature(unsigned char pixelAddr)
{
  // Temperature registers are numbered 128-255
  // Each pixel has a lower and higher register
  unsigned char pixelLowRegister = TEMPERATURE_REGISTER_START + (2 * pixelAddr);
  int16_t temperature = getRegister(pixelLowRegister, 2);
  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }
  float DegreesC = temperature * 0.25;
  return DegreesC;
}
float AMG8833IR::getPixelTemperatureFahrenheit(unsigned char pixelAddr)
{
  // Temperature registers are numbered 128-255
  // Each pixel has a lower and higher register
  unsigned char pixelLowRegister = TEMPERATURE_REGISTER_START + (2 * pixelAddr);
  int16_t temperature = getRegister(pixelLowRegister, 2);
  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }
  float DegreesF = (temperature * 0.25) * 1.8 + 32;
  return DegreesF;
}
int16_t AMG8833IR::getPixelTemperatureRaw(unsigned char pixelAddr)
{
  // Temperature registers are numbered 128-255
  // Each pixel has a lower and higher register
  unsigned char pixelLowRegister = TEMPERATURE_REGISTER_START + (2 * pixelAddr);
  int16_t temperature = getRegister(pixelLowRegister, 2);
  return temperature;
}
/********************************************************
 * Functions for retreiving the temperature of
 * the device according to the embedded thermistor. 
 ******************************************************** 
 * 
 * getDeviceTemperature() - returns float Celsius
 * 
 * getDeviceTemperatureFahrenheit() - returns float Fahrenheit
 * 
 * getDeviceTemperatureRaw() - returns int16_t contents of
 *    both thermistor temperature registers concatinated
 *    
 ********************************************************/
float AMG8833IR::getDeviceTemperature()
{
  int16_t temperature = getRegister(THERMISTOR_REGISTER_LSB, 2);
    // temperature is reported as 12-bit twos complement
    // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }
  float realTemperature = temperature * 0.0625;
  return realTemperature;
}
float AMG8833IR::getDeviceTemperatureFahrenheit()
{
  int16_t temperature = getRegister(THERMISTOR_REGISTER_LSB, 2);
    // temperature is reported as 12-bit twos complement
    // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }
  float realTemperatureF = (temperature * 0.0625) * 1.8 + 32;
  return realTemperatureF;
}
int16_t AMG8833IR::getDeviceTemperatureRaw()
{
  int16_t temperature = getRegister(THERMISTOR_REGISTER_LSB, 2);
  return temperature;
}
/********************************************************
 * Functions for manipulating Framerate
 ******************************************************** 
 * 
 * Internal framerate of the device is always 10fps
 * When operating in 1FPS mode, each frame is an average
 * of 10 readings.
 * 
 * setFramerate1FPS() - sets framerate to 1 Frame per Second
 * 
 * setFramerate10FPS() - sets framerate to 10 Frames per Second
 * 
 * isFramerate10FPS() - returns true if framerate is currently
 *    set to 10 Frames per Second (device default)
 *    
 ********************************************************/
void AMG8833IR::setFramerate1FPS()
{
    setRegister(FRAMERATE_REGISTER, 1);  
}
void AMG8833IR::setFramerate10FPS()
{
    setRegister(FRAMERATE_REGISTER, 0);  
}
bool AMG8833IR::isFramerate10FPS()
{
    if(getRegister(FRAMERATE_REGISTER, 1) == 0){
      return true;
    }else{
      return false;
    }
}
/********************************************************
 * Functions for manipulating Operating Mode
 ******************************************************** 
 * 
 * Device defaults to normal mode on reset. 
 * When the device is in standby mode, the temperature
 * register is only updated intermittently.
 * 
 * wake() - returns device to normal mode from any
 *    other state.
 *    
 * sleep() - puts device into sleep mode, temperature
 *    register is not updated
 * 
 * standby60seconds() - puts device into standby mode
 *    with 60 second update frequency
 *    
 * standby10seconds() - puts device into standby mode
 *    with 10 second update frequency
 *    
 ********************************************************/

void AMG8833IR::wake(){
    setRegister(POWER_CONTROL_REGISTER, 0x00);
}
void AMG8833IR::sleep(){
    setRegister(POWER_CONTROL_REGISTER, 0x10);
}
void AMG8833IR::standby60seconds(){
    setRegister(POWER_CONTROL_REGISTER, 0x20);
}
void AMG8833IR::standby10seconds(){
    setRegister(POWER_CONTROL_REGISTER, 0x21);
}
/********************************************************
 * Functions for manipulating Interrupt Control Register
 ******************************************************** 
 * 
 * interruptPinEnable() - Enable INT pin to pull low on 
 *    interrupt flag
 *    
 * interruptPinDisable() - Put INT pin into Hi-Z state
 * 
 * setInterruptModeAbsolute() - Set interrupt mode to
 *    "Absolute Value" mode
 *    
 * setInterruptModeDifference() - Set interrupt mode to
 *    "Difference" mode
 *    
 * interruptPinEnabled() - returns true if the INT pin
 *    is enabled. Returns false if INT pin is in Hi-Z 
 * 
 ********************************************************/
void AMG8833IR::interruptPinEnable(){
    int16_t ICRValue = getRegister(INT_CONTROL_REGISTER, 1);
    ICRValue |= (1 << 0);
    setRegister(INT_CONTROL_REGISTER, ICRValue & 0xFF);   
}
void AMG8833IR::interruptPinDisable(){
    int16_t ICRValue = getRegister(INT_CONTROL_REGISTER, 1);
    ICRValue &= ~(1 << 0);
    setRegister(INT_CONTROL_REGISTER, ICRValue & 0xFF);
}
void AMG8833IR::setInterruptModeAbsolute(){
    int16_t ICRValue = getRegister(INT_CONTROL_REGISTER, 1);   
    ICRValue |= (1 << 1);    
    setRegister(INT_CONTROL_REGISTER, ICRValue & 0xFF); 
}
void AMG8833IR::setInterruptModeDifference(){
    int16_t ICRValue = getRegister(INT_CONTROL_REGISTER, 1);   
    ICRValue &= ~(1 << 1);
    setRegister(INT_CONTROL_REGISTER, ICRValue & 0xFF);     
}
bool AMG8833IR::interruptPinEnabled(){
  int16_t ICRValue = getRegister(INT_CONTROL_REGISTER, 1);
  if(ICRValue & (1 << 0)){
    return true;
  }else{
    return false;
  }
}
/********************************************************
 * Functions for manipulating Status/Clear Registers
 ******************************************************** 
 * 
 * interruptFlagSet() - returns true if there is an 
 *    interrupt flag in the status register
 *    
 * pixelTemperatureOutputOK() - returns false if temperature
 *    output overflow flag is present in status register
 * 
 * deviceTemperatureOutputOK() - returns false if thermistor
 *    output overflow flag is present in status register
 *    
 * clearInterruptFlag() - clears interrupt flag in the 
 *    status register
 *    
 * clearPixelTemperatureOverflow() - clears temperature
 *    output overflow flag in status register
 *    
 * clearDeviceTemperatureOverflow() - clears thermistor
 *    output overflow flag in status register
 *    
 * clearAllOverflow() - clears both thermistor and 
 *    temperature overflow flags in status register but
 *    leaves interrupt flag untouched
 *    
 * clearAllStatusFlags() - clears all flags in status 
 *    register
 * 
 ********************************************************/
bool AMG8833IR::interruptFlagSet(){
  int16_t StatRegValue = getRegister(STATUS_REGISTER, 1);
  if(StatRegValue & (1 << 1)){
    return true;
  }else{
    return false;
  }  
}
bool AMG8833IR::pixelTemperatureOutputOK(){
  int16_t StatRegValue = getRegister(STATUS_REGISTER, 1);
  if(StatRegValue & (1 << 2)){
    return false;
  }else{
    return true;
  }  
}
bool AMG8833IR::deviceTemperatureOutputOK(){
  int16_t StatRegValue = getRegister(STATUS_REGISTER, 1);
  if(StatRegValue & (1 << 3)){
    return false;
  }else{
    return true;
  }  
}
void AMG8833IR::clearInterruptFlag(){  
    setRegister(STATUS_CLEAR_REGISTER, 0x02); 
}
void AMG8833IR::clearPixelTemperatureOverflow(){   
    setRegister(STATUS_CLEAR_REGISTER, 0x04); 
}
void AMG8833IR::clearDeviceTemperatureOverflow(){
    setRegister(STATUS_CLEAR_REGISTER, 0x08); 
}
void AMG8833IR::clearAllOverflow(){
    setRegister(STATUS_CLEAR_REGISTER, 0x0C);   
}
void AMG8833IR::clearAllStatusFlags(){
    setRegister(STATUS_CLEAR_REGISTER, 0x0E);   
}
/********************************************************
 * Function for reading Interrupt Table Register
 ******************************************************** 
 * 
 * pixelInterruptSet() - Returns true if interrupt flag 
 * is set for the specified pixel
 * 
 ********************************************************/
bool AMG8833IR::pixelInterruptSet(uint8_t pixelAddr){
  unsigned char interruptTableRegister = INT_TABLE_REGISTER_INT0 + (pixelAddr / 8);
  uint8_t pixelPosition = (pixelAddr % 8);
  int16_t interruptTableRow = getRegister(interruptTableRegister, 1);
  if(interruptTableRow & (1 << pixelPosition)){
    return true;
  }else{
    return false;
  }    
}
/********************************************************
 * Functions for manipulating Average Register
 ******************************************************** 
 * 
 * Moving Average Mode enable and disable are only 
 * referenced in some of the documentation for this 
 * device but not in all documentation. Requires writing
 * in sequence to a reserved register. I'm not sure it 
 * does anything.
 * 
 * movingAverageEnable() - enable "Twice Moving Average" 
 * 
 * movingAverageDisable() - disable "Twice Moving Average"
 * 
 * movingAverageEnabled() - returns true if enabled
 * 
 ********************************************************/
void AMG8833IR::movingAverageEnable(){
    setRegister(RESERVED_AVERAGE_REGISTER, 0x50); 
    setRegister(RESERVED_AVERAGE_REGISTER, 0x45); 
    setRegister(RESERVED_AVERAGE_REGISTER, 0x57); 
    setRegister(AVERAGE_REGISTER, 0x20); 
    setRegister(RESERVED_AVERAGE_REGISTER, 0x00);   
}
void AMG8833IR::movingAverageDisable(){
    setRegister(RESERVED_AVERAGE_REGISTER, 0x50); 
    setRegister(RESERVED_AVERAGE_REGISTER, 0x45); 
    setRegister(RESERVED_AVERAGE_REGISTER, 0x57); 
    setRegister(AVERAGE_REGISTER, 0x00); 
    setRegister(RESERVED_AVERAGE_REGISTER, 0x00);   
}
bool AMG8833IR::movingAverageEnabled(){
  int16_t AVGRegValue = getRegister(AVERAGE_REGISTER, 1);
  if(AVGRegValue & (1 << 5)){
    return true;
  }else{
    return false;
  }    
}
/********************************************************
 * Functions for manipulating Interrupt Level Register
 ******************************************************** 
 * 
 * setUpperInterruptValue() - accepts float Celsius 
 * 
 * setUpperInterruptValueRaw() - accepts int16_t register
 *    configuration
 * 
 * setUpperInterruptValueFahrenheit() - accepts float 
 *    Fahrenheit
 *  
 * setLowerInterruptValue() - accepts float Celsius 
 * 
 * setLowerInterruptValueRaw() - accepts int16_t register
 *    configuration
 * 
 * setLowerInterruptValueFahrenheit() - accepts float 
 *    Fahrenheit
 * 
 * setInterruptHysteresis() - accepts float Celsius
 * 
 * setInterruptHysteresisRaw() - accepts int16_t register
 *    configuration
 * 
 * setInterruptHysteresisFahrenheit() - accepts float 
 *    Fahrenheit
 *    
 * getUpperInterruptValue() - returns float Celsius 
 * 
 * getUpperInterruptValueRaw() - returns int16_t register
 *    contents
 * 
 * getUpperInterruptValueFahrenheit() - returns float 
 *    Fahrenheit
 *  
 * getLowerInterruptValue() - returns float Celsius 
 * 
 * getLowerInterruptValueRaw() - returns int16_t register
 *    contents
 * 
 * getLowerInterruptValueFahrenheit() - returns float 
 *    Fahrenheit
 * 
 * getInterruptHysteresis() - returns float Celsius
 * 
 * getInterruptHysteresisRaw() - returns int16_t register
 *    contents
 * 
 * getInterruptHysteresisFahrenheit() - returns float 
 *    Fahrenheit   
 * 
 ********************************************************/

void AMG8833IR::setUpperInterruptValue(float DegreesC){
  bool isNegative = false;
  if(DegreesC < 0){
    DegreesC = abs(DegreesC);
    isNegative = true;
  }  
  int16_t temperature = 0;  
  temperature = round(DegreesC*4);
  if(isNegative){
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  }  
  setRegister(INT_LEVEL_REGISTER_UPPER_LSB, temperature & 0xFF);
  setRegister(INT_LEVEL_REGISTER_UPPER_MSB, temperature >> 8);  
}
void AMG8833IR::setUpperInterruptValueRaw(int16_t regValue){  
  setRegister(INT_LEVEL_REGISTER_UPPER_LSB, regValue & 0xFF);
  setRegister(INT_LEVEL_REGISTER_UPPER_MSB, regValue >> 8);  
}
void AMG8833IR::setUpperInterruptValueFahrenheit(float DegreesF){
  bool isNegative = false;
  float DegreesC = (DegreesF - 32) / 1.8;
  if(DegreesC < 0){
    DegreesC = abs(DegreesC);
    isNegative = true;
  }  
  int16_t temperature = 0;  
  temperature = round(DegreesC*4);
  if(isNegative){
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  }  
  setRegister(INT_LEVEL_REGISTER_UPPER_LSB, temperature & 0xFF);
  setRegister(INT_LEVEL_REGISTER_UPPER_MSB, temperature >> 8);  
}
void AMG8833IR::setLowerInterruptValue(float DegreesC){
  bool isNegative = false;
  if(DegreesC < 0){
    DegreesC = abs(DegreesC);
    isNegative = true;
  }  
  int16_t temperature = 0;  
  temperature = round(DegreesC*4);
  if(isNegative){
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  }  
  setRegister(INT_LEVEL_REGISTER_LOWER_LSB, temperature & 0xFF);
  setRegister(INT_LEVEL_REGISTER_LOWER_MSB, temperature >> 8); 
}
void AMG8833IR::setLowerInterruptValueRaw(int16_t regValue){
  setRegister(INT_LEVEL_REGISTER_LOWER_LSB, regValue & 0xFF);
  setRegister(INT_LEVEL_REGISTER_LOWER_MSB, regValue >> 8);  
}
void AMG8833IR::setLowerInterruptValueFahrenheit(float DegreesF){
  bool isNegative = false;
  float DegreesC = (DegreesF - 32) / 1.8;
  if(DegreesC < 0){
    DegreesC = abs(DegreesC);
    isNegative = true;
  }  
  int16_t temperature = 0;  
  temperature = round(DegreesC*4);
  if(isNegative){
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  }  
  setRegister(INT_LEVEL_REGISTER_LOWER_LSB, temperature & 0xFF);
  setRegister(INT_LEVEL_REGISTER_LOWER_MSB, temperature >> 8);  
}
void AMG8833IR::setInterruptHysteresis(float DegreesC){
  bool isNegative = false;
  if(DegreesC < 0){
    DegreesC = abs(DegreesC);
    isNegative = true;
  }
  int16_t temperature = 0;  
  temperature = round(DegreesC*4);
  if(isNegative){
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  } 
  setRegister(INT_LEVEL_REGISTER_HYST_LSB, temperature & 0xFF);
  setRegister(INT_LEVEL_REGISTER_HYST_MSB, temperature >> 8);  
}
void AMG8833IR::setInterruptHysteresisRaw(int16_t regValue){ 
  setRegister(INT_LEVEL_REGISTER_HYST_LSB, regValue & 0xFF);
  setRegister(INT_LEVEL_REGISTER_HYST_MSB, regValue >> 8);  
}
void AMG8833IR::setInterruptHysteresisFahrenheit(float DegreesF){
  bool isNegative = false;
  float DegreesC = (DegreesF - 32) / 1.8;
  if(DegreesC < 0){
    DegreesC = abs(DegreesC);
    isNegative = true;
  }  
  int16_t temperature = 0;  
  temperature = round(DegreesC*4);
  if(isNegative){
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  }  
  setRegister(INT_LEVEL_REGISTER_HYST_LSB, temperature & 0xFF);
  setRegister(INT_LEVEL_REGISTER_HYST_MSB, temperature >> 8);  
}
float AMG8833IR::getUpperInterruptValue()
{
  int16_t temperature = getRegister(INT_LEVEL_REGISTER_UPPER_LSB, 2);
  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }
  float DegreesC = temperature * 0.25;
  return DegreesC;
}
int16_t AMG8833IR::getUpperInterruptValueRaw()
{
  return getRegister(INT_LEVEL_REGISTER_UPPER_LSB, 2);
}
float AMG8833IR::getUpperInterruptValueFahrenheit()
{
  int16_t temperature = getRegister(INT_LEVEL_REGISTER_UPPER_LSB, 2);
  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }
  float DegreesF = (temperature * 0.25) * 1.8 + 32;
  return DegreesF;
}
float AMG8833IR::getLowerInterruptValue()
{
  int16_t temperature = getRegister(INT_LEVEL_REGISTER_LOWER_LSB, 2);
  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }
  float DegreesC = temperature * 0.25;
  return DegreesC;
}
float AMG8833IR::getLowerInterruptValueFahrenheit()
{
  int16_t temperature = getRegister(INT_LEVEL_REGISTER_LOWER_LSB, 2);
  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }
  float DegreesF = (temperature * 0.25) * 1.8 + 32;
  return DegreesF;
}
int16_t AMG8833IR::getLowerInterruptValueRaw()
{
  return getRegister(INT_LEVEL_REGISTER_LOWER_LSB, 2);
}
float AMG8833IR::getInterruptHysteresis()
{
  int16_t temperature = getRegister(INT_LEVEL_REGISTER_HYST_LSB, 2);
  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }
  float DegreesC = temperature * 0.25;
  return DegreesC;
}
float AMG8833IR::getInterruptHysteresisFahrenheit()
{
  int16_t temperature = getRegister(INT_LEVEL_REGISTER_HYST_LSB, 2);
  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }
  float DegreesF = (temperature * 0.25) * 1.8 + 32;
  return DegreesF;
}
int16_t AMG8833IR::getInterruptHysteresisRaw()
{
  return getRegister(INT_LEVEL_REGISTER_HYST_LSB, 2);
}
/********************************************************
 * Functions for setting and getting registers over I2C
 ******************************************************** 
 * 
 * setRegister() - set unsigned char value at unsigned char register
 * 
 * getRegister() - get up to INT16 value from unsigned char register
 * 
 ********************************************************/
void AMG8833IR::setRegister(unsigned char reg, unsigned char val)
{
    	//_i2cPort->beginTransmission(_deviceAddress);
    	//_i2cPort->write(reg);
    	//_i2cPort->write(val);
    	//_i2cPort->endTransmission();
	//wiringPiI2CWriteReg8(_deviceAddress, reg, val);// deb100  
	i2c_write(_deviceAddress, reg, val);// deb100  
}
int16_t AMG8833IR::getRegister(unsigned char reg, int8_t len)
{
    	int16_t result;
	/*
    _i2cPort->beginTransmission(_deviceAddress);
    _i2cPort->write(reg);
    _i2cPort->endTransmission(false);
    _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)len);
    while(_i2cPort->available())    // client may send less than requested
    {
      // Get bytes from sensor
      uint8_t lsb = _i2cPort->read(); 
      uint8_t msb = _i2cPort->read(); 
  
      // concat bytes into int
      result = (uint16_t)msb << 8 | lsb;
    }
    _i2cPort->endTransmission();	
	*/
	//wiringPiI2CWriteReg8(_deviceAddress, reg, 0);	
	//uint8_t lsb = wiringPiI2CReadReg16(_deviceAddress, reg);
	//uint8_t msb = wiringPiI2CReadReg16(_deviceAddress, reg);
        //int i2c_read(u8 slave_addr, u8 reg, u8 *result) {
	//uint8_t lsb = i2c_read(_deviceAddress, reg);
	//uint8_t msb = i2c_read(_deviceAddress, reg);
	i2c_write(_deviceAddress, reg, 0);	
	//uint8_t lsb; 
	uint8_t lsb; 
	i2c_read(_deviceAddress, reg, &lsb);
	uint8_t msb; 
	i2c_read(_deviceAddress, reg, &msb);
	//result = ((uint16_t)msb << 8) | lsb;	
	result = (msb << 16) | lsb;	
//deb100
    	return result;                        
}
