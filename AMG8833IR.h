//#include <vector>
#include <stdint.h> 

//Registers
#define POWER_CONTROL_REGISTER        0x00
#define RESET_REGISTER                0x01
#define FRAMERATE_REGISTER            0x02
#define INT_CONTROL_REGISTER          0x03
#define STATUS_REGISTER               0x04
#define STATUS_CLEAR_REGISTER         0x05
#define AVERAGE_REGISTER              0x07
#define INT_LEVEL_REGISTER_UPPER_LSB  0x08
#define INT_LEVEL_REGISTER_UPPER_MSB  0x09
#define INT_LEVEL_REGISTER_LOWER_LSB  0x0A
#define INT_LEVEL_REGISTER_LOWER_MSB  0x0B
#define INT_LEVEL_REGISTER_HYST_LSB   0x0C
#define INT_LEVEL_REGISTER_HYST_MSB   0x0D
#define THERMISTOR_REGISTER_LSB       0x0E
#define THERMISTOR_REGISTER_MSB       0x0F
#define INT_TABLE_REGISTER_INT0       0x10
#define RESERVED_AVERAGE_REGISTER     0x1F
#define TEMPERATURE_REGISTER_START    0x80
#define DEFAULT_ADDRESS 0x69
class AMG8833IR {
  public:
    	//By default use the default I2C address, and use Wire port
    	void begin(uint8_t deviceAddress = DEFAULT_ADDRESS/*, TwoWire &wirePort = Wire*/);  //deb100

	float getPixelTemperature(unsigned char pixelAddr);
	int16_t getPixelTemperatureRaw(unsigned char pixelAddr);
	float getPixelTemperatureFahrenheit(unsigned char pixelAddr);
	
	float getDeviceTemperature();
	int16_t getDeviceTemperatureRaw();
	float getDeviceTemperatureFahrenheit();	
	
	void setFramerate1FPS();
	void setFramerate10FPS();
	bool isFramerate10FPS();
	
	void wake();
	void sleep();
	void standby60seconds();
	void standby10seconds();
	
	void interruptPinEnable();
	void interruptPinDisable();
	void setInterruptModeAbsolute();
	void setInterruptModeDifference();
	bool interruptPinEnabled();
	
	bool interruptFlagSet();
	bool pixelTemperatureOutputOK();
	bool deviceTemperatureOutputOK();
	void clearInterruptFlag();
	void clearPixelTemperatureOverflow();
	void clearDeviceTemperatureOverflow();
	void clearAllOverflow();
	void clearAllStatusFlags();
	
	bool pixelInterruptSet(uint8_t pixelAddr);
	
	void movingAverageEnable();
	void movingAverageDisable();
	bool movingAverageEnabled();
	
	void setUpperInterruptValue(float DegreesC);
	void setUpperInterruptValueRaw(int16_t regValue);
	void setUpperInterruptValueFahrenheit(float DegreesF);
	
	void setLowerInterruptValue(float DegreesC);
	void setLowerInterruptValueRaw(int16_t regValue);
	void setLowerInterruptValueFahrenheit(float DegreesF);
	
	void setInterruptHysteresis(float DegreesC);
	void setInterruptHysteresisRaw(int16_t regValue);
	void setInterruptHysteresisFahrenheit(float DegreesF);
	
	float getUpperInterruptValue();
	int16_t getUpperInterruptValueRaw();
	float getUpperInterruptValueFahrenheit();
	
	float getLowerInterruptValue();
	int16_t getLowerInterruptValueRaw();
	float getLowerInterruptValueFahrenheit();
	
	float getInterruptHysteresis();
	int16_t getInterruptHysteresisRaw();
	float getInterruptHysteresisFahrenheit();
	
	void setRegister(unsigned char reg, unsigned char val);
	int16_t getRegister(unsigned char reg, int8_t len);

   // 	void setI2CAddress(uint8_t addr); //Set the I2C address we read and write to
};
