
#include "driver/i2c.h"
#include "driver/gpio.h"


#define I2C_MASTER_SDA_IO           13
#define I2C_MASTER_SCL_IO           14
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0 /*!< I2C master doesn't need buffer */
#define MCP23017_ADDRESS            0x20
#define EXPANDER_READ               0x01
#define ACK_CHECK_EN                1
#define I2C_MASTER_PORT             0
#define MCP23017_RESET              15
#define GPIOA                       0x12 
#define GPIOB                       0x13  
#define INPUT                       1
#define OUTPUT                      0   
#define ENABLE                      1
#define DISABLE                     0

esp_err_t I2cMasterInit(void);
esp_err_t ExpanderRegWrite(i2c_port_t write_i2cPort, uint8_t writeRegAddress, uint8_t dataWrite, uint8_t size);
uint8_t ExpanderRegRead(i2c_port_t read_i2cPort, uint8_t readRegAddress);
void ExpanderGpioDirection(i2c_port_t i2cPort,uint8_t gpioPort ,uint8_t pinNumber ,uint8_t direction);
void ExpanderGpioWrite(i2c_port_t i2cPort ,uint8_t gpioPort , uint8_t pinNumber , uint8_t level);
uint8_t ExpanderGpioRead(i2c_port_t i2cPort , uint8_t gpioPort , uint8_t pinNumber);
void ExpanderPullupConfig(i2c_port_t i2cPort , uint8_t gpioPort , uint8_t pinNumber , uint8_t onOff);
void ExpanderRstGpioInit(void);
void ExpanderRstGpioWrite(uint8_t level);

