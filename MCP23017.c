
#include "gpio_port_expander.h"


esp_err_t I2cMasterInit() 
{
  i2c_config_t config;
  config.mode = I2C_MODE_MASTER,
  config.sda_io_num = I2C_MASTER_SDA_IO,
  config.sda_pullup_en = GPIO_PULLUP_DISABLE,
  config.scl_io_num = I2C_MASTER_SCL_IO,
  config.scl_pullup_en = GPIO_PULLUP_DISABLE,
  config.master.clk_speed = I2C_MASTER_FREQ_HZ,
  i2c_param_config(I2C_MASTER_PORT, & config);
  return i2c_driver_install(I2C_MASTER_PORT, config.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void ExpanderPullupConfig(i2c_port_t i2cPort , uint8_t gpioPort , uint8_t pinNumber , uint8_t onOff)
{
  uint8_t pullUpPinMask;
  pullUpPinMask = ExpanderRegRead(i2cPort , (gpioPort-0x12)+0x0C);
  pullUpPinMask = onOff > 0 ?  pullUpPinMask | (1<<pinNumber) : pullUpPinMask & (~(1<<pinNumber));
  ExpanderRegWrite(i2cPort , (gpioPort-0x12)+0x0C , pullUpPinMask , 1);

}


void ExpanderGpioDirection(i2c_port_t i2cPort, uint8_t gpioPort , uint8_t pinNumber , uint8_t direction)
{
  uint8_t dirPinMask;
  dirPinMask = ExpanderRegRead(i2cPort , gpioPort-0x12);
  dirPinMask = direction > 0 ?  dirPinMask | (1<<pinNumber) : dirPinMask & (~(1<<pinNumber));
  ExpanderRegWrite(i2cPort , gpioPort-0x12 , dirPinMask , 1);

}


esp_err_t ExpanderRegWrite(i2c_port_t gpioInit, uint8_t regAddress, uint8_t dataWrite, uint8_t size)
{
  i2c_cmd_handle_t writeCmd;
  esp_err_t ret;

  writeCmd = i2c_cmd_link_create();
  i2c_master_start(writeCmd);
  i2c_master_write_byte(writeCmd, MCP23017_ADDRESS << 1, ACK_CHECK_EN);
  i2c_master_write_byte(writeCmd, regAddress, ACK_CHECK_EN);
  i2c_master_write_byte(writeCmd, dataWrite, ACK_CHECK_EN);
  i2c_master_stop(writeCmd);
  ret = i2c_master_cmd_begin(gpioInit, writeCmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(writeCmd);
  return ret;

}

uint8_t ExpanderRegRead(i2c_port_t read_i2cPort, uint8_t readRegAddress) 
{
  i2c_cmd_handle_t readCmd;
  uint8_t data;
  readCmd = i2c_cmd_link_create();
  i2c_master_start(readCmd);
  i2c_master_write_byte(readCmd, MCP23017_ADDRESS << 1, ACK_CHECK_EN);
  i2c_master_write_byte(readCmd, readRegAddress, ACK_CHECK_EN);
  i2c_master_start(readCmd);
  i2c_master_write_byte(readCmd, MCP23017_ADDRESS << 1 | EXPANDER_READ, ACK_CHECK_EN);
  i2c_master_read_byte(readCmd, & data, ACK_CHECK_EN);
  i2c_master_stop(readCmd);
  i2c_master_cmd_begin(read_i2cPort, readCmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(readCmd);
  return data;

}


void ExpanderGpioWrite(i2c_port_t i2cPort,uint8_t gpioPort , uint8_t pinNumber , uint8_t level)
{
  uint8_t mask;
  uint8_t regCurrentVal;
  regCurrentVal = ExpanderRegRead(i2cPort , gpioPort);
  mask = level > 0 ?  regCurrentVal | (1<<pinNumber) : regCurrentVal & (~(1<<pinNumber));
  ExpanderRegWrite(i2cPort ,gpioPort,mask,1);
      
}


uint8_t ExpanderGpioRead(i2c_port_t i2cPort , uint8_t gpioPort , uint8_t pinNumber)
{
  uint8_t tempData;
  tempData = ExpanderRegRead(i2cPort, gpioPort);
  tempData = tempData & (1<<pinNumber);
  tempData = tempData >> pinNumber;  
  return tempData;
}

void ExpanderRstGpioInit() 
{
  gpio_config_t io_config;
  io_config.intr_type = GPIO_PIN_INTR_DISABLE;
  io_config.mode = GPIO_MODE_OUTPUT;
  io_config.pin_bit_mask = (1ULL << MCP23017_RESET);
  io_config.pull_down_en = 0;
  io_config.pull_up_en = 0;
  gpio_config( & io_config);
}

void ExpanderRstGpioWrite(uint8_t level)
{
  gpio_set_level(MCP23017_RESET, level);
}
