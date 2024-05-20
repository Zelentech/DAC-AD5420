
#include <SPI.h>
#include "ad5420.h"


DAC_AD5420::DAC_AD5420(const uint8_t LATCHp, const uint8_t CLEARp, const uint8_t FAULTp) 
{

Serial.println("DAC_AD5420");

    _latch = LATCHp;
    _clear = CLEARp;
    _fault = FAULTp;
}


// Constructor
void DAC_AD5420::begin()
{
Serial.println("begin()");

    //UNO,MEGA,Yun,nano,duemilanove and other 8 bit arduino's
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV128);
    SPI.setDataMode(SPI_MODE0);

    pinMode(_latch, OUTPUT);
    pinMode(_clear, OUTPUT);
    pinMode(_fault, INPUT);
    latchport = portOutputRegister(digitalPinToPort(_latch));//pinMode(_cs, OUTPUT);
    latchpinmask = digitalPinToBitMask(_latch);
    *latchport |= latchpinmask;//hi
}


uint32_t  DAC_AD5420::ad5420_write(uint32_t data)
{
    uint32_t result = 0;
    // write highest byte
    //result |=  ad5420_spi_wrap((uint8_t)(data >> 16));
    result |=  SPI.transfer((uint8_t)(data >> 16));
    result <<= 8;
    // write middle byte
    //result |=  ad5420_spi_wrap((uint8_t)(data >> 8));
    result |=  SPI.transfer((uint8_t)(data >> 8));
    result <<= 8;
    // write lowest byte
    //result |= ad5420_spi_wrap((uint8_t)(data));
    result |= SPI.transfer((uint8_t)(data));
	
    return result;
}

void  DAC_AD5420::ad5420_write_ctrl(uint16_t data)
{
    // clear unused bits
    uint16_t d = (data & 0x3FFF);
    // write data
    ad5420_write((AD5420_CONTROL_REG_ADDR << 16)+d);

    //AD5420_LATCH_HIGH;
    digitalWrite(_latch, HIGH);
    AD5420_LATCH_CONTROL_WRITE_DELAY;
    //AD5420_LATCH_LOW;
    digitalWrite(_latch, LOW);
}

void  DAC_AD5420::ad5420_write_data(uint16_t data)
{
    ad5420_write((AD5420_DATA_REG_ADDR << 16) + data);
    //AD5420_LATCH_HIGH;
    digitalWrite(_latch, HIGH);
    //AD5420_LATCH_LOW;
    digitalWrite(_latch, LOW);
}

void  DAC_AD5420::ad5420_set_slewrate(uint16_t sr_clock, uint16_t sr_step, uint8_t sr_enabled)
{
    _uint24_t ctrl_reg = ad5420_read_reg(AD5420_CONTROL_REG);
    if (sr_enabled) {
        ctrl_reg &=~0x000FE0;
        ctrl_reg |= (sr_clock | sr_step | AD5420_SREN);
    } else  ctrl_reg &=~(AD5420_SREN);
    ad5420_write_ctrl((uint16_t)ctrl_reg);
}

uint16_t  DAC_AD5420::ad5420_read_reg(uint8_t reg_name)
{
    // write read register command with reg name at the end of the 3rd byte
    ad5420_write(0x020000+reg_name);
    //AD5420_LATCH_HIGH;
    digitalWrite(_latch, HIGH);
    //AD5420_LATCH_LOW;
    digitalWrite(_latch, LOW);
    // now register data would appear in next
    // three bytes
    _uint24_t r = ad5420_write(0x000000);
    //AD5420_LATCH_HIGH;
    digitalWrite(_latch, HIGH);
    //AD5420_LATCH_LOW;
    digitalWrite(_latch, LOW);
    return (uint16_t)r;
}

uint8_t  DAC_AD5420::ad5420_get_status()
{
    uint8_t r = ad5420_read_reg(AD5420_STATUS_REG);
    return r;
}

void  DAC_AD5420::ad5420_clear()
{
    //AD5420_CLEAR_HIGH;
    digitalWrite(_clear, HIGH);
    //AD5420_CLEAR_LOW;
    digitalWrite(_clear, LOW);
}

void  DAC_AD5420::ad5420_reset()
{
Serial.println("ad5420_reset()");

    ad5420_write((AD5420_RESET_REG_ADDR << 16) + 1);
    //AD5420_LATCH_HIGH;
    digitalWrite(_latch, HIGH);
    AD5420_LATCH_CONTROL_WRITE_DELAY;
    //AD5420_LATCH_LOW;
    digitalWrite(_latch, LOW);
}
