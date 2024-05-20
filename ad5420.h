
/*! \file ad5420.h

\brief Header file of AD5420 driver.



AD5420 16-bit
The output current range is programmable at 4 mA to 20 mA, 0 mA to 20 mA.
Or an overrange function of 0 mA to 24 mA.

The output is open-circuit protected.

The device operates with a power supply (AV DD ) range from 10.8 V to 60 V.
Output loop compliance is 0 V to AV DD  2.5 V.
The device also includes a power-on reset function, ensuring that the device powers
up in a known state, and an asynchronous CLEAR pin that sets the output to the
low end of the selected current range. The total unadjusted error is typically
0.01% FSR.

From SPI point of view DAC has 3 internal registers which can be accessed
through read and write commands. CONTROL register set output parameters,
DATA register stores output code, STATUS register is read-only and is used to
check whether there some faults on DAC operation.


Zelentech signal Card layout file K245-0100-01

           	SIGCARD               AD5420
        	PA18       <--------> FAULT
		PA1 (SCK)  <--------> SCK
		PA00(MOSI) <--------> SDIN
		PB23 (MISO) <--------> SDO
		PA15        <--------> CLEAR
		PA07        <--------> LATCH
		
to do:
	Check / edit AD5420_xxx_BIT,AD5420_xxx_OUT and other macros to make them compatible
to do: Check / edit AD5420_xxx_HIGH and AD5420_xxx_LOW macros as well.

to do: Check #AD5420_LATCH_CONTROL_WRITE_DELAY consistent with your code for delay for 5 to 10 usec.

Code to initiate the IC
	<code>

		// Configures GPIO pins to work with AD5420, sends a reset command

		ad5420_init();	// Configures GPIO PINS
		ad5420_reset();	// Resets the IC




		// Configures AD5420 to work with external reference R in 4 to 20 mA range.
		// TODO: We don't have external ref, edit as required.

		// For other macros here, check out description of ad5420_write_ctrl().
		
		ad5420_write_ctrl(AD5420_OUTEN | AD5420_REXT | AD5420_4_20_RANGE);

		// Last statement sets output current to the middle of the output range.
		ad5420_write_data(0x8000);
	</code>
	 



This DAC also has an automatic slew rate control so you can vary rate at
which it changes output current. In very wide range between 2 msec and 20
sec (sic!). This thing might be done either with ad5420_write_ctrl() function
or with specific ad5420_set_slewrate() function. E.x.:

	<code>
		// TODO
		// Put this in the init part. Make it short as default.
		// This sets the lowest slewrate of output current.

		ad5420_set_slewrate(AD5420_SR_CLK_3300, AD5420_SR_STEPSIZE_1, 1);
	</code>


Current DAC configuration and output code can be read through ad5420_read_reg() function
Returns DAC CONTROL, STATUS or DATA register value.

*/



#ifndef AD5420_H_
#define AD5420_H_

#include "Arduino.h"
#include <avr/io.h>
#include <SPI.h>


/*! \brief Pin of the CLEAR signal.*/
#define AD5420_CLEAR_BIT  PA15

/*! \brief Pin of the LATCH signal.*/
#define AD5420_LATCH_BIT  PA07

/*! \brief Pin of the FAULT signal.*/
#define AD5420_FAULT_BIT  PB0

/*! \brief Input register name of the CLEAR signal.*/
#define AD5420_CLEAR_IN   PINB

/*! \brief Input register name of the LATCH signal.*/
#define AD5420_LATCH_IN   PINB

/*! \brief Input register name of the FAULT signal.*/
#define AD5420_FAULT_IN   PINB

/*! \brief Direction register name of the CLEAR signal.*/
#define AD5420_CLEAR_DIR  DDRB

/*! \brief Direction register name of the LATCH signal.*/
#define AD5420_LATCH_DIR  DDRB

/*! \brief Direction register name of the FAULT signal.*/
#define AD5420_FAULT_DIR  DDRB

/*! \brief Output register name of the CLEAR signal.*/
#define AD5420_CLEAR_OUT  PORTB

/*! \brief Output register name of the LATCH signal.*/
#define AD5420_LATCH_OUT  PORTB

/*! \brief Output register name of the FAULT signal.*/
#define AD5420_FAULT_OUT  PORTB

/*! \brief Shorthand macro for setting LATCH pin HIGH.*/
#define AD5420_LATCH_HIGH  AD5420_LATCH_OUT |= (1 << AD5420_LATCH_BIT)

/*! \brief Shorthand macro for setting LATCH pin LOW.*/
#define AD5420_LATCH_LOW   AD5420_LATCH_OUT &=~(1 << AD5420_LATCH_BIT)

/*! \brief Shorthand macro for a 10 us delay required by write to control
register of DAC.*/
#define AD5420_LATCH_CONTROL_WRITE_DELAY   delayMicroseconds(10)

/*! \brief Shorthand macro for setting CLEAR pin HIGH.*/
#define AD5420_CLEAR_HIGH  AD5420_CLEAR_OUT |= (1 << AD5420_CLEAR_BIT)

/*! \brief Shorthand macro for setting CLEAR pin LOW.*/
#define AD5420_CLEAR_LOW   AD5420_CLEAR_OUT &=~(1 << AD5420_CLEAR_BIT)

/*! \brief Totally portable function that initialize discrete in-out pins
of uC.*/
//extern void ad5420_init();

/*! \brief This is a wrap function to user's realization of a read-write cycle
of 1 byte via SPI interface. Bit order is from MSB to LSB.

\param data Byte that would be sent to SPI bus.

\return Byte that was read back from SPI bus while sending data.*/
//extern uint8_t ad5420_spi_wrap(uint8_t data);

/*! \brief Internal address of DAC STATUS register.*/
#define AD5420_STATUS_REG        0x00

/*! \brief Internal address of DAC DATA register.*/
#define AD5420_DATA_REG          0x01

/*! \brief Internal address of DAC CONTROL register.*/
#define AD5420_CONTROL_REG       0x02

/*! \brief Internal flag of using REXT as reference resistance. Please set this
bit before or during changing #AD5420_OUTEN bit.*/
#define AD5420_REXT              0x2000

/*! \brief Internal flag of switching DAC on.*/
#define AD5420_OUTEN             0x1000

/*! \brief Internal flag of switching slew rate control on.*/
#define AD5420_SREN              0x0010

/*! \brief Internal flag of setting daisy-chain mode of SPI bus.*/
#define AD5420_DCEN              0x0008

/*! \brief Internal flag of setting 4-20 mA output range.*/
#define AD5420_4_20_RANGE        0x0005

/*! \brief Internal flag of setting 0-20 mA output range.*/
#define AD5420_0_20_RANGE        0x0006

/*! \brief Internal flag of setting 0-24 mA output range.*/
#define AD5420_0_24_RANGE        0x0007

/*! \brief Internal flag of output current fault in status register.*/
#define AD5420_IOUT_FAULT        0x0004

/*! \brief Internal flag of slew rate being active in status register.*/
#define AD5420_SR_ACTIVE         0x0002

/*! \brief Internal flag of overheat fault of DAC in status register.*/
#define AD5420_OVRHEAT_FAULT     0x0001


/******************************************************************************
**                                                                           **
**      Set of slew rate control defines                                     **
**                                                                           **
******************************************************************************/

/*! \brief Bit value that sets slewrate to 257,730 Hz.*/
#define AD5420_SR_CLK_257730     0x0000

/*! \brief Bit value that sets slewrate to 198,410 Hz.*/
#define AD5420_SR_CLK_198410     0x0100

/*! \brief Bit value that sets slewrate to 152,440 Hz.*/
#define AD5420_SR_CLK_152440     0x0200

/*! \brief Bit value that sets slewrate to 131,580 Hz.*/
#define AD5420_SR_CLK_131580     0x0300

/*! \brief Bit value that sets slewrate to 115,740 Hz.*/
#define AD5420_SR_CLK_115740     0x0400

/*! \brief Bit value that sets slewrate to 69,440 Hz.*/
#define AD5420_SR_CLK_69440      0x0500

/*! \brief Bit value that sets slewrate to 37,590 Hz.*/
#define AD5420_SR_CLK_37590      0x0600

/*! \brief Bit value that sets slewrate to 25,770 Hz.*/
#define AD5420_SR_CLK_25770      0x0700

/*! \brief Bit value that sets slewrate to 20,160 Hz.*/
#define AD5420_SR_CLK_20160      0x0800

/*! \brief Bit value that sets slewrate to 16,030 Hz.*/
#define AD5420_SR_CLK_16030      0x0900

/*! \brief Bit value that sets slewrate to 10,290 Hz.*/
#define AD5420_SR_CLK_10290      0x0A00

/*! \brief Bit value that sets slewrate to 8,280 Hz.*/
#define AD5420_SR_CLK_8280       0x0B00

 /*! \brief Bit value that sets slewrate to 6,900 Hz.*/
#define AD5420_SR_CLK_6900       0x0C00

/*! \brief Bit value that sets slewrate to 5,530 Hz.*/
#define AD5420_SR_CLK_5530       0x0D00

/*! \brief Bit value that sets slewrate to 4,240 Hz.*/
#define AD5420_SR_CLK_4240       0x0E00

/*! \brief Bit value that sets slewrate to 3,300 Hz.*/
#define AD5420_SR_CLK_3300       0x0F00

/*! \brief Bit value that sets slewrate step to 1 LSB.*/
#define AD5420_SR_STEPSIZE_1     0x0000

/*! \brief Bit value that sets slewrate step to 2 LSB.*/
#define AD5420_SR_STEPSIZE_2     0x0020

/*! \brief Bit value that sets slewrate step to 4 LSB.*/
#define AD5420_SR_STEPSIZE_4     0x0040

/*! \brief Bit value that sets slewrate step to 8 LSB.*/
#define AD5420_SR_STEPSIZE_8     0x0060

/*! \brief Bit value that sets slewrate step to 16 LSB.*/
#define AD5420_SR_STEPSIZE_16    0x0080

/*! \brief Bit value that sets slewrate step to 32 LSB.*/
#define AD5420_SR_STEPSIZE_32    0x00A0

/*! \brief Bit value that sets slewrate step to 64 LSB.*/
#define AD5420_SR_STEPSIZE_64    0x00C0

/*! \brief Bit value that sets slewrate step to 128 LSB.*/
#define AD5420_SR_STEPSIZE_128   0x00E0

/*! \brief This type is specifically defined because almost
all transmissions between DAC and uC are 24-bit long.*/
typedef uint32_t _uint24_t;

/*! \brief Internal address of DAC CONTROL register.*/
#define AD5420_CONTROL_REG_ADDR  0x55UL

/*! \brief Internal address of DAC DATA register.*/
#define AD5420_DATA_REG_ADDR     0x01UL

/*! \brief Internal address of DAC RESET register.*/
#define AD5420_RESET_REG_ADDR    0x56UL


/* init function */
//void ad5420_init() {
//	AD5420_CLEAR_DIR |= (1 << AD5420_CLEAR_BIT);
//	AD5420_LATCH_DIR |= (1 << AD5420_LATCH_BIT);
//	AD5420_FAULT_DIR &=~(1 << AD5420_FAULT_BIT);
//	AD5420_LATCH_OUT &=~(1 << AD5420_LATCH_BIT);
//	AD5420_CLEAR_OUT &=~(1 << AD5420_CLEAR_BIT);
//}

//uint8_t ad5420_spi_wrap(uint8_t data) {
    //return spi_read(data);
//    return SPI.transfer(data);
//}




class DAC_AD5420 {
  public:

    DAC_AD5420(const uint8_t LATCHp, const uint8_t CLEARp, const uint8_t FAULTp);
    void begin();

    uint32_t ad5420_write(uint32_t data);

    /*! \brief This function resets DAC to it's initial state by sending a specific command to it.*/
    void ad5420_reset();

    /*! \brief This function sets output of the DAC to it's minimum value using CLEAR pin.*/
    void ad5420_clear();

    /*! \brief Writes data to DAC. This function changes sets output current.
        \param data - 16-bit data to be written to DAC.*/
    void ad5420_write_data(uint16_t data);

    /*! \brief Writes control bits to DAC.

        \param data - data to be written to DAC. Possible value is a logical OR of one or more of next values:

        - #AD5420_REXT determines whether external resistor is used to make a zero offset;
	- #AD5420_OUTEN determines whether output current is enabled;
	- #AD5420_SREN determines whether output slew rate limits are enabled;
	- one of #AD5420_0_20_RANGE, #AD5420_4_20_RANGE, #AD5420_0_24_RANGE
	  to set the output current range.

        This function defines current state of the DAC CONTROL register. Slew rate
        value is determined by ad5420_set_slewrate() function. But you can do it
        in one piece if you add #AD5420_SREN flag and some of these:
        AD5420_SR_CLK_xxx and AD5420_SR_STEPSIZE_xxx.*/
    void ad5420_write_ctrl(uint16_t data);

    /*! \brief This function sets parameters of DAC slewrate control.
        \param sr_clock Sets clock rate of update and must be one of the AD5420_SR_CLK_xxx macros.
        \param sr_step Sets update step of DAC output value. Must be one of the AD5420_SR_STEPSIZE_xxx macros.
        \param sr_enabled This param determines whether the slewrate function is enabled.
	   
        \warning Setting input parameters to lowest value makes output to change very slowly.
        Time that is need to switch between max and min codes is varied from about 1.5 ms
        (without slewrate control) to about 15 s (with slowest slew rate).*/
    void ad5420_set_slewrate(uint16_t sr_clock, uint16_t sr_step, uint8_t sr_enabled);

    /*! \brief This function reads DAC internal register.
        \param reg_name - Name of the reading register. Must be one of the follows:
        - AD5420_STATUS_REG to read STATUS register;
	- AD5420_CONTROL_REG to read CONTROL register;
	- AD5420_DATA_REG to read DATA register.
        \return Returns 16-bit data that had been read.*/
    uint16_t ad5420_read_reg(uint8_t reg_name);

    /*! \brief This function gets status bits.
        \return Returns 8-bit int, last 3 bits of which is used as status flags. To work with them
        #AD5420_IOUT_FAULT, #AD5420_SR_ACTIVE, #AD5420_OVRHEAT_FAULT macros are provided.*/
    uint8_t ad5420_get_status();

    uint8_t   _cs, cspinmask, _latch, _clear, _fault;
    uint8_t   latchpinmask, clearpinmask, faultpinmask;

  private:

    volatile  uint8_t *csport, *latchport, *clearport, *faultport;
};



#endif /* AD5420_H_ */
