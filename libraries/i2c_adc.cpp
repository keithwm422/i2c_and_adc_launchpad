/***************************/
////FOR I2C and ADC comms on TM4C///
#include "i2c_adc.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <inc/hw_i2c.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_gpio.h>
#include <inc/hw_ints.h>
#include <inc/hw_pwm.h>
#include <driverlib/i2c.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/adc.h>
#include <driverlib/timer.h>
#include <driverlib/interrupt.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/udma.h>
#include <driverlib/pwm.h>
#include <driverlib/ssi.h>
#include <driverlib/systick.h>
#include <driverlib/adc.h>
#include <string.h>


//initialize I2C module 0
//Slightly modified version of TI's example code
void InitI2C0(void)
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
 
    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
     
    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
 
    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
     
    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
 
    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
     
    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}
void InitI2C1(void)
{
    //enable I2C module 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
 
    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
     
    //enable GPIO peripheral that contains I2C 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
 
    // Configure the pin muxing for I2C1 functions on port A6 and A7.
// Need to add in GPIOAFSEL and GPIO open drain, etc from page 998 sec 16.2 for configuring other I2C pins. 
 //   GPIODirModeSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_DIR_MODE_HW);

    // Select the I2C function for these pins.
    
    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);
     
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
 
    // Enable and initialize the I2C1 master module.  Use the system clock for
    // the I2C1 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), false);
     
    //clear I2C FIFOs
    HWREG(I2C1_BASE + I2C_O_FIFOCTL) = 80008000;
}
void InitI2C(uint8_t port)
{

    if(port==0){
        SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
        SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
        GPIOPinConfigure(GPIO_PB2_I2C0SCL);
        GPIOPinConfigure(GPIO_PB3_I2C0SDA);
        GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
        GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
        I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
        HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
    }
    else if(port==1){
        SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
        SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        GPIOPinConfigure(GPIO_PA6_I2C1SCL);
        GPIOPinConfigure(GPIO_PA7_I2C1SDA);
        GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
        GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
        I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), false);
        HWREG(I2C1_BASE + I2C_O_FIFOCTL) = 80008000;
    }
    else if(port==2){
        SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
        SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
        GPIOPinConfigure(GPIO_PE4_I2C2SCL);
        GPIOPinConfigure(GPIO_PE5_I2C2SDA);
        GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);
        GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);
        I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), false);
        HWREG(I2C2_BASE + I2C_O_FIFOCTL) = 80008000;
    }
    else if(port==3){
        SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
        SysCtlPeripheralReset(SYSCTL_PERIPH_I2C3);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
        GPIOPinConfigure(GPIO_PD0_I2C3SCL);
        GPIOPinConfigure(GPIO_PD1_I2C3SDA);
        GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
        GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);
        I2CMasterInitExpClk(I2C3_BASE, SysCtlClockGet(), false);
        HWREG(I2C3_BASE + I2C_O_FIFOCTL) = 80008000;
    }
    else {
        return;
    }
}

void InitADC(void){
  // 
  // Enable the PLL when using ADC modules. 
  //
  SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

  //
  // The ADC0 peripheral must be enabled for use. 
  // There are two ADC "modules" , ADC0 or ADC1 and
  // this enables that module.
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  SysCtlDelay(3);

    // Enables the clock to the GPIO section that the ADC inputs.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  //
  // The first argument is the port base of the adc pin desired.
  // The second argument is a uint8_t where each pin is a bit in order [0,7]. 
  // Any high bits enable that adc pin on this port base. 4=[0000,0100]
  // PE2 ADC is enabled.
  // 
  GPIOPinTypeADC(GPIO_PORTE_BASE,4);
  //
  // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
  // will do a single sample when the processor sends a singal to start the
  // conversion.  Each ADC module (ADC0_BASE here) has 4 programmable sequences, sequence 0
  // to sequence 3, where each sequence has different number of samples and depth of FIFO
  // This example is arbitrarily using sequence 3 (1 sample, depth of FIFO i 1).
  // The last argument is the priority which is zero here, highest priority, and 
  // needs to be different than other same sample sequence priorities.
  //
  ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
  //
  // Configure step 0 on sequence 3.  Sample the temperature sensor
  // (ADC_CTL_TS) and configure the interrupt flag (ADC_CTL_IE) to be set
  // when the sample is done.  Tell the ADC logic that this is the last
  // conversion on sequence 3 (ADC_CTL_END).  Sequence 3 has only one
  // programmable step.  Sequence 1 and 2 have 4 steps, and sequence 0 has
  // 8 programmable steps.  Since we are only doing a single conversion using
  // sequence 3 we will only configure step 0.  For more information on the
  // ADC sequences and steps, reference the datasheet.
  // The last argument is the config stuff, a bunch of ORs.
  // ADC_CTL_CH[0,23] are the ADC channel inputs to be configured with this sequence step. 
  // 
  //
  ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
  //
  // Since sample sequence 3 is now configured, it must be enabled.
  //
  ADCSequenceEnable(ADC0_BASE, 3);
  //  new: use hardware sampling function 
  // can turn this off or change  second argument to any number x [0-6] (2^x samples)
  ADCHardwareOversampleConfigure(ADC0_BASE,2);
  //
  // Clear the interrupt status flag.  This is done to make sure the
  // interrupt flag is cleared before we sample.
  //
  ADCIntClear(ADC0_BASE, 3);
    
}

//sends an I2C command to the specified slave. See page 1001 of the microcontroller manual
uint32_t I2CSend2bytes(uint8_t base,uint8_t slave_addr, uint8_t num_of_args, ...)
{
    uint32_t error_return;
    uint32_t base_to_use;
    if (base==0) base_to_use=I2C0_BASE;
    else if(base==1) base_to_use=I2C1_BASE;
    else if(base==2) base_to_use=I2C2_BASE;
    else if(base==3) base_to_use=I2C3_BASE;
    else return 255;
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    // Step 1 under 16.3.1.5
    // false here is the receive or not receive value from master point of view.
    //so false means master is not receiving (so its sending duh). 
    I2CMasterSlaveAddrSet(base_to_use, slave_addr, false); // 7 bit slave address. 
     
    //stores list of variable number of arguments
    va_list vargs;
     
    //specifies the va_list to "open" and the last fixed argument
    //so vargs knows where to start looking
    va_start(vargs, num_of_args);
    //Step 2 of sec 16.3.1.5 
    //put data to be sent into FIFO
    I2CMasterDataPut(base_to_use, va_arg(vargs, uint32_t));
     
    //if there is only one argument, we only need to use the
    //single send I2C function
    if(num_of_args == 1)
    {
        //Initiate send of data from the MCU
        // if only argument, single send, then not the section im currently commenting
        I2CMasterControl(base_to_use, I2C_MASTER_CMD_SINGLE_SEND);
         
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(base_to_use));
        error_return = I2CMasterErr(base_to_use);
        //"close" variable argument list
        va_end(vargs);
    }
     
    //otherwise, we start transmission of multiple bytes on the
    //I2C bus
    else
    {
        //Initiate send of data from the MCU
        // repeated start is differnt than this maybe? looks the same in the doc
        // specifically section 16.3.1.5
        I2CMasterControl(base_to_use, I2C_MASTER_CMD_BURST_SEND_START);
         
        // Wait until MCU is done transferring.
        // "when the busy bit is 0" step 3 sec 16.3.1.5
        while(I2CMasterBusy(base_to_use));
         
        //send num_of_args-2 pieces of data, using the
        //BURST_SEND_CONT command of the I2C module
        for(uint8_t i = 1; i < (num_of_args - 1); i++)
        {
            //put next piece of data into I2C FIFO
            I2CMasterDataPut(base_to_use, va_arg(vargs, uint32_t));
            //send next data that was just placed into FIFO
            // curious this "SEND_CONT" is 0x1 in the file: 
            ///home/keith/.energia15/packages/Arduino-Tiva/hardware/tivac/1.0.2/system/driverlib/i2c.h
            // but the 16.3.1.5 sec calls for 0x3 to be sent?
            I2CMasterControl(base_to_use, I2C_MASTER_CMD_BURST_SEND_CONT);
     
            // Wait until MCU is done transferring.
            while(I2CMasterBusy(base_to_use));
        }
     
        //put last piece of data into I2C FIFO
        I2CMasterDataPut(base_to_use, va_arg(vargs, uint32_t));
        //send next data that was just placed into FIFO
        I2CMasterControl(base_to_use, I2C_MASTER_CMD_BURST_SEND_FINISH);
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(base_to_use));
        error_return = I2CMasterErr(base_to_use);

        //"close" variable args list
        va_end(vargs);
    }

    // compare errors to these vals:
    //
    //#define I2C_MASTER_ERR_NONE     0
    //#define I2C_MASTER_ERR_ADDR_ACK 0x00000004
    //#define I2C_MASTER_ERR_DATA_ACK 0x00000008
    //#define I2C_MASTER_ERR_ARB_LOST 0x00000010
    //#define I2C_MASTER_ERR_CLK_TOUT 0x00000080
    // custom error, return 255 when invalid base is input.
    return error_return;
}


//sends an I2C command to the specified slave
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...)
{
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
     
    //stores list of variable number of arguments
    va_list vargs;
     
    //specifies the va_list to "open" and the last fixed argument
    //so vargs knows where to start looking
    va_start(vargs, num_of_args);
     
    //put data to be sent into FIFO
    I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));
     
    //if there is only one argument, we only need to use the
    //single send I2C function
    if(num_of_args == 1)
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
         
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));
         
        //"close" variable argument list
        va_end(vargs);
    }
     
    //otherwise, we start transmission of multiple bytes on the
    //I2C bus
    else
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
         
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));
         
        //send num_of_args-2 pieces of data, using the
        //BURST_SEND_CONT command of the I2C module
        for(uint8_t i = 1; i < (num_of_args - 1); i++)
        {
            //put next piece of data into I2C FIFO
            I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));
            //send next data that was just placed into FIFO
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
     
            // Wait until MCU is done transferring.
            while(I2CMasterBusy(I2C0_BASE));
        }
     
        //put last piece of data into I2C FIFO
        I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));
        //send next data that was just placed into FIFO
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));
         
        //"close" variable args list
        va_end(vargs);
    }
}

//sends an array of data via I2C to the specified slave
void I2CSendString(uint8_t slave_addr, char array[])
{
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
     
    //put data to be sent into FIFO
    I2CMasterDataPut(I2C0_BASE, array[0]);
     
    //if there is only one argument, we only need to use the
    //single send I2C function
    if(array[1] == '\0')
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
         
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));
    }
     
    //otherwise, we start transmission of multiple bytes on the
    //I2C bus
    else
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
         
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));
         
        //initialize index into array
        uint8_t i = 1;
 
        //send num_of_args-2 pieces of data, using the
        //BURST_SEND_CONT command of the I2C module
        while(array[i + 1] != '\0')
        {
            //put next piece of data into I2C FIFO
            I2CMasterDataPut(I2C0_BASE, array[i++]);
 
            //send next data that was just placed into FIFO
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
     
            // Wait until MCU is done transferring.
            while(I2CMasterBusy(I2C0_BASE));
        }
     
        //put last piece of data into I2C FIFO
        I2CMasterDataPut(I2C0_BASE, array[i]);
 
        //send next data that was just placed into FIFO
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
 
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));
    }
}


//read specified slave device one byte
uint8_t I2CReceiveslave(uint8_t slave_addr)
{

    //specify that we are reading the
    //slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);

     //Initiate send of data from the MCU
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    //I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
         
    // Wait until MCU is done transferring.
    while(I2CMasterBusy(I2C0_BASE));

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    
    return I2CMasterDataGet(I2C0_BASE);

/*
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
              
//    return I2CMasterDataGet(I2C0_BASE);     
    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);
     
    //send control byte and read from the register we
    //specified
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
     
    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));
     
//    //return data pulled from the specified register
    return I2CMasterDataGet(I2C0_BASE);
*/
}

//read specified register on slave device
uint32_t I2CReceive(uint8_t base, uint8_t slave_addr)
{
    uint32_t base_to_use;
    if (base==0) base_to_use=I2C0_BASE;
    else if(base==1) base_to_use=I2C1_BASE;
    else if(base==2) base_to_use=I2C2_BASE;
    else if(base==3) base_to_use=I2C3_BASE;
    //specify that we are reading using the 7-bit slave address
    I2CMasterSlaveAddrSet(base_to_use, slave_addr, true);
  
    I2CMasterControl(base_to_use, I2C_MASTER_CMD_SINGLE_RECEIVE);
     
    //wait for MCU to finish transaction
    while(I2CMasterBusy(base_to_use));
     
//    //return data pulled from the specified register
    return I2CMasterDataGet(base_to_use);
}

uint32_t ADC_Read(){
  uint32_t ADCValues[1];
  //
  // Trigger the ADC conversion.
  //
  ADCProcessorTrigger(ADC0_BASE, 3);
  //
  // Wait for conversion to be completed.
  //
  while(!ADCIntStatus(ADC0_BASE, 3, false)){
  }
  //
  // Clear the ADC interrupt flag.
  //
  ADCIntClear(ADC0_BASE, 3);
  //
  // Read ADC Value.
  //
  ADCSequenceDataGet(ADC0_BASE, 3, ADCValues);
  SysCtlDelay(80000000 / 12);
  return ADCValues[0];
}

// END TM4C I2C and ADC comms
/***************************************************/ 

