#ifndef I2C_ADC_H
#define I2C_ADC_H


#define PART_TM4C123GH6PM

void InitI2C0(void);

void InitI2C1(void);
void InitI2C(uint8_t port);
void InitADC(void);
uint32_t I2CSend2bytes(uint8_t base, uint8_t slave_addr, uint8_t num_of_args, ...);

void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...);

void I2CSendString(uint8_t slave_addr, char array[]);

uint8_t I2CReceiveslave(uint8_t slave_addr);

uint32_t I2CReceive(uint8_t base, uint8_t slave_addr);

uint32_t ADC_Read();

#endif I2C_ADC_H
// END TM4C I2C and ADC comms
/***************************************************/ 

