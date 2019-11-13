

// Including other codes to be used in this program
#include "libraries/i2c_adc.h"

// I declare all variables and constants used in the program 

//
// This array is used for storing the data read from the ADC FIFO. It
// must be as large as the FIFO for the sequencer in use.  This example
// uses sequence 3 which has a FIFO depth of 1.  If another sequence
// was used with a deeper FIFO, then the array size must be changed.
// The FIFO gets the ADC sample and is a 32 bit word, so that is why 
// these are 32 bit variables. But nly the final 12 bits are adc 
// conversion values 
//
uint32_t ADCValues[1];
int i; // "int is a kind of variable" or type of variable
uint8_t to_write;
uint8_t command_byte;
uint8_t base_DAC;
uint32_t err_from_slave;
uint8_t PCF_I2C_WRITE_ADDRESS;
uint16_t write_2_bytes;
#define PCF8574_I2C_ADDRESS 32 // needs to be 7 bit address!
#define MAX_I2C_ADDRESS 32  // This is a special kind of type that is arbitray in size. 
#define PCF8574_I2C_READ_ADDRESS 32
// Energia specific code
void setup() {
    base_DAC=1;
    Serial.begin(9600);
    Serial.print("hello\n");
//    InitI2C(base_DAC);
    InitADC();
//    InitI2C1();
//    I2C_Init();
    i=0;
    command_byte=0;
}

void loop() {
  // put your main code here, to run repeatedly: 

  delay(1000);// do nothing/wait 1 second 
  Serial.print("i is ");
  Serial.print(i);
  Serial.print("\n"); // new line
  if(i<=255){
    to_write = (uint8_t) i;
//    write_2_bytes=((uint16_t)command_byte << 8) | to_write; 
//    err_from_slave = PCF(base_DAC,to_write);
//    err_from_slave = MAX(base_DAC,command_byte,to_write);

//    I2CSend2bytes(PCF8574_I2C_ADDRESS,write_2_bytes);  
//    I2CSend(PCF8574_I2C_ADDRESS,1, to_write);
//    read_from_slave = I2CReceive(PCF8574_I2C_ADDRESS,0);
//    read_from_slave = I2CReceiveslave(PCF8574_I2C_READ_ADDRESS);
    Serial.print("off:");
//    Serial.print(err_from_slave);
    Serial.print("\n");
    delay(50);
//    uint32_t read_val= I2CReceive(base_DAC, MAX_I2C_ADDRESS);
    delay(50);
    uint32_t read_adc_val=ADC_Read();
//    Serial.print("reading I2C: ");
//    Serial.print(read_val);
//    Serial.print("\n");
    Serial.print("reading ADC: ");
    Serial.print(read_adc_val);
    Serial.print("\n");
    i++;
  }
  else{
    i=0;
/*    to_write = 255;
//    write_2_bytes=((uint16_t)command_byte << 8) | to_write; 
//    err_from_slave= PCF(base_DAC,to_write);
    err_from_slave = MAX(base_DAC,command_byte,to_write);

//    I2CSend2bytes(PCF8574_I2C_ADDRESS,write_2_bytes);  
//    I2CSend(PCF8574_I2C_ADDRESS,1, to_write);    
//    read_from_slave = I2CReceive(PCF8574_I2C_ADDRESS,0);
//    read_from_slave = I2CReceiveslave(PCF8574_I2C_READ_ADDRESS);
    Serial.print("on: ");
    Serial.print(err_from_slave);
    Serial.print("\n");
//    delay(50);
//    uint32_t read_val= I2CReceive(base_DAC, MAX_I2C_ADDRESS);
    delay(50);
    uint32_t read_adc_val=ADC_Read();
//    Serial.print("reading I2C: ");
//    Serial.print(read_val);
//    Serial.print("\n");
    Serial.print("reading ADC: ");
    Serial.print(read_adc_val);
    Serial.print("\n");
*/
  }
  
}

uint32_t PCF(uint8_t base, uint8_t to_write){
    uint32_t err_from_slave=I2CSend2bytes(base, PCF8574_I2C_ADDRESS,1, to_write);
    return err_from_slave;
}

uint32_t MAX(uint8_t base, uint8_t command_byte, uint8_t to_write){
      uint32_t err_from_slave=I2CSend2bytes(base, MAX_I2C_ADDRESS,2,command_byte, to_write);
      return err_from_slave;
}
