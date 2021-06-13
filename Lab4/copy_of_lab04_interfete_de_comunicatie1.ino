/***** Master Program *****/

/***** *****************************************************************/
/***** Libraries *****/
/***** *****************************************************************/
#include <Wire.h>

/***** *****************************************************************/
/***** Constant & Macro Definitions *****/
/***** *****************************************************************/

// The baudrate used to set the UART interface
#define UART_BAUDRATE 115200

// The slave address to which this master will send data
#define I2C_SLAVE_ADDRESS 0x09


/***** *****************************************************************/
/***** Application Global Variables *****/
/***** *****************************************************************/

// The byte variable in which Serial reads from PC are performed
static char uart_read_byte;


/***** *****************************************************************/
/***** Main application *****/
/***** *****************************************************************/
void setup()
{
  // TODO 2a: Set the serial baudrate
	Serial.begin(UART_BAUDRATE);		
  
  // TODO 2b: Set the master i2c communication
	Wire.begin();
  
}

void loop()
{
  if (Serial.available() > 0) {
  	uart_read_byte = Serial.read();
    
    Wire.beginTransmission(I2C_SLAVE_ADDRESS);
  	Wire.write(uart_read_byte);
    Wire.endTransmission();
  }
  // TODO 2c: If something is available from the PC:
  //       a. read a character
  //       b. send it via i2c interface 2 the slave

    
}