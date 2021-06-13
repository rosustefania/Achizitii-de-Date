/***** Slave Program *****/

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

// The byte variable in which I2C reads from the master are performed
static char i2c_read_character = 0;


/***** *****************************************************************/
/***** Application Function Definitions *****/
/***** *****************************************************************/
void receiveEvent(int numBytes) {
  // TODO 3: read a character sent by the I2C master
  if (numBytes > 0 ) {
  	i2c_read_character = Wire.read();
  }
}

/***** *****************************************************************/
/***** Main application *****/
/***** *****************************************************************/
void setup()
{
  // TODO 4a: Set the serial baudrate
	Serial.begin(UART_BAUDRATE);
  
  // TODO 4b: Set the I2C slave address
	Wire.begin(I2C_SLAVE_ADDRESS);
  
  // TODO 4c: Set the on receive call-back function
	Wire.onReceive(receiveEvent);
}

void loop()
{
  if (i2c_read_character) {
  	Serial.print(i2c_read_character);
    i2c_read_character = 0;
  }
  // TODO 4d: If we recevide somthing on the I2C bus (other than zero)
  //       a. Print the caracter on the UART interface
  //       b. Reset the charecter to zero

    
}