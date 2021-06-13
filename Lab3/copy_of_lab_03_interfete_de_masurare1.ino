/***** *****************************************************************/
/***** Constant & Macro Definitions *****/
/***** *****************************************************************/

// Analog pin for the ADC
#define ADC_SENSOR_INPUT_PIN A0

// Temperature sensor voltage offset
#define TEMPERATURE_VOLTAGE_OFFSET 0.5

// ADC voltage reference
#define ADC_VOLTAGE_REFERENCE 5.0

// ADC 10-bit resolution 
#define ADC_RESOLUTION 1024

// Ultrasound pin
#define ULTRASOUND_SENSOR_PIN 12

// Ultrasound resolution in [pulse width / cm]
// See datasheet at https://www.cypress.com/file/55476/download
#define ULTRASONIC_PULSE_WIDTH_PER_CM 29

// The speed of sound propagation in [m/s]
#define SPEED_OF_SOUND 340

// The print counter limit
#define SENSORS_PRINT_UPDATE 100

/***** *****************************************************************/
/***** Application Function Definitions *****/
/***** *****************************************************************/
float adcReadingToVoltage(float adcRawReadingValue){
    float voltage = 0;
    voltage = adcRawReadingValue * 4.8 /1000;
    // TODO 2a: Convert the analog raw value to voltage value
    return voltage;
}

float computeTemperature(float voltage)
{
    float temperature = 0; 
    temperature = (voltage - TEMPERATURE_VOLTAGE_OFFSET) * 100;
    // TODO 2b: Compute the temperature from the a voltage value
    // Note: Don't forget to substract the voltage offset!
    return temperature;
}

void pulseOutUltrasonic(int ultrasoundPin){
  // TODO 3a: Send a pulse from the ultrasond sensor 
  // Note that according to the datasheet: 
  // - the pulse width must be 5 us long
  // - pulses must be separated by at least 2 us 
  	pinMode(ultrasoundPin, OUTPUT);
  // Set the ultrasound pin mode as output such that we can emit a pulse (see pinMode)
  	digitalWrite(ultrasoundPin, LOW);
  	delayMicroseconds(2);
  // Set the ultrasound pin low and wait 2 us
  	digitalWrite(ultrasoundPin, HIGH);
  	delayMicroseconds(5);
  // Set the ultrasound pin high and wait 5 us
	digitalWrite(ultrasoundPin, LOW);
  // Set the ultrasound pin low
  	pinMode(ultrasoundPin, INPUT);
  // Set the ultrasound pin mode as input such that we can recieve back the pulse
}	

float computeDistance(float duration){ 
    float cm;
    // TODO 3b: Convert the pulse duration to centimiters.
	cm = (duration / 2) * 340 * 0.0001;
    // Note: 
    // - The speed of sound is 340 m/s or 29 microseconds per centimeter (see defines).
    // - The pulse we send travels out and back, so to find the distance of the object
    //   take half of the duration travelled.

    return cm;
}

/***** *****************************************************************/
/***** Application Global Variables *****/
/***** *****************************************************************/
// Print counter
static long printStartTick;

// Variable to hold the raw ADC value
static float adcRawReadingValue;

// Variable to hold the measured voltage
static float voltage;

// Variable to hold the measured temperature
static float temperature;

// Variable to hold the pulse duration in micro-seconds
static float duration;

// Variable to hold the measured distance
static float distance;

/***** *****************************************************************/
/***** Main application *****/
/***** *****************************************************************/
void setup()
{
  Serial.begin(9600);
  
  pinMode(ADC_SENSOR_INPUT_PIN,INPUT);
    
  printStartTick = millis();
}

void loop()
{
  float adcRawReadingValue  = analogRead(ADC_SENSOR_INPUT_PIN);
  // TODO 3a: Read the analog raw value (see analogRead())
  float voltage = adcReadingToVoltage(adcRawReadingValue);
  // TODO 3b: Convert the analog value to voltage
  float temperature = computeTemperature(voltage);
  // TODO 3c: Convert the voltage value to temperature
  pulseOutUltrasonic(ULTRASOUND_SENSOR_PIN);
  // TODO 3d: Pulse the ultrasound sensor
  float duration = pulseIn(ULTRASOUND_SENSOR_PIN, HIGH);
  // TODO 3e: Get the duration of the ultrasound pin high state (see pulseIn())
  float distance = computeDistance(duration);
  // TODO 3f: Compute the distance from the duration
  
  // Printing measurements
  if((millis() - printStartTick) >= SENSORS_PRINT_UPDATE){
    printStartTick = millis();
    
    Serial.print(temperature);
    Serial.print(",");
    Serial.print(distance);
    Serial.println();
  }
}