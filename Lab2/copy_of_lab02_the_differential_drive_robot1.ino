#include <Encoder.h>
#include <limits.h>

/***** *****************************************************************/
/***** Constant & Macro Definitions *****/
/***** *****************************************************************/
// Sign macro
#define sign(x) ((x) < 0 ? -1 : 1)

// PWM Output Pins for Motor 1 H-Bridge
#define H1_A_PIN 5
#define H1_B_PIN 10

// PWM Output Pins for Motor 2 H-Bridge
#define H2_A_PIN 6
#define H2_B_PIN 11

// Encoder Input Pins for Motor 1 
#define ENC1_A_PIN 3
#define ENC1_B_PIN 7

// Encoder Input Pins for Motor 2 
#define ENC2_A_PIN 2
#define ENC2_B_PIN 8

// PWM Max Pulse Width
#define PWM_MAX 255

// PWM Max Duty Cycle
#define DUTY_CYCLE_MAX 100

// Encoder Count Per Revolution (cpr)
#define ENC_MAX_COUNT 2064

// Closed-Loop Update Time in ms
#define CLOSED_LOOP_UPDATE 20

// THe frequency at wich we will compute the (measured) rpm or rad/s
#define UPDATE_FREQ (1000/CLOSED_LOOP_UPDATE)

/***** *****************************************************************/
/***** Application Type Definitions *****/
/***** *****************************************************************/
// Motor ID's
typedef enum Motor{
  MOTOR1 = 0,
  MOTOR2 = 1
}Motor;

// Motor direction of rotation
typedef enum Direction{
  FORWARD = 1,
  BACKWARD = -1
}Direction;

// Motor spped estimator 
typedef struct SpeedEstimator{
  Encoder* enc;
  long old_enc_count;
}SpeedEstimator;

// PID
typedef struct PID{
  float setpoint;
  float input;
  float integral;
  float last_error;
  float saturation;
  float smoothing;
  float output;
  float kp;
  float ki;
  float kd;
}PID;

// Robot
typedef struct Robot{
  float r;
  float l;
  float omega_right;
  float omega_left;
  float x;
  float y;
  float theta;
}Robot;

/***** *****************************************************************/
/***** Application Function Definitions *****/
/***** *****************************************************************/

// TODO 2a: Initialize a Robot object
void initRobot(struct Robot* robot, float r, float l){
	robot->r = r;
  	robot->l = l;
  	robot->omega_right = 0;
  	robot->omega_left = 0;
  	robot->x = 0;
  	robot->y = 0;
  	robot->theta = 0;
}	

// TODO 2b: Compute commands for omega left and right
void setRobotCommand(struct Robot* robot, float v, float omega){
  // Compute omega left
  	robot->omega_left = (2 * v - omega * robot->l) / (2 * robot->r);
  // Compute omega right
  	robot->omega_right = (2 * v + omega * robot->l) / (2 * robot->r);
  
}

// TODO 2c: Compute the pose of the Robot
void updateRobotPose(struct Robot* robot, float rotation_left, float rotation_right){
  	float dl = 2 * PI * robot->r * rotation_left;	
  // Compute left wheel traveled distance
    float dr = 2 * PI * robot->l * rotation_right;
  // Compute right wheel traveled distance
    float d = (dl + dr) / 2;
  // Compute robot traveled distance
  	robot->x = robot->x + d * cos(robot->theta);
  // Comptue robot x coordinate
  	robot->y = robot->y + d * sin(robot->theta);
  // Comptue robot y coordinate
  	robot->theta = robot->theta + (dl - dr) / robot->l * sin(robot->theta);
  // Comptue robot heading (theta)
  
  // Note: There's an (algorithmic) problem with the heading computation.
  //       Do you know what exactly is the problem and how to solve it?
}

// TODO 3a: Initialize a SpeedEstimator object
void initSpeedEstimator(struct SpeedEstimator* est, struct Encoder *enc){
	est->enc = enc;
  	est->old_enc_count = 0;
}

// TODO 3b: Compute the motor rotation delta between two steps
float estimateSpeed(struct SpeedEstimator* est, int direction){
  // Read encoder
  	float encoder = est->enc->read();
  // Compute delta count
  	float delta = encoder - est->old_enc_count;
  // Take care of integer overflow while accounting for direction
  	if ((direction == FORWARD) && (delta < 0)) {
    	delta += LONG_MAX;
  	} else if ((direction == BACKWARD) && (delta > 0)) {
    	delta -= LONG_MAX;
  	} 
  // Compute the motor axel delta rotation in this instant
 	float rotation = delta / ENC_MAX_COUNT;
  // Store old count for next iteration
 	est->old_enc_count = encoder;
  
  return rotation;
}

// TODO 3c: Compute the motor axle rpm based on the update freq
float get_rpm(float delta_rotation, long update_freq){
  	float rpm = delta_rotation * update_freq * 60;
  	return rpm;
}

// TODO 3d: Compute the motor axle rad/s based on the update freq
float get_rad_per_second(float delta_rotation, long update_freq){
  	float rps = delta_rotation * update_freq * 2 * PI;
  	return rps;
}

// Initializes a pid controller
void initPID(struct PID* pid, float kp, float ki, float kd, float saturation, float smoothing){
  pid->setpoint = 0;
  pid->input = 0;
  pid->integral = 0;
  pid->last_error = 0;
  pid->output = 0;
  pid->saturation = saturation;
  pid->smoothing = smoothing;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

// Computes the PID output
void runPID(struct PID* pid){
  float P, I, D;
  float filtered_derivative;
  float derivative;
  
  // Compute the error
  float error = pid->setpoint - pid->input;

  // Take care of the sign
  if(pid->setpoint < 0) error = -error;
  
  // Compute the proportional term
  P = pid->kp * error;
  
  // Compute the integral term with clamping
  if(!(abs(pid->output) == pid->saturation && (sign(error) == sign(pid->output)))){
    pid->integral += error; 
  }
  
  // Compute the integral term\a
  I = pid->ki*pid->integral;
  
  // Compute the derivative
  derivative = pid->last_error - error;
  
  // Low pass filter the derivative
  filtered_derivative =  (1.0 - pid->smoothing)*pid->last_error + pid->smoothing*derivative;
  
  // Compute the derivative term
  D = pid->kd*filtered_derivative;
  
  // Compute the output
  pid->output = P + I + D;
  
  // Saturate the output
  if(pid->output > pid->saturation){
    pid->output = pid->saturation;
  }else if(pid->output < -pid->saturation){
    pid->output = -pid->saturation;
  }
  
  // Store last error for next iteration
  pid->last_error = error;
  
}

// Sets a motor axle direction of rotation PWM duty_cycle 
void setPWM(int motor, int direction, int duty_cycle){
  switch(motor){
    case MOTOR1:
    if(direction == FORWARD){
      analogWrite(H1_A_PIN, duty_cycle);
      analogWrite(H1_B_PIN, 0);
    }else{
      analogWrite(H1_A_PIN, 0);
      analogWrite(H1_B_PIN, duty_cycle);
    }
    break;
    case MOTOR2:
    if(direction == FORWARD){
      analogWrite(H2_A_PIN, duty_cycle);
      analogWrite(H2_B_PIN, 0);
    }else{
      analogWrite(H2_A_PIN, 0);
      analogWrite(H2_B_PIN, duty_cycle);
    }
    break;
  }
}

/***** *****************************************************************/
/***** Application Global Variables *****/
/***** *****************************************************************/
// Time stamps to track closed loop updates and plogs
long closed_loop_start_tick = 0;
long print_start_tick = 0;

// Encoder declaration for motor 1
Encoder enc1(ENC1_A_PIN,ENC1_B_PIN);

// Encoder declaration for motor 2
Encoder enc2(ENC2_A_PIN,ENC2_B_PIN);

// Speed estimator for left motor, i.e. motor 1
SpeedEstimator est_left;

// Speed estimator right motor, i.e. motor 2
SpeedEstimator est_right;

// Direction of the left motor, i.e. motor 1
int direction_left;

// Direction of the right motor, i.e. motor 2
int direction_right;

// Delta rotation for the left motor, i.e. motor 1
float delta_rotation_left;

// // Delta rotation for the right motor, i.e. motor 2
float delta_rotation_right;

// PID controller for left motor, i.e. motor 1
PID pid_left;

// PID controller for motor 2
PID pid_right;

// Robot
Robot robot;

/***** *****************************************************************/
/***** Main application *****/
/***** *****************************************************************/
void setup()
{
  // Set serial interface for printing.
  Serial.begin(115200);
  Serial.println("PID demo.");
  
  // Setting the H-bridge 1 pins
  pinMode(H1_A_PIN, OUTPUT);
  pinMode(H1_B_PIN, OUTPUT);
  
  // Setting the H-bridge 2 pins
  pinMode(H2_A_PIN, OUTPUT);
  pinMode(H2_B_PIN, OUTPUT);
  
  // Reset closed loop and print ticks
  closed_loop_start_tick = 0;
  print_start_tick = 0;

  // Intializes the speed estiomators for the left & right motors
  initSpeedEstimator(&est_left, &enc1);
  initSpeedEstimator(&est_right, &enc2);
  
  // Intializes the PID's for the left & right motors
  initPID(&pid_left, 1.2, 0.8, 0.012, (float) PWM_MAX, 1);  
  initPID(&pid_right, 1.2, 0.8, 0.012, (float) PWM_MAX, 1);
  
  // Reset delta rotations for the left & right motors
  delta_rotation_left = 0;
  delta_rotation_right = 0;
  
  // Initializes the robot
  initRobot(&robot, 0.04, 0.1);
}

void loop()
{
  	setRobotCommand(&robot, 0.5, 0.5);
  // TODO 4a: Set a robot command (ex: v = 0.5 m/s and omega = 0.5 rad/s)
  	if (robot.omega_left > 0 ){
    	direction_left = FORWARD;
  	} else {
    	direction_left = BACKWARD;
  	}
  
  	if (robot.omega_right > 0 ){
    	direction_right = FORWARD;
  	} else {
    	direction_right = BACKWARD;
  	}
  // TODO 4a: Compute the two motor axel direction of rotation
  
  
  // Motor control loop step 
  if((millis() - closed_loop_start_tick) >= CLOSED_LOOP_UPDATE){
    closed_loop_start_tick = millis();
    
   	pid_left.setpoint = robot.omega_left;
    pid_right.setpoint = robot.omega_right;
    // TODO 4b: Set left & right PID setpoints

	delta_rotation_left = estimateSpeed(&est_left, direction_left);
    delta_rotation_right = estimateSpeed(&est_right, direction_right);
    // TODO 4c: Compute the left & right delta rotations 
    
    pid_left.input = get_rad_per_second(delta_rotation_left, UPDATE_FREQ); 
    // TODO 4d: Set the left PID input
    runPID(&pid_left);
    // TODO 4d: Compute the left PID output
    pid_right.input = get_rad_per_second(delta_rotation_right, UPDATE_FREQ);
    // TODO 4d: Set the right PID input 
	runPID(&pid_right);
    // TODO 4d: Compute the right PID output
    setPWM(MOTOR1, direction_left, (int)pid_left.output);
    setPWM(MOTOR2, direction_right, (int)pid_right.output);
    // TODO 4e: Set the PWM for the left & right motors
    updateRobotPose(&robot, delta_rotation_left, delta_rotation_right);
    // TODO 4f: Update the robot pose
    
  }
  
  // Logging RMP loop step 
  if((millis() - print_start_tick) >= 100){
     print_start_tick = millis();
    
     // Print motor's rpm 
     Serial.print(get_rpm(delta_rotation_left, UPDATE_FREQ));
     Serial.print(", ");
     Serial.print(get_rpm(delta_rotation_right, UPDATE_FREQ));
     Serial.println();
  
    
     // Print robot pose
     
     Serial.print(robot.x);
     Serial.print(", ");
     Serial.print(robot.y);
	 Serial.print(", ");
	 Serial.print(robot.theta);
   	 Serial.println();
     
  }

}